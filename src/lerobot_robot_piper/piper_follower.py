import logging
import math
import time
from functools import cached_property
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_piper_follower import PiperFollowerConfig

logger = logging.getLogger(__name__)

# Piper joint limits in degrees
JOINT_LIMITS_DEG = {
    "joint_1": (-150.0, 150.0),
    "joint_2": (0.0, 180.0),
    "joint_3": (-170.0, 0.0),
    "joint_4": (-100.0, 100.0),
    "joint_5": (-70.0, 70.0),
    "joint_6": (-120.0, 120.0),
}
GRIPPER_RANGE_MM = (0.0, 70.0)

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def clamp_to_limits(goal_deg: dict[str, float]) -> dict[str, float]:
    """Clamp joint positions (deg) and gripper (mm) to the Piper's safe ranges.

    Keys are bare joint names (``joint_1`` .. ``joint_6``) and ``gripper``.
    """
    out = dict(goal_deg)
    for name, (lo, hi) in JOINT_LIMITS_DEG.items():
        if name in out:
            out[name] = float(np.clip(out[name], lo, hi))
    if "gripper" in out:
        out["gripper"] = float(np.clip(out["gripper"], *GRIPPER_RANGE_MM))
    return out


def apply_slew_limit(
    goal_deg: dict[str, float], current_deg: dict[str, float], max_delta: float
) -> dict[str, float]:
    """Limit each joint/gripper move to ``±max_delta`` per step, relative to ``current_deg``.

    Only keys present in both ``goal_deg`` and ``current_deg`` are limited. This is the
    per-step slew-rate guard against sudden large commands (sensor glitch, leader jump).
    """
    out = dict(goal_deg)
    for name in out:
        if name in current_deg:
            diff = float(np.clip(out[name] - current_deg[name], -max_delta, max_delta))
            out[name] = current_deg[name] + diff
    return out


class PiperFollower(Robot):
    """LeRobot-compatible driver for AgileX Piper robot arm.

    Units at API level:
      - Joint positions: degrees
      - Gripper position: mm (stroke)

    The piper_sdk uses 0.001 degree and 0.001 mm internally.
    """

    config_class = PiperFollowerConfig
    name = "piper_follower"

    def __init__(self, config: PiperFollowerConfig):
        super().__init__(config)
        self.config = config
        self.piper: Any = None  # piper_sdk C_PiperInterface_V2, set on connect()
        self._is_connected = False
        self.cameras = make_cameras_from_configs(config.cameras)

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {
            f"{name}.pos": float for name in JOINT_NAMES
        }
        features["gripper.pos"] = float
        for cam_name in self.cameras:
            cam_cfg = self.config.cameras[cam_name]
            features[cam_name] = (cam_cfg.height, cam_cfg.width, 3)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        features: dict[str, type] = {f"{name}.pos": float for name in JOINT_NAMES}
        features["gripper.pos"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._is_connected and all(
            cam.is_connected for cam in self.cameras.values()
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        from piper_sdk import C_PiperInterface_V2

        self.piper = C_PiperInterface_V2(self.config.can_port)
        self.piper.ConnectPort()

        # Enable all motors using EnablePiper (blocks until confirmed)
        logger.info("Enabling Piper arm...")
        enable_attempts = 0
        while not self.piper.EnablePiper():
            time.sleep(0.01)
            enable_attempts += 1
            if enable_attempts > 500:
                raise RuntimeError("Failed to enable Piper arm after 5 seconds")
        logger.info("Piper arm enabled.")

        # Prevent startup rush: the arm controller remembers the last
        # JointCtrl target from the previous session. If we enable MOVE_J
        # at full speed, it rushes to that old position.
        # Fix: enable at minimum speed, immediately send hold-in-place to
        # overwrite the stale target, then ramp up to normal speed.
        joint_msgs = self.piper.GetArmJointMsgs()
        js = joint_msgs.joint_state

        # Start MOVE_J at 1% speed — even if the stale target fires, movement is minimal
        self.piper.MotionCtrl_2(0x01, 0x01, 1, 0xAD)

        # Immediately overwrite stale target with current position (send multiple
        # times to ensure at least one is processed before the stale command)
        for _ in range(5):
            self.piper.JointCtrl(
                js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6
            )
        time.sleep(0.1)

        # Now safe to switch to normal speed
        self.piper.MotionCtrl_2(0x01, 0x01, self.config.speed_rate, 0xAD)

        # Enable gripper
        gripper_msgs = self.piper.GetArmGripperMsgs()
        current_grip = abs(gripper_msgs.gripper_state.grippers_angle)
        self.piper.GripperCtrl(current_grip, self.config.gripper_effort, 0x01, 0)

        for cam in self.cameras.values():
            cam.connect()

        self._is_connected = True
        logger.info("PiperFollower connected on %s", self.config.can_port)

        if self.config.go_home_on_connect:
            self._move_to_home()

    def _move_to_home(self) -> None:
        """Smoothstep interpolation to home position after connect."""
        logger.info("Moving to home position...")
        try:
            keys = [f"{n}.pos" for n in JOINT_NAMES] + ["gripper.pos"]
            current = self._get_current_deg()
            target = self.config.home_position_deg

            max_delta = max(abs(target[k] - current[k]) for k in keys)
            duration = max(max_delta / self._SAFE_SPEED, self._MIN_DURATION)

            steps = max(int(duration * self._CONTROL_RATE), 1)
            dt = 1.0 / self._CONTROL_RATE
            for i in range(steps):
                t = (i + 1) / steps
                t = t * t * (3 - 2 * t)  # smoothstep
                action = {k: current[k] + t * (target[k] - current[k]) for k in keys}
                self._send_action_deg(action)
                time.sleep(dt)
            logger.info("Home position reached.")
        except Exception as e:
            logger.warning("Failed to reach home position: %s", e)

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        joint_msgs = self.piper.GetArmJointMsgs()
        gripper_msgs = self.piper.GetArmGripperMsgs()

        js = joint_msgs.joint_state
        # SDK returns 0.001 degree; convert to degrees first
        j1 = js.joint_1 / 1000.0
        j2 = js.joint_2 / 1000.0
        j3 = js.joint_3 / 1000.0
        j4 = js.joint_4 / 1000.0
        j5 = js.joint_5 / 1000.0
        j6 = js.joint_6 / 1000.0
        grip = gripper_msgs.gripper_state.grippers_angle / 1000.0

        if self.config.unit == "rad":
            j1 = math.radians(j1)
            j2 = math.radians(j2)
            j3 = math.radians(j3)
            j4 = math.radians(j4)
            j5 = math.radians(j5)
            j6 = math.radians(j6)
            grip = grip / 1000.0  # mm → meters

        obs: RobotObservation = {
            "joint_1.pos": j1,
            "joint_2.pos": j2,
            "joint_3.pos": j3,
            "joint_4.pos": j4,
            "joint_5.pos": j5,
            "joint_6.pos": j6,
            "gripper.pos": grip,
        }

        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        goal = {
            key.removesuffix(".pos"): val
            for key, val in action.items()
            if key.endswith(".pos")
        }

        # If unit is rad, convert to degrees for internal processing
        if self.config.unit == "rad":
            for name in JOINT_NAMES:
                if name in goal:
                    goal[name] = math.degrees(goal[name])
            if "gripper" in goal:
                goal["gripper"] = goal["gripper"] * 1000.0  # meters → mm

        # Clamp to joint limits (always in degrees)
        goal = clamp_to_limits(goal)

        # Safety: limit relative movement per step (in degrees)
        if self.config.max_relative_target is not None:
            # get_observation returns in configured unit, convert to deg for comparison
            current_obs = self.get_observation()
            max_delta = self.config.max_relative_target
            if self.config.unit == "rad":
                max_delta = math.degrees(max_delta)
            current_deg: dict[str, float] = {}
            for name in JOINT_NAMES:
                key = f"{name}.pos"
                if key in current_obs:
                    current = current_obs[key]
                    current_deg[name] = (
                        math.degrees(current) if self.config.unit == "rad" else current
                    )
            if "gripper.pos" in current_obs:
                grip = current_obs["gripper.pos"]
                current_deg["gripper"] = (
                    grip * 1000.0 if self.config.unit == "rad" else grip
                )
            goal = apply_slew_limit(goal, current_deg, max_delta)

        if self.config.use_mit_mode:
            # MIT mode: send per-joint (pos, vel, kp, kd, t_ref). pos_ref in radians.
            # move_mode=0x04 (MOVE M) per SDK demo V2_piper_ctrl_joint_mit.py.
            self.piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
            for i, name in enumerate(JOINT_NAMES):
                self.piper.JointMitCtrl(
                    motor_num=i + 1,
                    pos_ref=math.radians(goal.get(name, 0.0)),
                    vel_ref=0.0,
                    kp=self.config.joint_kp,
                    kd=self.config.joint_kd,
                    t_ref=0.0,
                )
        else:
            # Position control: firmware's internal high-kp controller.
            # 0xAD here is trajectory smoothing flag in MOVE J context (see doc/03).
            j = [int(round(goal.get(name, 0.0) * 1000)) for name in JOINT_NAMES]
            self.piper.MotionCtrl_2(0x01, 0x01, self.config.speed_rate, 0xAD)
            self.piper.JointCtrl(j[0], j[1], j[2], j[3], j[4], j[5])

        # Gripper: convert mm to 0.001 mm
        gripper_val = int(round(goal.get("gripper", 0.0) * 1000))
        self.piper.GripperCtrl(abs(gripper_val), self.config.gripper_effort, 0x01, 0)

        return {
            f"{name}.pos": goal.get(name, 0.0) for name in JOINT_NAMES + ["gripper"]
        }

    @check_if_not_connected
    def disconnect(self) -> None:
        # Move to rest position before disabling to prevent the arm from dropping
        self._move_to_rest()

        if self.piper is not None:
            self.piper.DisableArm()
            self.piper.GripperCtrl(0, 0, 0x00, 0)
        for cam in self.cameras.values():
            cam.disconnect()
        self._is_connected = False
        logger.info("PiperFollower disconnected.")

    _SAFE_SPEED = 30.0  # deg/s
    _CONTROL_RATE = 100.0  # Hz
    _MIN_DURATION = 0.3  # seconds

    def _get_current_deg(self) -> dict[str, float]:
        """Get current joint positions in degrees, regardless of unit config."""
        obs = self.get_observation()
        keys = [f"{n}.pos" for n in JOINT_NAMES] + ["gripper.pos"]
        current = {}
        for k in keys:
            v = obs[k]
            if self.config.unit == "rad":
                if k == "gripper.pos":
                    v = v * 1000.0  # meters → mm
                else:
                    v = math.degrees(v)
            current[k] = v
        return current

    def _send_action_deg(self, action_deg: dict[str, float]) -> None:
        """Send action in degrees, converting if unit=rad."""
        if self.config.unit == "rad":
            action = {}
            for k, v in action_deg.items():
                if k == "gripper.pos":
                    action[k] = v / 1000.0  # mm → meters
                else:
                    action[k] = math.radians(v)
        else:
            action = action_deg
        self.send_action(action)

    def _move_to_rest(self) -> None:
        """Smoothstep interpolation to rest position before disconnect."""
        logger.info("Moving to rest position...")
        try:
            keys = [f"{n}.pos" for n in JOINT_NAMES] + ["gripper.pos"]
            current = self._get_current_deg()
            target = self.config.rest_position_deg

            max_delta = max(abs(target[k] - current[k]) for k in keys)
            duration = max(max_delta / self._SAFE_SPEED, self._MIN_DURATION)

            steps = max(int(duration * self._CONTROL_RATE), 1)
            dt = 1.0 / self._CONTROL_RATE
            for i in range(steps):
                t = (i + 1) / steps
                t = t * t * (3 - 2 * t)  # smoothstep
                action = {k: current[k] + t * (target[k] - current[k]) for k in keys}
                self._send_action_deg(action)
                time.sleep(dt)
            logger.info("Rest position reached.")
        except Exception as e:
            logger.warning("Failed to reach rest position: %s", e)
