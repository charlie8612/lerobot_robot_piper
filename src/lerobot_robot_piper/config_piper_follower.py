from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@dataclass
class PiperFollowerBaseConfig:
    """Base configuration for AgileX Piper robot arm (not registered with draccus)."""

    # CAN interface name (e.g., "piper_left", "can0")
    can_port: str = "piper_left"

    # Speed rate percentage for MotionCtrl_2 (0-100)
    speed_rate: int = 50

    # Max relative target per step (degrees). Limits sudden large movements.
    # Set to None to disable safety clamping.
    max_relative_target: float | None = None

    # Gripper default effort in 0.001 N*m (1000 = 1 N*m)
    gripper_effort: int = 1000

    # Control mode selector:
    #   False (default) = MOVE J + JointCtrl (firmware's internal high-kp position
    #     controller + trajectory smoothing). Precise waypoint tracking. Use for
    #     scripted motion, waypoint replay, recording.
    #   True = MOVE M + JointMitCtrl (user-controlled PD: kp, kd, t_ref).
    #     Tunable stiffness, useful when carrying payload or for impedance control.
    use_mit_mode: bool = False

    # MIT mode per-joint gains (only used when use_mit_mode=True).
    # Higher kp = stiffer position tracking (better under load, but more vibration).
    # Defaults are SDK reference values. SDK ranges: kp [0, 500], kd [-5, 5].
    joint_kp: float = 10.0
    joint_kd: float = 0.8

    # Unit for joint angles: "deg" (default, for your own datasets) or "rad" (for ISdept etc.)
    # When "rad", the plugin converts rad↔deg internally so the API uses radians
    # but the hardware still receives degrees.
    unit: str = "deg"

    # Move to home position on connect. Ensures a consistent starting pose.
    go_home_on_connect: bool = False

    # Home position in degrees (always degrees, converted internally if unit=rad)
    home_position_deg: dict[str, float] = field(default_factory=lambda: {
        "joint_1.pos": 0.0,
        "joint_2.pos": 0.0,
        "joint_3.pos": 0.0,
        "joint_4.pos": 0.0,
        "joint_5.pos": 0.0,
        "joint_6.pos": 0.0,
        "gripper.pos": 0.0,
    })

    # Log each policy inference chunk to console
    log_inference: bool = False

    # Cameras (empty by default, add in Phase 3)
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


@RobotConfig.register_subclass("piper_follower")
@dataclass
class PiperFollowerConfig(RobotConfig, PiperFollowerBaseConfig):
    """Configuration for AgileX Piper robot arm as a follower."""
    pass
