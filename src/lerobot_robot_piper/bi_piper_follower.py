import logging
from functools import cached_property

from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_bi_piper_follower import BiPiperFollowerConfig
from .config_piper_follower import PiperFollowerConfig
from .piper_follower import PiperFollower
from .subprocess_arm import SubprocessFollower

logger = logging.getLogger(__name__)


class BiPiperFollower(Robot):
    """Bimanual (dual) Piper follower.

    Left arm runs in the main process. Right arm runs in a subprocess to avoid
    GIL contention from piper_sdk's background CAN receive threads.
    """

    config_class = BiPiperFollowerConfig
    name = "bi_piper_follower"

    def __init__(self, config: BiPiperFollowerConfig):
        super().__init__(config)
        self.config = config

        left_arm_config = PiperFollowerConfig(
            id=f"{config.id}_left" if config.id else None,
            can_port=config.left_arm_config.can_port,
            speed_rate=config.left_arm_config.speed_rate,
            max_relative_target=config.left_arm_config.max_relative_target,
            gripper_effort=config.left_arm_config.gripper_effort,
            cameras=config.left_arm_config.cameras,
        )

        self.left_arm = PiperFollower(left_arm_config)
        self.right_arm = SubprocessFollower(config.right_arm_config)
        self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        left_ft = self.left_arm.observation_features
        right_ft = self.right_arm.observation_features
        return {
            **{f"left_{k}": v for k, v in left_ft.items()},
            **{f"right_{k}": v for k, v in right_ft.items()},
        }

    @cached_property
    def action_features(self) -> dict[str, type]:
        left_ft = self.left_arm.action_features
        right_ft = self.right_arm.action_features
        return {
            **{f"left_{k}": v for k, v in left_ft.items()},
            **{f"right_{k}": v for k, v in right_ft.items()},
        }

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.left_arm.connect(calibrate)
        self.right_arm.connect(calibrate)

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        left_obs = self.left_arm.get_observation()
        right_obs = self.right_arm.get_observation()
        return {
            **{f"left_{k}": v for k, v in left_obs.items()},
            **{f"right_{k}": v for k, v in right_obs.items()},
        }

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        left_action = {k.removeprefix("left_"): v for k, v in action.items() if k.startswith("left_")}
        right_action = {k.removeprefix("right_"): v for k, v in action.items() if k.startswith("right_")}

        sent_left = self.left_arm.send_action(left_action)
        sent_right = self.right_arm.send_action(right_action)

        return {
            **{f"left_{k}": v for k, v in sent_left.items()},
            **{f"right_{k}": v for k, v in sent_right.items()},
        }

    @check_if_not_connected
    def disconnect(self) -> None:
        self.left_arm.disconnect()
        self.right_arm.disconnect()
