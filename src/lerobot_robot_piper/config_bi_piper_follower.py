from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig

from .config_piper_follower import PiperFollowerBaseConfig


@RobotConfig.register_subclass("bi_piper_follower")
@dataclass
class BiPiperFollowerConfig(RobotConfig):
    """Configuration for bimanual (dual) Piper follower arms."""

    left_arm_config: PiperFollowerBaseConfig = field(
        default_factory=lambda: PiperFollowerBaseConfig(can_port="piper_left")
    )
    right_arm_config: PiperFollowerBaseConfig = field(
        default_factory=lambda: PiperFollowerBaseConfig(can_port="piper_right")
    )
