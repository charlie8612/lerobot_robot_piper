from lerobot_robot_piper import PiperFollower, PiperFollowerConfig
from lerobot_robot_piper.piper_follower import (
    GRIPPER_RANGE_MM,
    JOINT_LIMITS_DEG,
    apply_slew_limit,
    clamp_to_limits,
)


# --- config / schema -------------------------------------------------------


def test_config_defaults():
    cfg = PiperFollowerConfig()
    assert cfg.can_port == "piper_left"
    assert cfg.unit == "deg"
    assert cfg.max_relative_target is None
    assert cfg.speed_rate == 50


def test_features_schema():
    robot = PiperFollower(PiperFollowerConfig())
    expected = [f"joint_{i}.pos" for i in range(1, 7)] + ["gripper.pos"]
    assert list(robot.action_features) == expected
    # observation has the same joint/gripper keys (plus cameras, none here)
    for k in expected:
        assert k in robot.observation_features


# --- clamp_to_limits -------------------------------------------------------


def test_clamp_over_and_under_limits():
    goal = {"joint_1": 999.0, "joint_3": 999.0, "gripper": 999.0}
    out = clamp_to_limits(goal)
    assert out["joint_1"] == JOINT_LIMITS_DEG["joint_1"][1]  # clamped to upper
    assert out["joint_3"] == JOINT_LIMITS_DEG["joint_3"][1]  # joint_3 upper is 0.0
    assert out["gripper"] == GRIPPER_RANGE_MM[1]


def test_clamp_leaves_in_range_untouched():
    goal = {"joint_1": 10.0, "gripper": 35.0}
    out = clamp_to_limits(goal)
    assert out["joint_1"] == 10.0
    assert out["gripper"] == 35.0


def test_clamp_does_not_mutate_input():
    goal = {"joint_1": 999.0}
    clamp_to_limits(goal)
    assert goal["joint_1"] == 999.0


# --- apply_slew_limit ------------------------------------------------------


def test_slew_limits_large_jump():
    goal = {"joint_1": 100.0}
    current = {"joint_1": 0.0}
    out = apply_slew_limit(goal, current, max_delta=5.0)
    assert out["joint_1"] == 5.0  # capped to current + max_delta


def test_slew_allows_small_move():
    goal = {"joint_1": 3.0}
    current = {"joint_1": 0.0}
    out = apply_slew_limit(goal, current, max_delta=5.0)
    assert out["joint_1"] == 3.0


def test_slew_negative_direction():
    goal = {"joint_2": -100.0}
    current = {"joint_2": 0.0}
    out = apply_slew_limit(goal, current, max_delta=5.0)
    assert out["joint_2"] == -5.0


def test_slew_ignores_keys_without_current():
    goal = {"joint_1": 100.0, "joint_6": 50.0}
    current = {"joint_1": 0.0}  # no joint_6
    out = apply_slew_limit(goal, current, max_delta=5.0)
    assert out["joint_1"] == 5.0
    assert out["joint_6"] == 50.0  # untouched (no reference position)
