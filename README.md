# lerobot_robot_piper

A [LeRobot](https://github.com/huggingface/lerobot) plugin for the **AgileX Piper**
6-DoF robot arm. Drop-in `--robot.type=piper_follower` for `lerobot-record`,
`lerobot-replay`, `lerobot-teleoperate`, and policy evaluation.

This plugin is built from real hardware experience. Its focus is **not** being the
first Piper wrapper — it's being the **safe** one: no startup lunge, smooth
home/rest interpolation, per-step slew-rate limiting, and proper unit handling
for both your own datasets (deg) and pretrained checkpoints (rad).

> Uses LeRobot's [plugin system](https://huggingface.co/docs/lerobot) (introduced
> in lerobot PR #2123), so it installs alongside LeRobot without patching core.

## Why this plugin (vs. a plain wrapper)

Most public Piper×LeRobot plugins are thin single-arm wrappers: they enable the
arm at full speed, send a single `JointCtrl` and disconnect, and only speak
degrees/percent. On real hardware that means the arm **lunges to its last
session's target on connect** and **can drop on disconnect**. This plugin fixes
each of those.

| Capability | This plugin | Typical plain wrapper |
|---|---|---|
| **Anti startup-lunge** — enable at 1% speed, overwrite the controller's stale `JointCtrl` target with current pose, *then* ramp to normal speed | ✅ `piper_follower.py` `connect()` (L95–114) | ❌ enable at full speed → lunges to old target |
| **Smooth home on connect** — optional smoothstep interpolation to a known pose | ✅ `_move_to_home()` (L130–152), `go_home_on_connect` | ❌ none |
| **Safe rest on disconnect** — smoothstep to a folded, power-off-safe pose so the arm doesn't drop | ✅ `_move_to_rest()` + `REST_STATE_DEG` (L281–343) | ❌ single `DisableArm()`, arm can fall |
| **Per-step slew-rate limit** — clamp relative motion per step | ✅ `max_relative_target` (L217–239) | ❌ none |
| **Joint-limit clamping** — every command clipped to safe ranges | ✅ `JOINT_LIMITS_DEG` (L18–25, L210–215) | ⚠️ often unclamped |
| **Units** — radians *or* degrees, converted internally (rad for pretrained checkpoints like ISdept, deg for your own datasets) | ✅ `unit` config (deg/rad) | ❌ deg/percent only |
| **MIT impedance mode** — per-joint `kp`/`kd` PD control for payload/compliance | ✅ `use_mit_mode` + `JointMitCtrl` (L241–253) | ❌ position control only |
| **Bimanual** — two Piper arms as one robot | ✅ `bi_piper_follower` | ⚠️ usually single-arm |

Line numbers refer to [`src/lerobot_robot_piper/piper_follower.py`](src/lerobot_robot_piper/piper_follower.py)
and [`config_piper_follower.py`](src/lerobot_robot_piper/config_piper_follower.py).

## Install

```bash
# In your LeRobot environment
pip install -e .
```

This pulls in `piper-sdk` and `numpy`. You also need a working CAN interface for
the Piper arm (e.g. `piper_left` / `can0`) brought up before connecting.

## Usage

The plugin self-registers with LeRobot via
`@RobotConfig.register_subclass("piper_follower")`, so after install the robot
type is available on every LeRobot CLI:

```bash
# Teleoperate / record / replay — pick your leader and dataset
lerobot-teleoperate \
    --robot.type=piper_follower \
    --robot.can_port=piper_left \
    --robot.speed_rate=50 \
    --teleop.type=...

# Use radians (e.g. when feeding a pretrained checkpoint trained in rad)
lerobot-record \
    --robot.type=piper_follower \
    --robot.unit=rad \
    --robot.max_relative_target=5.0 \
    ...
```

### Key config options

| Option | Default | Meaning |
|---|---|---|
| `can_port` | `piper_left` | CAN interface name (`can0`, `piper_left`, …) |
| `speed_rate` | `50` | MOVE-J speed percentage (0–100) |
| `unit` | `deg` | API unit: `deg` for your datasets, `rad` for pretrained checkpoints |
| `max_relative_target` | `None` | Per-step slew-rate limit (deg). `None` disables |
| `go_home_on_connect` | `False` | Smoothstep to `home_position_deg` after connect |
| `use_mit_mode` | `False` | `True` = MIT impedance (`JointCtrl`→`JointMitCtrl`, tunable `kp`/`kd`) |
| `gripper_effort` | `1000` | Gripper effort, 0.001 N·m units |

See [`config_piper_follower.py`](src/lerobot_robot_piper/config_piper_follower.py)
for the full list.

### Units at the API boundary

- **deg mode (default):** joint positions in degrees, gripper in mm.
- **rad mode:** joint positions in radians, gripper in meters.

The plugin always talks degrees/mm to the hardware (the `piper_sdk` uses
0.001-deg / 0.001-mm integer units internally) and converts at the boundary, so
your policy/dataset sees one consistent unit.

## Package layout

```
src/lerobot_robot_piper/
├── piper_follower.py            # PiperFollower driver (safety logic lives here)
├── config_piper_follower.py     # PiperFollowerConfig (registered as "piper_follower")
├── bi_piper_follower.py         # BiPiperFollower — two arms as one robot
├── config_bi_piper_follower.py  # BiPiperFollowerConfig
└── subprocess_arm.py            # subprocess worker for bimanual control
```

## License

[Apache-2.0](LICENSE).

This is an independent, community-maintained plugin and is not affiliated with
AgileX or Hugging Face. "Piper" and "AgileX" are trademarks of their respective
owners.
