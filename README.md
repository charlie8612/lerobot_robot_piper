# LeRobot + AgileX Piper Integration

Brings a [LeRobot](https://github.com/huggingface/lerobot) integration for the
**AgileX Piper** 6-DoF arm. Registers `piper_follower` (and `bi_piper_follower`
for two arms), so it works with `lerobot-teleoperate`, `lerobot-record`,
`lerobot-replay`, and policy evaluation without patching LeRobot.

The focus is being the **safe** Piper plugin: no startup lunge, smooth
home/rest interpolation, per-step slew-rate limiting, and consistent unit
handling for both your own datasets (deg) and pretrained checkpoints (rad).

## Getting Started

```bash
# In your LeRobot environment
pip install git+https://github.com/charlie8612/lerobot_robot_piper.git

# Bring up the Piper CAN interface first (e.g. piper_left / can0), then:
lerobot-teleoperate \
    --robot.type=piper_follower \
    --robot.can_port=piper_left \
    --robot.speed_rate=50 \
    --teleop.type=...
```

The plugin self-registers via `@RobotConfig.register_subclass("piper_follower")`,
so the robot type is available on every LeRobot CLI after install.

## Why this plugin

Most public Piper×LeRobot wrappers enable the arm at full speed, send a single
`JointCtrl`, and disconnect — so the arm **lunges to its last target on connect**
and **can drop on disconnect**. This plugin fixes each:

| Capability | This plugin | Typical wrapper |
|---|---|---|
| Anti startup-lunge (enable at 1%, overwrite stale target, then ramp) | ✅ | ❌ |
| Smooth home on connect / safe rest on disconnect | ✅ | ❌ |
| Per-step slew-rate limit (`max_relative_target`) | ✅ | ❌ |
| Joint-limit clamping on every command | ✅ | ⚠️ |
| Units: radians *or* degrees, converted internally | ✅ | ❌ deg only |
| MIT impedance mode (tunable `kp` / `kd`) | ✅ | ❌ |
| Bimanual (`bi_piper_follower`) | ✅ | ⚠️ |

## Configuration

| Option | Default | Meaning |
|---|---|---|
| `can_port` | `piper_left` | CAN interface (`can0`, `piper_left`, …) |
| `speed_rate` | `50` | MOVE-J speed percentage (0–100) |
| `unit` | `deg` | API unit: `deg` for datasets, `rad` for pretrained checkpoints |
| `max_relative_target` | `None` | Per-step slew-rate limit (deg); `None` disables |
| `go_home_on_connect` | `False` | Smoothstep to `home_position_deg` after connect |
| `use_mit_mode` | `False` | MIT impedance control (tunable `kp` / `kd`) |
| `gripper_effort` | `1000` | Gripper effort, in 0.001 N·m units |

See [`config_piper_follower.py`](src/lerobot_robot_piper/config_piper_follower.py)
for the full list. The plugin always talks degrees/mm to the hardware and
converts at the API boundary, so your policy/dataset sees one consistent unit.

## Development

```bash
git clone https://github.com/charlie8612/lerobot_robot_piper.git
cd lerobot_robot_piper
pip install -e ".[dev]"
pytest -q
```

## License

[Apache-2.0](LICENSE). Independent, community-maintained plugin, not affiliated
with AgileX or Hugging Face. "Piper" and "AgileX" are trademarks of their
respective owners.
