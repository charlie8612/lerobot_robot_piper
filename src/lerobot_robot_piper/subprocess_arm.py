"""Run a PiperFollower in a subprocess to avoid GIL contention.

The piper_sdk C_PiperInterface_V2 creates a background CAN receive thread.
When two instances exist in the same process, their background threads compete
for the Python GIL, causing ~10x slowdown on all operations. Running the second
arm in a subprocess gives it its own GIL, matching single-arm performance.
"""
import multiprocessing as mp
import logging

logger = logging.getLogger(__name__)


def _follower_worker(config_dict, pipe):
    """Subprocess entry point for PiperFollower."""
    from .config_piper_follower import PiperFollowerConfig
    from .piper_follower import PiperFollower

    config = PiperFollowerConfig(**config_dict)
    follower = PiperFollower(config)

    try:
        while True:
            msg = pipe.recv()
            cmd = msg[0]

            if cmd == "connect":
                follower.connect(msg[1])
                pipe.send(("ok",))
            elif cmd == "get_observation":
                obs = follower.get_observation()
                pipe.send(("ok", obs))
            elif cmd == "send_action":
                result = follower.send_action(msg[1])
                pipe.send(("ok", result))
            elif cmd == "disconnect":
                follower.disconnect()
                pipe.send(("ok",))
                break
            elif cmd == "observation_features":
                pipe.send(("ok", follower.observation_features))
            elif cmd == "action_features":
                pipe.send(("ok", follower.action_features))
            elif cmd == "is_connected":
                pipe.send(("ok", follower.is_connected))
    except (EOFError, BrokenPipeError):
        try:
            follower.disconnect()
        except Exception:
            pass


class SubprocessFollower:
    """Proxy that runs a PiperFollower in a separate process."""

    def __init__(self, config):
        self.config = config
        self._parent_pipe, self._child_pipe = mp.Pipe()
        self._process = None
        # Cache features (don't need subprocess for these)
        from .config_piper_follower import PiperFollowerConfig
        from .piper_follower import PiperFollower
        temp = PiperFollower(PiperFollowerConfig(
            can_port=config.can_port,
            speed_rate=config.speed_rate,
            max_relative_target=config.max_relative_target,
            gripper_effort=config.gripper_effort,
            cameras=config.cameras,
        ))
        self._observation_features = temp.observation_features
        self._action_features = temp.action_features
        self.cameras = temp.cameras

    @property
    def observation_features(self):
        return self._observation_features

    @property
    def action_features(self):
        return self._action_features

    @property
    def is_connected(self):
        return self._process is not None and self._process.is_alive()

    def connect(self, calibrate=True):
        config_dict = {
            "can_port": self.config.can_port,
            "speed_rate": self.config.speed_rate,
            "max_relative_target": self.config.max_relative_target,
            "gripper_effort": self.config.gripper_effort,
            "cameras": self.config.cameras,
        }
        self._process = mp.Process(
            target=_follower_worker,
            args=(config_dict, self._child_pipe),
            daemon=True,
        )
        self._process.start()
        self._parent_pipe.send(("connect", calibrate))
        resp = self._parent_pipe.recv()
        if resp[0] != "ok":
            raise RuntimeError(f"Subprocess connect failed: {resp}")
        logger.info("SubprocessFollower connected (pid=%d)", self._process.pid)

    def get_observation(self):
        self._parent_pipe.send(("get_observation",))
        resp = self._parent_pipe.recv()
        return resp[1]

    def send_action(self, action):
        self._parent_pipe.send(("send_action", action))
        resp = self._parent_pipe.recv()
        return resp[1]

    def disconnect(self):
        if self._process and self._process.is_alive():
            self._parent_pipe.send(("disconnect",))
            try:
                self._parent_pipe.recv()
            except EOFError:
                pass
            self._process.join(timeout=3)
            if self._process.is_alive():
                self._process.terminate()
        logger.info("SubprocessFollower disconnected.")
