"""Hardware-independent bridge from ROS 2-like messages to episode data.

Callbacks accept duck-typed messages, so the core remains usable in tests
without importing rclpy. A ROS 2 node can forward its message fields here.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any

JOINT_COUNT = 16

class BridgeError(ValueError):
    pass

@dataclass(frozen=True)
class JointSample:
    timestamp_ns: int
    positions: tuple[float, ...]

class EpisodeBridge:
    def __init__(self, *, joint_count: int = JOINT_COUNT):
        if joint_count <= 0:
            raise BridgeError("joint_count must be positive")
        self.joint_count = joint_count
        self.commands: list[JointSample] = []
        self.states: list[JointSample] = []

    def on_joint_command(self, message: Any, *, timestamp_ns: int) -> None:
        self.commands.append(self._sample(message, timestamp_ns))

    def on_joint_state(self, message: Any, *, timestamp_ns: int) -> None:
        self.states.append(self._sample(message, timestamp_ns))

    def episode(self) -> dict[str, list[dict[str, object]]]:
        return {"commands": [self._as_record(s) for s in self.commands], "states": [self._as_record(s) for s in self.states]}

    def write_hdf5(self, path: str) -> None:
        try:
            import h5py
            import numpy as np
        except ImportError as exc:
            raise BridgeError("install h5py and numpy for HDF5 export") from exc
        episode = self.episode()
        with h5py.File(path, "w") as handle:
            handle.attrs["schema_version"] = 1
            handle.attrs["joint_count"] = self.joint_count
            for name in ("commands", "states"):
                rows = episode[name]
                handle.create_dataset(f"{name}/timestamp_ns", data=np.asarray([r["timestamp_ns"] for r in rows], dtype=np.int64))
                handle.create_dataset(f"{name}/positions", data=np.asarray([r["positions"] for r in rows], dtype=np.float64).reshape((-1, self.joint_count)))

    def _sample(self, message: Any, timestamp_ns: int) -> JointSample:
        values = getattr(message, "positions", message)
        try:
            positions = tuple(float(x) for x in values)
        except (TypeError, ValueError) as exc:
            raise BridgeError("message positions must be numeric") from exc
        if len(positions) != self.joint_count:
            raise BridgeError(f"expected {self.joint_count} joint positions")
        if timestamp_ns < 0:
            raise BridgeError("timestamp_ns must be non-negative")
        return JointSample(timestamp_ns, positions)

    @staticmethod
    def _as_record(sample: JointSample) -> dict[str, object]:
        return {"timestamp_ns": sample.timestamp_ns, "positions": sample.positions}
