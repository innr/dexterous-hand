"""S6-03 ROS-independent validation for dedicated JointCommand.

ROS2 message serialization and DDS delivery are tested separately in the
Jazzy integration workflow. This function guards application semantics.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

COUNT = 16
FUTURE_TOLERANCE_US = 10_000
MAX_AGE_US = 100_000


@dataclass(frozen=True)
class JointCommandData:
    stamp_us: int
    epoch: int
    sequence: int
    names: tuple[str, ...]
    positions: tuple[float, ...]


def validate_command(command: JointCommandData, *, names: Sequence[str],
                     lower: Sequence[float], upper: Sequence[float],
                     epoch: int, last_sequence: int, sim_time_us: int) -> list[float]:
    if command.epoch != epoch:
        raise ValueError("stale simulation epoch")
    if not 0 <= command.sequence < 2**64 or command.sequence <= last_sequence:
        raise ValueError("duplicate or out-of-order sequence")
    if len(command.names) != COUNT or len(set(command.names)) != COUNT or (
            set(command.names) != set(names)):
        raise ValueError("expected 16 unique official joint names")
    if len(command.positions) != COUNT or any(not math.isfinite(v) for v in command.positions):
        raise ValueError("invalid command positions")
    if command.stamp_us < 0 or command.stamp_us > sim_time_us + FUTURE_TOLERANCE_US or (
            sim_time_us - command.stamp_us > MAX_AGE_US):
        raise ValueError("stale/future simulation-time command")
    lookup = dict(zip(command.names, command.positions))
    ordered = [float(lookup[name]) for name in names]
    if any(not lower[i] <= value <= upper[i] for i, value in enumerate(ordered)):
        raise ValueError("command exceeds joint/actuator/C++ limit intersection")
    return ordered
