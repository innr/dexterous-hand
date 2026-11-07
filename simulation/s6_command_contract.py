"""S6-03/S7-00 ROS-independent validation for dedicated JointCommand.

ROS2 message serialization and DDS delivery are tested separately in the
Jazzy integration workflow. This function guards application semantics.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .canonical import (
    CANONICAL_NAMES,
    COUNT,
    CommandContractError,
    CommandReject,
    FUTURE_TOLERANCE_US,
    MAX_AGE_US,
)


@dataclass(frozen=True)
class JointCommandData:
    stamp_us: int
    session_id: int
    epoch: int
    sequence: int
    names: tuple[str, ...]
    positions: tuple[float, ...]


def validate_command(command: JointCommandData, *, names: Sequence[str],
                     lower: Sequence[float], upper: Sequence[float],
                     session_id: int, epoch: int, last_sequence: int,
                     sim_time_us: int) -> list[float]:
    if command.session_id == 0 or session_id == 0:
        raise CommandContractError(CommandReject.ZERO_SESSION, "session_id 0 is invalid")
    if command.session_id != session_id:
        raise CommandContractError(CommandReject.STALE_SESSION, "stale simulation session")
    if command.epoch != epoch:
        raise CommandContractError(CommandReject.STALE_EPOCH, "stale simulation epoch")
    if not 0 <= command.sequence < 2**64 or command.sequence <= last_sequence:
        raise CommandContractError(CommandReject.BAD_SEQUENCE,
                                   "duplicate or out-of-order sequence")
    if len(command.names) != COUNT or len(set(command.names)) != COUNT or (
            set(command.names) != set(names)):
        raise CommandContractError(CommandReject.BAD_NAMES,
                                   "expected 16 unique official joint names")
    if len(command.positions) != COUNT or any(not math.isfinite(v) for v in command.positions):
        raise CommandContractError(CommandReject.NON_FINITE, "invalid command positions")
    if command.stamp_us < 0 or command.stamp_us > sim_time_us + FUTURE_TOLERANCE_US or (
            sim_time_us - command.stamp_us > MAX_AGE_US):
        raise CommandContractError(CommandReject.TTL, "stale/future simulation-time command")
    lookup = dict(zip(command.names, command.positions))
    ordered = [float(lookup[name]) for name in names]
    if any(not lower[i] <= value <= upper[i] for i, value in enumerate(ordered)):
        raise CommandContractError(
            CommandReject.LIMIT,
            "command exceeds joint/actuator/C++ limit intersection")
    return ordered
