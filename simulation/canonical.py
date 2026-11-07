"""S7-00 canonical 16-DOF types (radians). Unwired to the C++ control loop.

Degrees stay inside the S5 core and STS3215 transport. This module is the
application-facing layout: names, rad, session_id, epoch, sequence.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
import os
from pathlib import Path
from typing import Sequence

COUNT = 16
NAME_MAX = 31
FUTURE_TOLERANCE_US = 10_000
MAX_AGE_US = 100_000

# Frozen identity, hardware_id 0..15, from config/joint_mapping.json joints[].name.
CANONICAL_NAMES: tuple[str, ...] = (
    "index_mcp_side",
    "index_mcp_forward",
    "index_pip",
    "index_dip",
    "middle_mcp_side",
    "middle_mcp_forward",
    "middle_pip",
    "middle_dip",
    "ring_mcp_side",
    "ring_mcp_forward",
    "ring_pip",
    "ring_dip",
    "thumb_cmc_side",
    "thumb_cmc_forward",
    "thumb_mcp",
    "thumb_ip",
)


def project_root() -> Path:
    root = os.environ.get("LEAP_PROJECT_ROOT")
    if root:
        return Path(root)
    return Path(__file__).resolve().parents[1]


def resolve_identity_path() -> Path:
    mapping = os.environ.get("LEAP_JOINT_MAPPING")
    if mapping:
        return Path(mapping)
    return project_root() / "config" / "joint_mapping.json"


def resolve_menagerie_path() -> Path:
    mapping = os.environ.get("LEAP_MENAGERIE_MAPPING")
    if mapping:
        return Path(mapping)
    return project_root() / "config" / "menagerie_right_mapping.json"


def load_identity_joints(path: Path | None = None) -> list[dict]:
    data = json.loads((path or resolve_identity_path()).read_text(encoding="utf-8"))
    joints = data["joints"]
    if data.get("schema_version") != 1:
        raise ValueError("schema_version must be 1")
    if data.get("hardware", {}).get("position_unit") != "radian_at_interface":
        raise ValueError("hardware.position_unit must be radian_at_interface")
    if len(joints) != COUNT or [j["hardware_id"] for j in joints] != list(range(COUNT)):
        raise ValueError("identity mapping must have hardware_id 0..15")
    if tuple(j["name"] for j in joints) != CANONICAL_NAMES:
        raise ValueError("identity names must match frozen canonical table")
    return joints


class CommandReject(str, Enum):
    STALE_SESSION = "stale_session"
    ZERO_SESSION = "zero_session"
    STALE_EPOCH = "stale_epoch"
    BAD_SEQUENCE = "bad_sequence"
    BAD_NAMES = "bad_names"
    NON_FINITE = "non_finite"
    LIMIT = "limit"
    TTL = "ttl"


class CommandContractError(ValueError):
    def __init__(self, code: CommandReject, message: str) -> None:
        super().__init__(message)
        self.code = code.value


@dataclass(frozen=True)
class CanonicalCommand:
    stamp_us: int
    session_id: int
    epoch: int
    sequence: int
    names: tuple[str, ...]
    positions: tuple[float, ...]


@dataclass(frozen=True)
class CanonicalState:
    stamp_us: int
    session_id: int
    epoch: int
    sequence: int
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


def _reject(code: CommandReject, message: str) -> None:
    raise CommandContractError(code, message)


def reorder_to_canonical(
    names: Sequence[str],
    positions: Sequence[float],
    expected_names: Sequence[str] = CANONICAL_NAMES,
) -> list[float]:
    """Session-boundary reorder. Not for the 100 Hz control tick."""
    if (len(expected_names) != COUNT or len(names) != COUNT or
            len(positions) != COUNT):
        _reject(CommandReject.BAD_NAMES, "expected 16 names and positions")
    if any(not isinstance(n, str) or not n or len(n) > NAME_MAX for n in names):
        _reject(CommandReject.BAD_NAMES, "name empty or longer than 31 characters")
    if len(set(names)) != COUNT or set(names) != set(expected_names):
        _reject(CommandReject.BAD_NAMES, "expected 16 unique canonical names")
    if any(not math.isfinite(v) for v in positions):
        _reject(CommandReject.NON_FINITE, "invalid command positions")
    lookup = dict(zip(names, positions))
    return [float(lookup[name]) for name in expected_names]


def validate_canonical_command(
    command: CanonicalCommand, *,
    session_id: int,
    names: Sequence[str] = CANONICAL_NAMES,
    lower: Sequence[float],
    upper: Sequence[float],
    epoch: int,
    last_sequence: int,
    sim_time_us: int,
) -> list[float]:
    """Reject before mutating last_sequence / authorized target."""
    if command.session_id == 0:
        _reject(CommandReject.ZERO_SESSION, "session_id 0 is invalid")
    if session_id == 0:
        _reject(CommandReject.ZERO_SESSION, "expected session_id 0 is invalid")
    if command.session_id != session_id:
        _reject(CommandReject.STALE_SESSION, "stale simulation session")
    if command.epoch != epoch:
        _reject(CommandReject.STALE_EPOCH, "stale simulation epoch")
    if not 0 <= command.sequence < 2**64 or command.sequence <= last_sequence:
        _reject(CommandReject.BAD_SEQUENCE, "duplicate or out-of-order sequence")
    if command.stamp_us < 0 or command.stamp_us > sim_time_us + FUTURE_TOLERANCE_US or (
            sim_time_us - command.stamp_us > MAX_AGE_US):
        _reject(CommandReject.TTL, "stale/future simulation-time command")
    ordered = reorder_to_canonical(command.names, command.positions, names)
    if len(lower) != COUNT or len(upper) != COUNT:
        _reject(CommandReject.LIMIT, "limit vectors must have 16 entries")
    if any(not lower[i] <= value <= upper[i] for i, value in enumerate(ordered)):
        _reject(CommandReject.LIMIT, "command exceeds joint/actuator/C++ limit intersection")
    return ordered
