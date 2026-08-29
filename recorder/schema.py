"""Stable data contracts for recording and replay."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np

HDF5_SCHEMA_VERSION = 1
JOINT_COUNT = 16
CANONICAL_JOINT_NAMES = (
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

VALID_REASON_CODES = {
    "ok": 0,
    "missing_command": 1,
    "low_confidence": 2,
    "invalid_input": 3,
    "out_of_order_timestamp": 4,
    "backend_error": 5,
}


class SampleValidationError(ValueError):
    """Raised when an episode frame violates the data contract."""


class DataIntegrityError(ValueError):
    """Raised when an HDF5 episode is incomplete or corrupt."""


class OptionalDependencyError(RuntimeError):
    """Raised when an optional recording/export dependency is unavailable."""


@dataclass(frozen=True)
class EpisodeFrame:
    """One synchronized action/state sample."""

    timestamp_ns: int
    action_position_rad: Sequence[float]
    observation_position_rad: Sequence[float]
    observation_velocity_rad_s: Sequence[float]
    valid: bool = True
    valid_reason: int = VALID_REASON_CODES["ok"]


def _as_vector(value: Sequence[float], field_name: str) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError) as exc:
        raise SampleValidationError(f"{field_name} must be numeric") from exc
    if vector.shape != (JOINT_COUNT,):
        raise SampleValidationError(
            f"{field_name} must have shape ({JOINT_COUNT},), got {vector.shape}"
        )
    if not np.isfinite(vector).all():
        raise SampleValidationError(f"{field_name} must contain only finite values")
    return vector


def normalize_frame(frame: EpisodeFrame) -> EpisodeFrame:
    """Validate and normalize a frame before persistent storage."""

    if not isinstance(frame.timestamp_ns, (int, np.integer)):
        raise SampleValidationError("timestamp_ns must be an integer nanosecond value")
    if int(frame.timestamp_ns) < 0:
        raise SampleValidationError("timestamp_ns must be non-negative")
    if not isinstance(frame.valid, (bool, np.bool_)):
        raise SampleValidationError("valid must be boolean")
    if not isinstance(frame.valid_reason, (int, np.integer)):
        raise SampleValidationError("valid_reason must be an integer code")
    if not 0 <= int(frame.valid_reason) <= 255:
        raise SampleValidationError("valid_reason must fit in uint8")
    return EpisodeFrame(
        timestamp_ns=int(frame.timestamp_ns),
        action_position_rad=_as_vector(frame.action_position_rad, "action_position_rad"),
        observation_position_rad=_as_vector(
            frame.observation_position_rad, "observation_position_rad"
        ),
        observation_velocity_rad_s=_as_vector(
            frame.observation_velocity_rad_s, "observation_velocity_rad_s"
        ),
        valid=bool(frame.valid),
        valid_reason=int(frame.valid_reason),
    )


def metadata_json(metadata: Mapping[str, Any] | None) -> str:
    import json

    try:
        return json.dumps(dict(metadata or {}), ensure_ascii=False, sort_keys=True)
    except (TypeError, ValueError) as exc:
        raise SampleValidationError("metadata must be JSON serializable") from exc
