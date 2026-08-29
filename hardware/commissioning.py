"""Hardware-independent STS3215 commissioning calculations and safety helpers."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from typing import Any, Mapping, Sequence


class CommissioningError(ValueError):
    """Raised when a calibration or commissioning artifact is unsafe."""


@dataclass(frozen=True)
class DirectionObservation:
    hardware_id: int
    start_ticks: int
    end_ticks: int
    commanded_delta_rad: float

    @property
    def direction(self) -> int | None:
        """Return +1/-1, or None when movement is too small to decide."""
        delta_ticks = self.end_ticks - self.start_ticks
        if delta_ticks == 0 or self.commanded_delta_rad == 0:
            return None
        return 1 if delta_ticks * self.commanded_delta_rad > 0 else -1


def infer_direction(
    start_ticks: int,
    end_ticks: int,
    commanded_delta_rad: float,
    *,
    min_tick_delta: int = 2,
) -> int | None:
    """Infer encoder direction from a deliberately small test movement."""
    if not 0 <= start_ticks <= 4095 or not 0 <= end_ticks <= 4095:
        raise CommissioningError("encoder ticks must be in [0, 4095]")
    if not math.isfinite(commanded_delta_rad):
        raise CommissioningError("commanded_delta_rad must be finite")
    if min_tick_delta < 1:
        raise CommissioningError("min_tick_delta must be positive")
    delta_ticks = end_ticks - start_ticks
    if abs(delta_ticks) < min_tick_delta or commanded_delta_rad == 0:
        return None
    return 1 if delta_ticks * commanded_delta_rad > 0 else -1


def ticks_to_radians(ticks: int, *, reference_ticks: int = 2048) -> float:
    if not 0 <= ticks <= 4095:
        raise CommissioningError("encoder ticks must be in [0, 4095]")
    if not 0 <= reference_ticks <= 4095:
        raise CommissioningError("reference_ticks must be in [0, 4095]")
    return (ticks - reference_ticks) * 2.0 * math.pi / 4096.0


def radians_to_ticks(radians: float, *, reference_ticks: int = 2048) -> int:
    if not math.isfinite(radians):
        raise CommissioningError("radians must be finite")
    if not 0 <= reference_ticks <= 4095:
        raise CommissioningError("reference_ticks must be in [0, 4095]")
    return max(0, min(4095, round(reference_ticks + radians * 4096.0 / (2.0 * math.pi))))


def load_json_mapping(path) -> dict[str, Any]:
    import json
    from pathlib import Path

    try:
        mapping = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError) as exc:
        raise CommissioningError(f"cannot read mapping: {path}") from exc
    validate_mapping(mapping)
    return mapping


def validate_mapping(mapping: Mapping[str, Any]) -> None:
    joints = mapping.get("joints")
    if not isinstance(joints, list) or len(joints) != 16:
        raise CommissioningError("mapping must contain exactly 16 joints")
    ids = []
    for joint in joints:
        try:
            servo_id = int(joint["hardware_id"])
            limits = joint["limit_rad"]
            direction = int(joint.get("direction", 1))
            offset = float(joint.get("zero_offset_rad", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise CommissioningError("mapping joint entry is invalid") from exc
        if servo_id in ids or not 0 <= servo_id <= 253:
            raise CommissioningError("mapping hardware IDs must be unique and in [0, 253]")
        if (
            direction not in (-1, 1)
            or not isinstance(limits, (list, tuple))
            or len(limits) != 2
            or limits[0] > limits[1]
        ):
            raise CommissioningError(f"invalid limits or direction for hardware ID {servo_id}")
        if not math.isfinite(offset):
            raise CommissioningError(f"invalid zero offset for hardware ID {servo_id}")
        ids.append(servo_id)


def validate_capture(capture: Mapping[str, Any], mapping: Mapping[str, Any]) -> None:
    validate_mapping(mapping)
    if capture.get("schema_version") != 1:
        raise CommissioningError("unsupported calibration capture schema")
    reference_ticks = capture.get("reference_ticks")
    if not isinstance(reference_ticks, int) or not 0 <= reference_ticks <= 4095:
        raise CommissioningError("capture reference_ticks must be in [0, 4095]")
    observations = capture.get("observations")
    if not isinstance(observations, list) or not observations:
        raise CommissioningError("capture must contain observations")
    mapping_ids = {int(joint["hardware_id"]) for joint in mapping["joints"]}
    observed_ids: set[int] = set()
    for observation in observations:
        try:
            servo_id = int(observation["hardware_id"])
            ticks = int(observation["present_position_ticks"])
        except (KeyError, TypeError, ValueError) as exc:
            raise CommissioningError("capture observation is invalid") from exc
        if servo_id in observed_ids or servo_id not in mapping_ids:
            raise CommissioningError(f"capture contains unexpected or duplicate ID {servo_id}")
        if not 0 <= ticks <= 4095:
            raise CommissioningError(f"capture ticks for ID {servo_id} are out of range")
        observed_ids.add(servo_id)


def calibration_candidates(
    mapping: Mapping[str, Any], capture: Mapping[str, Any]
) -> list[dict[str, Any]]:
    """Return reviewed candidates without mutating either input mapping."""
    validate_capture(capture, mapping)
    joints_by_id = {int(joint["hardware_id"]): joint for joint in mapping["joints"]}
    candidates = []
    for observation in capture["observations"]:
        servo_id = int(observation["hardware_id"])
        candidate = {
            "hardware_id": servo_id,
            "name": joints_by_id[servo_id]["name"],
            "zero_offset_rad": ticks_to_radians(
                int(observation["present_position_ticks"]),
                reference_ticks=int(capture["reference_ticks"]),
            ),
        }
        if observation.get("direction") in (-1, 1):
            candidate["direction"] = int(observation["direction"])
        candidates.append(candidate)
    return candidates


def apply_calibration(
    mapping: Mapping[str, Any],
    capture: Mapping[str, Any],
    *,
    confirmed: bool = False,
    mark_verified: bool = False,
) -> dict[str, Any]:
    """Apply a reviewed capture to a copy of the mapping.

    ``confirmed`` is intentionally mandatory for writes. ``mark_verified``
    requires every captured joint to include a direction result.
    """
    if not confirmed:
        raise CommissioningError(
            "calibration application requires explicit confirmation; review the capture first"
        )
    candidates = calibration_candidates(mapping, capture)
    if mark_verified and (
        len(candidates) != 16
        or any("direction" not in candidate for candidate in candidates)
    ):
        raise CommissioningError(
            "cannot mark calibration verified without direction results for all 16 joints"
        )
    updated = deepcopy(dict(mapping))
    joints_by_id = {int(joint["hardware_id"]): joint for joint in updated["joints"]}
    for candidate in candidates:
        joint = joints_by_id[candidate["hardware_id"]]
        joint["zero_offset_rad"] = candidate["zero_offset_rad"]
        if "direction" in candidate:
            joint["direction"] = candidate["direction"]
    calibration = dict(updated.get("calibration", {}))
    calibration["status"] = "verified" if mark_verified else "candidate"
    calibration["reference_ticks"] = int(capture["reference_ticks"])
    calibration["captured_at"] = capture.get("captured_at")
    calibration["applied_at"] = datetime.now(timezone.utc).isoformat()
    updated["calibration"] = calibration
    validate_mapping(updated)
    return updated


def apply_joint_calibration(raw_angle_rad: float, *, zero_offset_rad: float, direction: int) -> float:
    """Convert a raw encoder-relative angle into the canonical joint angle."""
    if direction not in (-1, 1):
        raise CommissioningError("direction must be +1 or -1")
    if not math.isfinite(raw_angle_rad) or not math.isfinite(zero_offset_rad):
        raise CommissioningError("angles must be finite")
    return direction * (raw_angle_rad - zero_offset_rad)


def load_capture(path) -> dict[str, Any]:
    import json
    from pathlib import Path

    try:
        capture = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError) as exc:
        raise CommissioningError(f"cannot read calibration capture: {path}") from exc
    if not isinstance(capture, dict):
        raise CommissioningError("calibration capture must be a JSON object")
    return capture
