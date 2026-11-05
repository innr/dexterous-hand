"""Pure validation helpers shared by the ROS 2 node and tests."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


class CommandValidationError(ValueError):
    """A joint command is malformed or outside configured limits."""


def load_mapping(path: Path) -> dict[str, Any]:
    import json

    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def canonical_joint_names(mapping: dict[str, Any]) -> tuple[str, ...]:
    joints = sorted(mapping["joints"], key=lambda item: int(item["hardware_id"]))
    expected = tuple(range(len(joints)))
    actual = tuple(int(item["hardware_id"]) for item in joints)
    if actual != expected:
        raise CommandValidationError("hardware IDs must be contiguous from zero")
    return tuple(str(item["name"]) for item in joints)


def joint_limits(mapping: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    joints = sorted(mapping["joints"], key=lambda item: int(item["hardware_id"]))
    limits = np.asarray([item["limit_rad"] for item in joints], dtype=float)
    if limits.ndim != 2 or limits.shape[1] != 2 or np.any(limits[:, 0] >= limits[:, 1]):
        raise CommandValidationError("invalid joint limits")
    return limits[:, 0], limits[:, 1]


def validate_joint_command(
    positions: Any,
    names: list[str] | tuple[str, ...] | None,
    mapping: dict[str, Any],
) -> np.ndarray:
    """Return a canonical 16-vector or raise before touching a backend."""
    expected_names = canonical_joint_names(mapping)
    lower, upper = joint_limits(mapping)
    values = np.asarray(positions, dtype=float)
    if values.shape != lower.shape:
        raise CommandValidationError(f"expected {len(lower)} positions, got {values.shape}")
    if not np.all(np.isfinite(values)):
        raise CommandValidationError("positions must be finite")
    if names and tuple(names) != expected_names:
        raise CommandValidationError("name must be empty or canonical joint order")
    if np.any(values < lower) or np.any(values > upper):
        raise CommandValidationError("position is outside configured limits")
    return values.copy()
