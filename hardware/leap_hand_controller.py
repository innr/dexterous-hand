"""Safety-gated canonical position interface for a LEAP hand bus."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from simulation.leap_hand_model import joint_limits

from .sts3215 import STS3215Bus, radians_to_ticks, ticks_to_radians


def _canonical_joints(mapping: dict[str, Any]) -> list[dict[str, Any]]:
    joints = sorted(mapping["joints"], key=lambda entry: int(entry["hardware_id"]))
    if [int(entry["hardware_id"]) for entry in joints] != list(range(len(joints))):
        raise ValueError("hardware IDs must be contiguous from zero")
    return joints


def joint_positions_to_ticks(target: np.ndarray, mapping: dict[str, Any]) -> np.ndarray:
    """Convert canonical joint radians to STS3215 encoder ticks.

    The mapping convention is ``servo_angle = direction * joint_angle +
    zero_offset_rad``.  ``zero_offset_rad`` is the encoder angle measured at
    the mechanical joint zero and must be filled only after calibration.
    """
    target = np.asarray(target, dtype=float)
    joints = _canonical_joints(mapping)
    lower, upper = joint_limits(mapping)
    if target.shape != lower.shape:
        raise ValueError(f"Expected {len(joints)} target values, got {target.shape}")
    if np.any(target < lower) or np.any(target > upper):
        raise ValueError("target contains a joint position outside configured limits")
    return np.asarray(
        [
            radians_to_ticks(
                int(joint["direction"]) * float(angle) + float(joint["zero_offset_rad"])
            )
            for angle, joint in zip(target, joints)
        ],
        dtype=int,
    )


def ticks_to_joint_positions(ticks: np.ndarray, mapping: dict[str, Any]) -> np.ndarray:
    """Convert canonical STS3215 ticks to joint radians."""
    ticks = np.asarray(ticks, dtype=int)
    joints = _canonical_joints(mapping)
    if ticks.shape != (len(joints),):
        raise ValueError(f"Expected {len(joints)} tick values, got {ticks.shape}")
    return np.asarray(
        [
            int(joint["direction"])
            * (ticks_to_radians(int(value)) - float(joint["zero_offset_rad"]))
            for value, joint in zip(ticks, joints)
        ],
        dtype=float,
    )


@dataclass
class LeapHandBusController:
    """Explicit, calibration-gated bridge from canonical targets to servos."""

    bus: STS3215Bus
    mapping: dict[str, Any]
    allow_uncalibrated: bool = False

    def __post_init__(self) -> None:
        _canonical_joints(self.mapping)

    def _require_calibrated(self) -> None:
        status = self.mapping.get("calibration", {}).get("status")
        if status != "calibrated" and not self.allow_uncalibrated:
            raise RuntimeError(
                "hardware mapping is not calibrated; review calibration capture first "
                "or explicitly use allow_uncalibrated=True for bench testing"
            )

    def target_ticks(self, target: np.ndarray) -> np.ndarray:
        """Validate and convert a canonical target without writing the bus."""
        return joint_positions_to_ticks(target, self.mapping)

    def command_positions(self, target: np.ndarray, *, enable_torque: bool = False) -> np.ndarray:
        """Write one canonical target; torque is enabled only when requested."""
        self._require_calibrated()
        ticks = self.target_ticks(target)
        joints = _canonical_joints(self.mapping)
        if enable_torque:
            self.set_torque_all(True)
        for joint, value in zip(joints, ticks):
            self.bus.write_goal_position_ticks(int(joint["hardware_id"]), int(value))
        return ticks

    def read_positions(self) -> np.ndarray:
        """Read all servo positions and return canonical joint radians."""
        self._require_calibrated()
        joints = _canonical_joints(self.mapping)
        ticks = np.asarray(
            [self.bus.read_position_ticks(int(joint["hardware_id"])) for joint in joints],
            dtype=int,
        )
        return ticks_to_joint_positions(ticks, self.mapping)

    def set_torque_all(self, enabled: bool) -> None:
        """Explicitly enable or disable torque on every mapped servo."""
        for joint in _canonical_joints(self.mapping):
            self.bus.set_torque(int(joint["hardware_id"]), enabled)
