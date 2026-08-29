from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from hardware.leap_hand_controller import (
    LeapHandBusController,
    joint_positions_to_ticks,
    ticks_to_joint_positions,
)
from simulation.leap_hand_model import joint_limits, load_mapping


MAPPING_PATH = Path(__file__).parents[1] / "config" / "joint_mapping.json"


class FakeBus:
    def __init__(self) -> None:
        self.goal_writes: list[tuple[int, int]] = []
        self.torque_writes: list[tuple[int, bool]] = []
        self.positions = {index: 2048 for index in range(16)}

    def write_goal_position_ticks(self, servo_id: int, ticks: int) -> None:
        self.goal_writes.append((servo_id, ticks))
        self.positions[servo_id] = ticks

    def read_position_ticks(self, servo_id: int) -> int:
        return self.positions[servo_id]

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        self.torque_writes.append((servo_id, enabled))


def calibrated_mapping() -> dict:
    mapping = load_mapping(MAPPING_PATH)
    mapping = deepcopy(mapping)
    mapping["calibration"]["status"] = "calibrated"
    for index, joint in enumerate(mapping["joints"]):
        joint["direction"] = -1 if index % 2 else 1
        joint["zero_offset_rad"] = 0.05
    return mapping


def test_position_tick_conversion_respects_direction_and_zero_offset() -> None:
    mapping = calibrated_mapping()
    lower, upper = joint_limits(mapping)
    target = np.clip(np.zeros(16), lower, upper)
    ticks = joint_positions_to_ticks(target, mapping)
    recovered = ticks_to_joint_positions(ticks, mapping)
    np.testing.assert_allclose(recovered, target, atol=2 * np.pi / 4096)


def test_controller_blocks_uncalibrated_motion() -> None:
    mapping = load_mapping(MAPPING_PATH)
    controller = LeapHandBusController(FakeBus(), mapping)
    with pytest.raises(RuntimeError, match="not calibrated"):
        controller.command_positions(np.zeros(16))


def test_controller_writes_canonical_ids_and_torque_is_explicit() -> None:
    mapping = calibrated_mapping()
    bus = FakeBus()
    controller = LeapHandBusController(bus, mapping)
    target = np.zeros(16)
    ticks = controller.command_positions(target, enable_torque=True)
    assert bus.goal_writes == [(index, int(ticks[index])) for index in range(16)]
    assert bus.torque_writes == [(index, True) for index in range(16)]
    np.testing.assert_allclose(controller.read_positions(), target, atol=2 * np.pi / 4096)
