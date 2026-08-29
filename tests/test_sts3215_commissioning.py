from __future__ import annotations

import math

import pytest

from hardware.commissioning import (
    CommissioningError,
    apply_calibration,
    apply_joint_calibration,
    calibration_candidates,
    infer_direction,
    load_json_mapping,
    radians_to_ticks,
    ticks_to_radians,
)
from hardware.motion_test import MotionSafetyError, execute_motion_test, make_motion_plan


def make_mapping():
    return {
        "schema_version": 1,
        "joints": [
            {
                "hardware_id": index,
                "name": f"joint_{index}",
                "direction": 1,
                "zero_offset_rad": 0.0,
                "limit_rad": [-1.0, 1.0],
            }
            for index in range(16)
        ],
        "calibration": {"status": "not_calibrated"},
    }


def make_capture(*, include_direction=False):
    observations = []
    for index in range(2):
        observation = {
            "hardware_id": index,
            "present_position_ticks": 2048 + index * 64,
        }
        if include_direction:
            observation["direction"] = 1 if index == 0 else -1
        observations.append(observation)
    return {
        "schema_version": 1,
        "reference_ticks": 2048,
        "observations": observations,
    }


def test_direction_and_tick_conversion():
    assert infer_direction(2048, 2060, 0.1) == 1
    assert infer_direction(2048, 2036, 0.1) == -1
    assert infer_direction(2048, 2049, 0.1, min_tick_delta=2) is None
    with pytest.raises(CommissioningError):
        infer_direction(0, 4096, 0.1)

    ticks = radians_to_ticks(0.5)
    assert abs(ticks_to_radians(ticks) - 0.5) < 0.002


def test_candidates_do_not_mutate_mapping():
    mapping = make_mapping()
    capture = make_capture(include_direction=True)
    candidates = calibration_candidates(mapping, capture)
    assert candidates[0]["zero_offset_rad"] == 0.0
    assert candidates[1]["direction"] == -1
    assert mapping["joints"][1]["zero_offset_rad"] == 0.0


def test_apply_requires_review_and_writes_copy():
    mapping = make_mapping()
    capture = make_capture(include_direction=True)
    with pytest.raises(CommissioningError, match="confirmation"):
        apply_calibration(mapping, capture)

    updated = apply_calibration(mapping, capture, confirmed=True)
    assert updated is not mapping
    assert updated["calibration"]["status"] == "candidate"
    assert updated["joints"][1]["direction"] == -1
    assert mapping["calibration"]["status"] == "not_calibrated"

    with pytest.raises(CommissioningError, match="verified"):
        apply_calibration(mapping, make_capture(), confirmed=True, mark_verified=True)


def test_canonical_angle_transform():
    assert apply_joint_calibration(0.4, zero_offset_rad=0.1, direction=1) == pytest.approx(0.3)
    assert apply_joint_calibration(0.4, zero_offset_rad=0.1, direction=-1) == pytest.approx(-0.3)
    with pytest.raises(CommissioningError):
        apply_joint_calibration(math.nan, zero_offset_rad=0.0, direction=1)


class FakeBus:
    def __init__(self):
        self.calls = []

    def set_torque(self, servo_id, enabled):
        self.calls.append(("torque", servo_id, enabled))

    def write_goal_position_ticks(self, servo_id, ticks):
        self.calls.append(("goal", servo_id, ticks))

    def read_position_ticks(self, servo_id):
        self.calls.append(("read", servo_id))
        return 2048


def test_motion_test_has_explicit_safety_gate():
    plan = make_motion_plan(3, 0.1)
    with pytest.raises(MotionSafetyError, match="--confirm"):
        execute_motion_test(FakeBus(), plan, enable_torque=False, confirmed=False)

    bus = FakeBus()
    assert execute_motion_test(bus, plan, enable_torque=True, confirmed=True) == 2048
    assert bus.calls[0] == ("torque", 3, True)
    assert bus.calls[-1] == ("torque", 3, False)
