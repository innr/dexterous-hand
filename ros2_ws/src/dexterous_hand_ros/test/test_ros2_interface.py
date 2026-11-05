from pathlib import Path

import numpy as np
import pytest

from dexterous_hand_ros.backends import VirtualBackend
from dexterous_hand_ros.node import CommandWatchdog
from dexterous_hand_ros.validation import (
    CommandValidationError,
    canonical_joint_names,
    load_mapping,
    validate_joint_command,
)


MAPPING_PATH = Path(__file__).parents[4] / "config" / "joint_mapping.json"


def test_command_validation_accepts_canonical_positions() -> None:
    mapping = load_mapping(MAPPING_PATH)
    names = canonical_joint_names(mapping)
    values = np.zeros(16)
    result = validate_joint_command(values, names, mapping)
    np.testing.assert_array_equal(result, values)


@pytest.mark.parametrize(
    "values,names",
    [
        (np.zeros(15), []),
        (np.full(16, np.nan), []),
        (np.full(16, np.inf), []),
        (np.full(16, 100.0), []),
    ],
)
def test_command_validation_rejects_invalid_positions(values: np.ndarray, names: list[str]) -> None:
    mapping = load_mapping(MAPPING_PATH)
    with pytest.raises(CommandValidationError):
        validate_joint_command(values, names, mapping)


def test_command_validation_rejects_wrong_names() -> None:
    mapping = load_mapping(MAPPING_PATH)
    with pytest.raises(CommandValidationError):
        validate_joint_command(np.zeros(16), ["wrong"] * 16, mapping)


def test_watchdog_timeout_and_recovery() -> None:
    watchdog = CommandWatchdog(timeout_s=0.25, action="disable_torque")
    assert watchdog.expired(0.0)
    watchdog.command_received(1.0)
    assert not watchdog.expired(1.24)
    assert watchdog.expired(1.25)
    watchdog.timed_out = True
    watchdog.command_received(1.30)
    assert not watchdog.timed_out


def test_virtual_backend_round_trip() -> None:
    backend = VirtualBackend()
    target = np.linspace(-0.1, 0.1, 16)
    backend.command_positions(target)
    np.testing.assert_array_equal(backend.read_positions(), target)
    assert backend.command_count == 1
    backend.set_torque(False)
    assert not backend.torque_enabled
