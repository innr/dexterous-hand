import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "dexterous_hand_ros"
sys.path.insert(0, str(PACKAGE_ROOT))

from dexterous_hand_ros.validation import (  # noqa: E402
    CommandValidationError,
    canonical_joint_names,
    load_mapping,
    validate_joint_command,
)


MAPPING_PATH = Path(__file__).parents[1] / "config" / "joint_mapping.json"


def test_ros2_validation_accepts_canonical_command() -> None:
    mapping = load_mapping(MAPPING_PATH)
    names = canonical_joint_names(mapping)
    result = validate_joint_command([0.0] * 16, list(names), mapping)
    assert result.shape == (16,)


def test_ros2_validation_rejects_bad_command() -> None:
    mapping = load_mapping(MAPPING_PATH)
    try:
        validate_joint_command([float("nan")] * 16, [], mapping)
    except CommandValidationError:
        pass
    else:
        raise AssertionError("NaN command was accepted")
