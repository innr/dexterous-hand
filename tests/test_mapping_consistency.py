"""Identity JSON and Menagerie adapter join on canonical name."""
from __future__ import annotations

import json
from pathlib import Path

from simulation.canonical import (
    CANONICAL_NAMES,
    COUNT,
    load_identity_joints,
    resolve_identity_path,
    resolve_menagerie_path,
)


def test_identity_and_menagerie_join_by_canonical_name() -> None:
    identity = load_identity_joints()
    menagerie = json.loads(resolve_menagerie_path().read_text(encoding="utf-8"))["joints"]
    assert len(menagerie) == COUNT
    by_canonical = {row["canonical"]: row for row in menagerie}
    for joint in identity:
        adapter = by_canonical[joint["name"]]
        assert adapter["id"] == joint["hardware_id"]
    assert tuple(j["name"] for j in identity) == CANONICAL_NAMES
    data = json.loads(resolve_identity_path().read_text(encoding="utf-8"))
    assert data["calibration"]["status"] == "not_calibrated"
    assert data["hardware"]["position_unit"] == "radian_at_interface"


def test_side_limit_rejects_110() -> None:
    identity = load_identity_joints()
    assert identity[0]["limit_rad"][1] < 1.10
