"""S7-00 PR-2: canonical 16-DOF layout and reject codes. No JointTable, no S5 core."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from simulation.canonical import (
    CANONICAL_NAMES,
    COUNT,
    NAME_MAX,
    CanonicalCommand,
    CanonicalState,
    CommandContractError,
    CommandReject,
    reorder_to_canonical,
    validate_canonical_command,
)
from simulation.s6_command_contract import JointCommandData, validate_command

ROOT = Path(__file__).resolve().parents[1]
MAPPING = ROOT / "config" / "joint_mapping.json"


def test_canonical_names_match_joint_mapping_identity() -> None:
    joints = json.loads(MAPPING.read_text(encoding="utf-8"))["joints"]
    assert [j["hardware_id"] for j in joints] == list(range(COUNT))
    assert tuple(j["name"] for j in joints) == CANONICAL_NAMES
    assert max(len(n) for n in CANONICAL_NAMES) <= NAME_MAX
    assert len("middle_mcp_forward") == 18


def test_reorder_to_canonical_shuffled_names() -> None:
    names = tuple(reversed(CANONICAL_NAMES))
    positions = tuple(float(i) for i in range(COUNT))
    ordered = reorder_to_canonical(names, positions)
    assert ordered == [float(COUNT - 1 - i) for i in range(COUNT)]


def test_reorder_rejects_menagerie_names() -> None:
    names = ("if_rot",) + CANONICAL_NAMES[1:]
    with pytest.raises(CommandContractError) as exc:
        reorder_to_canonical(names, [0.0] * COUNT)
    assert exc.value.code == CommandReject.BAD_NAMES.value


def test_reorder_rejects_long_or_non_finite() -> None:
    long_name = "x" * (NAME_MAX + 1)
    names = (long_name,) + CANONICAL_NAMES[1:]
    with pytest.raises(CommandContractError) as exc:
        reorder_to_canonical(names, [0.0] * COUNT)
    assert exc.value.code == CommandReject.BAD_NAMES.value
    with pytest.raises(CommandContractError) as exc:
        reorder_to_canonical(CANONICAL_NAMES, [float("nan")] * COUNT)
    assert exc.value.code == CommandReject.NON_FINITE.value


def _cmd(**kwargs) -> CanonicalCommand:
    base = dict(
        stamp_us=100_000,
        session_id=99,
        epoch=2,
        sequence=8,
        names=tuple(reversed(CANONICAL_NAMES)),
        positions=tuple(0.01 * i for i in range(COUNT)),
    )
    base.update(kwargs)
    return CanonicalCommand(**base)


def test_validate_canonical_command_reorders_and_rejects_session() -> None:
    lower = [-1.0] * COUNT
    upper = [1.0] * COUNT
    ordered = validate_canonical_command(
        _cmd(), session_id=99, lower=lower, upper=upper,
        epoch=2, last_sequence=7, sim_time_us=100_000)
    assert ordered == [0.01 * (COUNT - 1 - i) for i in range(COUNT)]

    with pytest.raises(CommandContractError) as exc:
        validate_canonical_command(
            _cmd(session_id=0), session_id=99, lower=lower, upper=upper,
            epoch=2, last_sequence=7, sim_time_us=100_000)
    assert exc.value.code == CommandReject.ZERO_SESSION.value

    with pytest.raises(CommandContractError) as exc:
        validate_canonical_command(
            _cmd(session_id=7), session_id=99, lower=lower, upper=upper,
            epoch=2, last_sequence=7, sim_time_us=100_000)
    assert exc.value.code == CommandReject.STALE_SESSION.value


def test_validate_canonical_command_codes() -> None:
    lower = [-0.5] * COUNT
    upper = [0.5] * COUNT

    def check(command, **kwargs):
        args = dict(session_id=99, lower=lower, upper=upper, epoch=2,
                    last_sequence=7, sim_time_us=100_000)
        args.update(kwargs)
        validate_canonical_command(command, **args)

    with pytest.raises(CommandContractError) as exc:
        check(_cmd(epoch=1))
    assert exc.value.code == CommandReject.STALE_EPOCH.value
    with pytest.raises(CommandContractError) as exc:
        check(_cmd(sequence=7))
    assert exc.value.code == CommandReject.BAD_SEQUENCE.value
    with pytest.raises(CommandContractError) as exc:
        check(_cmd(stamp_us=0), sim_time_us=100_001)
    assert exc.value.code == CommandReject.TTL.value
    with pytest.raises(CommandContractError) as exc:
        check(_cmd(positions=tuple([1.0] * COUNT)))
    assert exc.value.code == CommandReject.LIMIT.value


def test_canonical_state_layout_is_radians() -> None:
    state = CanonicalState(
        stamp_us=1, session_id=2, epoch=3, sequence=4,
        names=CANONICAL_NAMES, positions=(0.1,) * COUNT, velocities=(0.0,) * COUNT)
    assert state.names == CANONICAL_NAMES
    assert state.positions[0] == 0.1


def test_s6_validate_command_emits_reject_codes() -> None:
    names = [f"joint_{i}" for i in range(16)]
    cmd = JointCommandData(100000, 99, 2, 8, tuple(names[::-1]),
                           tuple(float(i) / 100 for i in range(16)))
    assert validate_command(
        cmd, names=names, lower=[-0.5] * 16, upper=[0.5] * 16,
        session_id=99, epoch=2, last_sequence=7, sim_time_us=100000) == [
            float(i) / 100 for i in range(15, -1, -1)]
    with pytest.raises(CommandContractError) as extra:
        validate_command(
            JointCommandData(100000, 99, 1, 8, tuple(names), tuple([0.0] * 16)),
            names=names, lower=[-0.5] * 16, upper=[0.5] * 16,
            session_id=99, epoch=2, last_sequence=7, sim_time_us=100000)
    assert extra.value.code == CommandReject.STALE_EPOCH.value
    assert CommandContractError is not Exception
    assert issubclass(CommandContractError, ValueError)
