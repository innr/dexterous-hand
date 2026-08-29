from pathlib import Path

import numpy as np
import pytest

from simulation.leap_hand_model import (
    LeapPositionController,
    clamp_state_to_limits,
    home_pose,
    joint_limits,
    limit_target,
    load_actuated_model,
    load_mapping,
    official_urdf_path,
    reorder_hardware_to_sim,
)


MAPPING_PATH = Path(__file__).parents[1] / "config" / "joint_mapping.json"


def test_simulation_home_pose_is_explicitly_separate_from_hardware_calibration() -> None:
    mapping = load_mapping(MAPPING_PATH)
    assert np.allclose(home_pose(mapping), 0.0)
    assert mapping["calibration"]["status"] == "not_calibrated"


def test_limit_target_clips_position_and_rate() -> None:
    mapping = load_mapping(MAPPING_PATH)
    lower, upper = joint_limits(mapping)
    current = np.zeros(16)
    target = np.full(16, 100.0)
    limited = limit_target(
        target,
        mapping=mapping,
        current=current,
        dt=0.01,
        max_velocity_rad_s=2.0,
    )
    np.testing.assert_allclose(limited, 0.02)
    assert np.all(limited >= lower)
    assert np.all(limited <= upper)


def test_limit_target_reclips_when_current_state_is_outside_soft_limit() -> None:
    mapping = load_mapping(MAPPING_PATH)
    lower, upper = joint_limits(mapping)
    current = lower - 0.2
    target = np.zeros(16)
    limited = limit_target(target, mapping=mapping, current=current, dt=0.01, max_velocity_rad_s=1.0)
    np.testing.assert_allclose(limited, lower)
    assert np.all(limited >= lower)
    assert np.all(limited <= upper)


def test_actuated_model_has_canonical_position_actuators_when_submodule_is_present() -> None:
    mapping = load_mapping(MAPPING_PATH)
    try:
        official_urdf_path(MAPPING_PATH.parents[1])
    except FileNotFoundError:
        pytest.skip("official LEAP submodule is not initialized")

    import mujoco

    model = load_actuated_model(MAPPING_PATH.parents[1])
    assert model.nq == 16
    assert model.nu == 16
    assert [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)] == [
        f"act_{i}" for i in range(16)
    ]
    data = mujoco.MjData(model)
    data.qpos[:] = reorder_hardware_to_sim(home_pose(mapping), model, mapping)
    controller = LeapPositionController(model, mapping)
    command = controller.step(data, np.full(16, 100.0))
    assert np.all(command <= joint_limits(mapping)[1])
    assert np.all(command >= joint_limits(mapping)[0])


def test_state_limit_projection_clamps_soft_constraint_overshoot() -> None:
    mapping = load_mapping(MAPPING_PATH)
    try:
        official_urdf_path(MAPPING_PATH.parents[1])
    except FileNotFoundError:
        pytest.skip("official LEAP submodule is not initialized")

    import mujoco

    model = load_actuated_model(MAPPING_PATH.parents[1])
    data = mujoco.MjData(model)
    lower, _ = joint_limits(mapping)
    data.qpos[int(model.jnt_qposadr[0])] = lower[1] - 0.1
    data.qvel[int(model.jnt_dofadr[0])] = -1.0
    violation = clamp_state_to_limits(data, model, mapping)
    assert violation == pytest.approx(0.1)
    assert data.qpos[int(model.jnt_qposadr[0])] == pytest.approx(lower[1])
    assert data.qvel[int(model.jnt_dofadr[0])] == pytest.approx(0.0)
