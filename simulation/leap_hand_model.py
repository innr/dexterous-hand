"""Load and control the pinned official LEAP Hand model.

The upstream LEAP asset is an URDF and intentionally does not contain
MuJoCo actuators.  :func:`load_actuated_model` converts it to an in-memory
MJCF variant and adds one position actuator per joint.  The generated file is
temporary; the checked-in upstream asset is never modified.
"""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MAPPING_PATH = PROJECT_ROOT / "config" / "joint_mapping.json"


DEFAULT_SIMULATION_CONFIG: dict[str, Any] = {
    "actuator_kp": 20.0,
    "actuator_kv": 1.0,
    "actuator_force_limit_nm": 0.95,
    "joint_damping": 0.2,
    "joint_armature": 0.001,
    "joint_limit_solref": "0.02 1",
    "max_target_velocity_rad_s": 4.0,
    # This is a MuJoCo-frame start pose, not a calibrated hardware zero.
    "home_pose_rad": [0.0] * 16,
}


def load_mapping(path: Path = MAPPING_PATH) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _canonical_joints(mapping: dict[str, Any]) -> list[dict[str, Any]]:
    """Return mapping entries in canonical hardware/LEAP order."""
    joints = sorted(mapping["joints"], key=lambda entry: int(entry["hardware_id"]))
    expected = list(range(len(joints)))
    actual = [int(entry["hardware_id"]) for entry in joints]
    if actual != expected:
        raise ValueError(f"hardware IDs must be contiguous from zero, got {actual}")
    return joints


def joint_limits(mapping: dict[str, Any] | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Return lower and upper limits in canonical hardware order."""
    mapping = mapping or load_mapping()
    joints = _canonical_joints(mapping)
    limits = np.asarray([entry["limit_rad"] for entry in joints], dtype=float)
    if limits.shape != (len(joints), 2) or np.any(limits[:, 0] >= limits[:, 1]):
        raise ValueError("joint limit_rad entries must be [lower, upper]")
    return limits[:, 0], limits[:, 1]


def simulation_config(mapping: dict[str, Any] | None = None) -> dict[str, Any]:
    """Return validated simulation-only settings from the mapping file."""
    mapping = mapping or load_mapping()
    config = dict(DEFAULT_SIMULATION_CONFIG)
    config.update(mapping.get("simulation", {}))
    for key in (
        "actuator_kp",
        "actuator_kv",
        "actuator_force_limit_nm",
        "joint_damping",
        "joint_armature",
        "max_target_velocity_rad_s",
    ):
        config[key] = float(config[key])
        if config[key] <= 0:
            raise ValueError(f"simulation.{key} must be positive")
    if not isinstance(config["joint_limit_solref"], str) or len(config["joint_limit_solref"].split()) != 2:
        raise ValueError("simulation.joint_limit_solref must contain two MuJoCo solref values")
    home = np.asarray(config["home_pose_rad"], dtype=float)
    lower, upper = joint_limits(mapping)
    if home.shape != lower.shape:
        raise ValueError(f"simulation.home_pose_rad must contain {len(lower)} values")
    if np.any(home < lower) or np.any(home > upper):
        raise ValueError("simulation.home_pose_rad is outside configured joint limits")
    config["home_pose_rad"] = home
    return config


def home_pose(mapping: dict[str, Any] | None = None) -> np.ndarray:
    """Return the simulation start pose in canonical order.

    It deliberately does not apply ``zero_offset_rad``: those offsets are
    hardware calibration data and are not known until a physical hand exists.
    """
    return np.array(simulation_config(mapping)["home_pose_rad"], dtype=float, copy=True)


def limit_target(
    target: np.ndarray,
    *,
    mapping: dict[str, Any] | None = None,
    current: np.ndarray | None = None,
    dt: float | None = None,
    max_velocity_rad_s: float | np.ndarray | None = None,
) -> np.ndarray:
    """Clip a canonical target to joint limits and an optional rate limit.

    ``current`` and ``target`` are both in canonical hardware order.  The
    rate limit is applied after position clipping, so a malformed command can
    never request a position outside the configured range.
    """
    mapping = mapping or load_mapping()
    lower, upper = joint_limits(mapping)
    output = np.asarray(target, dtype=float)
    if output.shape != lower.shape:
        raise ValueError(f"Expected {len(lower)} target values, got {output.shape}")
    output = np.clip(output, lower, upper)
    if current is None:
        return output
    if dt is None or dt <= 0:
        raise ValueError("dt must be positive when current is supplied")
    current_array = np.asarray(current, dtype=float)
    if current_array.shape != lower.shape:
        raise ValueError(f"Expected {len(lower)} current values, got {current_array.shape}")
    if max_velocity_rad_s is None:
        max_velocity_rad_s = simulation_config(mapping)["max_target_velocity_rad_s"]
    velocity = np.asarray(max_velocity_rad_s, dtype=float)
    if velocity.ndim == 0:
        velocity = np.full_like(lower, float(velocity))
    if velocity.shape != lower.shape or np.any(velocity <= 0):
        raise ValueError("max_velocity_rad_s must be a positive scalar or one value per joint")
    max_step = velocity * float(dt)
    # Clip once more after the rate limiter: the simulated state can be a
    # little outside a soft MuJoCo joint limit after a large contact impulse.
    return np.clip(output, current_array - max_step, current_array + max_step).clip(lower, upper)


def official_urdf_path(project_root: Path = PROJECT_ROOT) -> Path:
    """Return the submodule's URDF, with an actionable error if not initialized."""
    mapping = load_mapping(project_root / "config" / "joint_mapping.json")
    path = project_root / mapping["model"]["relative_urdf"]
    if not path.is_file():
        raise FileNotFoundError(
            f"Official LEAP model is missing at {path}. "
            "Run: git submodule update --init --recursive"
        )
    return path


def mujoco_joint_names(model: Any) -> list[str]:
    """Read joint names in the order used by the loaded MuJoCo model."""
    import mujoco

    return [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
    ]


def sim_to_hardware_indices(model: Any, mapping: dict[str, Any] | None = None) -> list[int]:
    """Return the hardware ID for each MuJoCo joint index."""
    mapping = mapping or load_mapping()
    by_joint = {entry["mujoco_joint"]: entry["hardware_id"] for entry in mapping["joints"]}
    names = mujoco_joint_names(model)
    if any(name not in by_joint for name in names):
        raise ValueError(f"Model contains unmapped joints: {names}")
    return [by_joint[name] for name in names]


def reorder_sim_to_hardware(values: np.ndarray, model: Any, mapping: dict[str, Any] | None = None) -> np.ndarray:
    """Reorder a MuJoCo joint vector into official LEAP/hardware order."""
    values = np.asarray(values, dtype=float)
    indices = sim_to_hardware_indices(model, mapping)
    if values.shape[-1] != len(indices):
        raise ValueError(f"Expected {len(indices)} joint values, got {values.shape[-1]}")
    output = np.empty_like(values)
    for sim_index, hardware_id in enumerate(indices):
        output[..., hardware_id] = values[..., sim_index]
    return output


def reorder_hardware_to_sim(values: np.ndarray, model: Any, mapping: dict[str, Any] | None = None) -> np.ndarray:
    """Reorder a canonical hardware vector into MuJoCo joint order."""
    values = np.asarray(values, dtype=float)
    indices = sim_to_hardware_indices(model, mapping)
    if values.shape[-1] != len(indices):
        raise ValueError(f"Expected {len(indices)} joint values, got {values.shape[-1]}")
    output = np.empty_like(values)
    for sim_index, hardware_id in enumerate(indices):
        output[..., sim_index] = values[..., hardware_id]
    return output


def clamp_state_to_limits(data: Any, model: Any, mapping: dict[str, Any] | None = None) -> float:
    """Project hinge state back into configured limits and return overshoot.

    MuJoCo joint limits are constraint forces, so a large impulse can leave a
    low-mass link a few tenths of a radian beyond a range for one step.  This
    final safety projection keeps the software control loop bounded; it is not
    intended to replace physical hard stops or hardware commissioning.
    """
    mapping = mapping or load_mapping()
    lower, upper = joint_limits(mapping)
    indices = sim_to_hardware_indices(model, mapping)
    max_violation = 0.0
    for sim_index, hardware_id in enumerate(indices):
        qpos_index = int(model.jnt_qposadr[sim_index])
        qvel_index = int(model.jnt_dofadr[sim_index])
        value = float(data.qpos[qpos_index])
        if value < lower[hardware_id]:
            max_violation = max(max_violation, float(lower[hardware_id] - value))
            data.qpos[qpos_index] = lower[hardware_id]
            if data.qvel[qvel_index] < 0:
                data.qvel[qvel_index] = 0.0
        elif value > upper[hardware_id]:
            max_violation = max(max_violation, float(value - upper[hardware_id]))
            data.qpos[qpos_index] = upper[hardware_id]
            if data.qvel[qvel_index] > 0:
                data.qvel[qvel_index] = 0.0
    return max_violation


def load_official_model(project_root: Path = PROJECT_ROOT) -> Any:
    """Load the official URDF with MuJoCo."""
    import mujoco

    return mujoco.MjModel.from_xml_path(str(official_urdf_path(project_root)))


def _augmented_mjcf(xml_text: str, mapping: dict[str, Any]) -> str:
    """Add stable joint dynamics and canonical-order position actuators."""
    config = simulation_config(mapping)
    joints = _canonical_joints(mapping)
    root = ElementTree.fromstring(xml_text)

    model_joint_names = {
        element.attrib.get("name")
        for element in root.findall(".//joint")
        if element.attrib.get("name") is not None
    }
    expected_names = {str(entry["mujoco_joint"]) for entry in joints}
    missing = expected_names - model_joint_names
    if missing:
        raise ValueError(f"Official model is missing mapped joints: {sorted(missing)}")

    # The converted URDF has no damping/armature.  A small amount keeps the
    # low-mass fingertips numerically stable while leaving the upstream mesh
    # and kinematics untouched.
    for element in root.findall(".//joint"):
        if element.attrib.get("name") in expected_names:
            element.set("damping", str(config["joint_damping"]))
            element.set("armature", str(config["joint_armature"]))
            element.set("solreflimit", config["joint_limit_solref"])

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ElementTree.SubElement(root, "actuator")
    else:
        # Avoid silently creating duplicate controls if an upstream model adds
        # its own actuator section in the future.
        for child in list(actuator):
            actuator.remove(child)

    force = config["actuator_force_limit_nm"]
    for entry in joints:
        lower, upper = entry["limit_rad"]
        ElementTree.SubElement(
            actuator,
            "position",
            {
                "name": f"act_{entry['hardware_id']}",
                "joint": str(entry["mujoco_joint"]),
                "kp": str(config["actuator_kp"]),
                "kv": str(config["actuator_kv"]),
                "ctrlrange": f"{lower} {upper}",
                "ctrllimited": "true",
                "forcelimited": "true",
                "forcerange": f"{-force} {force}",
            },
        )
    return ElementTree.tostring(root, encoding="unicode")


def load_actuated_model(project_root: Path = PROJECT_ROOT) -> Any:
    """Load the official model with 16 canonical-order position actuators.

    MuJoCo resolves mesh paths relative to the XML file, so the temporary
    converted MJCF is placed beside the official URDF and removed after the
    compiled model is loaded.
    """
    import mujoco

    root = Path(project_root)
    mapping = load_mapping(root / "config" / "joint_mapping.json")
    urdf = official_urdf_path(root)
    source_model = mujoco.MjModel.from_xml_path(str(urdf))
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".leap_actuated_", suffix=".xml", dir=urdf.parent
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    try:
        mujoco.mj_saveLastXML(str(temporary_path), source_model)
        augmented = _augmented_mjcf(temporary_path.read_text(encoding="utf-8"), mapping)
        temporary_path.write_text(augmented, encoding="utf-8")
        return mujoco.MjModel.from_xml_path(str(temporary_path))
    finally:
        temporary_path.unlink(missing_ok=True)


@dataclass
class LeapPositionController:
    """Canonical-order position controller for an actuated LEAP model."""

    model: Any
    mapping: dict[str, Any]
    max_velocity_rad_s: float | np.ndarray | None = None

    def __post_init__(self) -> None:
        expected = len(self.mapping["joints"])
        if self.model.nu != expected:
            raise ValueError(f"Expected {expected} actuators, got {self.model.nu}")
        if self.max_velocity_rad_s is None:
            self.max_velocity_rad_s = simulation_config(self.mapping)["max_target_velocity_rad_s"]

    def step(
        self,
        data: Any,
        target: np.ndarray,
        *,
        dt: float | None = None,
    ) -> np.ndarray:
        """Apply one rate-limited canonical target and return what was sent."""
        if dt is None:
            dt = float(self.model.opt.timestep)
        current = reorder_sim_to_hardware(data.qpos, self.model, self.mapping)
        limited = limit_target(
            target,
            mapping=self.mapping,
            current=current,
            dt=dt,
            max_velocity_rad_s=self.max_velocity_rad_s,
        )
        # Actuators are emitted in hardware_id order by _augmented_mjcf.
        data.ctrl[:] = limited
        return limited
