"""Load the pinned official LEAP Hand model and reorder its joints."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MAPPING_PATH = PROJECT_ROOT / "config" / "joint_mapping.json"


def load_mapping(path: Path = MAPPING_PATH) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


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


def load_official_model(project_root: Path = PROJECT_ROOT) -> Any:
    """Load the official URDF with MuJoCo."""
    import mujoco

    return mujoco.MjModel.from_xml_path(str(official_urdf_path(project_root)))

