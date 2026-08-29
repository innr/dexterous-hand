"""Run a minimal headless check against the official 16-DOF LEAP model."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .leap_hand_model import load_mapping, load_official_model, reorder_sim_to_hardware


def run(*, seconds: float, project_root: Path) -> None:
    model = load_official_model(project_root)
    import mujoco

    data = mujoco.MjData(model)
    # The official URDF has no actuators. Hold the model at its zero pose and
    # report the reordered state; hardware control is added after calibration.
    steps = max(1, round(seconds / model.opt.timestep))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    hardware_qpos = reorder_sim_to_hardware(data.qpos, model, load_mapping(project_root / "config" / "joint_mapping.json"))
    print(f"loaded official LEAP model: nq={model.nq}, njnt={model.njnt}, nu={model.nu}")
    print(f"simulated {data.time:.3f}s; hardware-order qpos={np.round(hardware_qpos, 3)}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=0.1)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    if args.seconds <= 0:
        raise SystemExit("--seconds must be greater than zero")
    run(seconds=args.seconds, project_root=args.project_root)


if __name__ == "__main__":
    main()
