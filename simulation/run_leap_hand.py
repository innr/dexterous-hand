"""Run a headless position-control check against the official LEAP model."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .leap_hand_model import (
    LeapPositionController,
    clamp_state_to_limits,
    home_pose,
    joint_limits,
    load_actuated_model,
    load_mapping,
    reorder_hardware_to_sim,
    reorder_sim_to_hardware,
)


def close_pose(mapping: dict[str, object]) -> np.ndarray:
    """Return a conservative simulation-only close target.

    Direction signs and hardware zero offsets are intentionally not applied:
    those values remain unverified until the first physical hand is available.
    """
    lower, upper = joint_limits(mapping)
    return np.clip(lower + 0.6 * (upper - lower), lower, upper)


def run(*, seconds: float, project_root: Path, pose: str = "close") -> None:
    mapping = load_mapping(project_root / "config" / "joint_mapping.json")
    model = load_actuated_model(project_root)
    import mujoco

    data = mujoco.MjData(model)
    start = home_pose(mapping)
    data.qpos[:] = reorder_hardware_to_sim(start, model, mapping)
    mujoco.mj_forward(model, data)
    controller = LeapPositionController(model, mapping)
    target = start if pose == "home" else close_pose(mapping)
    steps = max(1, round(seconds / model.opt.timestep))
    max_raw_limit_violation = 0.0
    for _ in range(steps):
        controller.step(data, target)
        mujoco.mj_step(model, data)
        violation = clamp_state_to_limits(data, model, mapping)
        max_raw_limit_violation = max(max_raw_limit_violation, violation)
        if violation:
            mujoco.mj_forward(model, data)
    hardware_qpos = reorder_sim_to_hardware(data.qpos, model, mapping)
    error = np.max(np.abs(hardware_qpos - target))
    lower, upper = joint_limits(mapping)
    limit_violation = np.max(np.maximum(lower - hardware_qpos, hardware_qpos - upper).clip(min=0.0))
    print(f"loaded actuated official LEAP model: nq={model.nq}, njnt={model.njnt}, nu={model.nu}")
    print(
        f"pose={pose}; simulated {data.time:.3f}s; "
        f"max target error={error:.4f} rad; "
        f"max raw-limit overshoot clamped={max_raw_limit_violation:.4f} rad; "
        f"final limit violation={limit_violation:.4f} rad"
    )
    print(f"hardware-order qpos={np.round(hardware_qpos, 3)}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=1.0)
    parser.add_argument("--pose", choices=("home", "close"), default="close")
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    if args.seconds <= 0:
        raise SystemExit("--seconds must be greater than zero")
    run(seconds=args.seconds, project_root=args.project_root, pose=args.pose)


if __name__ == "__main__":
    main()
