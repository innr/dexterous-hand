"""运行首个四关节单指 MuJoCo 实验。"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np


MODEL_PATH = Path(__file__).with_name("models") / "single_finger.xml"


def target_positions(elapsed: float) -> np.ndarray:
    """返回四个关节的平滑屈曲目标角（弧度）。"""
    # 用余弦曲线在张开与闭合姿态之间平滑过渡，周期 4 秒。
    phase = 0.5 - 0.5 * np.cos(2.0 * np.pi * elapsed / 4.0)
    open_pose = np.array([0.0, 0.05, 0.05, 0.05])
    closed_pose = np.array([0.35, 1.05, 1.15, 0.85])
    return open_pose + phase * (closed_pose - open_pose)


def run(*, seconds: float, headless: bool) -> None:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    duration = seconds if seconds > 0 else float("inf")

    # 无界面模式：按固定步长推进到指定时长。
    if headless:
        while data.time < duration:
            data.ctrl[:] = target_positions(data.time)
            mujoco.mj_step(model, data)
        print(f"completed {data.time:.2f}s; qpos={np.round(data.qpos, 3)}")
        return

    # 带界面模式：实时渲染，并按物理步长补偿休眠以贴近真实时间。
    import mujoco.viewer

    started = time.monotonic()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and time.monotonic() - started < duration:
            step_started = time.monotonic()
            data.ctrl[:] = target_positions(data.time)
            mujoco.mj_step(model, data)
            viewer.sync()
            remaining = model.opt.timestep - (time.monotonic() - step_started)
            if remaining > 0:
                time.sleep(remaining)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run without a viewer")
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="stop after N simulated seconds; 0 keeps the viewer open",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.headless and args.seconds <= 0:
        raise SystemExit("--headless requires --seconds greater than zero")
    run(seconds=args.seconds, headless=args.headless)


if __name__ == "__main__":
    main()
