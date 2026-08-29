"""Safety-gated single-servo motion test."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math

from .commissioning import CommissioningError, radians_to_ticks
from .sts3215 import STS3215Bus, STS3215Error


class MotionSafetyError(RuntimeError):
    """Raised when a motion test lacks an explicit safety acknowledgement."""


@dataclass(frozen=True)
class MotionPlan:
    servo_id: int
    target_angle_rad: float
    target_ticks: int


def make_motion_plan(servo_id: int, target_angle_rad: float, reference_ticks: int = 2048) -> MotionPlan:
    if not 0 <= servo_id <= 253:
        raise MotionSafetyError("servo_id must be in [0, 253]")
    if not math.isfinite(target_angle_rad):
        raise MotionSafetyError("target angle must be finite")
    try:
        ticks = radians_to_ticks(target_angle_rad, reference_ticks=reference_ticks)
    except CommissioningError as exc:
        raise MotionSafetyError(str(exc)) from exc
    return MotionPlan(servo_id, target_angle_rad, ticks)


def execute_motion_test(
    bus: STS3215Bus,
    plan: MotionPlan,
    *,
    enable_torque: bool,
    confirmed: bool,
) -> int:
    if not enable_torque or not confirmed:
        raise MotionSafetyError(
            "motion requires both --enable-torque and --confirm; default mode is dry-run"
        )
    try:
        bus.set_torque(plan.servo_id, True)
        bus.write_goal_position_ticks(plan.servo_id, plan.target_ticks)
        return bus.read_position_ticks(plan.servo_id)
    finally:
        try:
            bus.set_torque(plan.servo_id, False)
        except STS3215Error:
            # Best effort: the original motion error is more useful to the caller.
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--angle-rad", type=float, required=True)
    parser.add_argument("--reference-ticks", type=int, default=2048)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--enable-torque", action="store_true")
    parser.add_argument("--confirm", action="store_true")
    args = parser.parse_args()
    plan = make_motion_plan(args.id, args.angle_rad, args.reference_ticks)
    print(f"servo_id={plan.servo_id} target_ticks={plan.target_ticks}")
    if not args.enable_torque or not args.confirm:
        print("dry-run only: no torque or goal command sent")
        return
    try:
        with STS3215Bus(args.port, baudrate=args.baudrate, timeout=args.timeout) as bus:
            present = execute_motion_test(
                bus, plan, enable_torque=args.enable_torque, confirmed=args.confirm
            )
    except STS3215Error as exc:
        raise SystemExit(f"motion test failed: {exc}") from exc
    print(f"present_ticks_after_command={present}")


if __name__ == "__main__":
    main()
