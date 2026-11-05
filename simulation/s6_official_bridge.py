"""S6-02 official LEAP Hand MJCF with deterministic C++ control-core bridge.

No ROS2 or serial dependency in this module. ROS2 is an optional adapter.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Callable

from .fetch_leap_menagerie import DEST
from .s6_mujoco_bridge import CONTROL_DT, ControlBridgeProcess, PROJECT_ROOT

MAPPING = PROJECT_ROOT / "config" / "menagerie_right_mapping.json"
COUNT = 16
TICK_US = 10_000
COMMAND_TTL_US = 100_000


def load_contract(mapping_path: Path = MAPPING) -> list[dict]:
    data = json.loads(mapping_path.read_text(encoding="utf-8"))
    joints = data["joints"]
    if len(joints) != COUNT or [j["id"] for j in joints] != list(range(COUNT)):
        raise ValueError("official model mapping must have IDs 0..15")
    for key in ("mujoco_joint", "actuator", "canonical"):
        if len({j[key] for j in joints}) != COUNT:
            raise ValueError(f"duplicate {key} in model mapping")
    return joints


class OfficialPlant:
    def __init__(self, model_path: Path, mapping_path: Path = MAPPING):
        import mujoco
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
        self.data = mujoco.MjData(self.model)
        self.mapping = load_contract(mapping_path)
        if self.model.njnt != COUNT or self.model.nu != COUNT:
            raise ValueError("official model must have 16 joints and 16 actuators")
        if not math.isclose(self.model.opt.timestep, 0.002, abs_tol=1e-9):
            self.model.opt.timestep = 0.002
        self.qadr, self.vadr, self.aid, self.lower, self.upper = [], [], [], [], []
        for entry in self.mapping:
            joint = self.model.joint(entry["mujoco_joint"])
            actuator = self.model.actuator(entry["actuator"])
            jid = int(joint.id)
            aid = int(actuator.id)
            if int(self.model.actuator_trnid[aid, 0]) != jid:
                raise ValueError(f"actuator does not drive mapped joint: {entry}")
            self.qadr.append(int(joint.qposadr[0]))
            self.vadr.append(int(joint.dofadr[0]))
            self.aid.append(aid)
            # C++ S5 MotionControlCore currently uses +/- 90 deg default limits.
            # Do not silently ask the official plant to exceed that tested range.
            low = max(float(self.model.jnt_range[jid, 0]),
                      float(self.model.actuator_ctrlrange[aid, 0]), -math.pi / 2)
            high = min(float(self.model.jnt_range[jid, 1]),
                       float(self.model.actuator_ctrlrange[aid, 1]), math.pi / 2)
            if not low < 0.0 < high:
                raise ValueError(f"unsupported joint range for default C++ core: {entry}")
            self.lower.append(low)
            self.upper.append(high)

    @property
    def time_us(self) -> int:
        return round(float(self.data.time) * 1_000_000)

    @property
    def names(self) -> list[str]:
        return [entry["mujoco_joint"] for entry in self.mapping]

    def feedback(self) -> tuple[list[float], list[float]]:
        return ([float(self.data.qpos[i]) for i in self.qadr],
                [float(self.data.qvel[i]) for i in self.vadr])

    def apply(self, command: list[float]) -> None:
        if len(command) != COUNT:
            raise ValueError("wrong actuator command count")
        for i, value in enumerate(command):
            if not math.isfinite(value) or not self.lower[i] <= value <= self.upper[i]:
                raise ValueError(f"out-of-range actuator command {i}: {value}")
        for i, aid in enumerate(self.aid):
            self.data.ctrl[aid] = command[i]

    def hold_and_advance(self) -> None:
        # No authorized command: hold measured pose; do not use old target.
        position, _ = self.feedback()
        self.apply(position)
        self.advance()

    def advance(self) -> None:
        for _ in range(5):
            self.mujoco.mj_step(self.model, self.data)

    def reset(self) -> None:
        self.mujoco.mj_resetData(self.model, self.data)
        self.mujoco.mj_forward(self.model, self.data)


class SimSession:
    """Single-owner simulation state machine. Only the owner calls these methods."""

    def __init__(self, plant: OfficialPlant, bridge_executable: Path,
                 bridge_factory: Callable[[Path], object] = ControlBridgeProcess):
        self.plant = plant
        self.bridge_executable = bridge_executable
        self.bridge_factory = bridge_factory
        self.bridge = bridge_factory(bridge_executable)
        self.epoch = 0
        self.state = "READY"
        self.target: list[float] | None = None
        self.target_stamp_us: int | None = None
        self.last_command: list[float] | None = None

    def set_target(self, names: list[str], positions: list[float],
                   *, epoch: int, stamp_us: int) -> None:
        if self.state not in ("READY", "NORMAL"):
            raise ValueError(f"commands forbidden in state {self.state}")
        if epoch != self.epoch:
            raise ValueError("stale simulation epoch")
        if len(names) != COUNT or len(positions) != COUNT or set(names) != set(self.plant.names):
            raise ValueError("expected 16 unique official joint names")
        if len(set(names)) != COUNT or any(not math.isfinite(v) for v in positions):
            raise ValueError("duplicate joint name or non-finite command")
        if stamp_us < 0 or stamp_us > self.plant.time_us + TICK_US or (
                self.plant.time_us - stamp_us > COMMAND_TTL_US):
            raise ValueError("stale/future simulation-time command")
        values = dict(zip(names, positions))
        target = [float(values[name]) for name in self.plant.names]
        if any(not self.plant.lower[i] <= v <= self.plant.upper[i]
               for i, v in enumerate(target)):
            raise ValueError("command exceeds official model and C++ intersection limits")
        self.target = target
        self.target_stamp_us = stamp_us

    def tick(self) -> None:
        if self.state in ("PAUSED", "SAFE_STOP", "FAULT"):
            return
        try:
            if self.target is None:
                self.plant.hold_and_advance()
                return
            if self.target_stamp_us is None or (self.plant.time_us - self.target_stamp_us > COMMAND_TTL_US):
                raise RuntimeError("ROS joint command TTL expired")
            p, v = self.plant.feedback()
            command = self.bridge.step(self.plant.time_us, p, v, self.target)
            self.plant.apply(command)
            self.plant.advance()
            self.last_command = command
            self.state = "NORMAL"
        except Exception:
            self.stop()
            raise

    def pause(self, enabled: bool) -> None:
        if self.state in ("SAFE_STOP", "FAULT"):
            raise ValueError("latched stop; explicit reset required")
        if enabled:
            self.target = None
            self.target_stamp_us = None
            self.bridge.close()  # Freeze old trajectory state; no queued commands survive resume.
            self.state = "PAUSED"
        elif self.state == "PAUSED":
            # Resume does not replay commands queued before pause.
            self.target = None
            self.target_stamp_us = None
            try:
                self.bridge = self.bridge_factory(self.bridge_executable)
                self.state = "READY"
            except Exception:
                self.state = "FAULT"
                raise

    def stop(self) -> None:
        self.state = "SAFE_STOP"
        self.target = None
        self.target_stamp_us = None
        self.last_command = None
        self.bridge.close()

    def reset(self) -> None:
        # Explicit authorized reset only. Recreate C++ watchdog, trajectory and TX gate.
        self.bridge.close()
        self.plant.reset()
        self.epoch += 1
        self.target = None
        self.target_stamp_us = None
        self.last_command = None
        self.state = "READY"
        try:
            self.bridge = self.bridge_factory(self.bridge_executable)
        except Exception:
            self.state = "FAULT"
            raise

    def close(self) -> None:
        self.bridge.close()


def smoke(executable: Path, model_path: Path, seconds: float) -> dict:
    if seconds <= 0:
        raise ValueError("seconds must be positive")
    plant = OfficialPlant(model_path)
    session = SimSession(plant, executable)
    try:
        session.set_target(plant.names, [0.20] * COUNT,
                           epoch=session.epoch, stamp_us=0)
        for _ in range(round(seconds / CONTROL_DT)):
            session.set_target(plant.names, [0.20] * COUNT,
                               epoch=session.epoch, stamp_us=plant.time_us)
            session.tick()
        position, _ = plant.feedback()
        return {"model": "official_menagerie_right", "epoch": session.epoch,
                "simulated_s": round(plant.data.time, 4),
                "joints": plant.model.njnt, "actuators": plant.model.nu,
                "max_abs_position_rad": max(abs(v) for v in position),
                "max_abs_command_rad": max(abs(v) for v in (session.last_command or [0]))}
    finally:
        session.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--model", type=Path, default=DEST / "right_hand.xml")
    parser.add_argument("--seconds", type=float, default=0.5)
    parser.add_argument("--assert-motion", action="store_true")
    args = parser.parse_args()
    result = smoke(args.bridge.resolve(), args.model.resolve(), args.seconds)
    if args.assert_motion and (result["max_abs_position_rad"] < 1e-5 or
                               result["max_abs_command_rad"] < 1e-5):
        raise AssertionError(f"official model did not move: {result}")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
