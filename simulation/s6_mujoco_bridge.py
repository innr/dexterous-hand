"""S6-01: 16-DOF MuJoCo surrogate driven by the existing C++17 control core.

No STS3215, no serial, no ROS2: this is a deterministic simulation-clock bridge.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
from xml.etree import ElementTree as ET

PROJECT_ROOT = Path(__file__).resolve().parents[1]
JOINT_COUNT = 16
CONTROL_DT = 0.01
PHYSICS_DT = 0.002


def load_joint_order(mapping_path: Path) -> list[str]:
    mapping = json.loads(mapping_path.read_text(encoding="utf-8"))
    joints = mapping["joints"]
    if len(joints) != JOINT_COUNT:
        raise ValueError("expected exactly 16 joint mappings")
    by_id = {int(j["hardware_id"]): str(j["mujoco_joint"]) for j in joints}
    if sorted(by_id) != list(range(JOINT_COUNT)) or len(set(by_id.values())) != JOINT_COUNT:
        raise ValueError("duplicate/missing hardware ID or MuJoCo joint name")
    return [by_id[i] for i in range(JOINT_COUNT)]


def make_surrogate_xml(joint_names: list[str]) -> str:
    """Create a small 16-hinge plant for integration; NOT the official LEAP URDF."""
    root = ET.Element("mujoco", model="s6_16dof_surrogate")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep=str(PHYSICS_DT), gravity="0 0 0")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", damping="0.3", armature="0.02")
    world = ET.SubElement(root, "worldbody")
    palm = ET.SubElement(world, "body", name="palm", pos="0 0 0")
    ET.SubElement(palm, "geom", type="box", size="0.06 0.03 0.012", mass="0.2")
    actuators = ET.SubElement(root, "actuator")
    for finger in range(4):
        parent = palm
        for segment in range(4):
            joint_id = 4 * finger + segment
            body = ET.SubElement(
                parent, "body", name=f"link_{joint_id}",
                pos=f"{-0.045 + 0.03 * finger if segment == 0 else 0} 0 "
                    f"{0.025 if segment == 0 else 0.03}",
            )
            ET.SubElement(body, "joint", name=joint_names[joint_id],
                          type="hinge", axis="0 1 0", range="-1.3 1.3")
            ET.SubElement(body, "geom", name=f"geom_{joint_id}", type="capsule",
                          fromto="0 0 0 0 0 0.03", size="0.006", density="500",
                          contype="0", conaffinity="0")
            ET.SubElement(actuators, "position", name=f"servo_{joint_id}",
                          joint=joint_names[joint_id], kp="12", kv="1",
                          ctrlrange="-1.3 1.3", ctrllimited="true")
            parent = body
    return ET.tostring(root, encoding="unicode")


def bridge_env() -> dict[str, str]:
    env = os.environ.copy()
    if "LEAP_JOINT_MAPPING" not in env and "LEAP_PROJECT_ROOT" not in env:
        env["LEAP_PROJECT_ROOT"] = str(PROJECT_ROOT)
    return env


class ControlBridgeProcess:
    def __init__(self, executable: Path):
        # JSON limits from joint_mapping.json apply to this surrogate too.
        # Keep side-joint targets at 0 so measured pose stays inside limit_rad.
        self.process = subprocess.Popen(
            [str(executable)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1, env=bridge_env(),
        )
        assert self.process.stdout is not None
        ready = self.process.stdout.readline().strip()
        if ready != "READY 16 S6-01":
            self.close()
            raise RuntimeError(f"bridge handshake failed: {ready!r}")

    def step(self, time_us: int, position: list[float],
             velocity: list[float], target: list[float]) -> list[float]:
        if any(len(x) != JOINT_COUNT for x in (position, velocity, target)):
            raise ValueError("expected 16 joint values in every vector")
        if not all(math.isfinite(v) for seq in (position, velocity, target) for v in seq):
            raise ValueError("non-finite input")
        assert self.process.stdin is not None and self.process.stdout is not None
        values = [str(time_us), *(format(v, ".12g") for v in position),
                  *(format(v, ".12g") for v in velocity),
                  *(format(v, ".12g") for v in target)]
        self.process.stdin.write("STEP " + " ".join(values) + "\n")
        self.process.stdin.flush()
        response = self.process.stdout.readline().strip().split()
        if len(response) != JOINT_COUNT + 1 or response[0] != "OK":
            raise RuntimeError(f"control core stopped or invalid response: {response!r}")
        command = [float(value) for value in response[1:]]
        if not all(math.isfinite(v) for v in command):
            raise RuntimeError("non-finite command")
        return command

    def close(self) -> None:
        if self.process.poll() is None:
            try:
                assert self.process.stdin is not None
                self.process.stdin.write("QUIT\n")
                self.process.stdin.flush()
                self.process.communicate(timeout=3)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                self.process.kill()
                self.process.communicate()

    def __enter__(self) -> "ControlBridgeProcess":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def run(executable: Path, seconds: float, *, assert_motion: bool = False) -> dict[str, float]:
    if seconds <= 0:
        raise ValueError("seconds must be positive")
    import mujoco
    joint_names = load_joint_order(PROJECT_ROOT / "config" / "joint_mapping.json")
    model = mujoco.MjModel.from_xml_string(make_surrogate_xml(joint_names))
    data = mujoco.MjData(model)
    if model.njnt != JOINT_COUNT or model.nu != JOINT_COUNT:
        raise RuntimeError("surrogate model must have 16 joints and 16 actuators")
    qadr = [model.joint(name).qposadr[0] for name in joint_names]
    vadr = [model.joint(name).dofadr[0] for name in joint_names]
    aid = [model.actuator(f"servo_{i}").id for i in range(JOINT_COUNT)]
    target = [0.0 if i % 4 == 0 else 0.35 for i in range(JOINT_COUNT)]
    steps = max(2, round(seconds / CONTROL_DT))
    last_command = [0.0] * JOINT_COUNT
    with ControlBridgeProcess(executable) as bridge:
        for tick in range(steps):
            position = [float(data.qpos[index]) for index in qadr]
            velocity = [float(data.qvel[index]) for index in vadr]
            last_command = bridge.step(tick * 10000, position, velocity, target)
            for i, index in enumerate(aid):
                data.ctrl[index] = last_command[i]
            for _ in range(5):
                mujoco.mj_step(model, data)
    actual = [float(data.qpos[index]) for index in qadr]
    result = {
        "simulated_s": round(float(data.time), 6),
        "joints": float(model.njnt),
        "actuators": float(model.nu),
        "max_abs_position_rad": max(abs(value) for value in actual),
        "max_abs_command_rad": max(abs(value) for value in last_command),
    }
    if assert_motion and (result["max_abs_position_rad"] < 1e-5 or
                          result["max_abs_command_rad"] < 1e-5):
        raise AssertionError(f"simulation did not respond to C++ commands: {result}")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge", type=Path, required=True,
                        help="compiled cpp/s6_control_bridge executable")
    parser.add_argument("--seconds", type=float, default=1.0)
    parser.add_argument("--assert-motion", action="store_true")
    args = parser.parse_args()
    print(json.dumps(run(args.bridge.resolve(), args.seconds,
                         assert_motion=args.assert_motion), indent=2))


if __name__ == "__main__":
    main()
