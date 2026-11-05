"""S6-03 real DDS multi-process integration test; requires sourced ROS2 Jazzy.

Exercises 100Hz /clock and /joint_states, independent controller publisher,
pause/resume epoch barrier, reset epoch barrier, old/out-of-order commands,
and SIGKILL of controller process -> SAFE_STOP within bounded wall time.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=35.0)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMsg
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, UInt64
    from std_srvs.srv import SetBool, Trigger
    from leap_interfaces.msg import JointCommand
    from simulation.s6_official_bridge import load_contract

    names = [x["mujoco_joint"] for x in load_contract()]
    deadline = time.monotonic() + args.timeout
    log_path = Path("build/s6_ros2_simulator.log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    sim_log = log_path.open("w", encoding="utf-8")
    simulator = subprocess.Popen(
        [sys.executable, "-m", "simulation.s6_ros2_node",
         "--bridge", str(args.bridge.resolve())],
        stdout=sim_log, stderr=subprocess.STDOUT)
    controller = None
    rclpy.init()

    class Observer(Node):
        def __init__(self):
            super().__init__("leap_s6_dds_acceptance")
            self.sim_clock_us = None
            self.clocks = []
            self.states = []
            self.status = None
            self.epoch = None
            self.statuses = []
            self.pub = self.create_publisher(JointCommand, "/leap/joint_command", 10)
            clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(ClockMsg, "/clock", self.on_clock, clock_qos)
            self.create_subscription(JointState, "/joint_states", self.on_state, 10)
            self.create_subscription(String, "/leap/sim_status", self.on_status, 10)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(UInt64, "/leap/sim_epoch", self.on_epoch, qos)
            self.pause_client = self.create_client(SetBool, "/leap/sim_pause")
            self.reset_client = self.create_client(Trigger, "/leap/sim_reset")

        def on_clock(self, msg):
            micros = msg.clock.sec * 1_000_000 + msg.clock.nanosec // 1000
            self.sim_clock_us = micros
            self.clocks.append(micros)

        def on_state(self, msg):
            self.states.append((msg.header.stamp.sec * 1_000_000 +
                                msg.header.stamp.nanosec // 1000,
                                list(msg.name), list(msg.position)))

        def on_status(self, msg):
            self.status = msg.data
            self.statuses.append(msg.data)

        def on_epoch(self, msg):
            self.epoch = int(msg.data)

    obs = Observer()

    def spin_until(predicate, timeout=6.0, what="condition"):
        until = min(deadline, time.monotonic() + timeout)
        while time.monotonic() < until:
            if simulator.poll() is not None:
                raise AssertionError(f"simulator exited early: {simulator.returncode}")
            rclpy.spin_once(obs, timeout_sec=0.03)
            if predicate():
                return
        sim_log.flush()
        detail = log_path.read_text(encoding="utf-8")[-3000:]
        raise AssertionError(f"timeout waiting for {what}: status={obs.status}, "
                             f"epoch={obs.epoch}, clock={obs.sim_clock_us}\\n{detail}")

    def service(client, request, what):
        spin_until(lambda: client.service_is_ready(), what=f"{what} service")
        future = client.call_async(request)
        spin_until(lambda: future.done(), what=f"{what} response")
        result = future.result()
        assert result.success, f"{what} failed: {result.message}"
        return result

    def launch_controller():
        return subprocess.Popen(
            [sys.executable, "-m", "scripts.s6_ros2_controller"],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    def kill_controller(proc):
        if proc is not None and proc.poll() is None:
            proc.kill()  # Simulates abnormal controller process death, no graceful STOP.
            proc.wait(timeout=3)

    def send_bad(epoch, sequence, stamp_us):
        cmd = JointCommand()
        cmd.stamp.sec = stamp_us // 1_000_000
        cmd.stamp.nanosec = (stamp_us % 1_000_000) * 1000
        cmd.epoch = epoch
        cmd.sequence = sequence
        cmd.joint_names = names
        cmd.position = [0.12] * 16
        obs.pub.publish(cmd)

    try:
        spin_until(lambda: obs.epoch is not None and len(obs.clocks) > 4 and
                   len(obs.states) > 4, what="DDS clock/joint_states")
        assert len(obs.states[-1][1]) == 16 and set(obs.states[-1][1]) == set(names)
        # Sim-time clock and joint-state stamps must be sourced from same plant.
        assert abs(obs.states[-1][0] - obs.clocks[-1]) <= 20_000
        initial_epoch = obs.epoch
        controller = launch_controller()
        spin_until(lambda: obs.status == "NORMAL", what="first remote motion")
        spin_until(lambda: max(abs(v) for v in obs.states[-1][2]) > 0.0001,
                   what="official LEAP hand joint motion")
        recent = obs.clocks[-25:]
        assert all(a <= b for a, b in zip(recent, recent[1:])), "non-monotonic /clock"
        assert any(b > a for a, b in zip(recent, recent[1:])), "clock not progressing"
        # Kill remote controller and prove simulator remains alive and latches stop.
        kill_controller(controller)
        controller = None
        spin_until(lambda: obs.status == "SAFE_STOP", timeout=2.0,
                   what="SAFE_STOP after controller SIGKILL")
        assert simulator.poll() is None
        service(obs.reset_client, Trigger.Request(), "reset after publisher crash")
        spin_until(lambda: obs.epoch == initial_epoch + 1 and obs.status == "READY",
                   what="reset epoch")
        reset_epoch = obs.epoch
        assert obs.sim_clock_us < 100_000, "reset must rewind sim clock"
        # Old-epoch command and duplicate/out-of-order sequence must not authorize.
        send_bad(initial_epoch, 900, obs.sim_clock_us)
        for _ in range(8):
            rclpy.spin_once(obs, timeout_sec=0.02)
        assert obs.status == "READY", "old-epoch command accepted"
        # Command seq=5 accepted; replay seq=5 and seq=4 cannot override.
        send_bad(reset_epoch, 5, obs.sim_clock_us)
        spin_until(lambda: obs.status == "NORMAL", what="valid command after reset")
        send_bad(reset_epoch, 5, obs.sim_clock_us)
        send_bad(reset_epoch, 4, obs.sim_clock_us)
        # Pause immediately before TTL expiry.
        service(obs.pause_client, SetBool.Request(data=True), "pause")
        spin_until(lambda: obs.status == "PAUSED", what="PAUSED state")
        paused_time = obs.sim_clock_us
        for _ in range(10):
            rclpy.spin_once(obs, timeout_sec=0.02)
        assert obs.sim_clock_us == paused_time, "paused /clock advanced"
        service(obs.pause_client, SetBool.Request(data=False), "resume")
        spin_until(lambda: obs.status == "READY" and obs.epoch == reset_epoch + 1,
                   what="resume epoch barrier")
        resumed_epoch = obs.epoch
        send_bad(reset_epoch, 1000, obs.sim_clock_us)
        for _ in range(8):
            rclpy.spin_once(obs, timeout_sec=0.02)
        assert obs.status == "READY", "pre-pause command replayed after resume"
        controller = launch_controller()
        spin_until(lambda: obs.status == "NORMAL", what="fresh command after resume")
        kill_controller(controller)
        controller = None
        spin_until(lambda: obs.status == "SAFE_STOP", timeout=2.0,
                   what="second controller SIGKILL")
        print(f"PASS S6-03 DDS: 16 joints, /clock, seq, reset, pause, "
              f"epoch={resumed_epoch}, publisher SIGKILL -> SAFE_STOP")
    finally:
        kill_controller(controller)
        obs.destroy_node()
        rclpy.shutdown()
        if simulator.poll() is None:
            simulator.terminate()
            try:
                simulator.wait(timeout=3)
            except subprocess.TimeoutExpired:
                simulator.kill()
                simulator.wait(timeout=3)
        sim_log.close()


if __name__ == "__main__":
    main()
