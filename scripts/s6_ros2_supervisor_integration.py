"""S6-04 ROS2 Jazzy lifecycle and independent process supervision acceptance.

Uses real DDS, real lifecycle ChangeState services, SIGKILL fault injection.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", type=Path, required=True)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String, UInt64
    from std_srvs.srv import SetBool
    from rosgraph_msgs.msg import Clock as ClockMsg
    from sensor_msgs.msg import JointState
    from leap_interfaces.msg import JointCommand

    rclpy.init()

    class Observer(Node):
        def __init__(self):
            super().__init__("leap_s6_supervisor_acceptance")
            self.supervisor_status = None
            self.sim_status = None
            self.epoch = None
            self.clock_us = None
            self.joint_motion = 0.0
            self.command_count = 0
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(String, "/leap/supervisor_status",
                                     self.on_supervisor, qos)
            self.create_subscription(String, "/leap/sim_status", self.on_sim, 10)
            self.create_subscription(UInt64, "/leap/sim_epoch", self.on_epoch, qos)
            self.create_subscription(ClockMsg, "/clock", self.on_clock, clock_qos)
            self.create_subscription(JointState, "/joint_states", self.on_joint, 10)
            self.create_subscription(JointCommand, "/leap/joint_command",
                                     self.on_command, 10)
            self.pause_client = self.create_client(SetBool, "/leap/supervisor_pause")

        def on_supervisor(self, msg):
            self.supervisor_status = msg.data

        def on_sim(self, msg):
            self.sim_status = msg.data

        def on_epoch(self, msg):
            self.epoch = int(msg.data)

        def on_clock(self, msg):
            self.clock_us = msg.clock.sec * 1_000_000 + msg.clock.nanosec // 1000

        def on_command(self, _msg):
            self.command_count += 1

        def on_joint(self, msg):
            if len(msg.position) == 16:
                self.joint_motion = max(abs(v) for v in msg.position)

    observer = Observer()
    processes = []
    logs = []
    pid_path = Path("build/s6_04_supervisor_pids.json")
    pid_path.parent.mkdir(parents=True, exist_ok=True)

    def wait(predicate, what, timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(observer, timeout_sec=0.03)
            if predicate():
                return
        tail = ""
        if logs:
            logs[-1].flush()
            p = Path(logs[-1].name)
            tail = p.read_text(encoding="utf-8")[-3000:]
        raise AssertionError(f"{what} timed out: supervisor={observer.supervisor_status} "
                             f"sim={observer.sim_status} epoch={observer.epoch}\\n{tail}")

    def start_supervisor(scenario):
        observer.supervisor_status = None
        observer.sim_status = None
        observer.epoch = None
        observer.joint_motion = 0.0
        if pid_path.exists():
            pid_path.unlink()
        log = open(f"build/s6_04_{scenario}.log", "w", encoding="utf-8")
        logs.append(log)
        proc = subprocess.Popen(
            [sys.executable, "-m", "scripts.s6_ros2_supervisor",
             "--bridge", str(args.bridge.resolve()),
             "--pid-file", str(pid_path.resolve())],
            stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        processes.append(proc)
        wait(lambda: pid_path.exists(), "supervisor PID file")
        pids = json.loads(pid_path.read_text(encoding="utf-8"))
        wait(lambda: observer.supervisor_status == "ACTIVE" and
             observer.sim_status == "NORMAL" and observer.joint_motion > 0.0001,
             "lifecycle configure/activate, arm, and real joint motion", 12.0)
        assert proc.poll() is None
        return proc, pids

    def pause(value):
        wait(observer.pause_client.service_is_ready, "supervisor pause service")
        future = observer.pause_client.call_async(SetBool.Request(data=value))
        wait(future.done, "supervisor pause response")
        assert future.result().success, future.result().message

    def pid_alive(pid):
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False

    def stop_supervisor(proc):
        # Also clean orphaned children after the supervisor itself is SIGKILLed.
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        if proc.poll() is None:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=5)

    try:
        proc, pids = start_supervisor("controller_exit")
        initial_epoch = observer.epoch
        pause(True)
        wait(lambda: observer.supervisor_status == "PAUSED" and
             observer.sim_status == "PAUSED", "coordinated deactivate")
        paused_clock = observer.clock_us
        for _ in range(8):
            rclpy.spin_once(observer, timeout_sec=0.025)
        assert observer.clock_us == paused_clock, "PAUSED simulation clock moved"
        pause(False)
        wait(lambda: observer.supervisor_status == "ACTIVE" and
             observer.epoch is not None and observer.epoch > initial_epoch and
             observer.sim_status == "NORMAL", "coordinated resume epoch barrier")
        os.kill(pids["controller"], signal.SIGKILL)
        wait(lambda: observer.supervisor_status is not None and
             observer.supervisor_status.startswith("LATCHED:"),
             "supervisor latch after controller SIGKILL", 3.0)
        wait(lambda: observer.sim_status == "SAFE_STOP",
             "external SAFE_STOP on controller SIGKILL", 3.0)
        assert not pid_alive(pids["controller"]), "dead controller restarted"
        assert pid_alive(pids["simulator"]), "simulator should publish SAFE_STOP"
        stop_supervisor(proc)

        proc2, pids2 = start_supervisor("simulator_exit")
        os.kill(pids2["simulator"], signal.SIGKILL)
        wait(lambda: observer.supervisor_status is not None and
             observer.supervisor_status.startswith("LATCHED:"),
             "supervisor latch after simulator SIGKILL", 3.0)
        wait(lambda: not pid_alive(pids2["controller"]),
             "controller must terminate after simulator death", 3.0)
        assert not pid_alive(pids2["simulator"]), "simulator restarted automatically"
        assert proc2.poll() is None, "external supervisor died with simulator"
        stop_supervisor(proc2)

        proc3, pids3 = start_supervisor("supervisor_exit")
        assert observer.command_count > 0, "controller did not publish"
        os.kill(proc3.pid, signal.SIGKILL)
        proc3.wait(timeout=3)
        wait(lambda: observer.sim_status == "SAFE_STOP",
             "simulator watchdog after external supervisor SIGKILL", 3.0)
        # The controller also has its own lease: no more command publication.
        for _ in range(12):
            rclpy.spin_once(observer, timeout_sec=0.025)
        after_expiry = observer.command_count
        for _ in range(16):
            rclpy.spin_once(observer, timeout_sec=0.025)
        assert observer.command_count == after_expiry, (
            "controller continued publishing after supervisor death")
        assert pid_alive(pids3["simulator"]), "simulator watchdog did not stay alive"
        stop_supervisor(proc3)
        print("PASS S6-04: lifecycle configure/activate, pause/resume epoch, "
              "controller SIGKILL -> SAFE_STOP, simulator SIGKILL -> controller killed, "
              "supervisor SIGKILL -> lease SAFE_STOP + controller inhibited, "
              "no automatic restart", flush=True)
    finally:
        for proc in processes:
            stop_supervisor(proc)
        for log in logs:
            log.close()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
