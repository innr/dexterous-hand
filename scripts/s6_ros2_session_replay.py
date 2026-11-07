"""S7-00 PR-4a: unmanaged process restart must reject the previous session_id."""
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=40.0)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String, UInt64
    from leap_interfaces.msg import JointCommand
    from simulation.canonical import CANONICAL_NAMES

    names = list(CANONICAL_NAMES)
    deadline = time.monotonic() + args.timeout
    log_path = Path("build/s6_session_replay.log")
    log_path.parent.mkdir(parents=True, exist_ok=True)

    def start_sim(log_file):
        return subprocess.Popen(
            [sys.executable, "-m", "simulation.s6_ros2_node",
             "--bridge", str(args.bridge.resolve())],
            stdout=log_file, stderr=subprocess.STDOUT)

    def start_controller():
        return subprocess.Popen(
            [sys.executable, "-m", "scripts.s6_ros2_controller"],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    def stop(proc):
        if proc is not None and proc.poll() is None:
            proc.kill()
            proc.wait(timeout=3)

    rclpy.init()

    class Observer(Node):
        def __init__(self):
            super().__init__("leap_s6_session_replay")
            self.status = None
            self.epoch = None
            self.session_id = None
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, "/leap/sim_status", self.on_status, 10)
            self.create_subscription(UInt64, "/leap/sim_epoch", self.on_epoch, qos)
            self.create_subscription(UInt64, "/leap/sim_session", self.on_session, qos)
            self.pub = self.create_publisher(JointCommand, "/leap/joint_command", 10)

        def on_status(self, msg):
            self.status = msg.data

        def on_epoch(self, msg):
            self.epoch = int(msg.data)

        def on_session(self, msg):
            self.session_id = int(msg.data)

        def inject(self, session_id, epoch, sequence, stamp_us=0):
            cmd = JointCommand()
            cmd.stamp.sec = stamp_us // 1_000_000
            cmd.stamp.nanosec = (stamp_us % 1_000_000) * 1000
            cmd.session_id = session_id
            cmd.epoch = epoch
            cmd.sequence = sequence
            cmd.joint_names = names
            cmd.position = [0.12] * 16
            self.pub.publish(cmd)

    obs = Observer()
    sim_log = log_path.open("w", encoding="utf-8")
    sim = None
    controller = None
    try:
        def spin_until(predicate, timeout=8.0, what="condition"):
            until = min(deadline, time.monotonic() + timeout)
            while time.monotonic() < until:
                if sim is not None and sim.poll() is not None:
                    raise AssertionError(f"simulator exited: {sim.returncode}")
                rclpy.spin_once(obs, timeout_sec=0.03)
                if predicate():
                    return
            raise AssertionError(f"timeout waiting for {what}: status={obs.status} "
                                 f"session={obs.session_id} epoch={obs.epoch}")

        sim = start_sim(sim_log)
        spin_until(lambda: obs.session_id not in (None, 0), what="session A")
        session_a = obs.session_id
        epoch_a = obs.epoch
        controller = start_controller()
        spin_until(lambda: obs.status == "NORMAL", what="controller motion on A")
        stop(controller)
        controller = None
        stop(sim)
        sim = None
        time.sleep(0.5)
        obs.status = None
        obs.session_id = None
        obs.epoch = None
        sim = start_sim(sim_log)
        spin_until(lambda: obs.session_id not in (None, 0, session_a), what="session B")
        session_b = obs.session_id
        epoch_b = obs.epoch
        assert epoch_b == 0
        obs.inject(session_a, epoch_a if epoch_a is not None else 0, 1)
        for _ in range(20):
            rclpy.spin_once(obs, timeout_sec=0.05)
        assert obs.status != "NORMAL", "stale session A accepted on process B"
        sim_log.flush()
        text = log_path.read_text(encoding="utf-8")
        assert "stale_session" in text
        obs.inject(session_b, epoch_b, 1)
        spin_until(lambda: obs.status == "NORMAL", what="fresh session B accepted")
        print("PASS S7-00 session replay: stale session rejected, new nonce accepted")
    finally:
        stop(controller)
        stop(sim)
        obs.destroy_node()
        rclpy.shutdown()
        sim_log.close()


if __name__ == "__main__":
    main()
