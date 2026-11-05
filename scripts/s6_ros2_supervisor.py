"""S6-04 independent ROS2 process supervisor; SIMULATION ONLY.

Starts both lifecycle nodes as children. A dead/stalled child latches the
supervisor and terminates the controller; it never auto-restarts a failed node.
No physical servo torque-off or hard real-time guarantee is provided.
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--pid-file", type=Path, required=True)
    parser.add_argument("--heartbeat-timeout", type=float, default=0.65)
    parser.add_argument("--startup-timeout", type=float, default=12.0)
    args = parser.parse_args()
    if args.heartbeat_timeout <= 0.1 or args.startup_timeout <= 1.0:
        parser.error("unsafe timeout configuration")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from std_msgs.msg import String, UInt64
    from std_srvs.srv import SetBool, Trigger

    class Supervisor(Node):
        def __init__(self):
            super().__init__("leap_s6_supervisor")
            self.mode = "BOOTING"
            self.reason = ""
            self.pending = None
            self.sim_status = None
            self.sim_seen = None
            self.controller_seen = None
            self.last_heartbeat_seq = -1
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_pub = self.create_publisher(
                String, "/leap/supervisor_status", qos)
            self.heartbeat_pub = self.create_publisher(
                UInt64, "/leap/supervisor_heartbeat", 10)
            self.heartbeat_seq = 0
            self.create_subscription(String, "/leap/sim_status", self.on_sim_status, 10)
            self.create_subscription(UInt64, "/leap/controller_heartbeat",
                                     self.on_controller_heartbeat, 10)
            self.create_service(SetBool, "/leap/supervisor_pause", self.on_pause)
            self.create_service(Trigger, "/leap/supervisor_stop", self.on_stop)
            self.sim_change = self.create_client(
                ChangeState, "/leap_s6_sim/change_state")
            self.controller_change = self.create_client(
                ChangeState, "/leap_s6_test_controller/change_state")
            self.sim_arm = self.create_client(SetBool, "/leap/supervisor_arm")
            self.sim_stop = self.create_client(Trigger, "/leap/sim_stop")
            self.publish()

        def publish(self):
            value = self.mode if not self.reason else f"{self.mode}:{self.reason}"
            self.status_pub.publish(String(data=value))

        def on_sim_status(self, msg):
            self.sim_status = msg.data
            self.sim_seen = time.monotonic()

        def on_controller_heartbeat(self, msg):
            if msg.data != self.last_heartbeat_seq:
                self.controller_seen = time.monotonic()
                self.last_heartbeat_seq = msg.data

        def on_pause(self, request, response):
            if request.data and self.mode == "ACTIVE" and self.pending is None:
                self.pending = "pause"
                response.success = True
                response.message = "pause requested"
            elif not request.data and self.mode == "PAUSED" and self.pending is None:
                self.pending = "resume"
                response.success = True
                response.message = "resume requested"
            else:
                response.success = False
                response.message = f"transition not permitted from {self.mode}"
            return response

        def on_stop(self, _request, response):
            if self.mode != "LATCHED":
                self.pending = "stop"
            response.success = True
            response.message = "stop requested; supervisor will latch"
            return response

    rclpy.init()
    node = Supervisor()
    args.pid_file.parent.mkdir(parents=True, exist_ok=True)
    simulator = None
    controller = None
    shutdown = False

    def handle_signal(_signum, _frame):
        nonlocal shutdown
        shutdown = True

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    def pump(duration=0.03):
        node.heartbeat_seq += 1
        node.heartbeat_pub.publish(UInt64(data=node.heartbeat_seq))
        rclpy.spin_once(node, timeout_sec=duration)

    def wait(predicate, timeout, what, *, check_children=True):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not shutdown:
            if check_children:
                if simulator is not None and simulator.poll() is not None:
                    raise RuntimeError(f"simulator exited during {what}")
                if controller is not None and controller.poll() is not None:
                    raise RuntimeError(f"controller exited during {what}")
            pump()
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {what}")

    def call(client, request, what, timeout=3.0, *, check_children=True):
        wait(client.service_is_ready, timeout, f"{what} service",
             check_children=check_children)
        future = client.call_async(request)
        wait(future.done, timeout, f"{what} response",
             check_children=check_children)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"{what} rejected: {getattr(response, 'message', '')}")
        return response

    def transition(client, transition_id, name):
        request = ChangeState.Request()
        request.transition.id = transition_id
        call(client, request, name)

    def arm():
        call(node.sim_arm, SetBool.Request(data=True), "arm simulation")

    def stop_process(proc):
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)

    def latch(reason):
        nonlocal controller
        if node.mode == "LATCHED":
            return
        node.mode = "LATCHED"
        node.reason = reason
        node.pending = None
        node.publish()
        print(f"S6-04 supervisor LATCHED: {reason}", flush=True)
        # Controller is terminated even if the simulator is already dead.
        stop_process(controller)
        # If simulator still exists, explicitly request its latched SAFE_STOP.
        if simulator is not None and simulator.poll() is None:
            try:
                call(node.sim_stop, Trigger.Request(), "simulation SAFE_STOP", 1.5,
                     check_children=False)
            except Exception as exc:
                print(f"SAFE_STOP service unconfirmed: {exc}", flush=True)

    try:
        simulator = subprocess.Popen(
            [sys.executable, "-m", "simulation.s6_ros2_node", "--managed",
             "--bridge", str(args.bridge.resolve())])
        controller = subprocess.Popen(
            [sys.executable, "-m", "scripts.s6_ros2_controller", "--managed"])
        temp_path = args.pid_file.with_suffix(".tmp")
        temp_path.write_text(json.dumps({
            "supervisor": os.getpid(),
            "simulator": simulator.pid,
            "controller": controller.pid
        }), encoding="utf-8")
        os.replace(temp_path, args.pid_file)

        wait(lambda: node.sim_seen is not None and
             node.controller_seen is not None, args.startup_timeout, "DDS heartbeats")
        transition(node.sim_change, Transition.TRANSITION_CONFIGURE, "sim configure")
        transition(node.controller_change, Transition.TRANSITION_CONFIGURE, "controller configure")
        transition(node.sim_change, Transition.TRANSITION_ACTIVATE, "sim activate")
        wait(lambda: node.sim_status == "READY", 3.0, "sim READY")
        arm()
        transition(node.controller_change, Transition.TRANSITION_ACTIVATE, "controller activate")
        node.mode = "ACTIVE"
        node.publish()

        while not shutdown:
            pump(0.02)
            if node.mode == "LATCHED":
                node.publish()
                continue
            if simulator.poll() is not None:
                latch("simulator_exit")
                continue
            if controller.poll() is not None:
                latch("controller_exit")
                continue
            now = time.monotonic()
            if node.sim_seen is None or now - node.sim_seen > args.heartbeat_timeout:
                latch("simulator_heartbeat_timeout")
                continue
            if node.controller_seen is None or now - node.controller_seen > args.heartbeat_timeout:
                latch("controller_heartbeat_timeout")
                continue
            if node.sim_status in ("SAFE_STOP", "FAULT"):
                latch("simulation_fault")
                continue
            if node.pending == "stop":
                latch("operator_stop")
                continue
            if node.pending == "pause":
                node.pending = None
                try:
                    transition(node.controller_change, Transition.TRANSITION_DEACTIVATE,
                               "controller deactivate")
                    transition(node.sim_change, Transition.TRANSITION_DEACTIVATE,
                               "sim deactivate")
                    wait(lambda: node.sim_status == "PAUSED", 3.0, "sim PAUSED")
                    node.mode = "PAUSED"
                    node.publish()
                except Exception as exc:
                    latch(f"pause_failure:{exc}")
            elif node.pending == "resume":
                node.pending = None
                try:
                    transition(node.sim_change, Transition.TRANSITION_ACTIVATE,
                               "sim reactivate")
                    wait(lambda: node.sim_status == "READY", 3.0, "sim READY on resume")
                    arm()
                    transition(node.controller_change, Transition.TRANSITION_ACTIVATE,
                               "controller reactivate")
                    node.mode = "ACTIVE"
                    node.publish()
                except Exception as exc:
                    latch(f"resume_failure:{exc}")
    except Exception as exc:
        latch(f"startup_or_supervision_failure:{exc}")
    finally:
        # No automatic restart. Explicit operator relaunch is needed.
        stop_process(controller)
        stop_process(simulator)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
