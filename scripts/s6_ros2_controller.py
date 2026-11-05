"""S6-04 lifecycle-managed ROS2 controller with steady heartbeat."""
from __future__ import annotations

import argparse
import time

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--position", type=float, default=0.18)
    parser.add_argument("--managed", action="store_true")
    args = parser.parse_args()
    import rclpy
    from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMsg
    from std_msgs.msg import UInt64
    from leap_interfaces.msg import JointCommand
    from simulation.s6_official_bridge import load_contract

    class Controller(LifecycleNode):
        def __init__(self):
            super().__init__("leap_s6_test_controller")
            self.names = [x["mujoco_joint"] for x in load_contract()]
            self.active = not args.managed
            self.last_supervisor_heartbeat = None
            self.supervisor_lease_s = 0.35
            self.heartbeat_seq = 0
            self.heartbeat_pub = self.create_publisher(
                UInt64, "/leap/controller_heartbeat", 10)
            self.clock = None
            self.epoch = None
            self.seq = 0
            self.pub = self.create_publisher(JointCommand, "/leap/joint_command", 10)
            clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(ClockMsg, "/clock", self.on_clock, clock_qos)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(UInt64, "/leap/sim_epoch", self.on_epoch, qos)
            self.create_subscription(UInt64, "/leap/supervisor_heartbeat",
                                     self.on_supervisor_heartbeat, 10)
            self.timer = self.create_timer(
                0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def on_configure(self, state):
            return TransitionCallbackReturn.SUCCESS

        def on_activate(self, state):
            if args.managed and not self.supervisor_alive():
                return TransitionCallbackReturn.FAILURE
            self.seq = 0
            self.clock = None
            self.epoch = None
            self.active = True
            return super().on_activate(state)

        def on_deactivate(self, state):
            self.active = False
            self.clock = None
            self.epoch = None
            return super().on_deactivate(state)

        def on_cleanup(self, state):
            self.active = False
            return TransitionCallbackReturn.SUCCESS

        def on_shutdown(self, state):
            self.active = False
            return TransitionCallbackReturn.SUCCESS

        def on_supervisor_heartbeat(self, _msg):
            self.last_supervisor_heartbeat = time.monotonic()

        def supervisor_alive(self):
            return (self.last_supervisor_heartbeat is not None and
                    time.monotonic() - self.last_supervisor_heartbeat <= self.supervisor_lease_s)

        def on_clock(self, msg):
            self.clock = msg.clock

        def on_epoch(self, msg):
            if self.epoch != msg.data:
                self.seq = 0
            self.epoch = msg.data

        def tick(self):
            self.heartbeat_seq += 1
            self.heartbeat_pub.publish(UInt64(data=self.heartbeat_seq))
            if args.managed and self.active and not self.supervisor_alive():
                self.active = False  # Fail closed if the external supervisor dies.
                self.get_logger().error("external supervisor heartbeat lease expired")
            if not self.active or self.clock is None or self.epoch is None:
                return
            cmd = JointCommand()
            cmd.stamp = self.clock
            cmd.epoch = self.epoch
            cmd.sequence = self.seq
            cmd.joint_names = self.names
            cmd.position = [args.position] * 16
            self.pub.publish(cmd)
            self.seq += 1

    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
