"""S6-03 independent ROS2 controller process (DDS publisher, no direct sim calls)."""
from __future__ import annotations

import argparse

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--position", type=float, default=0.18)
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMsg
    from std_msgs.msg import UInt64
    from leap_interfaces.msg import JointCommand
    from simulation.s6_official_bridge import load_contract

    class Controller(Node):
        def __init__(self):
            super().__init__("leap_s6_test_controller")
            self.names = [x["mujoco_joint"] for x in load_contract()]
            self.clock = None
            self.epoch = None
            self.seq = 0
            self.pub = self.create_publisher(JointCommand, "/leap/joint_command", 10)
            self.create_subscription(ClockMsg, "/clock", self.on_clock, 10)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(UInt64, "/leap/sim_epoch", self.on_epoch, qos)
            self.timer = self.create_timer(
                0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def on_clock(self, msg):
            self.clock = msg.clock

        def on_epoch(self, msg):
            if self.epoch != msg.data:
                self.seq = 0
            self.epoch = msg.data

        def tick(self):
            if self.clock is None or self.epoch is None:
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
