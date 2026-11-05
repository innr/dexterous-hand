"""Optional ROS2 rclpy adapter for S6-02. Run only inside a sourced ROS2 environment.

ROS command: /leap/joint_command sensor_msgs/JointState, names = official MJCF
joint names, position in rad, header.stamp = /clock time, frame_id = sim_epoch:N.
The epoch token is a simulation-specific convention, not a TF frame.
"""
from __future__ import annotations

import argparse
from pathlib import Path

from .fetch_leap_menagerie import DEST
from .s6_official_bridge import OfficialPlant, SimSession


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--model", type=Path, default=DEST / "right_hand.xml")
    args, ros_args = parser.parse_known_args()

    # Lazy imports: all model/safety tests work without a ROS2 installation.
    import rclpy
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMsg
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, UInt64
    from std_srvs.srv import SetBool, Trigger
    from builtin_interfaces.msg import Time

    def stamp_from_us(micros: int) -> Time:
        result = Time()
        result.sec = micros // 1_000_000
        result.nanosec = (micros % 1_000_000) * 1000
        return result

    def micros_from_stamp(stamp: Time) -> int:
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError("invalid command stamp")
        return stamp.sec * 1_000_000 + stamp.nanosec // 1000

    class LeapSimNode(Node):
        def __init__(self) -> None:
            super().__init__("leap_s6_sim")
            self.plant = OfficialPlant(args.model.resolve())
            self.session = SimSession(self.plant, args.bridge.resolve())
            clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            epoch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.clock_pub = self.create_publisher(ClockMsg, "/clock", clock_qos)
            self.state_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.status_pub = self.create_publisher(String, "/leap/sim_status", 10)
            self.epoch_pub = self.create_publisher(UInt64, "/leap/sim_epoch", epoch_qos)
            self.create_subscription(JointState, "/leap/joint_command", self.on_command, 10)
            self.create_service(SetBool, "/leap/sim_pause", self.on_pause)
            self.create_service(Trigger, "/leap/sim_reset", self.on_reset)
            self.create_service(Trigger, "/leap/sim_stop", self.on_stop)
            # STEADY_TIME drives physics even if ROS /clock is paused or not initialized.
            # No ROS-time timer: it would deadlock before the first /clock message.
            self.timer = self.create_timer(
                0.01, self.on_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.publish_state()

        def publish_state(self) -> None:
            stamp = stamp_from_us(self.plant.time_us)
            self.clock_pub.publish(ClockMsg(clock=stamp))
            positions, velocities = self.plant.feedback()
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = self.plant.names
            msg.position = positions
            msg.velocity = velocities
            self.state_pub.publish(msg)
            self.status_pub.publish(String(data=self.session.state))
            self.epoch_pub.publish(UInt64(data=self.session.epoch))

        def on_command(self, msg: JointState) -> None:
            try:
                token = f"sim_epoch:{self.session.epoch}"
                if msg.header.frame_id != token:
                    raise ValueError(f"epoch mismatch: expected {token}")
                self.session.set_target(
                    list(msg.name), list(msg.position), epoch=self.session.epoch,
                    stamp_us=micros_from_stamp(msg.header.stamp))
            except (ValueError, TypeError) as exc:
                # Reject bad command without moving. Existing authorized target
                # will expire by simulation-time TTL if not refreshed.
                self.get_logger().warning(f"Rejected joint command: {exc}")

        def on_tick(self) -> None:
            try:
                self.session.tick()
            except Exception as exc:
                self.get_logger().error(f"Latched simulation SAFE_STOP: {exc}")
            self.publish_state()

        def on_pause(self, request: SetBool.Request,
                     response: SetBool.Response) -> SetBool.Response:
            try:
                self.session.pause(request.data)
                response.success = True
                response.message = self.session.state
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            self.publish_state()
            return response

        def on_reset(self, _request: Trigger.Request,
                     response: Trigger.Response) -> Trigger.Response:
            try:
                self.session.reset()
                response.success = True
                response.message = f"new epoch {self.session.epoch}; reissue commands"
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            self.publish_state()
            return response

        def on_stop(self, _request: Trigger.Request,
                    response: Trigger.Response) -> Trigger.Response:
            self.session.stop()
            response.success = True
            response.message = "SAFE_STOP latched; reset required"
            self.publish_state()
            return response

        def destroy_node(self) -> bool:
            self.session.close()
            return super().destroy_node()

    rclpy.init(args=ros_args)
    node = None
    try:
        node = LeapSimNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
