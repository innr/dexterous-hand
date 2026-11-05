"""S6-04 optional managed ROS2 lifecycle simulation with fail-closed arming.

Unmanaged default preserves S6-03 regression behavior. Simulation only.
"""
from __future__ import annotations

import argparse
from pathlib import Path

from .fetch_leap_menagerie import DEST
from .s6_official_bridge import OfficialPlant, SimSession
from .s6_command_contract import JointCommandData


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--managed", action="store_true", help="Require lifecycle activation and supervisor arming")
    parser.add_argument("--model", type=Path, default=DEST / "right_hand.xml")
    args, ros_args = parser.parse_known_args()

    # Lazy imports: all model/safety tests work without a ROS2 installation.
    import rclpy
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMsg
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, UInt64
    from std_srvs.srv import SetBool, Trigger
    from builtin_interfaces.msg import Time
    from leap_interfaces.msg import JointCommand

    def stamp_from_us(micros: int) -> Time:
        result = Time()
        result.sec = micros // 1_000_000
        result.nanosec = (micros % 1_000_000) * 1000
        return result

    def micros_from_stamp(stamp: Time) -> int:
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError("invalid command stamp")
        return stamp.sec * 1_000_000 + stamp.nanosec // 1000

    class LeapSimNode(LifecycleNode):
        def __init__(self) -> None:
            super().__init__("leap_s6_sim")
            self.managed = args.managed
            self.lifecycle_active = not self.managed
            self.armed = not self.managed
            self.next_epoch = 1
            self.plant = None
            self.session = None
            if not self.managed:
                self._create_session()
            clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            epoch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.clock_pub = self.create_publisher(ClockMsg, "/clock", clock_qos)
            self.state_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.status_pub = self.create_publisher(String, "/leap/sim_status", 10)
            self.epoch_pub = self.create_publisher(UInt64, "/leap/sim_epoch", epoch_qos)
            self.create_subscription(JointCommand, "/leap/joint_command", self.on_command, 10)
            self.create_service(SetBool, "/leap/sim_pause", self.on_pause)
            self.create_service(Trigger, "/leap/sim_reset", self.on_reset)
            self.create_service(Trigger, "/leap/sim_stop", self.on_stop)
            self.create_service(SetBool, "/leap/supervisor_arm", self.on_arm)
            # STEADY_TIME drives physics even if ROS /clock is paused or not initialized.
            # No ROS-time timer: it would deadlock before the first /clock message.
            self.timer = self.create_timer(
                0.01, self.on_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.publish_state()

        def _create_session(self) -> None:
            self.plant = OfficialPlant(args.model.resolve())
            self.session = SimSession(self.plant, args.bridge.resolve())
            if self.managed:
                self.session.epoch = self.next_epoch
                self.next_epoch += 1

        def on_configure(self, state):
            try:
                if self.session is None:
                    self._create_session()
                self.publish_state()
                return TransitionCallbackReturn.SUCCESS
            except Exception as exc:
                self.get_logger().error(f"configure failed: {exc}")
                return TransitionCallbackReturn.FAILURE

        def on_activate(self, state):
            try:
                if self.session is None:
                    raise RuntimeError("not configured")
                if self.session.state == "PAUSED":
                    self.session.pause(False)  # Epoch barrier.
                if self.session.state != "READY":
                    raise RuntimeError(f"cannot activate from {self.session.state}")
                self.lifecycle_active = True
                self.armed = not self.managed
                self.publish_state()
                return super().on_activate(state)
            except Exception as exc:
                self.get_logger().error(f"activate failed: {exc}")
                return TransitionCallbackReturn.FAILURE

        def on_deactivate(self, state):
            self.lifecycle_active = False
            self.armed = False
            if self.session is not None and self.session.state in ("READY", "NORMAL"):
                self.session.pause(True)
            self.publish_state()
            return super().on_deactivate(state)

        def on_cleanup(self, state):
            self.lifecycle_active = False
            self.armed = False
            if self.session is not None:
                self.next_epoch = max(self.next_epoch, self.session.epoch + 1)
                self.session.close()
                self.session = None
                self.plant = None
            self.publish_state()
            return TransitionCallbackReturn.SUCCESS

        def on_shutdown(self, state):
            self.lifecycle_active = False
            self.armed = False
            if self.session is not None:
                self.session.close()
                self.session = None
                self.plant = None
            return TransitionCallbackReturn.SUCCESS

        def publish_state(self) -> None:
            if self.session is None:
                self.status_pub.publish(String(data="UNCONFIGURED"))
                return
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

        def on_command(self, msg: JointCommand) -> None:
            try:
                if not self.lifecycle_active or not self.armed or self.session is None:
                    raise ValueError("simulator not activated and supervisor-armed")
                self.session.accept_command(JointCommandData(
                    stamp_us=micros_from_stamp(msg.stamp),
                    epoch=int(msg.epoch),
                    sequence=int(msg.sequence),
                    names=tuple(msg.joint_names),
                    positions=tuple(msg.position)))
            except (ValueError, TypeError) as exc:
                # Reject bad command without moving. Existing authorized target
                # will expire by simulation-time TTL if not refreshed.
                self.get_logger().warning(f"Rejected joint command: {exc}")

        def on_tick(self) -> None:
            try:
                if self.lifecycle_active and self.session is not None:
                    self.session.tick()
            except Exception as exc:
                self.get_logger().error(f"Latched simulation SAFE_STOP: {exc}")
            self.publish_state()

        def on_pause(self, request: SetBool.Request,
                     response: SetBool.Response) -> SetBool.Response:
            try:
                if self.managed:
                    raise ValueError("managed simulator: use lifecycle deactivate/activate")
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
                if self.managed and (not self.lifecycle_active or not self.armed):
                    raise ValueError("managed simulator requires supervisor authorization")
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
            self.armed = False
            if self.session is not None:
                self.session.stop()
            response.success = True
            response.message = "SAFE_STOP latched; reset required"
            self.publish_state()
            return response

        def on_arm(self, request: SetBool.Request,
                   response: SetBool.Response) -> SetBool.Response:
            if not request.data:
                self.armed = False
                if self.session is not None:
                    self.session.stop()  # Disarm is a latched safety action.
                response.success = True
                response.message = "disarmed and SAFE_STOP latched"
            elif not self.lifecycle_active or self.session is None or self.session.state != "READY":
                response.success = False
                response.message = "arm requires active READY lifecycle state"
            else:
                self.armed = True
                response.success = True
                response.message = f"armed epoch {self.session.epoch}"
            self.publish_state()
            return response

        def destroy_node(self) -> bool:
            if self.session is not None:
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
