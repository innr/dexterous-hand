"""ROS 2 node exposing the canonical LEAP Hand joint interface."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .backends import HandBackend, VirtualBackend
from .validation import CommandValidationError, load_mapping, validate_joint_command

try:  # Keep pure validation tests runnable without ROS 2 installed.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool
except ImportError:  # pragma: no cover - exercised only without ROS 2
    rclpy = None
    Node = object  # type: ignore[assignment]
    JointState = Any  # type: ignore[assignment]
    SetBool = Any  # type: ignore[assignment]


@dataclass
class CommandWatchdog:
    timeout_s: float = 0.25
    action: str = "disable_torque"
    last_command_s: float | None = None
    timed_out: bool = False

    def __post_init__(self) -> None:
        if self.timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if self.action not in {"disable_torque", "hold"}:
            raise ValueError("action must be disable_torque or hold")

    def command_received(self, now_s: float) -> None:
        self.last_command_s = float(now_s)
        self.timed_out = False

    def expired(self, now_s: float) -> bool:
        if self.last_command_s is None:
            return True
        return float(now_s) - self.last_command_s >= self.timeout_s


if rclpy is not None:

    class LeapHandNode(Node):
        def __init__(self) -> None:
            super().__init__("leap_hand_node")
            self.declare_parameter("mapping_path", "config/joint_mapping.json")
            self.declare_parameter("backend", "virtual")
            self.declare_parameter("publish_rate_hz", 50.0)
            self.declare_parameter("command_timeout_s", 0.25)
            self.declare_parameter("timeout_action", "disable_torque")
            self.declare_parameter("allow_uncalibrated", False)

            mapping_path = Path(str(self.get_parameter("mapping_path").value))
            self.mapping = load_mapping(mapping_path)
            backend_name = str(self.get_parameter("backend").value)
            if backend_name != "virtual":
                raise ValueError("only the virtual backend is available in this PR")
            self.backend: HandBackend = VirtualBackend(size=len(self.mapping["joints"]))
            self.watchdog = CommandWatchdog(
                timeout_s=float(self.get_parameter("command_timeout_s").value),
                action=str(self.get_parameter("timeout_action").value),
            )
            self.joint_names = tuple(
                sorted(self.mapping["joints"], key=lambda item: int(item["hardware_id"]))
            )
            self.name_values = [str(item["name"]) for item in self.joint_names]
            self.command_subscription = self.create_subscription(
                JointState, "~/joint_command", self._on_command, 10
            )
            self.state_publisher = self.create_publisher(JointState, "~/joint_states", 10)
            rate = float(self.get_parameter("publish_rate_hz").value)
            if rate <= 0:
                raise ValueError("publish_rate_hz must be positive")
            self.timer = self.create_timer(1.0 / rate, self._on_timer)

        def _on_command(self, message: JointState) -> None:
            try:
                positions = validate_joint_command(message.position, list(message.name), self.mapping)
                self.backend.command_positions(positions)
                now = self.get_clock().now().nanoseconds * 1e-9
                self.watchdog.command_received(now)
            except (CommandValidationError, ValueError) as exc:
                self.get_logger().warning(f"rejected joint_command: {exc}")

        def _on_timer(self) -> None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.watchdog.expired(now) and not self.watchdog.timed_out:
                if self.watchdog.action == "disable_torque":
                    self.backend.set_torque(False)
                self.watchdog.timed_out = True
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = self.name_values
            message.position = self.backend.read_positions().tolist()
            message.velocity = [0.0] * len(self.name_values)
            self.state_publisher.publish(message)


else:

    class LeapHandNode:  # pragma: no cover - ROS 2 is an optional runtime
        def __init__(self) -> None:
            raise RuntimeError("ROS 2 is not installed; source the Jazzy environment first")


def main(args: list[str] | None = None) -> None:
    if rclpy is None:
        raise SystemExit("ROS 2 is not installed; source the Jazzy environment first")
    rclpy.init(args=args)
    node = LeapHandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
