from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dexterous_hand_ros",
            executable="leap_hand_node",
            name="leap_hand_node",
            output="screen",
            parameters=[{"backend": "virtual", "publish_rate_hz": 50.0, "command_timeout_s": 0.25, "timeout_action": "disable_torque"}],
        )
    ])
