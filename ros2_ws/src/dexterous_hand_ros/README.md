# dexterous_hand_ros

This ROS 2 Jazzy package exposes the canonical 16-DOF LEAP Hand interface.

## Build

    source /opt/ros/jazzy/setup.bash
    cd ros2_ws
    colcon build --symlink-install
    source install/setup.bash

## Run the safe virtual backend

    ros2 launch dexterous_hand_ros leap_hand.launch.py

The node subscribes to ~/joint_command and publishes ~/joint_states. Commands
use sensor_msgs/msg/JointState, with 16 positions in mapping order and radians.
If no command arrives before command_timeout_s, the default action is
`disable_torque`. The launch file always uses the virtual backend; real hardware
requires a separate reviewed backend.
