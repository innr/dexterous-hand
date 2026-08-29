# 可运行示例

这些示例按“先无硬件、后硬件”的顺序排列。

## 无硬件

    python -m pytest -q
    cmake -S cpp/hand_retargeting -B build/hand_retargeting
    cmake --build build/hand_retargeting
    ctest --test-dir build/hand_retargeting --output-on-failure

## ROS 2 虚拟后端

    source /opt/ros/jazzy/setup.bash
    cd ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    ros2 launch dexterous_hand_ros leap_hand.launch.py

## 硬件到位后再执行

    python -m hardware.ports --json
    python -m hardware.scan_sts3215 --help
    python -m hardware.motion_test --help

最后三条只用于查看或受安全门控保护的硬件流程；未确认端口、ID、方向和零位前，不要启用扭矩。
