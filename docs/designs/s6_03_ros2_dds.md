# S6-03 — ROS2 Jazzy DDS 进程间验收与专用 JointCommand

## Goals
- 将 S6-02 的 JointState 命令换成 leap_interfaces/msg/JointCommand。
- 真正运行两个独立 ROS2 进程，通过 DDS 完成 16 路控制和 /clock、/joint_states 反馈。
- 验证 PAUSED、RESUME、RESET、旧 epoch、乱序 sequence、发布器意外退出的 SAFE_STOP。
- 不访问真实 STS3215；不将软件停止视作硬件断电。

## Protocol
ROS2 package: ros2_ws/src/leap_interfaces (ament_cmake + rosidl_default_generators).
JointCommand.msg: builtin_interfaces/Time stamp, uint64 epoch, uint64 sequence,
string[16] joint_names, float64[16] position.
Topic: /leap/joint_command, joint names are the 16 official Menagerie names, radians.
Other topics: /clock, /joint_states, /leap/sim_epoch, /leap/sim_status.
Services: /leap/sim_pause, /leap/sim_reset, /leap/sim_stop.

## Safety and time
- Stamp from current /clock. Commands must be no older than 100ms simulated time
  and at most 10ms in the future.
- Within each epoch sequence strictly increases. Invalid/duplicate/out-of-order
  commands rejected without changing authorized target.
- PAUSED: no physics ticks, /clock freezes, clear target and close C++ bridge.
- RESUME: epoch increments, sequence resets, C++ bridge recreated, fresh command required.
- RESET: MuJoCo time rewinds to zero, epoch increments, command/sequence/bridge reset.
- NORMAL with active target: no new command for >100ms sim time or >350ms
  steady time latches SAFE_STOP; explicit RESET required to recover.
- Controller publisher process exits unexpectedly: simulation remains running
  and its watchdog must latch SAFE_STOP. This is NOT a physical torque-off.
- Simulator process itself exits: its in-process watchdog cannot act. A
  separate hardware watchdog/supervisor is required for real actuators.

## Architecture
Independent controller process (scripts/s6_ros2_controller.py) subscribes
/clock and /leap/sim_epoch, publishes typed commands over DDS at 50Hz.
Simulator (simulation/s6_ros2_node.py) owns SimSession, C++17 S5 bridge,
official Menagerie MJCF, /clock and /joint_states at 100Hz.
Single-owner ROS callback executor avoids simultaneous mutation of SimSession;
a blocking callback may still delay watchdog, so no hard real-time claim.

## Acceptance
- ROS2 Jazzy colcon actually builds and imports leap_interfaces.
- Separate DDS publisher causes real official MJCF joint motion.
- /clock monotonically advances normally and freezes in PAUSED.
- RESUME/RESET increment epoch and reject stale messages.
- Sequence duplicate and out-of-order rejected.
- Terminating the remote publisher without graceful stop yields SAFE_STOP.
- Existing CTest S5/S6-02 regressions remain green.

## Reproduce on Ubuntu 24.04 + ROS2 Jazzy
```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_ws/src --build-base build/ros2_pkg --install-base build/ros2_install
source build/ros2_install/setup.bash
python3 -m simulation.fetch_leap_menagerie
cmake -S cpp -B build/cpp && cmake --build build/cpp
python3 scripts/s6_ros2_integration.py --bridge build/cpp/s6_control_bridge
```

CI workflow: .github/workflows/s6-ros2.yml runs the same in ros:jazzy-ros-base.
