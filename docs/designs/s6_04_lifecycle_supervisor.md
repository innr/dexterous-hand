# S6-04 — ROS2 Lifecycle 与独立安全监督器

## 目标与安全边界
S6-03 只能在仿真节点存活时检测控制发布者丢失。S6-04 将监督职责
移至独立进程 `scripts/s6_ros2_supervisor.py`，并为仿真与控制发布节点
添加 ROS2 lifecycle_msgs ChangeState/GetState 标准服务。

本阶段只用于 MuJoCo 仿真，不对 STS3215 真实舵机提供扭矩卸载、断电、
硬实时截止时间保证，也不实现网络分区下的分布式仲裁。

## 组件
- `simulation.s6_ros2_node --managed`：ROS2 LifecycleNode，
  初始 UNCONFIGURED、未授权；CONFIGURE 构建官方 MJCF 与 C++ bridge；
  ACTIVATE 进入 READY 但**仍未授权**；必须由监督器调用
  `/leap/supervisor_arm` 才接受 JointCommand。
- `scripts.s6_ros2_controller --managed`：LifecycleNode；
  只有 ACTIVE 时发布 JointCommand；即使 INACTIVE，仍以 steady timer
  发布 `/leap/controller_heartbeat`（单调递增序号）。
- `scripts.s6_ros2_supervisor`：独立进程，创建两个子进程，
  通过 lifecycle_msgs/srv/ChangeState 依次配置、激活、授权；
  监测 Popen 进程存活、仿真状态心跳和控制心跳。
- `/leap/supervisor_status`：TRANSIENT_LOCAL，BOOTING/ACTIVE/PAUSED/
  LATCHED:<reason>；`/leap/supervisor_pause` SetBool；
  `/leap/supervisor_stop` Trigger。
- `/leap/supervisor_arm` SetBool 只允许在 lifecycle ACTIVE 且
  SimSession READY 时授权；撤销授权会锁存 SAFE_STOP。
- 无 `--managed` 的旧 S6-03 CLI 继续保持兼容。

## 时序与状态机
1. 监督器启动两个进程，等待 /leap/sim_status 与 /leap/controller_heartbeat。
2. CONFIGURE sim/controller -> ACTIVATE sim -> 等 READY ->
   arm sim -> ACTIVATE controller -> ACTIVE。
3. 暂停：DEACTIVATE controller -> DEACTIVATE sim -> PAUSED；
   仿真时间冻结、命令清除、C++ bridge 关闭。
4. 恢复：ACTIVATE sim（SimSession 增加 epoch）-> READY ->
   arm -> ACTIVATE controller（清除旧 /clock、epoch、sequence）-> ACTIVE。
5. 监督器使用 steady-clock 心跳检测（默认 650ms）和进程退出检测。
   控制进程退出：锁存 LATCHED，终止控制进程，尝试调用
   /leap/sim_stop，仿真保持存活发布 SAFE_STOP。
   仿真进程退出：锁存 LATCHED，终止控制进程。
   SAFE_STOP/FAULT：锁存 LATCHED 并终止控制进程。
6. **绝不自动重启**；发生故障后需人工确认原因、终止监督器并
   重新启动新会话。仿真默认未授权避免重启后旧消息驱动。

## 失效模型与限制
- 监督器本身被 SIGKILL 时无法执行其 finally 清理；
  ROS2 仿真节点内部仍有 S6-03 的 100ms sim TTL/350ms steady watchdog，
  但不代表实物安全。部署真机需要独立 MCU/驱动器硬件看门狗。
- 监督器在单线程 ROS2 executor 中串行执行生命周期服务，
  启动/暂停期间阻塞可能延迟故障检测；非硬实时。
- ROS2 topic/service 不提供身份认证，`/leap/supervisor_arm` 并非
  安全加密授权。真机需要 ROS2 SROS2 / 网络隔离 / 独立安全链。
- 控制发布节点 ACTIVE 不代表闭环健康，后续可加入延迟分布、心跳
  jitter、命令有效性与控制质量指标。

## 验收
- Jazzy 下标准 lifecycle ChangeState 配置和激活两个真实节点。
- 16 路关节 DDS 指令推动官方 MuJoCo 模型。
- 通过监督器协调 PAUSE/RESUME，时钟冻结、epoch 递增。
- SIGKILL 控制节点 -> 外部监督器锁存，仿真 SAFE_STOP，无自动重启。
- SIGKILL 仿真节点 -> 外部监督器锁存并终止控制节点，无自动重启。
- S6-03 原 DDS 测试和 C++ CTest 不回退。

## 复现
```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base build/s6_ros2_log build --base-paths ros2_ws/src --build-base build/s6_ros2_pkg --install-base build/s6_ros2_install
source build/s6_ros2_install/setup.bash
python3 -m simulation.fetch_leap_menagerie
cmake -S cpp -B build/s6_ros2_cpp && cmake --build build/s6_ros2_cpp -j4
python3 scripts/s6_ros2_supervisor_integration.py --bridge build/s6_ros2_cpp/s6_control_bridge
```
