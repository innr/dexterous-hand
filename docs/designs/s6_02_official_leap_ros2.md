# S6-02 — 官方 LEAP Hand MJCF + ROS2 /clock 与安全会话

状态：Implemented / ROS2 运行时待 ROS2 Jazzy 环境 HIL-less 验收。仅仿真，不访问硬件。

## Context
S6-01 的 16DOF surrogate 已与 C++17 S5 核心闭环。S6-02 将仿真 plant 替换为 Google DeepMind MuJoCo Menagerie LEAP right-hand MJCF（上游 MIT），保留现有 `s6_control_bridge` 进程协议和所有 S5 stop gates。

## Goals / Non-goals
- Goals：官方 16 关节/16 actuator 的真实 MJCF 运动，语义映射，100Hz 控制+500Hz 物理步进，ROS2 `/clock`/`joint_states`/`joint_command`，暂停/重置/停机，旧 epoch/TTL 命令拒绝，CI 仿真验证。
- Non-goals：官方物理参数准确性标定、硬件 ID/零位确认、真实串口、ROS2 control controller_manager、ROS2 CI 实际中间件验收、左手模型。

## Source & mapping
- Menagerie commit: `98a70c569c4c1b6da467a20012f766ee1f8dc521`.
- `python -m simulation.fetch_leap_menagerie` 下载 pinned `right_hand.xml` + 10 OBJ + upstream LICENSE/README；下载步骤显式执行，运行时不联网。路径 `third_party/mujoco_menagerie/leap_hand` 不提交二进制。
- `config/menagerie_right_mapping.json` 是新的**仿真专用**逻辑 ID -> MJCF joint -> actuator 映射，运行时验证 16 个 actuator 确实驱动指定关节；`config/joint_mapping.json` 保持不变。
- 旧硬件映射与 Menagerie 的拇指关节顺序/运动范围不完全一致；不宣称仿真已证明 STS3215 硬件映射正确。
- 模型和 C++ 默认关节限位取交集，当前只支持每路包含零位的范围，且输出越界直接 SAFE_STOP，不做静默截断。

## Architecture
```text
ROS2 /leap/joint_command (JointState, 16 names, rad, sim_epoch:N)
     -> validate epoch + timestamp + name set + limits
SimSession (READY / NORMAL / PAUSED / SAFE_STOP / FAULT)
     -> OfficialPlant(qpos/qvel, model-specific indices)
     -> s6_control_bridge C++17 (S5 ControlLoop + MotionControlCore + safety)
     -> OfficialPlant 16 position actuators -> 5 x mj_step(2ms)
     -> /clock, /leap/joint_states, /leap/sim_status, /leap/sim_epoch
```
- ROS2 adapter `simulation/s6_ros2_node.py` imports rclpy lazily. Pure simulation core and tests do not require ROS2.
- Physics driver uses steady-clock 10ms timer; published timestamps always from MuJoCo time, never wall time. Simulation may run slower than wall time; this is not a hard realtime guarantee.
- `/leap/joint_states`: `sensor_msgs/msg/JointState`, names in official MJCF mapping order, position rad, velocity rad/s, effort empty, stamp = `/clock`. ROS2 consumers set `use_sim_time:=true`.
- `/leap/joint_command`: `sensor_msgs/msg/JointState`, 16 unique official names, positions rad, header stamp in current simulation time. **Simulation-only contract** requires `header.frame_id="sim_epoch:<N>"` to reject commands from previous reset epochs. This overloads frame_id and must be replaced with a custom stamped command message before production ROS2 deployment.
- `/leap/sim_epoch`: `std_msgs/msg/UInt64`, transient-local; `/leap/sim_status`: `std_msgs/msg/String`.
- Services: `/leap/sim_pause` `std_srvs/SetBool`, `/leap/sim_reset` `std_srvs/Trigger`, `/leap/sim_stop` `std_srvs/Trigger`.

## Safety/time behavior
- Command age is checked at acceptance AND every simulation tick. Stale >100ms (simulation time) => latched SAFE_STOP, bridge closed, no further movement until explicit reset.
- PAUSED: no `mj_step`, frozen `/clock`; clear command and close C++ bridge. RESUME: new bridge instance, state READY, fresh command required, no wall-clock catch-up.
- RESET: explicit service only; close old bridge, reset MuJoCo, increment epoch, rebuild C++ bridge, clear targets; `/clock` jumps backward to 0. Clients must clear trajectory/filter state on epoch changes and handle ROS time jumps.
- SAFE_STOP: close bridge and block commands; does not physically disable torque. RESET is the only re-arm path for simulation. No hardware access.
- On absence of authorized target: simulation may advance while actuator holds measured pose; no old command replay.
- Command invalid (duplicate/missing name, NaN, limits, stale epoch/time): reject and log; existing valid command expires by TTL. Protocol or physics failure: SAFE_STOP.
- Timeouts based on simulated time pause while `/clock` is frozen. A separate wall-time health monitor may be required for real hardware or remote ROS failures.

## Tests / Acceptance
- CTest `S6OfficialModelTest`: upstream official MJCF load, 16 joint/actuator map and motion, invalid commands, PAUSED clock frozen, resume new bridge, TTL -> SAFE_STOP, reset epoch increment, stale epoch rejected.
- CTest existing S1–S6-01 regressions.
- CI downloads pinned official model before configure. No real serial device used.
- ROS2 runtime smoke and QoS interoperability **not** exercised by standard Ubuntu CMake CI; must be run on ROS2 Jazzy workstation before calling middleware integration production-ready.

## Tradeoffs / next
- `sensor_msgs/JointState` as command and `frame_id` epoch is intentionally an interim learning interface; next PR should define dedicated `JointCommand.msg` with explicit epoch, and optionally a ROS2 package/launch test.
- Menagerie actuator PD and geometry are not STS3215 real motor/gearbox characteristics.
