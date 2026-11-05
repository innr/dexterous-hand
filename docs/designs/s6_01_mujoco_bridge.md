# S6-01：MuJoCo 16 关节仿真与 S5 C++17 控制核心对接

状态：Implemented（仿真替身模型，非官方 LEAP URDF）。本任务不引入真实硬件运动。

## Context
现有 `simulation/run_leap_hand.py` 仅加载官方 URDF 并执行无执行器的自由仿真；`cpp/robot_core` 已有 16 关节 `ControlLoop`、`MotionControlCore`、`CommunicationWorker`、`StopCoordinator`。需要让 MuJoCo 的真实 qpos/qvel 驱动 C++ 控制并将位置指令送回 MuJoCo actuator。

## Goals / Non-goals
- 目标：16 路闭环、同一映射顺序、rad/deg 明确、100Hz 控制与 500Hz 物理步进、仿真时钟、错误停机、CI 自动验证。
- 非目标：ROS2 节点（S6-02）、官方 LEAP URDF 16 路执行器/真实机械动力学标定（S6 后续）、Windows 串口/STS3215、真实 100Hz OS deadline 证明。

## Architecture
```text
MuJoCo Python (qpos/qvel rad, 500Hz step)
   -- STEP t_us, 16q,16dq,16target (rad) -->
C++ s6_control_bridge (single deterministic sim-thread)
   SimulatedServoTransport.read -> CommunicationWorker.pollFeedback(t_sim)
   -> FeedbackSafetyMonitor -> ControlLoop.tick(t_sim)
   -> MotionControlCore (16x trajectory + position PD + limiter)
   -> CommandMailbox -> CommunicationWorker.processCommands(t_sim)
   -> StopCoordinator -> SimulatedServoTransport.startSend/finishSend
   <-- OK 16 position commands (rad) --
MuJoCo Python -> data.ctrl[actuator_index] -> mj_step x5
```
- 仅复用 `robot_core`，不修改它的安全门禁；桥接进程没有串口驱动。
- 采用进程内 stdin/stdout 文本协议，避免新 pybind/ROS2 编译依赖。
- C++ 内部使用角度，接口和 MuJoCo 使用弧度。时间戳为非负微秒，映射到合成 `steady_clock::time_point`。
- 模型为**16 个铰链+16 个位置执行器的 surrogate**，关节名从 `config/joint_mapping.json` 获取，并按逻辑 ID 0..15 映射；不将未标定的硬件零位/方向应用到仿真。
- 不调用 `RealtimeExecutor`，因为其线程等待使用物理 wall clock；S6-01 是固定步长、确定性仿真，不是实时线程性能测试。

## Interface / failure
- 启动返回 `READY 16 S6-01`。
- 输入 `STEP <time_us> <q0..q15> <v0..v15> <target0..target15>`，空格分隔，单位 rad / rad/s。
- 输出 `OK <cmd0..cmd15>`，单位 rad；错误输出 `STOP`，且进程保持故障锁存；`QUIT` 返回 `BYE`。
- 拒绝负时间、重复/回退时间、非有限值、超限状态、超长 dt、控制门禁失败；不会输出部分 16 路命令。
- 桥接进程是单生产者顺序调用，不允许多 Python 线程并发写同一 stdin。
- 物理执行器的安全性不由仿真模型证明；`STOP` 不是硬件断扭矩。

## Configuration
- `CONTROL_DT=10ms`，`PHYSICS_DT=2ms`；C++ 位置外环默认参数沿用 S5-07，仅作仿真起点。
- 仿真模型关节范围 ±1.3rad，位置 actuator `kp=12,kv=1` 是 surrogate 参数，不代表 STS3215。
- `target` 默认各手指侧摆 0，其余关节 0.35rad，演示控制命令随仿真反馈闭环变化。

## Tests / AC
1. `S6BridgeProtocolTest`：10 个合成 10ms tick、16 路输出、首次从实测零位起步、非直通目标、时间重复后 STOP 锁存。
2. `S6MuJoCoSmokeTest`（安装 mujoco/numpy 时）：创建 16 关节 16 actuator，实际运行 `mj_step`，验证非零命令和非零关节运动。
3. 全量 S1～S5 CTest 不退化；CI 安装 mujoco/numpy 后运行完整 CTest。
4. 没有硬件 IO；默认不允许任何真实舵机使能。

## Hardware dependencies / tradeoffs
- 无硬件依赖；S6-01 不使用第三方 URDF 子模块，因此 CI 不需要 STL 网格。
- 用子进程文本协议换取易测、跨语言与独立进程崩溃隔离；性能和日志能力不等同未来 ROS2 DDS 接口。
- S6-02 需处理 ROS2 clock/pause/reset、官方模型驱动器、跨线程消息和安全恢复；S7 才验证真实硬件。
