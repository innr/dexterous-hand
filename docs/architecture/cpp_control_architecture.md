# LEAP dexterous hand — C++ control architecture

本文描述 **已实现** 的 C++17 分层，不是 S1–S3 教学草图。系统图与契约见 `docs/architecture/system_architecture.md` 与 `docs/designs/s7_00_contract_convergence.md`。

## Dependency flow

```text
s6_control_bridge / examples / tests
         |
         v
runtime/ControlLoop  --->  control/MotionControlCore
         |                         |
         |                         +--> JointTrajectoryPlanner / PD / AccelerationLimiter
         v
runtime/CommandMailbox
         |
         v
runtime/CommunicationWorker
         |
         v
safety/StopCoordinator          safety/FeedbackSafetyMonitor
         |                                 |
         |                                 +--> SafetyStateMachine (internal)
         v
bus/IServoTransport
     /            \
    v              v
VirtualServoBus   Sts3215Transport  (allow_motion=false)
                         |
                         v
                  PosixHalfDuplexPort (Linux TEMT)
```

教学路径（**不在 S6 上**）仍链入同一 `robot_core` STATIC 库：

```text
RobotController --> RobotHand / RobotJoint
                --> IBus / MockBus
```

新代码不要走 `IBus::send`，也不要 include `robot/robot_core.hpp` 作为运行时入口。

## Layout

```text
cpp/include/robot/
  core/       RobotJoint, RobotHand          (S1–S3 teaching)
  bus/        IBus, IServoTransport, STS3215, VirtualServoBus
  control/    RobotController (teaching), MotionControlCore, PD, trajectory
  safety/     SafetyStateMachine, StopCoordinator, FeedbackSafetyMonitor
  runtime/    ControlLoop, CommunicationWorker, mailbox, feedback, executor
cpp/src/      matching translation units
cpp/examples/ robot_control_demo, s6_control_bridge, sts3215_* probes
cpp/tests/    S1–S6 CTest
```

## Responsibilities

| Layer | Owns | Does not own |
|---|---|---|
| teaching core | 16 关节教学模型、同步下发 | 实时调度、串口 |
| runtime | 100 Hz tick、mailbox、分片反馈 | ROS2、MuJoCo |
| MotionControlCore | 轨迹 + 位置 PD + 限速/限加速度（**deg**） | 协议、rad 换算 |
| StopCoordinator | S4–S5 路径上唯一 `startSend` 线性化 | 物理断电 |
| FeedbackSafetyMonitor | 反馈年龄；经 `requestStop` 作用到 TX 门 | 会话 epoch |
| IServoTransport | `startSend` / `finishSend` / `read` | 运动规划 |
| Sts3215Transport | tick↔deg；`allow_motion` 默认 false | 标定策略的操作员授权 |
| s6_control_bridge | 文本协议；S6 路径上唯一 rad↔deg | 真实串口 |

## Units

C++ 运动核心、mailbox、transport、S5 遥测保持 **deg**。  
S6 桥 STEP/OK 为 **rad**。不要在 `MotionControlCore` 内做 rad 换算。  
S4 直通（未 `enableMotionControl`）仍硬限位 `±90 deg`，是遗留兼容，不是第三套单位政策。S6 桥必须 `enableMotionControl`。

## Safety limits

- `requestStop` 阻止新 TX；已启动的 UART 可能发完。不是物理急停。
- 16 路顺序发送不是原子事务。
- `SafetyStateMachine` 与 `StopCoordinator` **不合并**；前者在反馈监视器内部。
- 虚拟总线通过 ≠ 硬件安全。H-01/H-02/H-03 仍开放。

## Build and test

```sh
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/cpp --parallel
ctest --test-dir build/cpp --output-on-failure
```

S6 官方模型测试需要先 `python -m simulation.fetch_leap_menagerie`。Windows 上不要调用 `PosixHalfDuplexPort` 作为验收。
