# C++17 control core (S1–S6)

`robot_core` 是 STATIC 库。S1–S3 教学 API 与 S4–S5 运行时同库；**S6 运动路径只使用运行时接口**。架构见 `docs/architecture/cpp_control_architecture.md`，契约见 `docs/designs/s7_00_contract_convergence.md`。

## Layout

```text
cpp/
├── include/robot/
│   ├── core/       RobotJoint.hpp, RobotHand.hpp          (teaching)
│   ├── bus/        IBus.hpp (teaching), IServoTransport.hpp,
│   │               Sts3215Protocol.hpp, Sts3215Transport.hpp,
│   │               VirtualServoBus.hpp, PosixHalfDuplexPort.hpp
│   ├── control/    RobotController.hpp (teaching),
│   │               MotionControlCore.hpp, PD, trajectory, limiter
│   ├── safety/     SafetyStateMachine.hpp,
│   │               StopCoordinator.hpp, FeedbackSafetyMonitor.hpp
│   ├── runtime/    S4Pipeline.hpp (ControlLoop, CommunicationWorker),
│   │               mailbox, feedback, executor, telemetry
│   └── contract/   CanonicalCommand.hpp (rad, session_id; not on the 100 Hz path)
├── src/            matching .cpp
├── examples/       robot_control_demo, s6_control_bridge,
│                   sts3215_readonly_probe, sts3215_bench, telemetry/tuning
└── tests/          S1–S6 CTest
```

`robot/robot_core.hpp` 与 `JointTrajectoryPlanner.h` 是 **兼容聚合头**，只覆盖教学层。新代码包含具体模块头。

## Build and test

```sh
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/cpp --parallel
ctest --test-dir build/cpp --output-on-failure
```

S6 官方 MJCF 测试需要仓库根下已下载 Menagerie：

```sh
python -m simulation.fetch_leap_menagerie
```

## What is implemented

- S1–S3：同步 `RobotController` + `IBus` / `MockBus`（教学；不在 S6 路径）。
- S4：控制/通信拆分、mailbox、反馈快照、`StopCoordinator`。
- S5：实时执行器、STS3215 协议与传输（`allow_motion` 默认 false）、分片反馈、反馈安全、位置外环 PD、遥测。
- S6：`s6_control_bridge` 文本协议（rad STEP/OK ↔ 内部 deg）。仿真植物在 Python。

## Limits

- 单位：运行时 **deg**；桥接 **rad**。不要把 `CanonicalCommand` 弧度夹具送进 S5 单测。
- `IBus::send` 不是运行时 TX 门。运行时唯一线性化点是 `StopCoordinator::beginSend`。
- `Sts3215Transport` 不得从 S6 桥构造。只读 probe 不会使能扭矩。
- `PosixHalfDuplexPort` 依赖 Linux TEMT；Windows COM 不存在。
- 虚拟测试通过不是硬件安全证明。
