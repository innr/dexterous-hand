# LEAP 灵巧手：七阶段学习与能力验收路线（v1.0）

> 目标：从有嵌入式 C / Linux / FreeRTOS 经验出发，达到**能独立设计、实现、测试、分析故障**的 C++17 机器人控制软件工程能力。
> 本路线与根目录 README 的硬件准备、仿真和遥操作里程碑并行，不替换它们。
> 原则：**讨论过 ≠ 掌握；示例跑通 ≠ 独立完成；单元测试通过 ≠ 真实硬件安全。**

## 阶段总览

| 阶段 | 主线 | 必须交付 | 验收 |
|---|---|---|---|
| [S1](01-modern-cpp.md) | C++17 基础 | 无全局变量的 16 关节模型库 | 单测 + 独立编码 + 讲解 |
| [S2](02-cpp-engineering.md) | 工程化 | CMake 库、测试、CI、错误模型 | clean build + CI + sanitizer |
| [S3](03-concurrency.md) | 多线程与并发 | 快照、命令缓冲、线程生命周期 | 强制竞态 + TSAN + 压测 |
| [S4](04-control-core.md) | 机器人控制软件架构 | Control Core v0.1 + SafetySupervisor | 故障注入 + 状态机与并发测试 |
| [S5](05-robot-control.md) | 运动学与控制 | PD / 轨迹 / 状态估计 | 数值验证 + 稳定性实验 |
| [S6](06-simulation-ros2.md) | MuJoCo + ROS2 | 16 关节仿真与节点集成 | 回放、接口、端到端测试 |
| [S7](07-real-hardware.md) | STS3215 真机 | 受限条件下的软硬件闭环 | 实测时序、安全与恢复记录 |

## 能力状态：四种，不用百分比代替

- `NOT_STARTED`：未开始系统训练。
- `LEARNING`：学习/讨论/有示例，但独立交付或证据不完整。
- `READY_FOR_REVIEW`：实现、报告及测试证据齐备，待复核。
- `PASSED`：所有阻断项通过，附可复现的 commit SHA、测试日志和审查记录。

**当前默认状态**：S1 `LEARNING`；S2–S4 有概念预习与示例，但**尚未正式验收**；S5–S7 尚未正式验收。不要把此前的问答正确率视为阶段通过。

## 统一验收流程

1. 先冻结本阶段接口、用例与风险（Design Doc / issue）。
2. 本人独立实现关键代码、说明设计权衡；可用 Codex 辅助，但须识别并修复至少一个引入的缺陷。
3. 提交最小可运行工程、单元/集成测试、失败注入脚本和 README 运行命令。
4. 记录环境（OS、编译器、CMake、依赖版本、硬件修订）、commit SHA、时间戳、原始日志。
5. 执行自动化测试与人工答辩；**任何阻断测试失败不得标为 PASSED**。
6. 一阶段一验收 PR；未通过则记录原因、修复后重新运行。

统一证据模板见 [验收记录模板](ACCEPTANCE_TEMPLATE.md)，进度见 [验收看板](PROGRESS.md)。

## 共通质量门槛

- 源码 C++17；无未定义行为、未处理的资源泄漏或已知数据竞争。
- 单元测试稳定、正常及错误路径覆盖；禁用测试或仅演示打印结果不算通过。
- CI 通过 Debug/Release；可执行时运行 ASan/UBSan，S3 及以后额外运行 TSAN（平台不支持需说明替代方案）。
- 明确失败/超时/恢复语义；实时指标必须报告 P50/P99/max、样本量和测试环境，**不得把 PC 仿真结果当作硬件承诺**。
- 安全相关模块区分 stop requested / admission closed / TX drained / actuator safe；真机安全需独立验证。
- 后续阶段以之前阶段 `PASSED` 为正式准入前提；允许预习，但不记为完成。

## 现有工作如何归档

此前讨论的 LEAP Control Core v0.1、StopGate、SafetySupervisor、MockBus 思路属于 **S4 架构预习**，涉及 S2/S3；此文档**不宣称**这些模块已在仓库集成或完成验收。可在 S4 开始时把此前草案迁入正式 Design Doc，并由 PR 审查后冻结。

## Git 工作流

`stage/N-design -> stage/N-implementation -> stage/N-acceptance`（每次提交独立 PR，允许按模块细分），原则上一任务一 PR，验收 PR 提供证据索引。所有验收结论按实际运行结果填写。
