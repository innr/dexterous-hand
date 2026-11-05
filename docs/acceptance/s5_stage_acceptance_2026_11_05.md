# S5 阶段总验收 — 2026-11-05

> 验收基线：`main@2f67df5a5aa7f4d4d59cb11b5716e5d1bbc81c61`。
> 结论：**软件阶段条件通过（S6 仿真集成可启动）；真实硬件带载闭环未通过，禁止以本报告替代 HIL 验收。**
> 本次为 GitHub 源码/测试/CI 记录审查；未在真实 STS3215 上执行测试，未进行独立 Linux/ARM 本地实测。

## 1. 证据与验收口径

- `main` 的 [GitHub Actions run 41984313908](https://github.com/innr/dexterous-hand/actions/runs/41984313908)：
  Ubuntu、CMake Debug、构建和 CTest **17/17 通过**；这不是代码覆盖率 100%，也不证明实时 deadline。
- S5-01～S5-09 已分别合并 [PR #20](https://github.com/innr/dexterous-hand/pull/20) 至
  [PR #28](https://github.com/innr/dexterous-hand/pull/28)。
- 检查了 `cpp/include/robot`、`cpp/src`、`cpp/tests`、`cpp/CMakeLists.txt`、
  `.github/workflows/cpp-s4.yml`、`docs/designs/s5_*.md` 与 `scripts/plot_s5_telemetry.py`。
- 本报告的“软件通过”表示现有模拟/单元/双线程测试覆盖的行为通过；
  **不代表真实硬件、所有边界竞态、RT 性能或机械安全已经验收。**

## 2. 模块接口与测试追溯

| 任务 | 主要接口 | 对应验收 | 结论 |
| --- | --- | --- | --- |
| S5-01 | `StopCoordinator::beginSend/requestStop/recover` | `S5StopCoordinatorTest`：代次、在途许可、停机竞态 | 软件通过 |
| S5-02 | `RealtimeExecutor::start/stop/stats` | `S5RealtimeExecutorTest`：线程生命周期、慢通信隔离、时间戳 | 软件通过 |
| S5-03 | `Sts3215Protocol`、`Sts3215Transport`、`IHalfDuplexPort` | `S5Sts3215ProtocolTest`：黄金向量、CRC/ID/超时、FakePort | 模拟通过，实机待测 |
| S5-04 | `ReadOnlyBench::run/writeCsv` | `S5ReadOnlyBenchTest`：16 路慢轮询、失败、反馈年龄 | 模拟通过，实机待测 |
| S5-05 | `FeedbackBuffer::updateJoint`、`pollFeedbackSliceRealTime` | `S5FeedbackSlicingTest`：逐路可见、4 路分片、失败隔离 | 软件通过 |
| S5-06 | `FeedbackSafetyMonitor::evaluate`、`StopCoordinator` | `S5FeedbackSafetyTest`：启动宽限、过期/无效、锁存停机 | 软件通过，物理安全待测 |
| S5-07 | `MotionControlCore::step`、`ControlLoop::tick` | `S5MotionControlTest`：16 路限速/限加速度、PD、虚拟总线 | 软件通过，实机待测 |
| S5-08 | `analyzeStepResponse`、`rankVirtualCandidates` | `S5PerformanceMetricsTest`、`S5VirtualTuningCliTest`、`S5ResponseAnalyzerCliTest` | 离线通过 |
| S5-09 | `ControlTelemetryRing::tryPush/tryPop`、CSV/HTML | `S5TelemetryTest`、`S5TelemetryDemoTest`、`S5TelemetryPlotTest` | 虚拟总线端到端通过 |

其余 `RobotCoreTest`、`S3TrajectoryPlannerTest`、`S4RuntimeTest`、
`S4IntegrationTest` 属于前阶段回归，包含在总计 17 个 CTest 中。

## 3. 接口闭环审查

```text
setTargets (application thread)
    -> ControlLoop::tick (100Hz control thread)
    -> StateEstimator (16路 feedback snapshot + age)
    -> MotionControlCore (trajectory + position PD + output limiter)
    -> CommandMailbox (latest only, sequence/generation/time)
    -> CommunicationWorker::processCommands (communication thread)
    -> StopCoordinator::beginSend(generation)
    -> IServoTransport::startSend/finishSend
    -> Sts3215Transport or VirtualServoBus
    -> CommunicationWorker::pollFeedbackSliceRealTime
    -> FeedbackBuffer::updateJoint (per-servo timestamp)
    -> FeedbackSafetyMonitor (before next control tick)
    -> ControlTelemetryRing -> CSV -> offline plot
```

- 生产者/消费者线程划分合理：串口仅由通信线程访问；16 路目标通过 mutex
  快照更新；命令邮箱覆盖旧帧；SPSC 遥测满时丢日志，不等待消费者。
- `MotionControlCore` 输出为**位置命令**，不是转矩或电流命令；PD 是外环，
  不可把 STS3215 内部伺服环当成已经验证的模型。
- 反馈为每关节独立采样时间；安全监控与控制循环分别读取快照，
  不是原子化的“安全检查+控制计算”联合快照，需保留代次用于追溯。
- `StopCoordinator` 只阻止停机完成之后的新 TX 启动；
  已在途 TX 可完成；不具备物理断电/断扭矩能力。

## 4. 风险登记与关闭条件

| 编号 | 级别 | 证据 / 问题 | 关闭条件 |
| --- | --- | --- | --- |
| H-01 | **P0：硬件运动阻断** | `requestStop` 只禁止新位置写入，没有经验证的硬件急停、断扭矩、过温/过流链路 | 外部 E-stop/电源或扭矩切断，故障锁存，实测停止响应与故障恢复 |
| H-02 | **P0：硬件运动阻断** | 无真实 16 路 STS3215 的 TC/自动半双工方向/ID/零位/方向/限位实测；Linux 串口依赖 TEMT ioctl，Windows COM 不支持 | 先只读扫描、逐路标定与限位、校验适配器、记录实测结果 |
| H-03 | **P0：硬件运动阻断** | 16 路命令+反馈实际总线时间、反馈 P99/最大年龄、100Hz deadline 和最坏阻塞未实测 | 使用 S5-04 只读基准；在目标硬件验证分片读取、控制 deadline、故障注入 |
| S-01 | **P1：修复后才可实机运动** | `CommunicationWorker::processCommands(now)` 只在一整份 16 路命令发送前检查 `created_at` 年龄；中途可能继续发送过期目标 | 每路 TX 前重新读取单调时间并验证命令 TTL、代次和停机；慢总线回归 |
| S-02 | **P1：时序指标偏差** | `RealtimeExecutor::controlMain` 在 `ControlTelemetryRing::tryPush` 与 `stats_mutex_` 更新前记录 `finished`；`control_duration_us` 不含这些开销 | 区分算法耗时与完整 tick 开销，记录完整执行/发布后耗时；日志开关 A/B P99 测量 |
| S-03 | **P1：分析链路未闭合** | S5-09 的 16 路长表 CSV 无法直接输入 S5-08 单关节恒定阶跃分析器；缺少可靠分段/转换工具 | 增加按关节/阶跃分段转换，拒绝丢样和未发布记录，端到端测试 |
| S-04 | **P1：并发/性能证据不足** | CI 只有 Ubuntu Debug 的 17 个 CTest；没有 TSAN/ASAN/UBSAN、长时压力、ARM SoC、Windows、真实 P99/WCET | 增加 Sanitizer CI、压力测试和目标 SoC 实测；Release 下测试避免 assert 副作用 |
| S-05 | **P1：实机增益风险** | 速度来自编码器差分，无滤波/噪声量化验证；默认 PD/轨迹参数来自虚拟模型 | 单关节低速空载阶跃、延迟/噪声辨识、限幅与安全守护后再整定 |
| S-06 | **P2：观测语义** | S5-09 `feedback_valid` 仅表示读数有效和时间非未来，不等于未超时；未发布帧命令字段为占位值 | 可视化按有效性+年龄+发布状态筛选，导出 dropped 计数与安全事件 |

### 风险分级

- **P0**：阻断任何带载/使能运动的硬件验收；只读诊断可在电气安全条件下开展。
- **P1**：软件可进入 S6，但必须在进入真实闭环运动前修复或完成验证。
- **P2**：不阻断仿真，需在后续观测和验收中跟踪。

## 5. 阶段门禁决定

### S6（MuJoCo + ROS2）：**GO，带条件**

1. 只在 `VirtualServoBus` / MuJoCo 仿真环境中集成，禁止默认启用物理运动。
2. 保持 `IServoTransport` 抽象，ROS2 目标输入必须经过 `ControlLoop` 的目标校验、
   轨迹/限速和安全门禁，不能绕过核心直接写物理总线。
3. 仿真时间与 `steady_clock` 时间语义需显式适配，避免 ROS2 clock reset、
   pause/step 导致控制 `dt` 错误。
4. 针对 S-01～S-04 补测试；对历史开放 PR #1/#4/#13 等进行基线差异审查，
   **不得直接把旧分支视为已通过 S5 架构验收**。

### 真实 STS3215 台架：**NO-GO for actuated closed loop；只读准备 GO**

1. 首先确认适配器类型、1Mbps/半双工方向、TX TEMT 支持、16 路实际 ID、
   外部电源/硬件急停、每关节机械限位。
2. 先执行只读 probe/bench；检查通信成功率、每路/整轮耗时、P95/P99、
   控制线程时序、每路数据年龄、连续故障。
3. 再修复 S-01、S-02，并验证 H-01～H-03，建立明确的**单关节低速空载**
   使能授权清单；未通过前不允许 16 路带载运动。
4. 全部验证后逐步单关节 → 小组 → 16 路；不能用仿真最优 PD 增益直接上硬件。

## 6. 推荐下一步

**主线进入 S6-01：MuJoCo/ROS2 与 S5 控制核心接口设计及虚拟总线闭环验收。**
同时单独开一项 S5 遗留风险修复 PR，先处理 S-01（逐路命令 TTL）、
S-02（时序观测准确性），再开展硬件只读台架准备。

## 7. 复验命令

```bash
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/cpp --parallel
ctest --test-dir build/cpp --output-on-failure
./build/cpp/s5_telemetry_demo build/cpp/s5_telemetry.csv
python3 scripts/plot_s5_telemetry.py build/cpp/s5_telemetry.csv --joint 0 --output build/cpp/joint0.html
```

本报告不改变任何实时控制或舵机驱动行为。风险关闭必须附新的自动测试、
实测数据或复审记录，不得仅将勾选项改为“通过”。
