# S5-09：100Hz 控制数据采集、环形日志与闭环可视化

## 目标
将 S5-07 的 16 关节 PD 外环轨迹与 S5-08 的离线分析工具接上真实控制周期
的观测数据，而不是依靠虚拟模型凭空推测闭环效果。

## 数据流
```
RealtimeExecutor 控制线程 (100Hz)
  -> ControlLoop::tick(now, optional_trace)
  -> ControlTelemetryRing::tryPush (SPSC 固定容量, 非阻塞)
  -> 外部消费者 tryPop
  -> writeTelemetryCsvHeader / writeTelemetryCsvFrame (CSV长表)
  -> scripts/plot_s5_telemetry.py (离线 HTML/SVG)
```

## 数据字段与语义
- `tick`：本次运行单调递增的控制周期编号。
- `time_us`：steady_clock 自 epoch 的微秒，仅同一运行内可比较。
- `joint`：0..15 的逻辑关节索引，不是 STS3215 实际舵机 ID。
- `target_deg`：上层应用目标；`reference_deg`：规划器参考位置；
  `command_deg`：最终位置指令；`measured_deg`：控制计算所用的原始反馈位置。
- `measured_velocity_deg_s`：反馈速度；`feedback_age_us`：
  控制线程读取时刻减去该路反馈时间戳，`-1` 表示无效/未来时间。
- `feedback_valid`：快照中的通信有效标志与时间顺序检查；
  **不能单凭此字段判断未过期**，必须同时检查反馈年龄及安全状态。
- `status`：0=PUBLISHED，1=STOP_GATE，2=INVALID_FEEDBACK，
  3=NO_TARGET，4=MOTION_REJECTED，5=SAFETY_BLOCKED。
- `published`：仅 PUBLISHED 为 1；其他行的 target/reference/command
  可能为零值占位，不得用于闭环误差计算。
- `safety_state`：SafetyState 底层枚举值；需结合故障状态分析。
- `start_lateness_us`、`control_duration_us`：调度迟到和控制周期执行耗时。
- `feedback_sequence`：读取到的反馈快照代次；用于诊断反馈更新频率。

## 并发与性能约束
- 固定大小 SPSC 环，`std::vector` 仅在创建时分配；`tryPush` 不分配、
  不加互斥锁、不写文件、不等待消费者。
- 满时丢弃新日志并增加 `dropped()`；不得丢弃控制命令或覆盖
  消费者正在读取的槽。丢日志不应触发控制停机。
- **必须只有一个控制生产者、一个外部消费者**；第二个消费者或同时
  导出和消费是错误用法。调用方负责保证 ring 比运行器存活更久。
- 不提供内建日志后台线程，使用者可在诊断线程中持续 `tryPop` 并写入
  文件，或在 `executor.stop()` 之后导出。
- `ControlLoop` 仍单独读取反馈快照；安全监控在该周期更早读取一次，
  极端情况下两者代次可能不同。CSV 的反馈记录对应实际控制 tick 使用的
  快照；安全阻止控制时使用安全监控快照。
- 本阶段没有硬实时 WCET 证明；必须在目标 Linux SoC 测量打开日志前后的
  P99/最大控制周期耗时和调度迟到。

## 操作
```bash
cmake -S cpp -B build/cpp
cmake --build build/cpp -j
ctest --test-dir build/cpp --output-on-failure
./build/cpp/s5_telemetry_demo build/cpp/s5_telemetry.csv
python3 scripts/plot_s5_telemetry.py build/cpp/s5_telemetry.csv --joint 0 --output build/cpp/joint0.html
```
浏览器打开 `build/cpp/joint0.html`，可看到目标/参考/指令/实际角度、
反馈年龄、控制耗时。无需第三方 Python 包。

**注意：** `s5_telemetry_demo` 仅使用 `VirtualServoBus`，不会访问串口，
不会给真实舵机使能或发送运动命令。图中数据是虚拟执行器数据。
`s5_response_analyzer` 的输入是**单关节恒定阶跃**六列格式；
S5-09 CSV 是 16 关节长表，不能直接作为 S5-08 的输入。真实阶跃分析需
先按关节和稳定目标区间过滤、校验完整性后再转换。

## 验收与限制
- 环形缓冲：满队列丢弃计数、FIFO、wrap-around、并发生产消费。
- 双线程 100Hz 虚拟总线：真实 tick 生成记录，目标/指令/反馈字段及 CSV
  行数正确，退出后可安全导出。
- CSV demo 端到端：CTest 检查程序生成日志。
- 离线图表：只画有效反馈，未发布命令不画指令曲线；无硬件 IO。
- 尚未实测 16 路 STS3215；真实反馈年龄、控制抖动和串口长尾延迟需要
  后续 HIL 台架测试。
