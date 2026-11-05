# S5 遗留修复：完整控制周期耗时统计

## 问题
旧 `RealtimeExecutor::controlMain` 在遥测环入队和 `RuntimeStats` 更新之前取 `finished`，导致 `deadline_misses` 和 `max_control_duration_us` 低估完整周期负担。

## 修改
- 保留 `ControlTelemetryFrame::control_duration_us` 和 `RuntimeStats::max_control_duration_us` 为**控制计算阶段**耗时，避免破坏现有 CSV 语义。
- 新增 `RuntimeStats::max_full_tick_duration_us`，在遥测入队、统计更新后采样；以该完成时间判定 `deadline_misses` 和推进 `PeriodicScheduler`。
- 局限：解锁 `stats_mutex_` 和调度器自身 bookkeeping 的微小开销不计入；这仍非硬实时 WCET 证明。要在目标 SoC 实测 P99/最大值。
- 测试：慢总线和遥测运行器断言完整周期耗时 >= 算法耗时；全部既有 CTest 回归。
