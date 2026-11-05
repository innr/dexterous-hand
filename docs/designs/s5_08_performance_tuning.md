# S5-08：闭环性能评估与 PD 参数整定（离线）

## 目的与安全边界
本阶段不触发 STS3215 真实运动，不在线自动整定，不更改 S5-07 默认增益。
新增独立的**单关节阶跃响应分析器**和**确定性一阶虚拟执行器**，为未来
16 路 HIL 数据分析提供可复用计算工具。

S5-07 位置外环通过位置命令驱动 STS3215 内部位置环；
本阶段的虚拟模型 `dq/dt = clamp((q_cmd_delayed-q)/tau, ±v_max)`
**不是**真实 STS3215 识别模型，仿真排名不能直接部署。\n指令延迟为整周期排队；反馈延迟至少一个采样周期（离散仿真在周期起点使用上一时刻的状态）。

## 数据格式
单关节一次阶跃，一行一采样，CSV 表头必须为：
```csv
time_s,target_deg,reference_deg,command_deg,measured_deg,feedback_age_ms
0.0,20,0,0,0,20
0.01,20,0.01,0.001,0.0,20
```
- `time_s`：相对于阶跃时刻的单调时间，单位秒，严格递增。
- `target_deg`：整段必须恒定；若要分析多个阶跃，先按阶跃区间分割。
- `reference_deg`：规划器输出位置；`command_deg`：最终下发的位置指令。
- `measured_deg`：传感器测得的关节角度；`feedback_age_ms`：控制时刻
  减去反馈采样时刻，不能用日志写入时间代替。
- 必须含施加阶跃前的基线测量（第一行），且目标与基线差至少 0.5°。
- 当前 CSV 是**单关节**格式；16 路应分别导出分析，不能把异步反馈
  误认为同时采样。

## 指标定义
- `RMS reference error`：`sqrt(∫(q_ref-q_meas)²dt/T)`，梯形积分。
- `RMS target error`：`sqrt(∫(q_target-q_meas)²dt/T)`。
- `IAE target`：`∫|q_target-q_meas|dt`。
- `overshoot`：沿阶跃方向越过最终目标的最大角度，除以阶跃幅度为百分比。
- `rise time`：实际角度第一次跨越阶跃幅度 10% 和 90% 的时间差，线性插值。
- `settling time`：从阶跃时刻到**最后一次**超出 ±max(2%阶跃幅度,0.2°)
  的下一个采样时刻；后续必须持续在带内且末尾至少有 0.2s 的观察。
  未稳定输出 `not_reached`，不可写为 0。
- `steady-state error`：最后 20% 时间窗内 `target-measured` 的样本平均，
  是观察窗估计值，不等于严格意义上的无限时间稳态误差。
- `feedback_age_ms`：最大值与最近秩 P99，非有限/负年龄拒绝。

## 离线运行
```bash
cmake -S cpp -B build/cpp
cmake --build build/cpp -j
ctest --test-dir build/cpp --output-on-failure
./build/cpp/s5_virtual_tuning best_trace.csv
./build/cpp/s5_response_analyzer best_trace.csv
```
虚拟整定固定扫描 12 组 `(Kp,Kd)`，以
`IAE + 5*overshoot_deg + 3*abs(steady_state_error_deg) + (未稳定?10:0)`
排序。评分是示例多目标权重，不是稳定性证明。每组模型相同，便于复现。
`best_trace.csv` 可被分析器重新读取。

## 验收
- 手工构造阶跃：10%-90% 上升时间、超调、稳定时间、积分误差精确校验。
- 未稳定、不合法时间戳、变化目标、NaN 等输入拒绝或显式无结果。
- 同样虚拟模型和增益运行两次得完全一致指标，排序可复现。
- 全量 CTest 及 GitHub Actions 通过。

## S5-09 / S7 后续
- 将真实控制 tick、反馈采样时间戳、位置、轨迹参考、最终命令记录为
  **非阻塞环形日志**，离线导出上述 CSV；日志不能阻塞 100Hz 线程。
- HIL 先单关节、空载、低速、小幅度、限位远离机械边界，再扩展 16 路。
- 真实整定必须加上最大电流、温度、通信超时、过冲保护和硬件急停。
- 观察数据年龄、控制线程 P99 时延、接触时的机械弹性和振荡。
