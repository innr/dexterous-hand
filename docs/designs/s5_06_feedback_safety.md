# S5-06：反馈超时监控与安全状态机联动

## 目标
在 S5-05 逐路反馈时间戳基础上，按 100Hz 控制线程时刻评估全部 16 路反馈。
不能把整轮通信仍在运行等同于反馈新鲜，也不能把禁止新 TX 误称为物理急停。

## 接口与职责
- `FeedbackSafetyMonitor`：控制线程独占，消费 `FeedbackSnapshot` 和
  `steady_clock` 时刻，输出 `FeedbackSafetyDecision`。
- `SafetyStateMachine`：复用 NORMAL/DEGRADED/SAFE_STOP 状态，不自动恢复。
- `RealtimeExecutor`：先执行安全评估，再调用 `ControlLoop::tick`；
  第一次硬异常立即调用 `StopCoordinator::requestStop()` 并清空
  `CommandMailbox`，确保后续新 TX 无法启动。
- `ControlLoop`：增加 `clearPendingCommands()`；控制目标更新不改变
  已锁存的停机代次。
- `RuntimeStats`：增加反馈安全状态、无效/过期/预警路数、连续硬异常次数、
  启动等待和运动门禁锁存状态。

## 默认阈值（可配置，**不是实测硬件阈值**）
| 配置 | 默认值 | 语义 |
| --- | --- | --- |
| startup_grace | 100ms | 初次未收到全部有效反馈时的等待窗口 |
| warning_age | 20ms | 超过此年龄进入预警区 |
| hard_timeout | 30ms | 超过此年龄为硬异常 |
| warning_ticks | 2 | 连续预警达到次数后进入 DEGRADED |
| stop_ticks | 3 | 连续硬异常达到次数后进入 SAFE_STOP |

无效标志、未来时间戳、NaN/Inf 也属于硬异常。年龄恰好等于阈值不算超时。
任何一路异常都会触发整手保护策略（第一版保守策略）。

## 状态与动作
1. **启动**：首次控制 tick 起计算 `startup_grace`；未填满有效反馈时
   `ControlLoop` 自身不发布命令。超时仍不完整则触发硬异常。
2. **NORMAL**：16 路有效且年龄未超出警戒区，正常下发。
3. **DEGRADED**：连续警戒年龄达到 `warning_ticks`，记录降级；
   反馈仍低于硬阈值时可以继续下发。状态不会自动回到 NORMAL。
4. **硬异常首个 tick**：立即锁存 `StopCoordinator`（代次增加）并清空邮箱；
   即使还未达到 `stop_ticks`，也不再启动新运动 TX。
5. **SAFE_STOP**：连续硬异常达到 `stop_ticks` 后锁存状态、退出工作循环；
   不会因后续恢复反馈自动重新启动。必须经上层人工授权及硬件确认后
   创建新运行器。
6. **短暂硬异常后恢复**：连续计数清零，但运动门禁仍锁存；
   防止一次 CRC/超时后的自动恢复绕过安全审查。

## 线程竞态与边界
- `StopCoordinator::beginSend` 和 `requestStop` 共用门禁互斥锁；
  停机请求完成之后，旧 generation 的命令不能再启动新的 TX。
- 已经启动的 TX 可能继续完成；本阶段**不能**保证物理电机立刻断扭矩。
- 通信线程的 `read` 必须有界超时，否则退出线程的 `join` 仍可能等待。
- 停机状态下运行器不支持自动重启；`recover` 必须人工授权并确认故障
  清除和在途 TX 已完成，不能仅凭反馈变正常。
- `feedback_safety.enabled` 默认为 true；S5-02 慢总线隔离测试显式关闭，
  以免把该测试的慢通信故意注入解释成故障。
- 本阶段只覆盖反馈年龄/有效性，不包含过温、电流尖峰、外部急停、
  真实扭矩切断或硬件看门狗。

## 验收
- 虚拟时钟测试：启动宽限、阈值边界、预警降级、连续硬异常停机、
  未来时间戳、单路失效、非有限数据和锁存恢复策略。
- 双线程测试：通信连续失败后 SAFE_STOP、门禁关闭、邮箱清空，
  后续伪造新命令仍不能启动 TX。
- 全量 CTest + GitHub Actions；真实 16 舵机安全响应需要 S7 HIL 验收。
