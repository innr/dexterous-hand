# S5-01：发送门禁与停机线性化点

## 问题
原实现先 `commitStart()`，随后才调用阻塞 `bus.send()`。
两者之间停机可能生效，但总线仍可开始新的 TX。

## 冻结的接口与语义
- `IServoTransport::startSend()`：有界、非阻塞、无异常，返回 true 表示物理 TX 已启动；不能仅将命令放入软件队列。
- `StopCoordinator::beginSend(generation, callback)`：在同一把互斥锁下检查状态、执行短小的 TX 启动、计数在途事务，随后返回 RAII 许可。
- `requestStop()`：在同一把锁下锁存停止并推进 generation。返回后不会有新的 TX 启动被授权。
- `finishSend()`：在锁外等待 UART TC 或确认硬件已安全取消，完成后释放 RAII 许可。
- `recover()`：必须获得操作员授权、故障清除且 `inFlight()==0`。

## 限制
- 若真实驱动的 startSend 只是排队、可能阻塞或在返回后才触发 UART，线性化保证不成立。
- stop 不能取消已经启动的 TX，不能等价于硬件急停或切断电源。
- 真实 DMA 驱动须严格区分 DMA 完成和 UART TC，并对超时、断线、未知完成状态保持故障锁存。
- 当前仍为虚拟总线，没有真实 STS3215 物理验证。
- 不能在 startSend 回调内重入 StopCoordinator，以免死锁。

## 验收
- 失败启动不增加在途计数。
- stop 后禁止启动、旧 generation 失效。
- 在途未结束时拒绝恢复，完成后允许授权恢复。
- 并发启动与 stop 互斥；stop 返回后禁止新启动。
- S1～S4 回归测试全部通过。
