# S5 遗留修复：逐关节命令 TTL

- 问题：旧实现只在进入 `processCommands(now)` 时检查命令年龄；16 路慢总线可能发送后半段已过期的目标。
- 修复：每次 `startSend` 前重新检查 TTL，过期即停止本批剩余关节发送。为支持 S4 测试和 S6 虚拟时钟，采用调用入口的 `now` 加上本次调用经过的 `steady_clock` 时间；不直接比较不同 epoch。
- 限制：此修复保证**批次开始和每一路 TX 启动前**的 TTL，不取消已开始的 TX；若物理 UART 超时，应由 `finishSend` 及 StopCoordinator 处理。
- 验收：模拟首路 TX 12ms、命令 TTL 5ms，断言仅发第 1 路；预先过期时零发送；保留所有 S4/S5 回归。
