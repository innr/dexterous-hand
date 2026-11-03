# S4 实时控制架构设计与验收

## 实现范围
- 100 Hz 周期调度与 Deadline 监控，支持注入虚拟时间。
- 16 路反馈完整快照，逐路时间戳与有效位；线程安全发布。
- 最新值命令邮箱（Generation、序列号、生成时刻），无 FIFO 堆积。
- 状态估计：限时匀速预测，过期或未来反馈拒绝。
- StopCoordinator：许可申请与停机同锁，commitStart 二次校验；在途同步发送 RAII 跟踪。
- ControlLoop：读取反馈，检查有效性，发布限位位置目标。
- CommunicationWorker：单线程独占同步总线，逐路发送与轮询。
- VirtualServoBus：16 路模拟反馈、单路故障注入；CTest 自动化验证。

## 线程约束
ControlLoop::tick 与 CommunicationWorker 的方法由应用层调度；当前没有真正启动 100Hz 线程。
CommunicationWorker::processCommands 与 pollFeedback 必须由同一通信线程调用。

## 安全限制
- 本阶段的 send 为同步接口；真实 DMA/异步 UART 必须将 SendPermit 生命周期延长至 UART TC。
- requestStop 阻止新许可；已开始的发送可能继续，不能视为物理急停。
- 16 路顺序发送并非原子事务，途中失败可能产生部分关节更新。
- 当前仅检查整批发送开始时的命令年龄，真实硬件必须预算整批通信与舵机生效延迟。
- 虚拟反馈使用调用者时间；真实驱动应逐路记录实际采样时间。
- 当前 ControlLoop 发布限位位置目标，不是完整 PD 或接触控制器。
- 原有 SafetyStateMachine 尚未与 StopCoordinator 统一为原子状态机。
- 没有真实 STS3215 串口驱动、硬件急停、接触力限制或硬件在环测试。
- 不得将 PC 虚拟测试通过解释为真实硬件安全验证。

## 构建与验收
```sh
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/cpp --parallel
ctest --test-dir build/cpp --output-on-failure
```
