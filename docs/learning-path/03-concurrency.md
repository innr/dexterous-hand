# S3 — C++17 多线程、同步与资源生命周期

**入口**：S2 PASSED。**目标**：能识别并修复数据竞争、死锁、TOCTOU、线程退出和所有权漏洞。

## 必会知识
`std::thread`/join、RAII 线程生命周期、`mutex`/`lock_guard`、`condition_variable`、`atomic` 和 acquire/release、happens-before、双缓冲/快照、generation、CAS、优先级反转与有界等待。明确 C++ atomic 不等于实时保证。

## 项目任务
实现 `LatestValue<FeedbackSnapshot>`（每个关节独立时间戳）、`LatestCommand`、`WorkerLifecycle`；生产者模拟 16 关节轮询，消费者以 100 Hz 读取。设计不在控制线程执行 I/O 的消息链路。使用可控 barrier / 条件变量制造交错；`join()` 必须先于资源释放。

## 验收测试
- S3-T01：生产者/消费者并发 100k 次，无撕裂快照和无效 sequence。
- S3-T02：使用同步点覆盖“检查后被抢占再使用”TOCTOU；修复后重复通过。
- S3-T03：测试 stop / join / destructor 与 worker 退出顺序；重复 start/stop 1000 次。
- S3-T04：模拟读包 100 ms 卡顿：控制线程不调用/等待该 I/O，明确反馈陈旧策略。
- S3-T05：TSAN（可用平台）无数据竞争；用故意竞争的负样例验证工具能报错。
- S3-T06：证明 atomic flag 不能取消阻塞驱动调用，并描述解除阻塞/有界超时机制。
- S3-T07：单独解释死锁与优先级反转场景，给出资源锁顺序及测试结果。

## 通过标准
自动化断言全过，包含强制线程交错的确定性测试、压力统计与退出测试；TSAN 或替代平台说明；本人能够解释每个跨线程数据的 owner、同步边界和退出语义。
