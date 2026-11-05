// S5-02：双线程生命周期、慢通信与逐路时间戳回归测试。
#include "robot/runtime/RealtimeExecutor.hpp"
#include "robot/bus/VirtualServoBus.hpp"
#include "robot/runtime/PeriodicScheduler.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace {
// 16 路每次读取延迟 2 ms：完整轮询约 32 ms，明显超过 10 ms 控制周期。
class SlowTransport final : public robot::IServoTransport {
public:
    bool startSend(std::size_t, float) noexcept override { return true; }
    bool finishSend() noexcept override { return true; }
    bool read(std::size_t, float& p, float& v) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        p = 0.0f;
        v = 0.0f;
        return true;
    }
};
} // namespace

int main() {
    using namespace robot;
    using namespace std::chrono;

    const ServoClock::time_point origin{};
    PeriodicScheduler scheduler(milliseconds(10), origin + milliseconds(10));
    assert(scheduler.completeCycle(origin + milliseconds(11)) == origin + milliseconds(20));
    assert(scheduler.skippedCycles() == 0);
    assert(scheduler.completeCycle(origin + milliseconds(45)) == origin + milliseconds(50));
    assert(scheduler.skippedCycles() == 2); // 30ms、40ms 两个计划时刻被跳过。

    FeedbackBuffer feedback;
    CommandMailbox mailbox;
    StopCoordinator stop;
    SlowTransport bus;
    ControlLoop control(feedback, mailbox, stop, milliseconds(500));
    CommunicationWorker communication(bus, feedback, mailbox, stop, milliseconds(100));
    RealtimeExecutor::Config config;
    config.control_period = milliseconds(5);
    config.communication_pause = milliseconds(1);
    // 本测试专门验证慢总线下双线程独立性；安全超时由 S5-06 单独测试。
    config.feedback_safety.enabled = false;
    RealtimeExecutor executor(control, communication, feedback, stop, config);
    assert(executor.start());
    assert(!executor.start()); // 禁止重复启动。

    std::array<float, kServoCount> targets{};
    for (int n = 0; n < 40; ++n) {
        targets.fill(static_cast<float>(n % 40));
        control.setTargets(targets); // 与控制线程并发，验证目标更新不发生数据竞争。
        std::this_thread::sleep_for(milliseconds(3));
    }
    std::this_thread::sleep_for(milliseconds(80));
    const auto before_stop = executor.stats();
    assert(before_stop.control_ticks >= 10);
    assert(before_stop.communication_polls >= 2);
    assert(before_stop.feedback_batches >= before_stop.communication_polls * 4);
    assert(before_stop.feedback_reads == before_stop.feedback_batches * config.feedback_batch_size);
    assert(before_stop.control_ticks > before_stop.communication_polls * 2);
    assert(before_stop.max_start_lateness_us >= 0);
    assert(before_stop.max_control_duration_us >= 0);
    assert(before_stop.invalid_feedback_joints <= kServoCount);
    assert(executor.running());

    executor.stop();
    executor.stop(); // 停机幂等。
    assert(!executor.running());
    assert(!stop.commandsAllowed());
    assert(stop.inFlight() == 0);
    assert(!executor.start()); // 停机后不隐式恢复旧命令。

    // 慢总线逐路采集时刻应递增，不能全都使用轮询开始时刻。
    FeedbackBuffer feedback2;
    CommandMailbox mailbox2;
    StopCoordinator stop2;
    CommunicationWorker worker2(bus, feedback2, mailbox2, stop2, milliseconds(100));
    worker2.pollFeedbackRealTime();
    const auto snapshot = feedback2.read();
    for (std::size_t i = 0; i < kServoCount; ++i) assert(snapshot.valid[i]);
    assert(snapshot.timestamp[0] < snapshot.timestamp[kServoCount - 1]);

    // 不合法配置必须拒绝，避免 0 周期忙循环。
    bool invalid = false;
    try {
        RealtimeExecutor bad(control, communication, feedback, stop, {milliseconds(0), milliseconds(1)});
    } catch (const std::invalid_argument&) { invalid = true; }
    assert(invalid);

    invalid=false;
    RealtimeExecutor::Config bad_batch;
    bad_batch.feedback_batch_size=0;
    try {
        RealtimeExecutor bad(control,communication,feedback,stop,bad_batch);
    } catch (const std::invalid_argument&) { invalid=true; }
    assert(invalid);
}
