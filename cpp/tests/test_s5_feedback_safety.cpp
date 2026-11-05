// S5-06：虚拟时间边界、状态锁存和真实线程安全门禁联动。
#include "robot/safety/FeedbackSafetyMonitor.hpp"
#include "robot/runtime/RealtimeExecutor.hpp"
#include "robot/bus/IServoTransport.hpp"
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <thread>

namespace {
robot::FeedbackSnapshot fresh(robot::ServoClock::time_point now) {
    robot::FeedbackSnapshot data;
    data.valid.fill(true);
    data.position.fill(0.0f);
    data.velocity.fill(0.0f);
    data.timestamp.fill(now);
    return data;
}
class FailingReadBus final : public robot::IServoTransport {
public:
    std::atomic<std::size_t> starts{0};
    bool startSend(std::size_t,float) noexcept override {
        ++starts;
        return true;
    }
    bool finishSend() noexcept override { return true; }
    bool read(std::size_t,float&,float&) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return false;
    }
};
}

int main() {
    using namespace robot;
    using namespace std::chrono;
    const ServoClock::time_point t0{};
    FeedbackSafetyConfig cfg;
    cfg.startup_grace=milliseconds(100);
    cfg.warning_age=milliseconds(20);
    cfg.hard_timeout=milliseconds(30);
    cfg.warning_ticks=2;
    cfg.stop_ticks=3;

    // 无有效反馈的启动期：不允许 ControlLoop 发布命令，但监控不立即锁死。
    FeedbackSafetyMonitor startup(cfg);
    FeedbackSnapshot empty;
    auto d=startup.evaluate(empty,t0);
    assert(d.startup_waiting && !d.inhibit_motion);
    d=startup.evaluate(empty,t0+milliseconds(99));
    assert(d.startup_waiting && d.state==SafetyState::NORMAL);
    d=startup.evaluate(empty,t0+milliseconds(100));
    assert(!d.startup_waiting && d.inhibit_motion);
    assert(d.state==SafetyState::DEGRADED && d.consecutive_critical_ticks==1);
    d=startup.evaluate(empty,t0+milliseconds(110));
    assert(d.state==SafetyState::DEGRADED && d.consecutive_critical_ticks==2);
    d=startup.evaluate(empty,t0+milliseconds(120));
    assert(d.state==SafetyState::SAFE_STOP && d.request_safe_stop);
    d=startup.evaluate(fresh(t0+milliseconds(130)),t0+milliseconds(130));
    assert(d.state==SafetyState::SAFE_STOP && d.inhibit_motion);
    assert(d.request_safe_stop); // 恢复反馈不解除锁存。

    // 边界：warning_age 和 hard_timeout 本身仍属于允许年龄。
    FeedbackSafetyMonitor age(cfg);
    auto sample=fresh(t0);
    d=age.evaluate(sample,t0);
    assert(d.state==SafetyState::NORMAL);
    d=age.evaluate(sample,t0+milliseconds(20));
    assert(d.warning_joints==0 && !d.inhibit_motion);
    d=age.evaluate(sample,t0+milliseconds(21));
    assert(d.warning_joints==kServoCount && d.state==SafetyState::NORMAL);
    d=age.evaluate(sample,t0+milliseconds(22));
    assert(d.state==SafetyState::DEGRADED && !d.inhibit_motion);
    d=age.evaluate(sample,t0+milliseconds(30));
    assert(d.stale_joints==0 && !d.inhibit_motion);
    d=age.evaluate(sample,t0+milliseconds(31));
    assert(d.stale_joints==kServoCount && d.inhibit_motion);
    assert(d.state==SafetyState::DEGRADED && !d.request_safe_stop);
    d=age.evaluate(fresh(t0+milliseconds(32)),t0+milliseconds(32));
    assert(d.state==SafetyState::DEGRADED && d.inhibit_motion);
    assert(d.consecutive_critical_ticks==0); // 连续计数恢复，门禁仍锁存。

    // 单路错误、未来时间戳和非有限反馈都按硬异常处理。
    FeedbackSafetyMonitor single(cfg);
    sample=fresh(t0);
    (void)single.evaluate(sample,t0);
    sample.valid[3]=false;
    d=single.evaluate(sample,t0+milliseconds(1));
    assert(d.invalid_joints==1 && d.inhibit_motion);
    FeedbackSafetyMonitor future(cfg);
    sample=fresh(t0+milliseconds(10));
    (void)future.evaluate(fresh(t0),t0);
    d=future.evaluate(sample,t0+milliseconds(1));
    assert(d.invalid_joints==kServoCount && d.inhibit_motion);

    bool invalid=false;
    cfg.hard_timeout=cfg.warning_age;
    try { FeedbackSafetyMonitor bad(cfg); }
    catch (const std::invalid_argument&) { invalid=true; }
    assert(invalid);

    // 真正双线程：通信持续失败，控制线程锁存 stop 并清空旧邮箱。
    FeedbackBuffer feedback;
    CommandMailbox mailbox;
    StopCoordinator stop;
    FailingReadBus bus;
    ControlLoop control(feedback,mailbox,stop,milliseconds(30));
    CommunicationWorker communication(bus,feedback,mailbox,stop,milliseconds(100));
    feedback.publish(fresh(ServoClock::now()));
    RealtimeExecutor::Config runtime;
    runtime.control_period=milliseconds(5);
    runtime.communication_pause=milliseconds(1);
    runtime.feedback_batch_size=4;
    runtime.feedback_safety.startup_grace=milliseconds(0);
    runtime.feedback_safety.warning_age=milliseconds(10);
    runtime.feedback_safety.hard_timeout=milliseconds(20);
    runtime.feedback_safety.stop_ticks=2;
    RealtimeExecutor executor(control,communication,feedback,stop,runtime);
    assert(executor.start());
    for (int i=0; i<200 && executor.running(); ++i)
        std::this_thread::sleep_for(milliseconds(5));
    assert(!executor.running());
    const auto stats=executor.stats();
    assert(stats.feedback_safety_state==SafetyState::SAFE_STOP);
    assert(stats.feedback_motion_inhibited);
    assert(stats.consecutive_critical_ticks>=2);
    assert(!stop.commandsAllowed());
    executor.stop();
    assert(stop.inFlight()==0);
    assert(!mailbox.takeLatest());
    const auto sent_before=bus.starts.load();
    ServoCommand stale;
    stale.generation=stop.generation();
    stale.created_at=ServoClock::now();
    mailbox.publish(stale);
    assert(communication.processCommands(ServoClock::now())==0);
    assert(bus.starts.load()==sent_before); // 停机后不能启动任何新运动 TX。
}
