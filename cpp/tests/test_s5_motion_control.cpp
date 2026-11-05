// S5-07：虚拟时间下验证16关节限速/限加速度、PD位置外环与安全门禁。
#include "robot/control/MotionControlCore.hpp"
#include "robot/runtime/S4Pipeline.hpp"
#include "robot/runtime/RealtimeExecutor.hpp"
#include "robot/bus/VirtualServoBus.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace {
robot::FeedbackSnapshot feedbackAt(robot::ServoClock::time_point now,float angle) {
    robot::FeedbackSnapshot s;
    s.position.fill(angle);
    s.velocity.fill(0.0f);
    s.valid.fill(true);
    s.timestamp.fill(now);
    return s;
}
}

int main() {
    using namespace robot;
    using namespace std::chrono;
    const ServoClock::time_point origin{};
    MotionControlConfig config;
    config.nominal_period=milliseconds(10);
    config.max_step=milliseconds(50);
    for (auto& j:config.joints) {
        j.max_velocity_deg_s=20.0f;
        j.max_acceleration_deg_s2=100.0f;
        j.kp=1.0f;
        j.kd_seconds=0.05f;
    }
    MotionControlCore motion(config);
    std::array<float,kServoCount> targets{},measured{},velocity{};
    targets.fill(45.0f);
    measured.fill(15.0f);
    MotionStep step;
    float last_position=15.0f, last_velocity=0.0f;
    for (int n=0;n<150;++n) {
        assert(motion.step(targets,measured,velocity,origin+milliseconds(10*n),step));
        for (std::size_t i=0;i<kServoCount;++i) {
            assert(std::isfinite(step.command_deg[i]));
            assert(step.command_deg[i]>=-90.0f && step.command_deg[i]<=90.0f);
            assert(step.command_deg[i]>=15.0f && step.command_deg[i]<=45.5f);
            assert(std::abs(step.command_velocity_deg_s[i])<=20.001f);
            assert(std::abs(step.command_velocity_deg_s[i]-last_velocity)<=1.001f);
            assert(step.command_deg[i]==step.command_deg[0]);
        }
        assert(step.command_deg[0]>=last_position-0.001f);
        last_position=step.command_deg[0];
        last_velocity=step.command_velocity_deg_s[0];
    }
    assert(last_position>20.0f);
    targets.fill(-20.0f);
    for (int n=150;n<230;++n) {
        assert(motion.step(targets,measured,velocity,origin+milliseconds(10*n),step));
        assert(std::abs(step.command_velocity_deg_s[0])<=20.001f);
        assert(std::abs(step.command_velocity_deg_s[0]-last_velocity)<=1.001f);
        last_velocity=step.command_velocity_deg_s[0];
    }

    // KD 影响位置目标修正：速度误差会产生位置偏置，但不会输出电流/力矩。
    MotionControlConfig damping=config;
    damping.joints[0].kp=0.0f;
    damping.joints[0].kd_seconds=0.2f;
    MotionControlCore with_damping(damping);
    measured.fill(0.0f);
    velocity.fill(5.0f);
    targets.fill(0.0f);
    for (int n=0;n<20;++n)
        assert(with_damping.step(targets,measured,velocity,
                                 origin+milliseconds(10*n),step));
    assert(step.command_deg[0]<step.command_deg[1]); // 更大的 KD 产生更强速度阻尼。

    MotionControlCore invalid_motion(config);
    measured.fill(100.0f);
    assert(!invalid_motion.step(targets,measured,velocity,origin,step));
    measured.fill(0.0f);
    assert(invalid_motion.step(targets,measured,velocity,origin,step));
    assert(!invalid_motion.step(targets,measured,velocity,origin+milliseconds(60),step));
    assert(!invalid_motion.step(targets,measured,velocity,origin-milliseconds(1),step));

    bool invalid=false;
    config.joints[3].max_acceleration_deg_s2=0.0f;
    try { MotionControlCore bad(config); }
    catch (const std::invalid_argument&) { invalid=true; }
    assert(invalid);

    // ControlLoop 与 CommandMailbox 集成：初次反馈位置为15°，目标45°，
    // 第一帧必须从反馈位置平滑起步，不能直接跳到45°。
    FeedbackBuffer buffer;
    CommandMailbox mailbox;
    StopCoordinator stop;
    ControlLoop loop(buffer,mailbox,stop,milliseconds(30));
    MotionControlConfig integrated;
    integrated.nominal_period=milliseconds(10);
    loop.enableMotionControl(integrated);
    buffer.publish(feedbackAt(origin,15.0f));
    assert(!loop.tick(origin)); // 尚未明确设置目标，禁止隐式回零。
    std::array<float,kServoCount> target{};
    target.fill(45.0f);
    loop.setTargets(target);
    assert(loop.tick(origin));
    const auto first=mailbox.takeLatest();
    assert(first);
    for (float pos:first->position) assert(pos>=15.0f && pos<15.1f);
    assert(loop.tick(origin+milliseconds(10)));
    const auto second=mailbox.takeLatest();
    assert(second && second->position[0]>first->position[0]);
    // 缺少新反馈时不允许无期限依靠历史反馈控制。
    assert(!loop.tick(origin+milliseconds(40)));
    assert(!stop.commandsAllowed() || !mailbox.takeLatest());

    // 真实双线程运行器（虚拟总线）：确认默认接入外环，
    // 而非直接将应用目标30°转发给舵机。
    FeedbackBuffer live_feedback;
    CommandMailbox live_mailbox;
    StopCoordinator live_stop;
    VirtualServoBus virtual_bus;
    ControlLoop live_control(live_feedback,live_mailbox,live_stop,milliseconds(50));
    CommunicationWorker live_worker(virtual_bus,live_feedback,live_mailbox,
                                    live_stop,milliseconds(100));
    live_worker.pollFeedback(ServoClock::now());
    target.fill(30.0f);
    live_control.setTargets(target);
    RealtimeExecutor::Config runtime;
    runtime.control_period=milliseconds(10);
    runtime.communication_pause=milliseconds(1);
    runtime.motion.max_step=milliseconds(50);
    RealtimeExecutor executor(live_control,live_worker,live_feedback,live_stop,runtime);
    assert(executor.start());
    std::this_thread::sleep_for(milliseconds(150));
    const auto stats=executor.stats();
    executor.stop();
    float actual=0.0f, actual_velocity=0.0f;
    assert(virtual_bus.read(0,actual,actual_velocity));
    assert(stats.control_ticks>=5 && stats.published_commands>0);
    assert(virtual_bus.sendCount()>0);
    assert(actual>0.0f && actual<30.0f); // 不允许首次阶跃直接跳到30°。
}
