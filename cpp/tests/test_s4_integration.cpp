// S4 集成测试：16 路虚拟反馈、最新值命令、安全停机与线程安全。
#include "robot/bus/VirtualServoBus.hpp"
#include "robot/runtime/S4Pipeline.hpp"
#include <cassert>
#include <chrono>
#include <thread>
int main() {
    using namespace robot;
    using namespace std::chrono;
    const ServoClock::time_point origin{};
    FeedbackBuffer feedback;
    CommandMailbox mailbox;
    StopCoordinator stop;
    VirtualServoBus bus;
    ControlLoop control(feedback, mailbox, stop, milliseconds(20));
    CommunicationWorker worker(bus, feedback, mailbox, stop, milliseconds(20));
    assert(!control.tick(origin));
    worker.pollFeedback(origin);
    std::array<float, kServoCount> target{};
    target.fill(30.0f);
    control.setTargets(target);
    assert(control.tick(origin + milliseconds(10)));
    assert(worker.processCommands(origin + milliseconds(11)) == kServoCount);
    assert(bus.sendCount() == kServoCount);
    worker.pollFeedback(origin + milliseconds(12));
    assert(feedback.read().position[15] == 30.0f);
    assert(control.tick(origin + milliseconds(13)));
    assert(worker.processCommands(origin + milliseconds(40)) == 0);
    ServoCommand cmd;
    cmd.generation = stop.generation();
    cmd.created_at = origin + milliseconds(50);
    mailbox.publish(cmd);
    assert(worker.processCommands(origin + milliseconds(49)) == 0);
    stop.requestStop();
    bool started = false;
    assert(!stop.beginSend(cmd.generation, [&] { started = true; return true; }));
    assert(!started);
    assert(!stop.recover(false, true));
    assert(stop.recover(true, true));
    mailbox.publish(cmd);
    assert(worker.processCommands(origin + milliseconds(51)) == 0);
    auto active = stop.beginSend(stop.generation(), [&] { started = true; return true; });
    assert(active);
    stop.requestStop();
    assert(stop.inFlight() == 1);
    assert(!stop.recover(true, true));
    active.reset();
    assert(stop.recover(true, true));
    bus.setReadFailure(3, true);
    worker.pollFeedback(origin + milliseconds(60));
    assert(!feedback.read().valid[3]);
    assert(!control.tick(origin + milliseconds(61)));
    bus.setReadFailure(3, false);
    worker.pollFeedback(origin + milliseconds(62));
    assert(control.tick(origin + milliseconds(63)));
    ServoCommand a, b;
    a.sequence = 1; b.sequence = 2;
    mailbox.publish(a); mailbox.publish(b);
    auto latest = mailbox.takeLatest();
    assert(latest && latest->sequence == 2);
    assert(!mailbox.takeLatest());
    StateEstimator estimator(milliseconds(20));
    auto snap = feedback.read();
    snap.timestamp[0] = origin;
    snap.timestamp[1] = origin + milliseconds(70);
    auto estimated = estimator.estimate(snap, origin + milliseconds(63));
    assert(!estimated.valid[0] && !estimated.valid[1] && estimated.valid[2]);
    std::thread producer([&] {
        for (int n = 0; n < 500; ++n) {
            FeedbackSnapshot value;
            value.position.fill(static_cast<float>(n));
            feedback.publish(value);
        }
    });
    for (int n = 0; n < 500; ++n) {
        auto value = feedback.read();
        for (float p : value.position) assert(p == value.position[0]);
    }
    producer.join();
}
