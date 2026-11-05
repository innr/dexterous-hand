// S5 遗留任务：单批命令在第一个关节后过期，后续关节不得继续发送。
#include "robot/runtime/S4Pipeline.hpp"
#include "robot/bus/IServoTransport.hpp"
#include <cassert>
#include <chrono>
#include <thread>

namespace {
class SlowTxBus final : public robot::IServoTransport {
public:
    bool startSend(std::size_t id, float) noexcept override {
        assert(!active);
        assert(id==starts);
        active=true;
        ++starts;
        return true;
    }
    bool finishSend() noexcept override {
        assert(active);
        std::this_thread::sleep_for(std::chrono::milliseconds(12));
        active=false;
        return true;
    }
    bool read(std::size_t,float&,float&) override { return false; }
    std::size_t starts=0;
    bool active=false;
};
}

int main() {
    using namespace robot;
    using namespace std::chrono;
    FeedbackBuffer feedback;
    CommandMailbox mailbox;
    StopCoordinator stop;
    SlowTxBus bus;
    CommunicationWorker worker(bus,feedback,mailbox,stop,milliseconds(5));
    // 使用合成时钟原点，验证不会错误混用系统 steady_clock epoch。
    const ServoClock::time_point sim_now{};
    ServoCommand cmd;
    cmd.created_at=sim_now;
    cmd.generation=stop.generation();
    cmd.position.fill(10.0f);
    mailbox.publish(cmd);
    const auto sent=worker.processCommands(sim_now);
    assert(sent==1);
    assert(bus.starts==1);
    assert(stop.commandsAllowed()); // TTL 过期不是硬件故障，不必锁存停机。
    // 批次开始就过期时，零发送。
    cmd.created_at=sim_now-milliseconds(6);
    mailbox.publish(cmd);
    assert(worker.processCommands(sim_now)==0);
    assert(bus.starts==1);
}
