// S5-09：环形队列边界/顺序/并发、实际100Hz运行器和16路CSV数据。
#include "robot/runtime/ControlTelemetry.hpp"
#include "robot/runtime/RealtimeExecutor.hpp"
#include "robot/bus/VirtualServoBus.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <thread>

int main() {
    using namespace robot;
    using namespace std::chrono;
    bool invalid=false;
    try { ControlTelemetryRing bad(0); }
    catch (const std::invalid_argument&) { invalid=true; }
    assert(invalid);
    ControlTelemetryRing small(2);
    ControlTelemetryFrame f;
    assert(!small.tryPop(f));
    f.tick=1;
    assert(small.tryPush(f));
    f.tick=2;
    assert(small.tryPush(f));
    f.tick=3;
    assert(!small.tryPush(f));
    assert(small.dropped()==1);
    assert(small.tryPop(f) && f.tick==1);
    assert(small.tryPop(f) && f.tick==2);
    assert(!small.tryPop(f));
    f.tick=4;
    assert(small.tryPush(f));
    assert(small.tryPop(f) && f.tick==4);

    // SPSC 真正并发；生产者不因队列满等待，消费顺序严格递增。
    ControlTelemetryRing concurrent(64);
    std::atomic<bool> done{false};
    std::thread producer([&] {
        for (std::uint64_t i=1;i<=100000;++i) {
            ControlTelemetryFrame frame;
            frame.tick=i;
            (void)concurrent.tryPush(frame);
        }
        done.store(true,std::memory_order_release);
    });
    std::uint64_t received=0,last=0;
    while (true) {
        if (concurrent.tryPop(f)) {
            assert(f.tick>last);
            last=f.tick;
            ++received;
        } else if (done.load(std::memory_order_acquire)) {
            break;
        } else {
            std::this_thread::yield();
        }
    }
    producer.join();
    assert(received+concurrent.dropped()==100000);

    FeedbackBuffer feedback;
    CommandMailbox mailbox;
    StopCoordinator stop;
    VirtualServoBus bus;
    ControlLoop control(feedback,mailbox,stop,milliseconds(50));
    CommunicationWorker communication(bus,feedback,mailbox,stop,milliseconds(100));
    communication.pollFeedback(ServoClock::now());
    std::array<float,kServoCount> targets{};
    targets.fill(25.0f);
    control.setTargets(targets);
    ControlTelemetryRing ring(1024);
    RealtimeExecutor::Config config;
    config.telemetry=&ring;
    config.control_period=milliseconds(10);
    config.communication_pause=milliseconds(1);
    RealtimeExecutor executor(control,communication,feedback,stop,config);
    assert(executor.start());
    std::this_thread::sleep_for(milliseconds(160));
    executor.stop();
    std::ostringstream csv;
    writeTelemetryCsvHeader(csv);
    std::size_t ticks=0,published=0;
    std::uint64_t last_tick=0;
    while (ring.tryPop(f)) {
        assert(f.tick==last_tick+1);
        last_tick=f.tick;
        ++ticks;
        if (f.status==ControlTickStatus::PUBLISHED) {
            ++published;
            assert(f.target_deg[0]==25.0f);
            assert(f.command_deg[0]>=0.0f && f.command_deg[0]<25.0f);
            assert(f.feedback_valid[0]);
            assert(f.feedback_age_us[0]>=0);
            assert(f.control_duration_us>=0);
        }
        writeTelemetryCsvFrame(csv,f);
    }
    assert(ticks>=5 && published>0);
    assert(ring.dropped()==0);
    assert(ticks==executor.stats().control_ticks);
    assert(executor.stats().max_full_tick_duration_us >=
           executor.stats().max_control_duration_us);
    const auto contents=csv.str();
    assert(contents.find("tick,time_us,joint,status,safety_state,published") == 0);
    std::size_t newline_count=0;
    for (char c:contents) if (c=='\n') ++newline_count;
    assert(newline_count==1+ticks*kServoCount);
}
