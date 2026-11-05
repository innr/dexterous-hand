// S5-09：只使用虚拟总线，不会连接串口或使能真实舵机。
#include "robot/bus/VirtualServoBus.hpp"
#include "robot/runtime/ControlTelemetry.hpp"
#include "robot/runtime/RealtimeExecutor.hpp"
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

int main(int argc,char** argv) {
    if (argc!=2) {
        std::cerr<<"usage: s5_telemetry_demo <output.csv>\n";
        return 2;
    }
    try {
        robot::FeedbackBuffer feedback;
        robot::CommandMailbox mailbox;
        robot::StopCoordinator stop;
        robot::VirtualServoBus bus;
        robot::ControlLoop control(feedback,mailbox,stop,std::chrono::milliseconds(50));
        robot::CommunicationWorker communication(bus,feedback,mailbox,stop,
                                                 std::chrono::milliseconds(100));
        communication.pollFeedback(robot::ServoClock::now());
        std::array<float,robot::kServoCount> targets{};
        targets.fill(25.0f);
        control.setTargets(targets);
        robot::ControlTelemetryRing ring(1024);
        robot::RealtimeExecutor::Config config;
        config.telemetry=&ring;
        robot::RealtimeExecutor executor(control,communication,feedback,stop,config);
        if (!executor.start()) throw std::runtime_error("cannot start virtual runtime");
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        executor.stop();
        std::ofstream out(argv[1]);
        if (!out) throw std::runtime_error("cannot open output CSV");
        robot::writeTelemetryCsvHeader(out);
        robot::ControlTelemetryFrame frame;
        std::size_t ticks=0;
        while (ring.tryPop(frame)) {
            robot::writeTelemetryCsvFrame(out,frame);
            ++ticks;
        }
        if (!out) throw std::runtime_error("cannot write CSV");
        std::cout<<"VIRTUAL BUS ONLY: ticks="<<ticks<<", dropped="<<ring.dropped()
                 <<", file="<<argv[1]<<"\n";
        if (ticks==0) return 1;
    } catch (const std::exception& e) {
        std::cerr<<"telemetry demo failed: "<<e.what()<<"\n";
        return 1;
    }
}
