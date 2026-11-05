// S6-01: deterministic MuJoCo process adapter. No serial/physical hardware access.
#include "robot/runtime/S4Pipeline.hpp"
#include "robot/safety/FeedbackSafetyMonitor.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace {
constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

struct StepInput {
    std::int64_t time_us = 0;
    std::array<float, robot::kServoCount> position_rad{};
    std::array<float, robot::kServoCount> velocity_rad_s{};
    std::array<float, robot::kServoCount> target_rad{};
};

bool parseStep(const std::string& line, StepInput& step) {
    std::istringstream in(line);
    std::string command;
    if (!(in >> command) || command != "STEP" || !(in >> step.time_us) ||
        step.time_us < 0) return false;
    for (auto* array : {&step.position_rad, &step.velocity_rad_s, &step.target_rad}) {
        for (float& value : *array)
            if (!(in >> value) || !std::isfinite(value)) return false;
    }
    std::string extra;
    return !(in >> extra);
}

class SimulatedServoTransport final : public robot::IServoTransport {
public:
    void updateFeedback(const std::array<float,robot::kServoCount>& p,
                        const std::array<float,robot::kServoCount>& v) noexcept {
        position_=p;
        velocity_=v;
    }
    bool startSend(std::size_t id,float position_deg) noexcept override {
        if (active_ || id>=robot::kServoCount || !std::isfinite(position_deg)) return false;
        command_[id]=position_deg;
        active_=true;
        return true;
    }
    bool finishSend() noexcept override {
        if (!active_) return false;
        active_=false;
        return true;
    }
    bool read(std::size_t id,float& p,float& v) override {
        if (id>=robot::kServoCount) return false;
        p=position_[id];
        v=velocity_[id];
        return true;
    }
    const std::array<float,robot::kServoCount>& commands() const noexcept { return command_; }
private:
    std::array<float,robot::kServoCount> position_{};
    std::array<float,robot::kServoCount> velocity_{};
    std::array<float,robot::kServoCount> command_{};
    bool active_=false;
};

class SimulationStepBridge {
public:
    SimulationStepBridge()
        : control_(feedback_,mailbox_,stop_,std::chrono::milliseconds(30)),
          worker_(transport_,feedback_,mailbox_,stop_,std::chrono::milliseconds(100)) {
        control_.enableMotionControl(robot::MotionControlConfig{});
    }
    bool step(const StepInput& input,std::array<float,robot::kServoCount>& command_rad) {
        if (failed_ || (last_time_us_>=0 && input.time_us<=last_time_us_))
            return fail();
        last_time_us_=input.time_us;
        const auto now=robot::ServoClock::time_point{}+
            std::chrono::microseconds(input.time_us);
        std::array<float,robot::kServoCount> p{},v{},target{};
        for (std::size_t i=0;i<robot::kServoCount;++i) {
            p[i]=input.position_rad[i]*kRadToDeg;
            v[i]=input.velocity_rad_s[i]*kRadToDeg;
            target[i]=input.target_rad[i]*kRadToDeg;
            if (!std::isfinite(p[i]) || !std::isfinite(v[i]) ||
                !std::isfinite(target[i])) return fail();
        }
        transport_.updateFeedback(p,v);
        worker_.pollFeedback(now); // 使用仿真时钟；不使用真实串口读取时刻。
        const auto safety=safety_.evaluate(feedback_.read(),now);
        if (safety.startup_waiting || safety.inhibit_motion || safety.request_safe_stop)
            return fail();
        try {
            control_.setTargets(target);
        } catch (...) { return fail(); }
        if (!control_.tick(now)) return fail();
        if (worker_.processCommands(now)!=robot::kServoCount) return fail();
        for (std::size_t i=0;i<robot::kServoCount;++i)
            command_rad[i]=transport_.commands()[i]*kDegToRad;
        return true;
    }
    void requestStop() noexcept { (void)fail(); }
private:
    bool fail() noexcept {
        if (!failed_) {
            failed_=true;
            stop_.requestStop();
            mailbox_.clear();
        }
        return false;
    }
    robot::FeedbackBuffer feedback_;
    robot::CommandMailbox mailbox_;
    robot::StopCoordinator stop_;
    SimulatedServoTransport transport_;
    robot::ControlLoop control_;
    robot::CommunicationWorker worker_;
    robot::FeedbackSafetyMonitor safety_;
    std::int64_t last_time_us_=-1;
    bool failed_=false;
};
} // namespace

int main() {
    SimulationStepBridge bridge;
    std::cout << "READY 16 S6-01" << std::endl;
    std::string line;
    while (std::getline(std::cin,line)) {
        if (line=="QUIT") {
            bridge.requestStop();
            std::cout << "BYE" << std::endl;
            return 0;
        }
        StepInput input;
        std::array<float,robot::kServoCount> commands{};
        if (!parseStep(line,input) || !bridge.step(input,commands)) {
            bridge.requestStop();
            std::cout << "STOP" << std::endl;
            continue;
        }
        std::cout << "OK" << std::setprecision(9);
        for (float command : commands) std::cout << ' ' << command;
        std::cout << std::endl;
    }
    bridge.requestStop();
    return 0;
}
