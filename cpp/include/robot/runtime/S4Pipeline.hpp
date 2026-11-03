#pragma once
#include "robot/runtime/FeedbackBuffer.hpp"
#include "robot/runtime/CommandMailbox.hpp"
#include "robot/safety/StopCoordinator.hpp"
#include "robot/bus/IServoTransport.hpp"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <array>

// S4 运行时组件：控制计算与通信分离，串口仅由通信线程持有。
namespace robot {
struct EstimatedState {
    std::array<float, kServoCount> position{};
    std::array<bool, kServoCount> valid{};
};
class StateEstimator {
public:
    explicit StateEstimator(std::chrono::milliseconds max_age);
    EstimatedState estimate(const FeedbackSnapshot& data, ServoClock::time_point now) const;
private:
    std::chrono::milliseconds max_age_;
};
class ControlLoop {
public:
    ControlLoop(FeedbackBuffer& feedback, CommandMailbox& mailbox, StopCoordinator& stop,
                std::chrono::milliseconds max_feedback_age);
    void setTargets(const std::array<float, kServoCount>& targets);
    bool tick(ServoClock::time_point now);
private:
    FeedbackBuffer& feedback_;
    CommandMailbox& mailbox_;
    StopCoordinator& stop_;
    StateEstimator estimator_;
    std::array<float, kServoCount> targets_{};
    std::uint64_t sequence_ = 0;
};
class CommunicationWorker {
public:
    CommunicationWorker(IServoTransport& bus, FeedbackBuffer& feedback,
                        CommandMailbox& mailbox, StopCoordinator& stop,
                        std::chrono::milliseconds max_command_age);
    std::size_t processCommands(ServoClock::time_point now);
    void pollFeedback(ServoClock::time_point now);
private:
    IServoTransport& bus_;
    FeedbackBuffer& feedback_;
    CommandMailbox& mailbox_;
    StopCoordinator& stop_;
    std::chrono::milliseconds max_command_age_;
};
} // namespace robot
