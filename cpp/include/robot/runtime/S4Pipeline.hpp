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
    // 反馈硬异常时清除尚未被通信线程取走的旧命令。
    void clearPendingCommands();
private:
    FeedbackBuffer& feedback_;
    CommandMailbox& mailbox_;
    StopCoordinator& stop_;
    StateEstimator estimator_;
    // setTargets 可能来自应用线程，tick 来自控制线程，必须同步目标数组。
    std::mutex targets_mutex_;
    std::array<float, kServoCount> targets_{};
    std::uint64_t sequence_ = 0;
};
// 分片轮询结果：completed_round 只在第 16 路完成时置位。
struct PollBatchResult {
    std::size_t attempted = 0;
    bool completed_round = false;
};
class CommunicationWorker {
public:
    CommunicationWorker(IServoTransport& bus, FeedbackBuffer& feedback,
                        CommandMailbox& mailbox, StopCoordinator& stop,
                        std::chrono::milliseconds max_command_age);
    std::size_t processCommands(ServoClock::time_point now);
    void pollFeedback(ServoClock::time_point now);
    // 实际通信轮询：每路读操作完成时独立打时间戳。
    void pollFeedbackRealTime();
    // 只允许单通信线程调用。分片完成后立即发布每路反馈。
    PollBatchResult pollFeedbackSliceRealTime(std::size_t max_joints);
private:
    IServoTransport& bus_;
    FeedbackBuffer& feedback_;
    CommandMailbox& mailbox_;
    StopCoordinator& stop_;
    std::chrono::milliseconds max_command_age_;
    std::size_t next_joint_ = 0; // 轮询游标，按 ID 0..15 公平轮转。
};
} // namespace robot
