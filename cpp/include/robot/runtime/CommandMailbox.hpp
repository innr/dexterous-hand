#pragma once
#include "robot/runtime/FeedbackBuffer.hpp"
#include <array>
#include <cstdint>
#include <mutex>
#include <optional>

// 最新值邮箱：覆盖旧位置命令，避免积压过期动作。
namespace robot {
struct ServoCommand {
    std::array<float, kServoCount> position{};
    std::uint64_t sequence = 0;
    std::uint64_t generation = 0;
    ServoClock::time_point created_at{};
};
class CommandMailbox {
public:
    void publish(const ServoCommand& command);
    std::optional<ServoCommand> takeLatest();
    void clear();
private:
    std::mutex mutex_;
    std::optional<ServoCommand> pending_;
};
} // namespace robot
