#include "robot/runtime/CommandMailbox.hpp"
namespace robot {
void CommandMailbox::publish(const ServoCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = command;
}
std::optional<ServoCommand> CommandMailbox::takeLatest() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto value = pending_;
    pending_.reset();
    return value;
}
void CommandMailbox::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
}
} // namespace robot
