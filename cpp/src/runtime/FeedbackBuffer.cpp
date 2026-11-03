#include "robot/runtime/FeedbackBuffer.hpp"
namespace robot {
void FeedbackBuffer::publish(const FeedbackSnapshot& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = value;
    ++value_.sequence;
}
FeedbackSnapshot FeedbackBuffer::read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}
} // namespace robot
