#include "robot/runtime/FeedbackBuffer.hpp"
namespace robot {
void FeedbackBuffer::publish(const FeedbackSnapshot& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = value;
    ++value_.sequence;
}
bool FeedbackBuffer::updateJoint(std::size_t joint, float position, float velocity,
                                 ServoClock::time_point sampled_at, bool valid) {
    if (joint >= kServoCount) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (valid) {
        value_.position[joint] = position;
        value_.velocity[joint] = velocity;
        value_.timestamp[joint] = sampled_at;
    }
    value_.valid[joint] = valid;
    ++value_.sequence;
    return true;
}
FeedbackSnapshot FeedbackBuffer::read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
}
} // namespace robot
