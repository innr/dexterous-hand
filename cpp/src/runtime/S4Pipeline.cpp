#include "robot/runtime/S4Pipeline.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot {
StateEstimator::StateEstimator(std::chrono::milliseconds max_age) : max_age_(max_age) {
    if (max_age_ <= std::chrono::milliseconds::zero()) throw std::invalid_argument("invalid feedback age");
}
EstimatedState StateEstimator::estimate(const FeedbackSnapshot& data, ServoClock::time_point now) const {
    EstimatedState result;
    for (std::size_t i = 0; i < kServoCount; ++i) {
        if (!data.valid[i] || now < data.timestamp[i] || now - data.timestamp[i] > max_age_ ||
            !std::isfinite(data.position[i]) || !std::isfinite(data.velocity[i])) continue;
        const auto dt = std::chrono::duration<float>(now - data.timestamp[i]).count();
        const float predicted = data.position[i] + data.velocity[i] * dt;
        if (!std::isfinite(predicted)) continue;
        result.position[i] = predicted;
        result.valid[i] = true;
    }
    return result;
}
ControlLoop::ControlLoop(FeedbackBuffer& feedback, CommandMailbox& mailbox, StopCoordinator& stop,
                         std::chrono::milliseconds max_feedback_age)
    : feedback_(feedback), mailbox_(mailbox), stop_(stop), estimator_(max_feedback_age) {}
void ControlLoop::setTargets(const std::array<float, kServoCount>& targets) {
    for (float target : targets) if (!std::isfinite(target)) throw std::invalid_argument("invalid target");
    targets_ = targets;
}
bool ControlLoop::tick(ServoClock::time_point now) {
    if (!stop_.commandsAllowed()) return false;
    const auto state = estimator_.estimate(feedback_.read(), now);
    // 第一版保守处理：任意一路反馈过期，禁止发布整手新动作。
    for (bool valid : state.valid) if (!valid) return false;
    ServoCommand command;
    for (std::size_t i = 0; i < kServoCount; ++i)
        command.position[i] = std::clamp(targets_[i], -90.0f, 90.0f);
    command.sequence = ++sequence_;
    command.generation = stop_.generation();
    command.created_at = now;
    mailbox_.publish(command); // 发送前通信线程再次验证停机代次。
    return true;
}
CommunicationWorker::CommunicationWorker(IServoTransport& bus, FeedbackBuffer& feedback,
        CommandMailbox& mailbox, StopCoordinator& stop, std::chrono::milliseconds max_command_age)
    : bus_(bus), feedback_(feedback), mailbox_(mailbox), stop_(stop),
      max_command_age_(max_command_age) {
    if (max_command_age_ <= std::chrono::milliseconds::zero())
        throw std::invalid_argument("invalid command age");
}
std::size_t CommunicationWorker::processCommands(ServoClock::time_point now) {
    const auto cmd = mailbox_.takeLatest();
    if (!cmd || now < cmd->created_at || now - cmd->created_at > max_command_age_) return 0;
    std::size_t sent = 0;
    for (std::size_t i = 0; i < kServoCount; ++i) {
        if (!std::isfinite(cmd->position[i])) {
            stop_.requestStop();
            break;
        }
        // TX 真正启动和停机请求使用同一把门禁锁，消除检查后启动竞态。
        auto permit = stop_.beginSend(cmd->generation, [&] {
            return bus_.startSend(i, cmd->position[i]);
        });
        if (!permit) break;

        // 阻塞等待 UART 完成必须在门禁锁外；许可在完成后才释放。
        if (!bus_.finishSend()) {
            stop_.requestStop();
            break;
        }
        ++sent;
    }
    return sent;
}
void CommunicationWorker::pollFeedback(ServoClock::time_point now) {
    auto data = feedback_.read();
    for (std::size_t i = 0; i < kServoCount; ++i) {
        float p = 0.0f, v = 0.0f;
        if (bus_.read(i, p, v) && std::isfinite(p) && std::isfinite(v)) {
            data.position[i] = p;
            data.velocity[i] = v;
            data.timestamp[i] = now; // 虚拟时间；真实驱动须记录每路实际采样时刻。
            data.valid[i] = true;
        } else data.valid[i] = false;
    }
    feedback_.publish(data);
}
} // namespace robot
