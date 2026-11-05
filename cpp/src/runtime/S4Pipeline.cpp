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
void ControlLoop::clearPendingCommands() { mailbox_.clear(); }
void ControlLoop::enableMotionControl(MotionControlConfig config) {
    motion_=std::make_unique<MotionControlCore>(config);
}
void ControlLoop::setTargets(const std::array<float, kServoCount>& targets) {
    for (float target : targets) if (!std::isfinite(target)) throw std::invalid_argument("invalid target");
    std::lock_guard<std::mutex> lock(targets_mutex_);
    targets_ = targets;
    targets_set_ = true;
}
bool ControlLoop::tick(ServoClock::time_point now, ControlTelemetryFrame* trace) {
    if (trace) trace->status=ControlTickStatus::STOP_GATE;
    if (!stop_.commandsAllowed()) return false;
    const auto snapshot=feedback_.read();
    if (trace) {
        trace->feedback_sequence=snapshot.sequence;
        for (std::size_t i=0;i<kServoCount;++i) {
            trace->measured_deg[i]=snapshot.position[i];
            trace->measured_velocity_deg_s[i]=snapshot.velocity[i];
            trace->feedback_valid[i]=snapshot.valid[i] && now>=snapshot.timestamp[i];
            trace->feedback_age_us[i]=trace->feedback_valid[i] ?
                std::chrono::duration_cast<std::chrono::microseconds>(
                    now-snapshot.timestamp[i]).count() : -1;
        }
        trace->status=ControlTickStatus::INVALID_FEEDBACK;
    }
    const auto state = estimator_.estimate(snapshot, now);
    // 第一版保守处理：任意一路反馈过期，禁止发布整手新动作。
    for (bool valid : state.valid) if (!valid) return false;
    std::array<float, kServoCount> targets;
    {
        std::lock_guard<std::mutex> lock(targets_mutex_);
        if (motion_ && !targets_set_) {
            if (trace) trace->status=ControlTickStatus::NO_TARGET;
            return false; // 未授权目标时不得自行回零。
        }
        targets = targets_;
    }
    if (trace) trace->target_deg=targets;
    ServoCommand command;
    if (motion_) {
        std::array<float,kServoCount> measured_velocity{};
        for (std::size_t i=0;i<kServoCount;++i) measured_velocity[i]=snapshot.velocity[i];
        if (!motion_->step(targets,state.position,measured_velocity,now,last_motion_step_)) {
            if (trace) trace->status=ControlTickStatus::MOTION_REJECTED;
            // 控制状态不可用（例如超长控制周期或越界），故障锁存优先于下发。
            stop_.requestStop();
            mailbox_.clear();
            return false;
        }
        command.position=last_motion_step_.command_deg;
        if (trace) trace->reference_deg=last_motion_step_.reference_deg;
    } else {
        // S4 单独测试和历史调用保留目标位置直通。
        for (std::size_t i=0;i<kServoCount;++i)
            command.position[i]=std::clamp(targets[i],-90.0f,90.0f);
    }
    if (trace) {
        trace->command_deg=command.position;
        trace->status=ControlTickStatus::PUBLISHED;
    }
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
PollBatchResult CommunicationWorker::pollFeedbackSliceRealTime(std::size_t max_joints) {
    if (max_joints == 0 || max_joints > kServoCount)
        throw std::invalid_argument("feedback batch must be in [1,16]");
    PollBatchResult result;
    for (std::size_t n = 0; n < max_joints; ++n) {
        const auto joint = next_joint_;
        float p = 0.0f, v = 0.0f;
        bool read_ok = false;
        try {
            read_ok = bus_.read(joint, p, v);
        } catch (...) {
            // 诊断时保留其他关节的数据；单路异常不能伪装为有效反馈。
            read_ok = false;
        }
        const auto sampled_at = ServoClock::now();
        feedback_.updateJoint(joint, p, v, sampled_at,
                              read_ok && std::isfinite(p) && std::isfinite(v));
        ++result.attempted;
        next_joint_ = (next_joint_ + 1) % kServoCount;
        if (next_joint_ == 0) result.completed_round = true;
    }
    return result;
}
void CommunicationWorker::pollFeedbackRealTime() {
    // 兼容旧接口：仍轮询完整 16 路，但每路完成后立即发布。
    (void)pollFeedbackSliceRealTime(kServoCount);
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
