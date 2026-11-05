#include "robot/runtime/RealtimeExecutor.hpp"
#include "robot/runtime/PeriodicScheduler.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace robot {
namespace {
using Clock = ServoClock;

std::int64_t micros(Clock::duration duration) {
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}
} // namespace

RealtimeExecutor::RealtimeExecutor(ControlLoop& control, CommunicationWorker& communication,
                                   FeedbackBuffer& feedback, StopCoordinator& stop)
    : RealtimeExecutor(control, communication, feedback, stop, Config{}) {}

RealtimeExecutor::RealtimeExecutor(ControlLoop& control, CommunicationWorker& communication,
                                   FeedbackBuffer& feedback, StopCoordinator& stop, Config config)
    : control_(control), communication_(communication), feedback_(feedback),
      stop_(stop), config_(config), feedback_safety_(config.feedback_safety) {
    if (config_.control_period <= std::chrono::milliseconds::zero() ||
        config_.communication_pause <= std::chrono::milliseconds::zero() ||
        config_.feedback_batch_size == 0 || config_.feedback_batch_size > kServoCount)
        throw std::invalid_argument("invalid runtime period");
    // RealtimeExecutor 构造在启动线程之前完成，避免控制线程看到半配置状态。
    config_.motion.nominal_period=config_.control_period;
    control_.enableMotionControl(config_.motion);
}

RealtimeExecutor::~RealtimeExecutor() { stop(); }

bool RealtimeExecutor::start() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (started_once_ || !stop_.commandsAllowed()) return false;
    started_once_.store(true);
    stopping_.store(false);
    try {
        control_thread_ = std::thread([this] { controlMain(); });
        communication_thread_ = std::thread([this] { communicationMain(); });
    } catch (...) {
        stopping_.store(true);
        stop_.requestStop();
        wake_cv_.notify_all();
        if (control_thread_.joinable()) control_thread_.join();
        if (communication_thread_.joinable()) communication_thread_.join();
        throw;
    }
    return true;
}

void RealtimeExecutor::stop() {
    std::thread control;
    std::thread communication;
    {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (!started_once_) return;
        stopping_.store(true);
        // 停机优先于 join：通信线程即使仍在阻塞读，也不得再启动新 TX。
        stop_.requestStop();
        wake_cv_.notify_all();
        control = std::move(control_thread_);
        communication = std::move(communication_thread_);
    }
    if (control.joinable()) control.join();
    if (communication.joinable()) communication.join();
}

bool RealtimeExecutor::running() const noexcept {
    return started_once_.load() && !stopping_.load();
}

RuntimeStats RealtimeExecutor::stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void RealtimeExecutor::failSafe() noexcept {
    // 异常不能逃出 std::thread 入口，否则 std::terminate。
    stopping_.store(true);
    stop_.requestStop();
    wake_cv_.notify_all();
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.worker_faults;
}

void RealtimeExecutor::controlMain() noexcept {
    try {
        PeriodicScheduler scheduler(config_.control_period, Clock::now() + config_.control_period);
        bool gate_inhibited = false;
        while (!stopping_.load()) {
            const auto scheduled = scheduler.nextTime();
            {
                std::unique_lock<std::mutex> lock(wake_mutex_);
                if (wake_cv_.wait_until(lock, scheduled, [this] { return stopping_.load(); }))
                    break;
            }
            if (stopping_.load()) break;

            const auto begin = Clock::now();
            const auto feedback = feedback_.read();
            std::size_t invalid = 0;
            std::int64_t oldest_age_us = -1;
            for (std::size_t i = 0; i < kServoCount; ++i) {
                if (!feedback.valid[i] || begin < feedback.timestamp[i]) {
                    ++invalid;
                    continue;
                }
                oldest_age_us = std::max(oldest_age_us, micros(begin - feedback.timestamp[i]));
            }

            const auto safety = feedback_safety_.evaluate(feedback, begin);
            if (safety.inhibit_motion && !gate_inhibited) {
                // 先关闭原子发送门禁，再清理待发送邮箱；在途 TX 不会被伪装为取消。
                stop_.requestStop();
                control_.clearPendingCommands();
                gate_inhibited = true;
            }
            if (safety.request_safe_stop) {
                stopping_.store(true);
                wake_cv_.notify_all();
            }
            const bool published = !safety.startup_waiting && !safety.inhibit_motion &&
                                   !safety.request_safe_stop &&
                                   control_.tick(begin);
            const auto finished = Clock::now();
            const bool missed = finished > scheduled + config_.control_period;
            scheduler.completeCycle(finished); // 只跳过后续过期时刻，不补跑。
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.control_ticks;
                if (published) ++stats_.published_commands;
                if (missed) ++stats_.deadline_misses;
                stats_.skipped_cycles = scheduler.skippedCycles();
                stats_.max_start_lateness_us =
                    std::max(stats_.max_start_lateness_us, micros(begin - scheduled));
                stats_.max_control_duration_us =
                    std::max(stats_.max_control_duration_us, micros(finished - begin));
                stats_.invalid_feedback_joints = invalid;
                stats_.feedback_safety_state = safety.state;
                stats_.stale_feedback_joints = safety.stale_joints;
                stats_.warning_feedback_joints = safety.warning_joints;
                stats_.consecutive_critical_ticks = safety.consecutive_critical_ticks;
                stats_.feedback_startup_waiting = safety.startup_waiting;
                stats_.feedback_motion_inhibited = safety.inhibit_motion;
                stats_.max_feedback_age_us =
                    std::max(stats_.max_feedback_age_us, oldest_age_us);
                deadline_.record(missed);
                stats_.deadline_health = deadline_.health();
            }
        }
    } catch (...) {
        failSafe();
    }
}

void RealtimeExecutor::communicationMain() noexcept {
    try {
        while (!stopping_.load()) {
            // 只有本线程访问总线；读一整轮超过 10 ms 不会阻塞控制线程。
            const auto sent = communication_.processCommands(Clock::now());
            if (stopping_.load()) break;
            const auto batch = communication_.pollFeedbackSliceRealTime(config_.feedback_batch_size);
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.sent_servos += sent;
                ++stats_.feedback_batches;
                stats_.feedback_reads += batch.attempted;
                if (batch.completed_round) ++stats_.communication_polls;
            }
            std::unique_lock<std::mutex> lock(wake_mutex_);
            if (wake_cv_.wait_for(lock, config_.communication_pause,
                                  [this] { return stopping_.load(); }))
                break;
        }
    } catch (...) {
        failSafe();
    }
}

} // namespace robot
