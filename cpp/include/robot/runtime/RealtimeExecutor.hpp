#pragma once

#include "robot/runtime/DeadlineMonitor.hpp"
#include "robot/runtime/ControlTelemetry.hpp"
#include "robot/control/MotionControlCore.hpp"
#include "robot/runtime/FeedbackBuffer.hpp"
#include "robot/runtime/S4Pipeline.hpp"
#include "robot/safety/StopCoordinator.hpp"
#include "robot/safety/FeedbackSafetyMonitor.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

// S5-02：控制与通信双线程运行器。线程生命周期由运行器独占管理。
namespace robot {

struct RuntimeStats {
    std::uint64_t control_ticks = 0;
    std::uint64_t published_commands = 0;
    std::uint64_t deadline_misses = 0;
    std::uint64_t skipped_cycles = 0;
    std::uint64_t communication_polls = 0; // 完整 16 路轮询次数。
    std::uint64_t feedback_batches = 0; // 分片执行次数。
    std::uint64_t feedback_reads = 0; // 尝试读取的舵机总数。
    std::uint64_t sent_servos = 0;
    std::uint64_t worker_faults = 0;
    std::int64_t max_start_lateness_us = 0;
    std::int64_t max_control_duration_us = 0; // 控制计算阶段，不含日志入队与统计更新。
    std::int64_t max_full_tick_duration_us = 0; // 含遥测入队与统计更新，作为 deadline 依据。
    std::int64_t max_feedback_age_us = -1; // -1 表示尚无有效反馈。
    std::size_t invalid_feedback_joints = kServoCount;
    DeadlineHealth deadline_health = DeadlineHealth::NORMAL;
    SafetyState feedback_safety_state = SafetyState::NORMAL;
    std::size_t stale_feedback_joints = 0;
    std::size_t warning_feedback_joints = 0;
    std::size_t consecutive_critical_ticks = 0;
    bool feedback_startup_waiting = false;
    bool feedback_motion_inhibited = false;
};

class RealtimeExecutor {
public:
    struct Config {
        std::chrono::milliseconds control_period{10}; // 100 Hz
        std::chrono::milliseconds communication_pause{1};
        std::size_t feedback_batch_size = 4; // 每批最多 4 路，优先给新命令发送机会。
        FeedbackSafetyConfig feedback_safety{}; // 默认启用；测试可显式关闭。
        MotionControlConfig motion{}; // 默认开启位置外环 PD + 双重轨迹限制。
        ControlTelemetryRing* telemetry = nullptr; // 可选；调用方保证比运行器存活更久。
    };

    RealtimeExecutor(ControlLoop& control, CommunicationWorker& communication,
                     FeedbackBuffer& feedback, StopCoordinator& stop);
    RealtimeExecutor(ControlLoop& control, CommunicationWorker& communication,
                     FeedbackBuffer& feedback, StopCoordinator& stop, Config config);
    ~RealtimeExecutor();

    RealtimeExecutor(const RealtimeExecutor&) = delete;
    RealtimeExecutor& operator=(const RealtimeExecutor&) = delete;

    // 单次启动：停止后禁止自动重启，必须由更高层完成授权恢复与重新建对象。
    bool start();
    // 幂等：先锁存停机，再唤醒周期等待并 join；通信读操作需保证有界超时。
    void stop();
    bool running() const noexcept;
    RuntimeStats stats() const;

private:
    void controlMain() noexcept;
    void communicationMain() noexcept;
    void failSafe() noexcept;

    ControlLoop& control_;
    CommunicationWorker& communication_;
    FeedbackBuffer& feedback_;
    StopCoordinator& stop_;
    Config config_;

    mutable std::mutex lifecycle_mutex_;
    std::atomic<bool> started_once_{false};
    std::thread control_thread_;
    std::thread communication_thread_;
    std::atomic<bool> stopping_{false};

    std::mutex wake_mutex_;
    std::condition_variable wake_cv_;

    mutable std::mutex stats_mutex_;
    RuntimeStats stats_;
    DeadlineMonitor deadline_;
    FeedbackSafetyMonitor feedback_safety_; // 仅由控制线程调用。
    std::uint64_t telemetry_tick_ = 0; // 仅控制线程写。
};

} // namespace robot
