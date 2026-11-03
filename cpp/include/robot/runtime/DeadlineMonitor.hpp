#pragma once

#include <array>
#include <cstddef>

// Deadline 健康监控：固定容量滑动窗口，O(1) 更新。
namespace robot {
enum class DeadlineHealth { NORMAL, WARNING, CRITICAL };

class DeadlineMonitor {
public:
    static constexpr std::size_t kWindowSize = 100;
    struct Config {
        std::size_t min_samples = 20;
        double warning_rate = 0.10;
        std::size_t critical_consecutive = 5;
    };

    DeadlineMonitor();
    explicit DeadlineMonitor(Config config);

    void record(bool missed) noexcept;
    void reset() noexcept;
    double missRate() const noexcept;
    std::size_t sampleCount() const noexcept { return size_; }
    std::size_t consecutiveMisses() const noexcept { return consecutive_; }
    DeadlineHealth health() const noexcept;

private:
    Config config_;
    std::array<bool, kWindowSize> samples_{};
    std::size_t head_ = 0;
    std::size_t size_ = 0;
    std::size_t missed_ = 0;
    std::size_t consecutive_ = 0;
};
} // namespace robot
