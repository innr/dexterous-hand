#pragma once
#include "robot/runtime/FeedbackBuffer.hpp"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <vector>

// S5-04：只读台架测试。通信轮询与 100Hz 观察线程并行，禁止任何运动写入。
namespace robot {
struct BenchConfig {
    std::size_t rounds = 100;
    std::chrono::milliseconds control_period{10};
    std::chrono::milliseconds max_feedback_age{20};
};

struct BenchReadSample {
    std::size_t round = 0;
    std::size_t joint = 0;
    bool success = false;
    std::uint16_t raw_ticks = 0;
    std::int64_t read_us = 0;
};
struct BenchTickSample {
    std::size_t tick = 0;
    std::int64_t start_lateness_us = 0;
    std::int64_t oldest_age_us = -1;
    std::size_t invalid = 0;
    std::size_t stale = 0;
    bool deadline_miss = false;
};
struct BenchLatency {
    std::int64_t min_us = 0;
    double mean_us = 0;
    std::int64_t p95_us = 0;
    std::int64_t max_us = 0;
};
struct BenchReport {
    std::vector<BenchReadSample> reads;
    std::vector<BenchTickSample> ticks;
    std::vector<std::int64_t> round_us;
    std::array<std::uint64_t,kServoCount> successful_by_joint{};
    std::array<std::uint64_t,kServoCount> failed_by_joint{};
    std::uint64_t successful_reads = 0;
    std::uint64_t failed_reads = 0;
    std::uint64_t deadline_misses = 0;
    std::uint64_t skipped_control_cycles = 0;
    std::uint64_t stale_observations = 0;
    std::int64_t max_start_lateness_us = 0;
    std::int64_t max_feedback_age_us = -1;
    BenchLatency round_latency{};
    BenchLatency read_latency{};
};

class ReadOnlyBench {
public:
    // 回调只允许读取当前位置原始 tick；不得写目标位置、扭矩或 EEPROM。
    using ReadRaw = std::function<bool(std::size_t joint, std::uint16_t& ticks)>;
    static BenchReport run(const ReadRaw& read_raw, BenchConfig config = {});
    static void writeCsv(std::ostream& out, const BenchReport& report);
};
} // namespace robot
