#include "robot/runtime/ReadOnlyBench.hpp"
#include "robot/runtime/PeriodicScheduler.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <stdexcept>
#include <thread>
#include <utility>

namespace robot {
namespace {
std::int64_t us(ServoClock::duration duration) {
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}
BenchLatency summarize(std::vector<std::int64_t> values) {
    BenchLatency result;
    if (values.empty()) return result;
    std::sort(values.begin(), values.end());
    result.min_us = values.front();
    result.max_us = values.back();
    long double total = 0;
    for (auto value : values) total += value;
    result.mean_us = static_cast<double>(total / values.size());
    // 最近秩定义：ceil(0.95*N)-1，N=1 时为 0。
    const std::size_t index = (95 * values.size() + 99) / 100 - 1;
    result.p95_us = values[index];
    return result;
}
}

BenchReport ReadOnlyBench::run(const ReadRaw& read_raw, BenchConfig config) {
    if (!read_raw || config.rounds == 0 || config.rounds > 10000 ||
        config.control_period <= std::chrono::milliseconds::zero() ||
        config.max_feedback_age <= std::chrono::milliseconds::zero())
        throw std::invalid_argument("invalid read-only benchmark config");

    BenchReport report;
    report.reads.reserve(config.rounds * kServoCount);
    report.round_us.reserve(config.rounds);
    FeedbackBuffer feedback;
    FeedbackSnapshot snapshot;
    std::atomic<bool> done{false};
    std::exception_ptr control_error;
    std::uint64_t skipped = 0;

    // 控制线程只观察数据年龄和调度指标，不下发任何动作。
    std::thread control([&] {
        try {
            PeriodicScheduler scheduler(config.control_period,
                                        ServoClock::now() + config.control_period);
            while (!done.load()) {
                const auto planned = scheduler.nextTime();
                std::this_thread::sleep_until(planned);
                if (done.load()) break;
                const auto started = ServoClock::now();
                const auto data = feedback.read();
                BenchTickSample tick;
                tick.tick = report.ticks.size();
                tick.start_lateness_us = std::max<std::int64_t>(0, us(started - planned));
                for (std::size_t i=0; i<kServoCount; ++i) {
                    if (!data.valid[i] || data.timestamp[i] > started) {
                        ++tick.invalid;
                        continue;
                    }
                    const auto age = us(started - data.timestamp[i]);
                    tick.oldest_age_us = std::max(tick.oldest_age_us, age);
                    if (started - data.timestamp[i] > config.max_feedback_age)
                        ++tick.stale;
                }
                const auto ended = ServoClock::now();
                tick.deadline_miss = ended > planned + config.control_period;
                scheduler.completeCycle(ended);
                report.ticks.push_back(tick);
            }
            skipped = scheduler.skippedCycles();
        } catch (...) {
            control_error = std::current_exception();
            done.store(true);
        }
    });

    try {
        for (std::size_t round=0; round<config.rounds && !done.load(); ++round) {
            const auto round_start = ServoClock::now();
            for (std::size_t joint=0; joint<kServoCount && !done.load(); ++joint) {
                std::uint16_t ticks = 0;
                const auto begin = ServoClock::now();
                bool ok = false;
                try {
                    ok = read_raw(joint, ticks) && ticks <= 4095;
                } catch (...) {
                    ok = false; // 读取异常不应终止其他关节的诊断。
                }
                const auto completed = ServoClock::now();
                BenchReadSample sample;
                sample.round = round;
                sample.joint = joint;
                sample.success = ok;
                sample.raw_ticks = ok ? ticks : 0;
                sample.read_us = us(completed - begin);
                report.reads.push_back(sample);

                // 每路采集完成立即发布，保留各关节独立数据年龄。
                snapshot.valid[joint] = ok;
                snapshot.timestamp[joint] = completed;
                if (ok) {
                    snapshot.position[joint] = static_cast<float>(ticks);
                    ++report.successful_reads;
                    ++report.successful_by_joint[joint];
                } else {
                    ++report.failed_reads;
                    ++report.failed_by_joint[joint];
                }
                feedback.publish(snapshot);
            }
            report.round_us.push_back(us(ServoClock::now() - round_start));
        }
    } catch (...) {
        done.store(true);
        control.join();
        throw;
    }
    done.store(true);
    control.join();
    if (control_error) std::rethrow_exception(control_error);
    report.skipped_control_cycles = skipped;
    std::vector<std::int64_t> read_us;
    read_us.reserve(report.reads.size());
    for (const auto& sample : report.reads) read_us.push_back(sample.read_us);
    report.read_latency = summarize(std::move(read_us));
    report.round_latency = summarize(report.round_us);
    for (const auto& tick : report.ticks) {
        if (tick.deadline_miss) ++report.deadline_misses;
        report.stale_observations += tick.stale;
        report.max_start_lateness_us =
            std::max(report.max_start_lateness_us, tick.start_lateness_us);
        report.max_feedback_age_us =
            std::max(report.max_feedback_age_us, tick.oldest_age_us);
    }
    return report;
}

void ReadOnlyBench::writeCsv(std::ostream& out, const BenchReport& report) {
    out << "type,round,tick,joint,success,raw_ticks,read_us,round_us,"
           "start_lateness_us,oldest_age_us,invalid,stale,deadline_miss\n";
    for (const auto& sample : report.reads)
        out << "read," << sample.round << ",," << sample.joint << ","
            << (sample.success ? 1 : 0) << "," << sample.raw_ticks << ","
            << sample.read_us << ",,,,,,\n";
    for (std::size_t i=0; i<report.round_us.size(); ++i)
        out << "round," << i << ",,,,,," << report.round_us[i] << ",,,,,\n";
    for (const auto& tick : report.ticks)
        out << "tick,," << tick.tick << ",,,,,,"
            << tick.start_lateness_us << "," << tick.oldest_age_us << ","
            << tick.invalid << "," << tick.stale << ","
            << (tick.deadline_miss ? 1 : 0) << "\n";
}
} // namespace robot
