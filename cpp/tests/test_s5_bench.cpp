// S5-04：模拟 16 路慢轮询、故障注入、反馈过期与 CSV 格式。
#include "robot/runtime/ReadOnlyBench.hpp"
#include <cassert>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <thread>

int main() {
    using namespace robot;
    using namespace std::chrono;
    BenchConfig cfg;
    cfg.rounds=5;
    cfg.control_period=milliseconds(5);
    cfg.max_feedback_age=milliseconds(10);
    std::size_t calls=0;
    const auto report=ReadOnlyBench::run([&](std::size_t joint,std::uint16_t& ticks) {
        ++calls;
        std::this_thread::sleep_for(milliseconds(2));
        if (joint==3) return false;
        ticks=static_cast<std::uint16_t>(2048+joint);
        return true;
    },cfg);
    assert(calls==80);
    assert(report.reads.size()==80 && report.round_us.size()==5);
    assert(report.successful_reads==75 && report.failed_reads==5);
    assert(report.failed_by_joint[3]==5 && report.successful_by_joint[0]==5);
    assert(report.reads[0].success && report.reads[0].raw_ticks==2048);
    assert(report.round_latency.min_us>=25000);
    assert(report.ticks.size()>5); // 整轮约 32ms，但控制观察线程仍以 5ms 运行。
    assert(report.max_feedback_age_us>=0);
    assert(report.stale_observations>0);
    assert(report.read_latency.p95_us>=report.read_latency.min_us);
    assert(report.round_latency.max_us>=report.round_latency.p95_us);
    std::ostringstream csv;
    ReadOnlyBench::writeCsv(csv,report);
    assert(csv.str().find("type,round,tick,joint") == 0);
    assert(csv.str().find("read,0,,0,1,2048,") != std::string::npos);
    assert(csv.str().find("round,0,") != std::string::npos);
    assert(csv.str().find("tick,,") != std::string::npos);

    bool invalid=false;
    try {
        ReadOnlyBench::run({},cfg);
    } catch (const std::invalid_argument&) { invalid=true; }
    assert(invalid);
    cfg.rounds=0;
    invalid=false;
    try {
        ReadOnlyBench::run([](std::size_t,std::uint16_t&) {return true;},cfg);
    } catch (const std::invalid_argument&) { invalid=true; }
    assert(invalid);
}
