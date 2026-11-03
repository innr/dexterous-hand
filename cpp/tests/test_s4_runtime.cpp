// S4 运行时单元测试：使用虚拟时间点验证边界，不依赖实际 sleep 精度。
#include "robot/runtime/PeriodicScheduler.hpp"
#include "robot/runtime/DeadlineMonitor.hpp"

#include <cassert>
#include <chrono>
#include <stdexcept>

int main() {
    using namespace std::chrono;
    using Clock = robot::PeriodicScheduler::Clock;
    const Clock::time_point origin{};
    const auto ms = [](int n) { return milliseconds(n); };

    bool invalid = false;
    try { robot::PeriodicScheduler bad(ms(0), origin); }
    catch (const std::invalid_argument&) { invalid = true; }
    assert(invalid);

    robot::PeriodicScheduler scheduler(ms(10), origin + ms(30));
    assert(scheduler.advance(origin + ms(29)) == origin + ms(30));
    assert(scheduler.skippedCycles() == 0);
    assert(scheduler.advance(origin + ms(30)) == origin + ms(40));
    assert(scheduler.skippedCycles() == 1);
    scheduler.reset(origin + ms(30));
    assert(scheduler.advance(origin + ms(52)) == origin + ms(60));
    assert(scheduler.skippedCycles() == 3);
    scheduler.reset();
    assert(scheduler.skippedCycles() == 0);

    robot::DeadlineMonitor monitor;
    assert(monitor.missRate() == 0);
    for (int i = 0; i < 5; ++i) monitor.record(i < 2);
    assert(monitor.missRate() == 0.4);
    assert(monitor.health() == robot::DeadlineHealth::NORMAL);
    monitor.reset();
    for (int i = 0; i < 20; ++i) monitor.record(i == 0 || i == 5 || i == 10);
    assert(monitor.health() == robot::DeadlineHealth::WARNING);
    monitor.reset();
    for (int i = 0; i < 5; ++i) monitor.record(true);
    assert(monitor.health() == robot::DeadlineHealth::CRITICAL);
    monitor.record(false);
    assert(monitor.consecutiveMisses() == 0);

    monitor.reset();
    for (int i = 0; i < 100; ++i) monitor.record(i < 10);
    assert(monitor.missRate() == 0.10);
    monitor.record(false);
    assert(monitor.missRate() == 0.09);

    invalid = false;
    try { robot::DeadlineMonitor bad({0, 0.1, 5}); }
    catch (const std::invalid_argument&) { invalid = true; }
    assert(invalid);
}
