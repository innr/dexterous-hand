// S5-01 安全门禁竞态回归：确定性验证停机线性化点、在途发送与代次。
#include "robot/safety/StopCoordinator.hpp"

#include <atomic>
#include <cassert>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

int main() {
    using robot::StopCoordinator;

    StopCoordinator stop;
    const auto generation = stop.generation();
    int starts = 0;
    assert(!stop.beginSend(generation, [] { return false; }));
    assert(stop.inFlight() == 0);
    auto permit = stop.beginSend(generation, [&] { ++starts; return true; });
    assert(permit && starts == 1 && stop.inFlight() == 1);
    stop.requestStop();
    assert(!stop.beginSend(generation, [&] { ++starts; return true; }));
    assert(starts == 1);
    assert(!stop.recover(true, true)); // 在途发送未完成，不允许恢复。
    permit.reset();
    assert(stop.inFlight() == 0);
    assert(stop.recover(true, true));
    assert(!stop.beginSend(generation, [&] { ++starts; return true; }));

    // 两线程交错：start 回调尚未返回时，停机必须等待门禁线性化点。
    StopCoordinator concurrent;
    std::mutex mutex;
    std::condition_variable cv;
    bool inside_start = false;
    bool allow_start_return = false;
    std::atomic<bool> stop_completed{false};
    std::thread tx([&] {
        auto active = concurrent.beginSend(concurrent.generation(), [&] {
            {
                std::lock_guard<std::mutex> lock(mutex);
                inside_start = true;
            }
            cv.notify_all();
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&] { return allow_start_return; });
            return true;
        });
        assert(active);
        // 模拟 UART 传输已完成，释放在途许可。
    });
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [&] { return inside_start; });
    }
    std::thread stopper([&] {
        concurrent.requestStop();
        stop_completed.store(true);
    });
    {
        std::lock_guard<std::mutex> lock(mutex);
        allow_start_return = true;
    }
    cv.notify_all();
    tx.join();
    stopper.join();
    assert(stop_completed.load());
    assert(!concurrent.beginSend(concurrent.generation(), [] { return true; }));

    // 重复竞争：停机完成后不能再启动任何新的 TX。
    for (int round = 0; round < 200; ++round) {
        StopCoordinator gate;
        const auto g = gate.generation();
        std::atomic<bool> go{false};
        std::thread sender([&] {
            while (!go.load()) std::this_thread::yield();
            auto active = gate.beginSend(g, [] { return true; });
        });
        std::thread breaker([&] {
            while (!go.load()) std::this_thread::yield();
            gate.requestStop();
        });
        go.store(true);
        sender.join();
        breaker.join();
        assert(gate.inFlight() == 0);
        assert(!gate.beginSend(gate.generation(), [] { return true; }));
    }
}
