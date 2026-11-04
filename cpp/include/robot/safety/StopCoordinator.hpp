#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>

// 发送门禁：停机状态检查与非阻塞 TX 启动共享一个线性化点。
// 注意：startSend 回调必须立即启动传输且有界、非阻塞，不得重入本类。
namespace robot {
class StopCoordinator {
public:
    class SendPermit {
    public:
        SendPermit(const SendPermit&) = delete;
        SendPermit& operator=(const SendPermit&) = delete;
        SendPermit(SendPermit&& other) noexcept;
        SendPermit& operator=(SendPermit&& other) noexcept;
        ~SendPermit();
    private:
        friend class StopCoordinator;
        explicit SendPermit(StopCoordinator* owner) noexcept;
        void release() noexcept;
        StopCoordinator* owner_;
    };

    // 原子检查 generation + 启动一次 TX；失败不创建在途许可。
    // start 回调不得阻塞、抛异常、排队后才启动，也不得调用 StopCoordinator。
    std::optional<SendPermit> beginSend(std::uint64_t generation,
                                        const std::function<bool()>& start);
    // 请求停机只禁止后续启动，不等待已在途的 UART 传输。
    void requestStop();
    // 恢复前必须确认所有已启动传输已完成。
    bool recover(bool operator_authorized, bool fault_cleared);
    std::uint64_t generation() const;
    bool commandsAllowed() const;
    std::size_t inFlight() const;
private:
    friend class SendPermit;
    mutable std::mutex mutex_;
    bool stopped_ = false;
    std::uint64_t generation_ = 1;
    std::size_t in_flight_ = 0;
};
} // namespace robot
