#pragma once
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

// 统一发送门禁：停机后不再授权新的发送；在途发送仍须等待完成。
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
        bool commitStart();
    private:
        friend class StopCoordinator;
        SendPermit(StopCoordinator* owner, std::uint64_t generation);
        void release() noexcept;
        StopCoordinator* owner_;
        std::uint64_t generation_;
        bool started_ = false;
    };
    std::optional<SendPermit> tryAcquire(std::uint64_t generation);
    void requestStop();
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
