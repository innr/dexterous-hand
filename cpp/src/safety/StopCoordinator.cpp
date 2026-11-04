#include "robot/safety/StopCoordinator.hpp"

#include <utility>

namespace robot {
StopCoordinator::SendPermit::SendPermit(StopCoordinator* owner) noexcept : owner_(owner) {}

StopCoordinator::SendPermit::SendPermit(SendPermit&& other) noexcept
    : owner_(std::exchange(other.owner_, nullptr)) {}

StopCoordinator::SendPermit& StopCoordinator::SendPermit::operator=(SendPermit&& other) noexcept {
    if (this != &other) {
        release();
        owner_ = std::exchange(other.owner_, nullptr);
    }
    return *this;
}

StopCoordinator::SendPermit::~SendPermit() { release(); }

void StopCoordinator::SendPermit::release() noexcept {
    if (!owner_) return;
    std::lock_guard<std::mutex> lock(owner_->mutex_);
    --owner_->in_flight_;
    owner_ = nullptr;
}

std::optional<StopCoordinator::SendPermit> StopCoordinator::beginSend(
        std::uint64_t generation, const std::function<bool()>& start) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_ || generation != generation_ || !start) return std::nullopt;

    // start() 必须是有界、非阻塞的真实 TX 启动操作。
    // 此锁保证 requestStop() 不会在线性化点之前完成。
    if (!start()) return std::nullopt;
    ++in_flight_;
    return SendPermit(this);
}

void StopCoordinator::requestStop() {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    ++generation_;
}

bool StopCoordinator::recover(bool operator_authorized, bool fault_cleared) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stopped_ || !operator_authorized || !fault_cleared || in_flight_ != 0) return false;
    ++generation_;
    stopped_ = false;
    return true;
}

std::uint64_t StopCoordinator::generation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
}

bool StopCoordinator::commandsAllowed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !stopped_;
}

std::size_t StopCoordinator::inFlight() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return in_flight_;
}
} // namespace robot
