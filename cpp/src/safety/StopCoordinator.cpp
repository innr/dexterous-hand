#include "robot/safety/StopCoordinator.hpp"
#include <utility>
namespace robot {
StopCoordinator::SendPermit::SendPermit(StopCoordinator* owner, std::uint64_t generation)
    : owner_(owner), generation_(generation) {}
StopCoordinator::SendPermit::SendPermit(SendPermit&& other) noexcept
    : owner_(std::exchange(other.owner_, nullptr)), generation_(other.generation_),
      started_(std::exchange(other.started_, false)) {}
StopCoordinator::SendPermit& StopCoordinator::SendPermit::operator=(SendPermit&& other) noexcept {
    if (this != &other) {
        release();
        owner_ = std::exchange(other.owner_, nullptr);
        generation_ = other.generation_;
        started_ = std::exchange(other.started_, false);
    }
    return *this;
}
StopCoordinator::SendPermit::~SendPermit() { release(); }
void StopCoordinator::SendPermit::release() noexcept {
    if (!owner_) return;
    if (started_) {
        std::lock_guard<std::mutex> lock(owner_->mutex_);
        --owner_->in_flight_;
    }
    owner_ = nullptr;
}
bool StopCoordinator::SendPermit::commitStart() {
    if (!owner_ || started_) return false;
    std::lock_guard<std::mutex> lock(owner_->mutex_);
    if (owner_->stopped_ || owner_->generation_ != generation_) return false;
    ++owner_->in_flight_;
    started_ = true;
    return true;
}
std::optional<StopCoordinator::SendPermit> StopCoordinator::tryAcquire(std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_ || generation != generation_) return std::nullopt;
    return SendPermit(this, generation);
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
