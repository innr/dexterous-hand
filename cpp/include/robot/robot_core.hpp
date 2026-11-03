#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace robot {
constexpr std::size_t kJointCount = 16;

class RobotJoint {
public:
    RobotJoint(int id, float min_deg, float max_deg)
        : id_(id), min_(min_deg), max_(max_deg), position_(0.0f) {
        if (id < 0 || id >= static_cast<int>(kJointCount) ||
            !std::isfinite(min_deg) || !std::isfinite(max_deg) || min_deg > max_deg)
            throw std::invalid_argument("invalid joint limits/id");
        position_ = std::clamp(0.0f, min_, max_);
    }
    int getId() const { return id_; }
    float getPosition() const { return position_; }
    float clamp(float value) const { return std::clamp(value, min_, max_); }
    void setPosition(float value) {
        if (!std::isfinite(value)) throw std::invalid_argument("nonfinite joint command");
        position_ = clamp(value);
    }
private:
    int id_;
    float min_, max_, position_;
};

class RobotHand {
public:
    RobotHand(float min_deg = -90.0f, float max_deg = 90.0f) {
        for (int i = 0; i < static_cast<int>(kJointCount); ++i)
            joints_.push_back(RobotJoint(i, min_deg, max_deg));
    }
    void setPosition(std::size_t id, float pos) { joints_.at(id).setPosition(pos); }
    float getPosition(std::size_t id) const { return joints_.at(id).getPosition(); }
    float clamp(std::size_t id, float pos) const { return joints_.at(id).clamp(pos); }
private:
    std::vector<RobotJoint> joints_;
};

struct IBus {
    virtual ~IBus() = default;
    virtual bool send(int id, float position_deg) = 0;
};

class MockBus final : public IBus {
public:
    bool send_result = true;
    int fail_count = 0;
    int send_count = 0;
    int last_id = -1;
    float last_position = 0.0f;
    bool send(int id, float position_deg) override {
        ++send_count;
        last_id = id;
        last_position = position_deg;
        if (fail_count > 0) { --fail_count; return false; }
        return send_result;
    }
};

// S2 educational synchronous bus controller; not a real-time S4 communication layer.
class RobotController {
public:
    explicit RobotController(std::unique_ptr<IBus> bus)
        : bus_(std::move(bus)) {
        if (!bus_) throw std::invalid_argument("bus is null");
    }
    bool setJointPosition(int id, float pos) {
        if (id < 0 || id >= static_cast<int>(kJointCount) || !std::isfinite(pos)) return false;
        const float limited = hand_.clamp(static_cast<std::size_t>(id), pos);
        if (!bus_->send(id, limited)) return false;
        hand_.setPosition(static_cast<std::size_t>(id), limited);
        return true;
    }
    float commandedPosition(std::size_t id) const { return hand_.getPosition(id); }
private:
    RobotHand hand_;
    std::unique_ptr<IBus> bus_;
};

class PDController {
public:
    PDController(float kp, float kd) : kp_(kp), kd_(kd) {
        if (kp < 0 || kd < 0) throw std::invalid_argument("negative gain");
    }
    float update(float target_pos, float measured_pos, float measured_vel) const {
        return kp_ * (target_pos - measured_pos) - kd_ * measured_vel;
    }
private:
    float kp_, kd_;
};

struct TrajectoryState {
    float position_deg = 0.0f;
    float velocity_deg_s = 0.0f;
};

inline void validateMotion(float limit, float dt) {
    if (!std::isfinite(limit) || limit <= 0.0f || !std::isfinite(dt) || dt <= 0.0f)
        throw std::invalid_argument("motion limit/dt must be positive and finite");
}

inline float calculateStoppingDistance(float velocity, float max_deceleration) {
    validateMotion(max_deceleration, 1.0f);
    return velocity * velocity / (2.0f * max_deceleration);
}
inline float calculateRequiredBrakingDistance(float velocity, float max_deceleration, float dt) {
    validateMotion(max_deceleration, dt);
    return calculateStoppingDistance(velocity, max_deceleration) + std::abs(velocity) * dt;
}
inline float calculateAllowedSpeed(float remaining_distance, float max_velocity, float max_acceleration) {
    validateMotion(max_acceleration, 1.0f);
    validateMotion(max_velocity, 1.0f);
    return std::min(max_velocity, std::sqrt(2.0f * max_acceleration * std::abs(remaining_distance)));
}
inline float calculateDesiredVelocityV2(float current_position, float target_position,
                                         float max_velocity, float max_acceleration) {
    const float delta = target_position - current_position;
    if (delta == 0.0f) return 0.0f;
    return std::copysign(calculateAllowedSpeed(delta, max_velocity, max_acceleration), delta);
}
inline TrajectoryState predictNextState(const TrajectoryState& current, float next_velocity, float dt) {
    validateMotion(1.0f, dt);
    return {current.position_deg + 0.5f * (current.velocity_deg_s + next_velocity) * dt,
            next_velocity};
}
// Narrow predicate: when moving away from target, no *approach braking* is required.
// At target with nonzero speed is explicitly infeasible.
inline bool isBrakingFeasible(const TrajectoryState& state, float target_position, float max_deceleration) {
    validateMotion(max_deceleration, 1.0f);
    const float delta = target_position - state.position_deg;
    if (delta == 0.0f) return state.velocity_deg_s == 0.0f;
    const bool approaching = (delta > 0.0f && state.velocity_deg_s > 0.0f) ||
                             (delta < 0.0f && state.velocity_deg_s < 0.0f);
    return !approaching || calculateStoppingDistance(state.velocity_deg_s, max_deceleration) <= std::abs(delta);
}
inline bool isNextStateBrakingFeasible(const TrajectoryState& current, float next_velocity,
                                       float target_position, float max_deceleration, float dt) {
    return isBrakingFeasible(predictNextState(current, next_velocity, dt), target_position, max_deceleration);
}
inline bool isAccelerationFeasible(float current_velocity, float next_velocity,
                                    float max_acceleration, float dt) {
    validateMotion(max_acceleration, dt);
    return std::abs(next_velocity - current_velocity) <= max_acceleration * dt + 1e-5f;
}
inline bool isCandidateVelocityFeasible(const TrajectoryState& current, float next_velocity,
                                        float target_position, float max_acceleration, float dt) {
    return isAccelerationFeasible(current.velocity_deg_s, next_velocity, max_acceleration, dt) &&
           isNextStateBrakingFeasible(current, next_velocity, target_position, max_acceleration, dt);
}

class AccelerationLimiter {
public:
    explicit AccelerationLimiter(float max_acceleration) : max_acceleration_(max_acceleration) {
        validateMotion(max_acceleration_, 1.0f);
    }
    float update(float desired_velocity, float dt) {
        validateMotion(max_acceleration_, dt);
        const float max_step = max_acceleration_ * dt;
        velocity_ += std::clamp(desired_velocity - velocity_, -max_step, max_step);
        return velocity_;
    }
    float velocity() const { return velocity_; }
private:
    float max_acceleration_;
    float velocity_ = 0.0f;
};

class JointTrajectoryPlanner {
public:
    JointTrajectoryPlanner(float initial_position, float max_velocity, float max_acceleration)
        : max_velocity_(max_velocity), max_acceleration_(max_acceleration),
          limiter_(max_acceleration), state_{initial_position, 0.0f} {
        validateMotion(max_velocity_, 1.0f);
        validateMotion(max_acceleration_, 1.0f);
        if (!std::isfinite(initial_position)) throw std::invalid_argument("invalid initial position");
    }
    TrajectoryState update(float target_position, float dt) {
        validateMotion(max_acceleration_, dt);
        if (!std::isfinite(target_position)) throw std::invalid_argument("invalid target");
        const float desired = calculateDesiredVelocityV2(
            state_.position_deg, target_position, max_velocity_, max_acceleration_);
        const float next_velocity = limiter_.update(desired, dt);
        state_ = predictNextState(state_, next_velocity, dt);
        return state_;
    }
    TrajectoryState state() const { return state_; }
private:
    float max_velocity_, max_acceleration_;
    AccelerationLimiter limiter_;
    TrajectoryState state_;
};

enum class SafetyState { NORMAL, DEGRADED, SAFE_STOP, FAULT };
class SafetyStateMachine {
public:
    SafetyState state() const { return state_; }
    void degrade() { if (state_ == SafetyState::NORMAL) state_ = SafetyState::DEGRADED; }
    void requestStop() { if (state_ != SafetyState::FAULT) state_ = SafetyState::SAFE_STOP; }
    void fault() { state_ = SafetyState::FAULT; }
    bool authorizeRecovery(bool operator_authorized, bool fault_cleared) {
        if (!operator_authorized || !fault_cleared || state_ == SafetyState::NORMAL) return false;
        state_ = SafetyState::NORMAL;
        return true;
    }
    bool commandsAllowed() const { return state_ == SafetyState::NORMAL || state_ == SafetyState::DEGRADED; }
private:
    SafetyState state_ = SafetyState::NORMAL;
};
} // namespace robot
