#include "robot/safety/SafetyStateMachine.hpp"
namespace robot {
SafetyState SafetyStateMachine::state() const { return state_; }
void SafetyStateMachine::degrade() {
    if (state_ == SafetyState::NORMAL) state_ = SafetyState::DEGRADED;
}
void SafetyStateMachine::requestStop() {
    if (state_ != SafetyState::FAULT) state_ = SafetyState::SAFE_STOP;
}
void SafetyStateMachine::fault() { state_ = SafetyState::FAULT; }
bool SafetyStateMachine::authorizeRecovery(bool operator_authorized, bool fault_cleared) {
    if (!operator_authorized || !fault_cleared || state_ == SafetyState::NORMAL) return false;
    state_ = SafetyState::NORMAL;
    return true;
}
bool SafetyStateMachine::commandsAllowed() const {
    return state_ == SafetyState::NORMAL || state_ == SafetyState::DEGRADED;
}
} // namespace robot
