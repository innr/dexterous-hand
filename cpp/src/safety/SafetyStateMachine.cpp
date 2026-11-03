#include "robot/safety/SafetyStateMachine.hpp"

namespace robot {

SafetyState SafetyStateMachine::state() const { return state_; }

void SafetyStateMachine::degrade() {
    // 仅允许 NORMAL 降级为 DEGRADED，避免覆盖更严重的状态。
    if (state_ == SafetyState::NORMAL) state_ = SafetyState::DEGRADED;
}

void SafetyStateMachine::requestStop() {
    // 请求停机的优先级低于 FAULT：处于 FAULT 时保持不变。
    if (state_ != SafetyState::FAULT) state_ = SafetyState::SAFE_STOP;
}

void SafetyStateMachine::fault() { state_ = SafetyState::FAULT; }

bool SafetyStateMachine::authorizeRecovery(bool operator_authorized, bool fault_cleared) {
    // 恢复条件：操作员授权 + 故障已清除 + 当前不在 NORMAL 态。
    if (!operator_authorized || !fault_cleared || state_ == SafetyState::NORMAL) return false;
    state_ = SafetyState::NORMAL;
    return true;
}

bool SafetyStateMachine::commandsAllowed() const {
    // 仅 NORMAL / DEGRADED 允许下发控制指令。
    return state_ == SafetyState::NORMAL || state_ == SafetyState::DEGRADED;
}

} // namespace robot
