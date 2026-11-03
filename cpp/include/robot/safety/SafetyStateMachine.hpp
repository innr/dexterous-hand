#pragma once
namespace robot {
enum class SafetyState { NORMAL, DEGRADED, SAFE_STOP, FAULT };
class SafetyStateMachine {
public:
    SafetyState state() const;
    void degrade();
    void requestStop();
    void fault();
    bool authorizeRecovery(bool operator_authorized, bool fault_cleared);
    bool commandsAllowed() const;
private:
    SafetyState state_ = SafetyState::NORMAL;
};
} // namespace robot
