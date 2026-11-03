#pragma once
// S3 | Simplified safety-mode state machine for unit testing.
// FAULT is latched until authorized recovery and cleared fault are provided.
// This is not a hardware emergency-stop implementation or a thread-safe gate.
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
