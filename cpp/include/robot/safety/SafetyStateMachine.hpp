#pragma once

// 安全状态机：NORMAL / DEGRADED / SAFE_STOP / FAULT 四态，决定控制指令是否放行。
namespace robot {

enum class SafetyState { NORMAL, DEGRADED, SAFE_STOP, FAULT };

class SafetyStateMachine {
public:
    SafetyState state() const;

    // NORMAL -> DEGRADED（仅从 NORMAL 降级）。
    void degrade();

    // 请求安全停机；FAULT 优先级最高，不会被覆盖。
    void requestStop();

    // 进入故障态。
    void fault();

    // 仅在操作员授权且故障已清除时，才允许从非 NORMAL 态恢复为 NORMAL。
    bool authorizeRecovery(bool operator_authorized, bool fault_cleared);

    // 只有 NORMAL 与 DEGRADED 两种状态允许下发控制指令。
    bool commandsAllowed() const;

private:
    SafetyState state_ = SafetyState::NORMAL;
};

} // namespace robot
