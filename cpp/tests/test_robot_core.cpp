// 核心单元测试：关节/手/总线/控制器/PD/轨迹数学/规划器/安全状态机。
// 使用 assert，需以 Debug 构建以保留断言。
#include "robot/robot_core.hpp"

#include <cassert>
#include <cmath>
#include <memory>
#include <stdexcept>

int main() {
    constexpr float tol = 1e-4f;

    // --- 关节与手：限位裁剪、越界索引 ---
    robot::RobotJoint j(1, -90, 90);
    j.setPosition(100);
    assert(j.getPosition() == 90);
    j.setPosition(-100);
    assert(j.getPosition() == -90);

    robot::RobotHand hand;
    hand.setPosition(15, 40);
    assert(hand.getPosition(15) == 40);
    bool threw = false;
    try {
        (void)hand.getPosition(16);
    } catch (const std::out_of_range&) {
        threw = true;
    }
    assert(threw);

    // --- 控制器：裁剪后下发、失败处理、状态回读 ---
    auto bus = std::make_unique<robot::MockBus>();
    auto* mock = bus.get();
    robot::RobotController controller(std::move(bus));
    assert(controller.setJointPosition(1, 100)); // 裁剪到 90 后成功
    assert(mock->send_count == 1 && mock->last_position == 90);
    assert(!controller.setJointPosition(16, 10)); // 越界 ID
    mock->fail_count = 1;
    assert(!controller.setJointPosition(2, 25)); // 总线失败
    assert(controller.commandedPosition(2) == 0); // 失败不更新状态
    assert(controller.setJointPosition(2, 25));

    // --- PD 与轨迹数学纯函数 ---
    robot::PDController pd(2, 0.5f);
    assert(std::abs(pd.update(30, 20, 4) - 18) < tol);
    assert(std::abs(robot::calculateStoppingDistance(10, 40) - 1.25f) < tol);
    assert(std::abs(robot::calculateRequiredBrakingDistance(10, 40, 0.01f) - 1.35f) < tol);
    assert(std::abs(robot::calculateAllowedSpeed(1, 20, 40) - std::sqrt(80.0f)) < tol);
    assert(robot::calculateDesiredVelocityV2(60, 20, 20, 40) < 0);
    const robot::TrajectoryState now{24, 10};
    const auto next = robot::predictNextState(now, 9.6f, 0.01f);
    assert(std::abs(next.position_deg - 24.098f) < tol);
    assert(robot::isAccelerationFeasible(10, 9.6f, 40, 0.01f));
    assert(!robot::isAccelerationFeasible(10, 8, 40, 0.01f));
    assert(!robot::isBrakingFeasible(now, 25, 40));
    assert(!robot::isNextStateBrakingFeasible(now, 9.6f, 25, 40, 0.01f));
    assert(!robot::isCandidateVelocityFeasible(now, 9.6f, 25, 40, 0.01f));
    assert(!robot::isBrakingFeasible({25, 10}, 25, 40));

    // --- 规划器：500 步内位置有限、速度与加速度不越界 ---
    robot::JointTrajectoryPlanner planner(60, 20, 40);
    float previous = 0;
    for (int i = 0; i < 500; ++i) {
        const auto s = planner.update(20, 0.01f);
        assert(std::isfinite(s.position_deg));
        assert(std::abs(s.velocity_deg_s) <= 20 + tol);
        assert(std::abs(s.velocity_deg_s - previous) <= 0.4f + tol);
        previous = s.velocity_deg_s;
    }

    // --- 安全状态机：降级、停机、恢复与故障优先级 ---
    robot::SafetyStateMachine safety;
    safety.degrade();
    assert(safety.state() == robot::SafetyState::DEGRADED);
    safety.requestStop();
    assert(!safety.commandsAllowed());
    assert(!safety.authorizeRecovery(false, true));
    assert(safety.authorizeRecovery(true, true));
    safety.fault();
    assert(!safety.authorizeRecovery(true, false));
}
