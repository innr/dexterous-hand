// 演示程序：用 Mock 总线驱动一个关节，串联“轨迹规划 + PD + 安全状态机 + 控制器”。
#include "robot/robot_core.hpp"

#include <iostream>
#include <memory>

int main() {
    auto bus = std::make_unique<robot::MockBus>();
    robot::RobotController controller(std::move(bus));

    // 规划器：起点 20°，最大速度 20°/s，最大加速度 40°/s²。
    robot::JointTrajectoryPlanner planner(20.0f, 20.0f, 40.0f);
    robot::PDController pd(2.0f, 0.1f);
    robot::SafetyStateMachine safety;

    // 以 10ms 步长运行 100 步（约 1 秒），目标角度 60°。
    for (int i = 0; i < 100; ++i) {
        const auto planned = planner.update(60.0f, 0.01f);
        const float effort = pd.update(planned.position_deg, 20.0f, 0.0f);

        // 安全状态允许时下发指令；下发失败则进入故障态。
        if (safety.commandsAllowed() && !controller.setJointPosition(0, planned.position_deg))
            safety.fault();

        // 每 20 步打印一次轨迹与 PD 输出。
        if (i % 20 == 0)
            std::cout << "step=" << i << " plan=" << planned.position_deg
                      << " vel=" << planned.velocity_deg_s << " pd=" << effort << '\n';
    }
}
