// S1-S3 demo | Mock transport and trajectory stepping; NOT real 100 Hz scheduling.
#include "robot/robot_core.hpp"
#include <iostream>
#include <memory>
int main() {
    auto bus = std::make_unique<robot::MockBus>();
    robot::RobotController controller(std::move(bus));
    robot::JointTrajectoryPlanner planner(20.0f, 20.0f, 40.0f);
    robot::PDController pd(2.0f, 0.1f);
    robot::SafetyStateMachine safety;
    for (int i = 0; i < 100; ++i) {
        const auto planned = planner.update(60.0f, 0.01f);
        const float effort = pd.update(planned.position_deg, 20.0f, 0.0f);
        if (safety.commandsAllowed() && !controller.setJointPosition(0, planned.position_deg))
            safety.fault();
        if (i % 20 == 0)
            std::cout << "step=" << i << " plan=" << planned.position_deg
                      << " vel=" << planned.velocity_deg_s << " pd=" << effort << '\n';
    }
}
