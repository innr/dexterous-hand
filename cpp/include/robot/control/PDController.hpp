#pragma once

// 位置式 PD 控制器：输出 = kp*(目标-实际) - kd*实际速度。
namespace robot {

class PDController {
public:
    // kp、kd 必须为有限非负值，否则抛异常。
    PDController(float kp, float kd);

    // 根据目标位置、当前测量位置与速度计算控制量。
    float update(float target_pos, float measured_pos, float measured_vel) const;

private:
    float kp_;
    float kd_;
};

} // namespace robot
