#pragma once

// 单个关节模型：保存关节 ID、角度限位与当前位置，并负责限位裁剪。
namespace robot {

// 项目约定固定 16 个关节，与 LEAP Hand 的 16 DOF 对应。
inline constexpr int kJointCount = 16;

class RobotJoint {
public:
    // 构造时校验 ID 与限位；初始位置会被裁剪到 [min_deg, max_deg]。
    RobotJoint(int id, float min_deg, float max_deg);

    int getId() const;

    float getPosition() const;

    // 将任意角度裁剪到本关节的限位范围内（不改变内部状态）。
    float clamp(float value) const;

    // 设置目标位置：非有限值抛异常，越界值会被裁剪。
    void setPosition(float value);

private:
    int id_;
    float min_;
    float max_;
    float position_;
};

} // namespace robot
