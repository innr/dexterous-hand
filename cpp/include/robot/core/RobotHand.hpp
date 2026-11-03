#pragma once

#include "robot/core/RobotJoint.hpp"

#include <cstddef>
#include <vector>

// 手部聚合：持有 16 个关节，对外提供按索引的位置读写与裁剪。
namespace robot {

class RobotHand {
public:
    // 构造 16 个关节，所有关节共享同一组角度限位。
    RobotHand(float min_deg = -90.0f, float max_deg = 90.0f);

    // 按关节索引写入位置；越界索引会抛 std::out_of_range。
    void setPosition(std::size_t id, float pos);

    float getPosition(std::size_t id) const;

    // 计算某个关节对给定角度的限位裁剪结果（不写入状态）。
    float clamp(std::size_t id, float pos) const;

private:
    std::vector<RobotJoint> joints_;
};

} // namespace robot
