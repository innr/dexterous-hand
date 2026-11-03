#pragma once

#include "robot/bus/IBus.hpp"
#include "robot/core/RobotHand.hpp"

#include <cstddef>
#include <memory>

// 教学用同步控制器：持有总线与手部状态，负责裁剪并下发单个关节位置。
// 注意：S4 需要把 I/O 与 100 Hz 控制循环解耦，本类目前是同步阻塞实现。
namespace robot {

class RobotController {
public:
    explicit RobotController(std::unique_ptr<IBus> bus);

    // 裁剪后下发关节位置；ID 非法、角度非有限或总线失败时返回 false。
    bool setJointPosition(int id, float pos);

    // 返回最近一次成功下发的关节位置。
    float commandedPosition(std::size_t id) const;

private:
    RobotHand hand_;
    std::unique_ptr<IBus> bus_;
};

} // namespace robot
