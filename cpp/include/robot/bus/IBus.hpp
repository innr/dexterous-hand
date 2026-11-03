#pragma once

// 总线抽象：控制器只依赖此接口，真实舵机总线与 Mock 总线均可替换实现。
namespace robot {

class IBus {
public:
    virtual ~IBus() = default;

    // 向指定 ID 的舵机下发目标角度；成功返回 true。
    virtual bool send(int id, float position_deg) = 0;
};

} // namespace robot
