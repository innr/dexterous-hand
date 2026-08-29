# P2：MediaPipe → LEAP 16DOF C++ 重定向设计

状态：已冻结  
目标平台：Ubuntu 24.04；C++17；可被 ROS 2 Jazzy 或其他上层程序调用

## 目标

将 MediaPipe Hands 风格的 21 个三维关键点转换为 LEAP canonical 顺序的 16
个关节目标角度（单位：弧度），并通过统一的硬件后端接口连接虚拟或真实
STS3215 总线。

## 边界

本任务包含：

- C++17、无摄像头依赖的重定向核心；
- 左右手镜像、关节限位、指数平滑和最大速度限制；
- 无效关键点、低置信度、时间戳错误和帧间隔过大的安全保持；
- `JointBackend` 抽象和 `VirtualSts3215Backend` 端到端测试；
- 使用 `config/joint_mapping.json` 的 16 组 `limit_rad`。

本任务不包含：

- 摄像头和 MediaPipe 进程本身；
- ROS 2 自定义消息或节点重写；
- 真实串口、电源、舵机 ID、方向和零位标定。

真实硬件驱动属于 P3。当前没有硬件，因此只能验证虚拟后端。

## 数据契约

输入为 21 个 `(x, y, z)` 点、置信度、秒级单调时间戳和可选左右手标记。点的
编号遵循 MediaPipe：腕部 0，拇指 1–4，食指 5–8，中指 9–12，无名指 13–16，
小指 17–20。

输出为 16 个 `position_rad`，顺序固定为：食指 0–3、中指 4–7、无名指 8–11、
拇指 12–15。该顺序与现有 `joint_mapping.json` 和 ROS 2 canonical 接口一致；
模块不自行处理 MuJoCo 观测顺序或硬件方向。

## 算法

1. 检查 21 个点是否有限，置信度是否达到 0.6，时间戳是否递增。
2. 使用腕部、中指根部、食指根部和小指根部建立手掌坐标系。
3. 使用解剖学坐标系消除左右手的整体镜像影响；`handedness` 保留在接口中，
   供未来图像坐标适配器在需要时显式反射。
4. 对每根手指计算 MCP 左右角、MCP 前后角、PIP 弯曲角和 DIP 弯曲角；拇指
   使用 CMC 左右角、CMC 前后角、MCP 弯曲角和 IP 弯曲角。
5. 按 `joint_mapping.json` 裁剪到合法范围。
6. 对连续有效帧应用指数平滑和每关节最大速度限制（默认 3 rad/s）。

首次有效输入直接建立目标；之后每帧按时间戳限速。无效输入保持上一安全目标；
第一帧无效时保持 `home_pose`，默认是限位内的零位。

## C++ 接口

```cpp
struct JointCommand {
  std::array<double, 16> position_rad;
  bool valid;
};

// The implementation exposes this as an alias of RetargetOutput and also
// includes reason/timestamp diagnostics.

class JointBackend {
 public:
  virtual void send_position(const std::array<double, 16>& position_rad) = 0;
  virtual JointState read_state() const = 0;
  virtual void disable_torque() = 0;
};
```

`VirtualSts3215Backend` 用于现在的无硬件测试；未来 `SerialSts3215Backend` 可以
在不修改重定向算法的情况下替换它。

## 验收标准

- CMake 可在无 ROS 2、无 MediaPipe、无硬件的环境构建；
- C++ 单元测试覆盖形状、弯曲趋势、左右手镜像、限位、异常保持、限速和虚拟
  后端扭矩关闭；
- 通过 mapping 文件加载 16 组官方限位；
- 输出始终是合法的 16 维弧度数组；
- 现有 Python、ROS 2 和虚拟总线代码不被本任务修改；
- 后续可以由 Python MediaPipe 适配器或 ROS 2 节点提供关键点输入。
