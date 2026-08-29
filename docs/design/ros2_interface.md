# ROS 2 统一关节接口设计

状态：已冻结  
任务：P1 — ROS 2 统一接口  
目标平台：Ubuntu 24.04 + ROS 2 Jazzy

## 范围

为 LEAP Hand 提供统一的 canonical 16 关节接口，后端先支持无硬件运行的
virtual backend；MuJoCo 和真实 STS3215 后端沿用现有映射与安全逻辑。

本任务不包含摄像头重定向、真实串口标定、ros2_control 或自定义消息包。

## 接口

节点名：`leap_hand_node`

| 接口 | 类型 | 约定 |
|---|---|---|
| `~/joint_command` | `sensor_msgs/msg/JointState` | 只使用 position；name 为空或必须匹配 canonical 关节顺序 |
| `~/joint_states` | `sensor_msgs/msg/JointState` | 发布 canonical name、position、velocity |
| `~/set_torque` | 预留 | 后续接入硬件时使用标准 SetBool |

参数：

- `mapping_path`：默认 `config/joint_mapping.json`
- `backend`：默认 `virtual`
- `publish_rate_hz`：默认 50
- `command_timeout_s`：默认 0.25 秒
- `timeout_action`：`disable_torque`（默认）或 `hold`
- `allow_uncalibrated`：默认 false

命令校验要求长度为 16、数值有限、名称正确且在关节限位内。超时后停止接受
过期运动目标；`disable_torque` 调用后端关闭扭矩，`hold` 保持最后状态。

## 目录

`ros2_ws/src/dexterous_hand_ros/` 下包含标准 ament_python 包、节点、后端、
校验模块和测试。ROS 2 未安装时，校验与虚拟后端仍可由普通 pytest 运行。

## 验收标准

- virtual 后端可完成 command → state 闭环；
- 非法命令不会到达后端；
- 超时策略可测试且默认关闭扭矩；
- ROS 2 Jazzy 环境可用 `colcon test` 构建；
- 现有 Python 测试保持通过。
