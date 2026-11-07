# Dexterous Hand System Architecture

本文是 **S1–S6 已实现基线** 与 **S7 之前契约** 的权威系统图。  
冻结合同：`docs/designs/s7_00_contract_convergence.md`（Frozen，2026-11-07）。  
更早的 Python-first / Design 001–008 / PR #1 merge gate 叙述已过期，见文末 Historical。

## 1. Purpose

栈负责表示、命令、仿真、观测一台 16-DOF LEAP 兼容灵巧手，并在标定与硬件门禁满足之前 **禁止普通真机运动**。

软件边界包括：

- 官方 Menagerie LEAP 右手 MJCF 仿真（S6）；
- C++17 S4–S5 运行时（100 Hz 控制 / 通信拆分 / TX 门禁 / STS3215 协议）；
- 进程外 `s6_control_bridge` 文本协议；
- ROS2 Jazzy 仿真命令与 lifecycle 监督（仅 Linux CI）；
- STS3215 只读工具与未使能的 C++ 传输。

物理标定、E-stop、16 路带载闭环在软件-only 验收之外（H-01 / H-02 / H-03）。S7 运动仍是 **NO-GO**。

## 2. Layer labels

文档与新代码必须使用这些标签，不要再发明第三套分层名。

| 标签 | 含义 | 权威代码 | 在 S6 路径上？ |
|---|---|---|---|
| S1–S3 teaching | 同步教学核心 | `RobotController`, `IBus`, `RobotHand` | 否 |
| S4–S5 runtime | 异步控制 / 通信 / 安全 / STS 协议 | `S4Pipeline.hpp`, `RealtimeExecutor`, `StopCoordinator`, `Sts3215*` | 是（经桥） |
| S6 sim adapter | Python 植物 + ROS2 + 文本桥 | `s6_official_bridge.py`, `s6_ros2_node.py`, `s6_control_bridge.cpp` | 是 |
| S7 hardware | 真实总线；本阶段运动 NO-GO | `sts3215_readonly_probe`, `Sts3215Transport` (`allow_motion=false`) | 否（不得从桥构造） |

`robot/robot_core.hpp` 只聚合 S1–S3 头。新代码包含具体模块头。

## 3. Current system (as built)

```text
ROS2 controller (Python, 50 Hz)
  leap_interfaces/msg/JointCommand
  rad, epoch, sequence, 16 Menagerie names (until S7-00 PR-4b)
        |
LeapSimNode / SimSession     独立 supervisor 租约 / arm
  READY / NORMAL / PAUSED / SAFE_STOP / FAULT
        |
stdin/stdout  s6_control_bridge
  rad <-> deg   READY 16 S6-01 / STEP / OK / STOP
        |
C++ S5 runtime
  ControlLoop -> MotionControlCore (deg)
  -> CommandMailbox -> CommunicationWorker
  -> StopCoordinator::beginSend -> IServoTransport
        |
OfficialPlant (Menagerie MJCF) -> 5 x mj_step(2 ms)
```

同一 STATIC 库 `robot_core` 仍含教学栈（`IBus` / `RobotController`）。S6/S7 运动不得调用 `IBus::send`。

`hardware/sts3215.py` 是 **tools-only 第二写路径**（扫描、零位采集；默认不使能扭矩）。运动 PR 不得调用它。S4–S5 运行时路径上唯一 `startSend` 线性化点是 `StopCoordinator::beginSend`。

## 4. Canonical 16-DOF contract

应用面（ROS2、Python 校验、未来重定向/录制）位置一律 **radian**，canonical 名，顺序 `0..15`：

```text
 0 index_mcp_side        1 index_mcp_forward     2 index_pip       3 index_dip
 4 middle_mcp_side       5 middle_mcp_forward    6 middle_pip      7 middle_dip
 8 ring_mcp_side         9 ring_mcp_forward     10 ring_pip       11 ring_dip
12 thumb_cmc_side       13 thumb_cmc_forward    14 thumb_mcp      15 thumb_ip
```

名字来源：`config/joint_mapping.json` 的 `joints[].name`。  
Menagerie MJCF 名（`if_rot` 等）与 STS3215 ID 只在后端适配。当前 ROS2 命令仍发 MJCF 名，直到 S7-00 **PR-4b**。

单位边界：

| 边界 | 单位 |
|---|---|
| `CanonicalCommand` / ROS2 命令与 `/joint_states` | rad |
| `s6_control_bridge` STEP / OK | rad（桥内换成 deg） |
| `ControlLoop` / `MotionControlCore` / mailbox / `IServoTransport` | deg |
| `ControlTelemetryFrame` / S5 CSV / S5 CTest | deg |
| `Sts3215Transport` 寄存器 | tick |

S6 路径上 **只有** 桥做 rad↔deg。硬件路径上 **只有** `Sts3215Transport` 做 deg↔tick。不要在 `MotionControlCore` 内部换算。

`session_id` 是进程 boot nonce，不是 `epoch`。S6-R01（跨进程命令重放）在 PR-4a 的 unmanaged CI 测试落地前 **仍开放**。

细节与类型布局见 `docs/designs/s7_00_contract_convergence.md`。

## 5. Mapping

| 文件 | 角色 |
|---|---|
| `config/joint_mapping.json` | 身份源：canonical 名、`hardware_id`、`limit_rad`、方向/零位占位、`calibration.status` |
| `config/menagerie_right_mapping.json` | 仿真适配：canonical → MJCF joint / actuator |

两文件暂不合并。`OfficialPlant` 必须两份都加载，按 `canonical == name` 连接（S7-00 PR-3）。

`calibration.status` 现为 `not_calibrated`。未标定 ⇒ 禁止普通运动。本系列不得改写仓库内该字段。

C++ 桥发现顺序（S7-00 PR-3）：`LEAP_JOINT_MAPPING`，否则 `${LEAP_PROJECT_ROOT}/config/joint_mapping.json`。禁止相对可执行文件 / cwd 搜索。Python `ControlBridgeProcess` 在环境为空时注入 `LEAP_PROJECT_ROOT`。

## 6. Safety vocabulary (do not merge implementations)

| 词汇 | 所有者 | 权威含义 |
|---|---|---|
| `Supervisor.mode` | `scripts/s6_ros2_supervisor.py` | 进程监督；BOOTING / ACTIVE / PAUSED / LATCHED |
| `LeapSimNode.armed` | `simulation/s6_ros2_node.py` | 是否接受 DDS 命令 |
| `SimSession.state` | `simulation/s6_official_bridge.py` | 仿真会话；READY / NORMAL / PAUSED / SAFE_STOP / FAULT |
| `SafetyStateMachine` | 仅 `FeedbackSafetyMonitor` 内部 | 反馈年龄分级 |
| `StopCoordinator` | C++ 运行时 | **唯一** 是否允许 `startSend` |

Python `armed` 不是 TX 许可。物理 E-stop / 断扭矩 **不存在**（H-01）；仿真 `SAFE_STOP` 不是硬件卸载。

## 7. Dependency direction

```text
applications / ROS2 / (future retargeting / replay)
                 |
                 v
       canonical command (rad) + session/epoch/seq
                 |
                 v
       S6 adapter (SimSession, OfficialPlant, text bridge)
                 |
                 v
       S4–S5 runtime (deg) + StopCoordinator
          /                    \
         v                      v
   SimulatedServoTransport   Sts3215Transport (motion disabled)
```

低层传输不得依赖 ROS2、MediaPipe 或数据集代码。

## 8. Invariants

- 未标定硬件不得接受普通运动命令；`allow_motion` 默认 false。
- 使能扭矩必须显式，且受标定策略约束。本系列所有调用点 `operator_allow_motion=false`。
- 应用面位置：canonical 顺序、弧度。
- 后端换算只发生在声明的边界。
- CI 绿 ≠ 硬件安全。
- 不要把 `IBus` 接到 `Sts3215Transport`。不要从 S6 桥构造 `Sts3215Transport`。

## 9. Platform

| 平台 | 范围 |
|---|---|
| Windows 开发机 | 文档、`robot_core` 编译、Python 单测；无真实串口；无 ROS2 作为默认环境 |
| Linux + ROS2 Jazzy | DDS、lifecycle supervisor、S6 integration CI |
| Linux 串口 | `PosixHalfDuplexPort`（1 Mbps + TEMT）；未在目标适配器上验收 |
| Windows COM | **不存在** |

## 10. Implementation record (current)

| 阶段 | 文档 |
|---|---|
| S4 | `docs/designs/s4_realtime_control.md` |
| S5 | `docs/designs/s5_01_safe_send.md` … `s5_09_control_telemetry.md` |
| S6 | `s6_01`–`s6_04`（`s6_02` 命令面已被 S6-03 取代） |
| 验收 | `docs/acceptance/s5_stage_acceptance_2026_11_05.md`、`s6_stage_acceptance_2026_11_05.md` |
| 下一契约 | `docs/designs/s7_00_contract_convergence.md` |

S6 软件/仿真阶段条件通过。S7-01 只读台架可准备。单关节使能与 16 路带载闭环 **NO-GO**，直到 H-01～H-03 与 S6-R01～R03 的硬件对应项关闭。

## 11. Ordered work after this baseline

见 S7-00 PR Plan：**文档（本文件所属）→ Canonical 类型 → JointTable 限位 → session_id → canonical 名 → STS3215 黄金向量**。

仍开放、不得假装关闭：S6-R01（至 PR-4a CI）、S6-R02、S6-R03、H-01、H-02、H-03。

未来、**非本系列**：MediaPipe 重定向、LeRobot/HDF5、物理调试与标定运动（原 backlog 006–008）。

## 12. Historical (do not implement against)

以下内容保留为史实，**不是当前 backlog**：

- Design 001–005（MuJoCo PD、safety policy、joint mapping、virtual bus、ROS2 interface）已被 S4–S6 + S7-00 取代。
- Design 006–008（MediaPipe、recording/replay、hardware commissioning）**仍未实现**，也不是 S7-00 系列的任务。
- PR #1 merge gate（“001/002/003 Frozen 之前不得合入 PR #1”）是早期流程。当前 `main` 已含 S5/S6。不要再把 PR #1 当作 in-flight 基线。
- 根 `README.md` 的 2026-08-25 硬件 BOM / 遥操作路线图是项目准备文档，不是运行时架构。
