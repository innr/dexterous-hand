# S6 阶段总验收 — 2026-11-05

> 基线：`main@1ac4b7141cf0a1f9ba59397d596437e6c61c4691`（PR #35 已合并）。
>
> **结论：S6-01～S6-04 软件/仿真阶段条件通过；S7 只读硬件台架准备 GO；真实 STS3215 使能运动、16 路带载闭环 NO-GO。**
>
> 证据类型：GitHub 仓库源码审查 + 已完成的 GitHub Actions 日志复核。
> 本次未在本地重跑 ROS2/MuJoCo、未连接 STS3215、未进行真实物理 E-stop 测量。
> “21/21”是 CTest 用例数，不是代码覆盖率或安全证明。

## 1. 验收证据

- S6-01 [PR #32](https://github.com/innr/dexterous-hand/pull/32)：MuJoCo surrogate + C++17 控制核心。
- S6-02 [PR #33](https://github.com/innr/dexterous-hand/pull/33)：官方 Menagerie LEAP right-hand MJCF + 16 路语义映射。
- S6-03 [PR #34](https://github.com/innr/dexterous-hand/pull/34)：ROS2 Jazzy 真实 DDS 双进程 + `leap_interfaces/msg/JointCommand`。
- S6-04 [PR #35](https://github.com/innr/dexterous-hand/pull/35)：LifecycleNode + 外部 supervisor + 三类 SIGKILL 故障注入。
- [main C++ CI #41994588291](https://github.com/innr/dexterous-hand/actions/runs/41994588291)：**21/21 CTest 通过**，包括 `S5CommandTtlTest`、`S6BridgeProtocolTest`、`S6MuJoCoSmokeTest`、`S6OfficialModelTest`。
- [main ROS2 Jazzy CI #41994588290](https://github.com/innr/dexterous-hand/actions/runs/41994588290)：`leap_interfaces` colcon 生成成功；`tests/test_s6_command_contract.py` 通过；`scripts/s6_ros2_integration.py` 报告 PASS；`scripts/s6_ros2_supervisor_integration.py` 报告 PASS。
- 这些日志证明的是 Ubuntu 24.04 GitHub runner + `ros:jazzy-ros-base` Docker + 固定模型下载后的**有限自动化场景**，不是所有 ROS2 DDS 配置、负载和平台组合的验收。

## 2. S6 任务、接口、测试追溯

| 阶段 | 关键实现与接口 | 已有自动验收 | 审查结论 |
| --- | --- | --- | --- |
| S6-01 | `cpp/examples/s6_control_bridge.cpp` 的 `STEP`/16 路 `OK`/`STOP` 文本协议；`simulation/s6_mujoco_bridge.py`；100Hz 控制/500Hz 物理 | `S6BridgeProtocolTest`、`S6MuJoCoSmokeTest` | **通过**：确定性虚拟闭环；非真实 OS deadline |
| S6-02 | `OfficialPlant`、`SimSession`、`config/menagerie_right_mapping.json`；官方 right-hand 16 joint/16 actuator | `S6OfficialModelTest` | **通过**：模型映射/运动/暂停/重置/TTL；模型参数非实物标定 |
| S6-03 | `leap_interfaces/msg/JointCommand`（stamp/epoch/sequence/names/position）；`/clock`、`/joint_states`、`/leap/sim_epoch` | 独立 ROS2 控制进程 + 仿真进程 DDS 测试、消息校验单测 | **通过**：ROS2 Jazzy 真正进程间通信；仅测指定 QoS/拓扑 |
| S6-04 | `LifecycleNode`、`ChangeState`、`/leap/supervisor_arm`、`/leap/supervisor_heartbeat`、`/leap/supervisor_status`、`scripts/s6_ros2_supervisor.py` | 生命周期启动/暂停/恢复、控制/仿真/监督器 SIGKILL | **通过**：仿真层故障锁存与禁止旧命令继续执行；非硬件断扭矩 |

前阶段回归：`S5CommandTtlTest` 验证逐路 TX 前 TTL；`S5RealtimeExecutorTest` 验证控制耗时与 full tick 指标分离。两项 S5 软件遗留已由 [PR #30](https://github.com/innr/dexterous-hand/pull/30)、[PR #31](https://github.com/innr/dexterous-hand/pull/31) 修复并通过 CI；**真实总线时序仍未测量**。

## 3. 端到端控制链路

```text
独立 ROS2 controller (50Hz, steady timer)
  /clock + /leap/sim_epoch
  -> leap_interfaces/JointCommand (16x rad, stamp, epoch, sequence)
  -> DDS /leap/joint_command
  -> LifecycleNode simulator: active + armed + supervisor lease
  -> SimSession.accept_command (epoch/seq/TTL/names/limits)
  -> OfficialPlant.feedback (qpos/qvel rad)
  -> C++ s6_control_bridge STEP (16q,16dq,16target, sim_time_us)
  -> S5 FeedbackBuffer / FeedbackSafetyMonitor / ControlLoop
  -> MotionControlCore (position PD + trajectory + limiter)
  -> CommandMailbox / CommunicationWorker / StopCoordinator
  -> C++ OK 16 position commands (rad)
  -> OfficialPlant.apply -> Menagerie 16 actuators -> 5 x mj_step(2ms)
  -> ROS2 /joint_states + /clock + /leap/sim_status

外部 supervisor (独立进程)
  -> lifecycle configure/activate/deactivate
  -> sim arm; 650ms 进程/消息健康监测
  -> /leap/supervisor_heartbeat
  -> 两个 managed 子节点各自 350ms steady-clock 租约
  -> 失效后锁存 SAFE_STOP / 终止控制发布进程
```

- 仿真时间来自 MuJoCo，不使用 `RealtimeExecutor` 的 wall-clock 线程调度；ROS2 10ms timer 仅驱动仿真 tick，不能当作已达到硬实时 100Hz 的证据。
- 单位边界明确：ROS2/MuJoCo 为 rad、rad/s；C++ S5 核心为 deg、deg/s，桥接显式换算。
- `config/menagerie_right_mapping.json` 是仿真专用映射，**不能替代 STS3215 ID/零位/方向/机械限位标定**。
- S6-02 设计文档描述的 `JointState` 命令/`frame_id` epoch 是历史接口；实际代码从 S6-03 起采用专用 `JointCommand.msg`，应以当前代码和 S6-03 文档为准。

## 4. 时钟、安全与故障验收

| 场景 | 已验证的行为 | 验证边界 |
| --- | --- | --- |
| 正常控制 | 16 路官方模型关节运动，`/clock` 与 `joint_states` 同源 | 不是物理精度/负载标定 |
| PAUSE | 先停控制发布，再停仿真，时钟冻结、目标清空 | 没有持续高负载/随机故障竞态覆盖 |
| RESUME | 新 epoch、清空 seq/目标、重建 C++ bridge，等待新命令 | 仅保证**同一仿真进程内** epoch 单调 |
| RESET（S6-03 unmanaged） | MuJoCo 时间回到零、epoch 增加，旧 epoch 拒绝 | S6-04 managed 由 supervisor 管理授权/重启，不能把任意 reset service 当故障恢复授权 |
| 命令无效 | 错名称、NaN、超限、重复/乱序、过期/未来 stamp 拒绝 | 不是消息源身份认证 |
| 控制节点 SIGKILL | supervisor 锁存，仿真 SAFE_STOP | 未给出统计置信区间或最坏延迟 |
| 仿真节点 SIGKILL | supervisor 锁存并终止控制节点 | 实物驱动器失去主控后的动作未验证 |
| supervisor SIGKILL | 子节点 350ms steady 租约到期后停止发布/仿真 SAFE_STOP | 可能留下孤儿进程；租约值不是已测最坏停机时间 |
| 故障恢复 | 监督器故障锁存后不自动重启；重新会话需人工确认 | 跨进程重启会话 ID 防重放仍需加强 |

## 5. 遗留风险登记（新增 S6 风险）

| ID | 级别 | 证据与影响 | 关闭条件 |
| --- | --- | --- | --- |
| **S6-R01** | **P1 / 实机运动阻断** | `LeapSimNode.next_epoch = 1`；新仿真进程再次从 epoch=1 开始。当前 epoch+seq 只能隔离**同进程** reset/resume，不能证明跨进程重启后旧 DDS 命令绝不被接纳。 | 加唯一 session/boot ID（随机 nonce 或持久单调代次）并纳入命令；独立进程重启 + 延迟旧消息注入测试 |
| **S6-R02** | **P1 / 实机运动阻断** | `/leap/supervisor_heartbeat` 的接收回调仅更新时间，未验证序号新鲜度/发送者身份；`/leap/supervisor_arm` 是普通 ROS service。非可信发布者或重放消息可破坏监督租约假设。 | 序号/会话绑定、来源信任边界、网络隔离/SROS2；真机安全链必须独立于 ROS2 |
| **S6-R03** | **P1 / 实机运动阻断** | 350ms lease / 650ms supervisor timeout 是**配置值**，不是测得的 worst-case stop latency；单线程 executor 与同步等待、`stop_process` 最多等待约 2s 会影响服务调用/响应时序。 | 注入阻塞、CPU/IO 压力、DDS 延迟，记录检测→禁止下发→状态发布的 P50/P95/P99/max，明确安全 deadline |
| **S6-R04** | **P1** | supervisor SIGKILL 后子进程可能存活（虽已验证停止发布/仿真停机）；再次启动 supervisor 时可能出现同名节点、旧 topic/service 及 PID 管理冲突。 | 外部进程组/cgroup/systemd 管理、单实例锁、孤儿进程清理和重复启动测试 |
| **S6-R05** | **P1** | CI 仅覆盖 Jazzy 容器与固定 DDS 环境，未覆盖网络分区、ROS2 QoS 变化、长时间压力、ARM/Windows、Sanitizer、时间跳变下游消费者。 | 多配置 DDS/QoS、长时随机故障/CPU 负载、TSAN/ASAN/UBSAN 与目标平台验证 |
| **S6-R06** | **P1 / 实机运动阻断** | 官方 Menagerie actuator、关节惯量/摩擦/碰撞、C++ PD 参数与 STS3215 真实硬件不一致。虚拟模型运动不能证明真实关节增益安全。 | 真实单关节空载低速标定/辨识；建立硬件专用映射、机械限位和安全增益 |
| **S6-R07** | **P2** | S6-02 文档保留过时的 JointState 命令语义；当前代码已换专用 JointCommand。 | 更新历史文档的 superseded 注记及文档索引 |
| **S6-R08** | **P2** | ROS2 CI 用脚本 assert 验收，没有系统化发布/订阅延迟直方图、丢包率、`/clock` jitter、关节误差 RMS/P99、模型接触稳定性统计。 | 添加可重复性能基准与自动报告，给出阈值和 CI 趋势 |

### 继承的 S5 硬件 P0

- **H-01 P0**：没有经验证的物理 E-stop、扭矩/电源切断、过温过流及独立看门狗。
- **H-02 P0**：1Mbps 半双工 UART、方向/TC/TEMT、16 路 ID、零位/方向/机械限位未在真实 STS3215 上验证。
- **H-03 P0**：16 路 TX+RX 实际总线时间、反馈年龄 P99/max、100Hz 控制 deadline、真实故障停止时延未测。

S5 的 S-01（逐路 TTL）和 S-02（时序统计）**软件修复已完成**，但其真实硬件时序验证仍属 H-03。S5 的 S-03（遥测长表到阶跃分析转换）、S-04（Sanitizer/长测）、S-05（速度噪声/增益）、S-06（日志语义）继续跟踪，不因 S6 通过自动关闭。

## 6. 阶段门禁决定

| 后续工作 | 决定 | 必须满足的条件 |
| --- | --- | --- |
| S6 软件学习/仿真继续扩展 | **GO** | 不接真实运动硬件，保持 S5 安全门禁；S6-R01～R05 纳入后续独立 PR |
| S7-01 真实 STS3215 **只读**台架 | **GO（受控准备）** | 外部电源/接线检查、限流、扭矩禁止、明确设备和 ID；仅读取位置/温度/电压/状态；不能将“只读工具”误认为零电气风险 |
| S7 单关节低速空载使能 | **NO-GO（待门禁）** | H-01～H-03 验收、真实零位/限位、物理 E-stop、独立硬件看门狗、S6-R01～R03 在真机架构中有对应防护 |
| S7 16 关节带载闭环 | **NO-GO** | 单关节→小组→16 路分级通过，控制/总线 P99/max 与机械/电流/温度故障响应均达到预定阈值 |

## 7. 推荐实施顺序

1. **S7-01 只读台架 Design Doc**：确认电源、1Mbps 半双工适配器、总线拓扑、ID 扫描、温度/电压/位置采样、每路数据年龄与失败统计；明确禁止 torque enable 和位置写入。
2. **独立 S6 遗留 PR**：优先 S6-R01 跨进程 session ID、S6-R02 supervisor 租约防重放，再做 S6-R03 延迟量化和 S6-R04 孤儿进程清理。不要让这些工作阻塞只读诊断，但不能以只读结果绕过运动门禁。
3. **硬件安全清单**：电源断开路径、独立急停、扭矩卸载/失效策略、温度/电流阈值、看门狗、关节限位、逐关节校准，留存实测记录后才申请单关节使能。

## 8. 复验命令

```bash
# Ubuntu 24.04 + ROS2 Jazzy
source /opt/ros/jazzy/setup.bash
colcon --log-base build/s6_ros2_log build --base-paths ros2_ws/src \
  --build-base build/s6_ros2_pkg --install-base build/s6_ros2_install
source build/s6_ros2_install/setup.bash
python3 -m simulation.fetch_leap_menagerie
cmake -S cpp -B build/s6_ros2_cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/s6_ros2_cpp --parallel 4
ctest --test-dir build/s6_ros2_cpp --output-on-failure
python3 tests/test_s6_command_contract.py
python3 scripts/s6_ros2_integration.py --bridge build/s6_ros2_cpp/s6_control_bridge --timeout 45
python3 scripts/s6_ros2_supervisor_integration.py --bridge build/s6_ros2_cpp/s6_control_bridge
```

本报告为**证据审查与阶段门禁**，没有修改控制代码或放宽安全阈值。
