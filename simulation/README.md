# 单指 MuJoCo 仿真

这是项目的第一个可运行里程碑：一个具有 4 个旋转关节和位置执行器的简化手指。
它不等同于最终 LEAP Hand 模型，主要用于验证环境、控制循环和关节约束。

## 官方 LEAP Hand 模型

官方模型通过固定提交的 Git 子模块接入，包含 16-DOF URDF 和 STL 网格：

```bash
git submodule update --init --recursive
dexterous-hand-leap-check --seconds 0.1
```

`config/joint_mapping.json` 记录了官方关节编号、MuJoCo 加载顺序、STS3215
舵机 ID、关节限位和仿真到硬件的重排规则。舵机方向与零位仍需在实物标定后填写，
当前配置不会直接驱动硬件。

## 安装

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[dev]"
```

## 运行

打开 MuJoCo 窗口：

```bash
dexterous-hand-sim
```

服务器或 CI 中无界面运行：

```bash
dexterous-hand-sim --headless --seconds 5
```

执行测试：

```bash
pytest
```

控制器会生成平滑的周期弯曲目标。下一阶段应将简化模型替换为 LEAP Hand
官方 URDF/MJCF，并建立与真实 STS3215 舵机一致的关节方向、零位和限位表。

## S6-01：16 关节 MuJoCo 与 C++ 控制核心闭环

```bash
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/cpp --parallel
python3 -m pip install "mujoco>=3.1,<4" "numpy>=1.26,<3"
python3 -m simulation.s6_mujoco_bridge --bridge build/cpp/s6_control_bridge --seconds 1 --assert-motion
ctest --test-dir build/cpp --output-on-failure
```

使用 16-DOF **替身动力学模型**（不是官方 LEAP URDF），Python MuJoCo 以
500Hz 步进，C++ `ControlLoop` / `MotionControlCore` / `CommunicationWorker`
以仿真时钟 100Hz 运行。输出统计包含关节实际运动幅度。
没有任何串口/硬件使能；官方 LEAP 模型和 ROS2 接入在后续 S6 任务完成。

## S6-02：官方 LEAP Hand MJCF 与 ROS2

使用 Google DeepMind MuJoCo Menagerie 的 **LEAP Hand 右手官方 MJCF**
（MIT 授权，固定提交 `98a70c569c4c1b6da467a20012f766ee1f8dc521`）。
仅用于仿真，不改变 STS3215 硬件映射。

```bash
python -m simulation.fetch_leap_menagerie
cmake -S cpp -B build/cpp
cmake --build build/cpp --config Debug
# Linux/macOS:
python -m simulation.s6_official_bridge --bridge build/cpp/s6_control_bridge --seconds 0.6 --assert-motion
# Windows:
python -m simulation.s6_official_bridge --bridge build/cpp/Debug/s6_control_bridge.exe --seconds 0.6 --assert-motion
ctest --test-dir build/cpp --output-on-failure -C Debug
```

可选 ROS2 环境（例如 Ubuntu + ROS2 Jazzy，需先 source ROS2 环境）：

```bash
python3 -m simulation.s6_ros2_node --bridge build/cpp/s6_control_bridge
ros2 topic echo /leap/joint_states
ros2 topic echo /leap/sim_status
ros2 topic echo /leap/sim_epoch
ros2 service call /leap/sim_pause std_srvs/srv/SetBool "{data: true}"
ros2 service call /leap/sim_pause std_srvs/srv/SetBool "{data: false}"
ros2 service call /leap/sim_reset std_srvs/srv/Trigger "{}"
```

控制输入：`/leap/joint_command` (`sensor_msgs/JointState`)，
16 个官方 MJCF 名称（见 `config/menagerie_right_mapping.json`），
`position` 单位 rad，`header.stamp` 使用当前 `/clock`，
`header.frame_id` 必须是 `sim_epoch:<N>`（**临时仿真协议**，不是 TF 坐标系）。
控制方至少每 100ms 仿真时间刷新一次，否则进入锁存 SAFE_STOP。
暂停冻结仿真时间并清除目标；恢复后需新命令。
重置将 epoch 加一，旧 epoch 命令不再接受。详情见
`docs/designs/s6_02_official_leap_ros2.md`。
