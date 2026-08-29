# 软件运行教程

本教程面向第一次运行项目的用户，目标是在没有真实舵机和摄像头的情况下完成软件验证。

## 1. 准备环境

推荐环境：

- Ubuntu 24.04
- Python 3.12
- ROS 2 Jazzy（运行 ROS 2 节点时需要）
- Git 和 CMake

克隆仓库并安装 Python 依赖：

    git clone https://github.com/innr/dexterous-hand.git
    cd dexterous-hand
    python3.12 -m venv .venv
    source .venv/bin/activate
    python -m pip install --upgrade pip
    python -m pip install -e ".[dev]"

## 2. 运行 Python 测试

    python -m pytest -q

测试覆盖关节映射、MuJoCo 配置、STS3215 协议、安全逻辑和数据接口。
测试不需要串口或硬件。

## 3. 运行 MuJoCo 仿真

    python -m simulation.run_single_finger
    python -m simulation.run_leap_hand

如果只想检查模型和关节映射：

    python -m simulation.run_leap_hand --check

## 4. 构建 C++ 重定向核心

    cmake -S cpp/hand_retargeting -B build/hand_retargeting
    cmake --build build/hand_retargeting
    ctest --test-dir build/hand_retargeting --output-on-failure
    ./build/hand_retargeting/retarget_demo

安装为可复用 CMake 包：

    cmake --install build/hand_retargeting --prefix "$HOME/.local"

## 5. 运行虚拟 STS3215

虚拟总线用于测试 ID、位置读写、超时和扭矩关闭逻辑。它不会打开串口：

    python -m pytest tests/test_sts3215.py -q

真实串口命令属于硬件阶段，未连接舵机时不要执行运动命令。

## 6. 运行 ROS 2 虚拟节点

先加载 Jazzy：

    source /opt/ros/jazzy/setup.bash
    cd ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    ros2 launch dexterous_hand_ros leap_hand.launch.py

另一个终端查看状态：

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    ros2 topic echo /leap_hand_node/joint_states

发送一条 16 关节命令时，必须使用弧度并保持 mapping 顺序。长时间没有新命令时，节点默认执行 disable_torque。

## 7. 录制和回放

录制接口要求每条命令和状态带纳秒时间戳。建议先使用虚拟后端录制，再回放检查：

    python -m pytest tests/test_ros2_recording_bridge.py -q

HDF5 导出需要安装 h5py：

    python -m pip install h5py

## 8. MediaPipe 输入

MediaPipe 适配层接收 21 个关键点，统一为 x/y/z 三元组，再交给重定向核心：

    python -m pytest tests/test_mediapipe_adapter.py -q

当前适配层不打开摄像头。摄像头接入属于后续软件扩展，真实手部和舵机运动仍需硬件验证。

## 9. 常见问题

- ROS 2 is not installed：先 source /opt/ros/jazzy/setup.bash。
- No serial ports found：没有硬件时是正常结果。
- 关节数量错误：LEAP 接口固定为 16 个关节。
- 单位错误：软件接口使用弧度，不使用角度。
- 运动前先确认 mapping、方向和零位；真实运动必须等硬件阶段。

## 软件完成标准

在没有硬件的情况下，以下项目全部通过即可认为软件闭环完成：

1. Python 和 C++ 测试通过。
2. MuJoCo 仿真可以启动。
3. ROS 2 虚拟节点可以发布和接收 16 关节消息。
4. 命令、状态可以录制并回放。
5. MediaPipe 关键点可以通过校验并送入重定向核心。

真实串口扫描、舵机方向确认、零位标定和实物运动不属于本教程的无硬件验证范围。
