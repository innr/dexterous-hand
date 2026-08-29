# 单指 MuJoCo 仿真

这是项目的第一个可运行里程碑：一个具有 4 个旋转关节和位置执行器的简化手指。
它不等同于最终 LEAP Hand 模型，主要用于验证环境、控制循环和关节约束。

## 官方 LEAP Hand 模型

官方模型通过固定提交的 Git 子模块接入，包含 16-DOF URDF 和 STL 网格：

```bash
git submodule update --init --recursive
dexterous-hand-leap-check --seconds 1 --pose close
```

`config/joint_mapping.json` 记录了官方关节编号、MuJoCo 加载顺序、STS3215
舵机 ID、关节限位和仿真到硬件的重排规则。运行检查时，程序会把官方 URDF
转换为临时 MJCF，加入 16 个按 canonical/hardware ID 排列的位置执行器、PD
参数、关节阻尼和力矩限幅；临时文件不会修改子模块。

`simulation.home_pose_rad` 是仅用于 MuJoCo 的零姿态，不能当作舵机零位。
舵机方向与零位仍需在实物标定后填写，当前配置不会直接驱动硬件。`--pose home`
可只验证保持零姿态，`--pose close` 则运行一个受限的弯曲目标。

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

控制器会生成平滑的周期弯曲目标；LEAP 检查入口现在已切换为官方 16-DOF
模型。下一阶段是在接入实物后，用扫描结果填写 STS3215 的方向、零位和限位表，
再把同一套 canonical 目标接到硬件总线。
