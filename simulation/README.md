# 单指 MuJoCo 仿真

这是项目的第一个可运行里程碑：一个具有 4 个旋转关节和位置执行器的简化手指。
它不等同于最终 LEAP Hand 模型，主要用于验证环境、控制循环和关节约束。

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

