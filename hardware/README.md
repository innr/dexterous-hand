# STS3215 通信与零位标定

通信实现位于 `hardware/sts3215.py`，使用 FEETECH SCS/STS 半双工 TTL 协议：
默认波特率 1 Mbps，位置寄存器为 12-bit（0–4095），位置读取不会改变舵机状态。

## 连接前检查

1. 使用支持半双工 TTL 的 USB 转换板；普通全双工 USB-TTL 不能直接代替总线收发器。
2. 舵机单独供电，电压严格匹配 STS3215 版本；USB 只负责通信。
3. 第一次只连接一个舵机，确认 ID 和波特率后再接入菊花链。
4. 标定采集工具只读取位置，不启用扭矩、不下发目标位置。

## 扫描

```bash
python -m hardware.scan_sts3215 --port /dev/ttyUSB0 --ids 0,1,2
```

## 零位采集

先把机械结构手动放在约定的零位，确认舵机可自由转动，再执行：

```bash
python -m hardware.calibrate_zero \
  --port /dev/ttyUSB0 \
  --ids 0,1,2,3 \
  --output calibration/capture.json
```

输出文件只记录当前编码器读数和相对参考角度。人工检查机械姿态、方向和限位后，
再把结果写回 `config/joint_mapping.json`；工具不会自动覆盖主映射文件。

## 软件侧位置接口

`hardware/leap_hand_controller.py` 提供 canonical 16 关节到 STS3215 tick 的转换，
统一应用 `direction`、`zero_offset_rad` 和关节限位。映射的
`calibration.status` 不是 `calibrated` 时，控制器默认拒绝下发运动指令；
`allow_uncalibrated=True` 只应在 bench 测试时显式使用。启用扭矩也必须通过
`command_positions(..., enable_torque=True)` 或 `set_torque_all(True)` 明确请求。
