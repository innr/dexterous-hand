# P3：STS3215 串口扫描、方向确认和零位标定设计

状态：已冻结（软件预处理部分）  
目标环境：Ubuntu 24.04 + Python 3.12；目标硬件：STS3215 TTL 半双工总线

## 范围

本任务分为可在无硬件时完成的软件部分和必须到货后执行的实物验证。

现在实现：

- 串口设备枚举，不打开、不写入设备；
- 基于现有 `STS3215Bus` 的 PING/读寄存器扫描；
- 零位 ticks → rad 换算和方向判断；
- 标定 capture 校验、候选结果和 reviewed mapping 输出；
- 单舵机运动 dry-run 和显式扭矩/确认安全门；
- 虚拟/fake bus 单元测试。

硬件到货后执行：

- USB-TTL、供电和 TTL 半双工接线确认；
- 实际串口端口和舵机 ID 扫描；
- 单舵机小幅度运动方向确认；
- LEAP home pose 下逐关节零位采集；
- 低速逐步运动和急停验证。

没有硬件时不声称完成上述物理验证。

## 现有协议边界

沿用 `hardware/sts3215.py`：默认 1 Mbps、50ms 超时、PING、寄存器读写、位置
读取、目标位置和扭矩开关。扫描命令只调用 PING/读操作，不启用扭矩、不下发目标
位置。

## 标定数据流

```text
机械手放到 home pose
        ↓
只读 capture present_position_ticks
        ↓
人工检查姿态、ID 和方向
        ↓
生成候选 mapping（默认 status=candidate）
        ↓
另存为新 JSON；不自动覆盖正式 mapping
```

ticks 与接口角度的关系为：

```text
raw_angle = (ticks - reference_ticks) * 2π / 4096
canonical_angle = direction * (raw_angle - zero_offset_rad)
```

只有同时满足 `--confirm` 且人工复核后才能生成更新后的 mapping；只有每个关节
都有方向结果时，才允许标记 `status=verified`。

## 运动安全门

`sts3215-motion-test` 默认只打印目标 ticks，不打开扭矩，也不发送目标位置。真实
运动必须同时提供：

```text
--enable-torque
--confirm
```

运动结束会在 `finally` 中尝试关闭该舵机扭矩。串口错误时不继续发送后续动作；关闭
扭矩是 best-effort，因为通信故障本身可能阻止写操作。

## 验收标准

- 串口枚举和扫描工具在无设备时也能安全运行；
- 扫描路径不调用扭矩或目标位置写操作；
- capture 校验拒绝未知/重复 ID、越界 ticks 和错误 schema；
- 未确认不能生成更新 mapping；
- 未提供两个运动安全参数时不会发送真实运动命令；
- 虚拟/fake bus 测试覆盖方向、零位、候选 mapping 和安全门；
- 实物接线、方向、零位和运动结果标记为硬件到货后的待办。
