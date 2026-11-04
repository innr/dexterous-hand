# S5-03：STS3215 半双工串口协议与适配

## 与现有实现对齐
协议字段和寄存器以仓库 `hardware/sts3215.py` 为基准：
`FF FF ID LEN INSTRUCTION PARAMS CHECKSUM`，校验和为包体求和取反；
READ=0x02，WRITE=0x03，目标位置地址 42，当前位置地址 56。
默认 1 Mbps、8N1。C++ 侧只做运行时位置读写，**不主动使能扭矩**。

## 设计边界
- `Sts3215Protocol` 纯编解码，校验帧头、长度、ID、Checksum、错误字节。
- `IHalfDuplexPort` 抽象串口 TX 启动、TC 完成、RX 固定长度读取与输入清空。
- `Sts3215Transport` 实现原有 `IServoTransport`，构造时显式提供 16 路物理 ID、
  中心 tick、方向、每度 tick 和机械限位。拒绝重复 ID、广播 ID 和越界映射。
- 默认 `allow_motion=false`；必须明确配置 true 才能发目标位置 WRITE。
- `startSend` 在 S5-01 门禁内只启动 TX；`finishSend` 在锁外等待 TC。
  TC 无法确认则锁存故障，禁止自动恢复。
- 写响应依设备状态返回等级配置 `expect_write_status`；默认不等待写 ACK。
- READ 单播当前位置两个字节；速度暂用连续位置差分估算，首次采样返回无效，
  不把未知速度伪装成 0。超时、错误 ID、校验失败、舵机错误返回无效反馈。
- `PosixHalfDuplexPort` 仅支持 Linux 1Mbps 8N1 + **自动收发方向控制的硬件**。
  通过 TIOCOUTQ 和 TIOCSERGETLSR/TEMT 确认真实 TX 完成；不支持此 ioctl 的
  USB 串口会失败关闭，不能把驱动缓冲区清空误当 UART TC。
- 没有 GPIO DE/RE 方向控制实现，也没有 Windows COM 后端；这些需依据实际硬件补充。

## 风险与未完成
- 当前仍未接入真实设备；真实转接器的 TEMT 能力、收发回波、状态返回等级、
  电气半双工方向、通信超时和 16 舵机实际 ID 必须在台架确认。
- 位置刻度默认 4096 tick/360°，中心 2048；每个关节的安装方向、零位与限位
  **必须实测标定**，不能直接假设默认值适用于真实灵巧手。
- 反馈采用位置差分估算速度，不代表舵机内部速度寄存器，需进一步校准。
- 实际传输超时、重试策略、分片轮询及物理急停尚需后续阶段验收。
- 测试仅为协议黄金向量与 FakePort 故障注入，**不是硬件在环测试**。

## 验收
```sh
cmake -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Debug
cmake --build build/cpp --parallel
ctest --test-dir build/cpp --output-on-failure
```
