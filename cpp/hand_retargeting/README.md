# C++ 手部重定向核心

这是 P2 的 C++17 实现：把 MediaPipe 风格的 21 个手部关键点转换成 LEAP
canonical 顺序的 16 个关节角度。核心不依赖摄像头、ROS 2 或第三方 JSON
库，因此可以在没有硬件的情况下测试。

## 构建和测试

在仓库根目录运行：

```bash
cmake -S cpp/hand_retargeting -B build/hand_retargeting
cmake --build build/hand_retargeting
ctest --test-dir build/hand_retargeting --output-on-failure
```

离线演示可以使用安全默认限位：

```bash
./build/hand_retargeting/retarget_demo
```

使用仓库的官方关节限位：

```bash
./build/hand_retargeting/retarget_demo config/joint_mapping.json
```

## 硬件边界

`JointBackend` 是未来真实 STS3215 串口驱动的替换点。当前
`VirtualSts3215Backend` 只模拟接受 16 维位置命令、返回状态和关闭扭矩；它不
声称已经验证真实串口、电源、舵机 ID 或零位。真实串口属于后续 P3 任务。
