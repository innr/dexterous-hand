# P2：录制、回放和 LeRobot/HDF5 数据管线

状态：已冻结  
目标环境：Ubuntu 24.04 + Python 3.12；ROS 2 Jazzy 适配可选

## 目标

保存灵巧手每一帧的目标动作和实际状态，支持离线回放，并在训练前导出为
LeRobotDataset。当前没有硬件和摄像头，因此先使用虚拟后端和离线 JSONL 输入。

## 格式分工

- HDF5 是本项目的本地原始格式，按 episode 追加写入，便于审计和回放。
- LeRobot 是训练格式；导出器使用可选的 `LeRobotDataset` API，HDF5 不伪装成
  LeRobot 原生格式。
- 当前版本不录制图像；以后加入摄像头时再增加视频和图像 feature。

## HDF5 schema v1

每个文件位于 `episodes/<episode_id>.h5`，包含：

```text
attrs:
  schema_version = 1
  joint_order_json = LEAP canonical 16 names
  position_unit = "rad"
  time_unit = "ns"
  task, metadata_json, finalized, frame_count

frames/
  timestamp_ns                  (N,) int64
  action_position_rad           (N,16) float32
  observation_position_rad     (N,16) float32
  observation_velocity_rad_s   (N,16) float32
  valid                         (N,) uint8
  valid_reason                  (N,) uint8
```

时间戳必须严格递增，所有数值必须有限。数据顺序固定为现有
`config/joint_mapping.json` 的 canonical 16 关节顺序，单位为弧度。

## 录制契约

`EpisodeRecorder.append(EpisodeFrame(...))` 接受一帧已经同步的数据。未来的 ROS 2
适配器以 `joint_states` 时间戳为主，寻找时间误差不超过 20ms 的最近 command；找
不到时写入 `valid=false` 和原因码，不写入 NaN。

当前还提供 JSONL 离线入口，用于无 ROS 2、无硬件测试：

```bash
dexterous-hand-data record-jsonl \
  --input samples/episode.jsonl \
  --output recordings/demo \
  --task open_hand
```

## 回放契约

默认 `speed=0`，不等待、按原始顺序读取；`speed=1` 按录制时间实时回放。
无效帧默认跳过，真实硬件不会被自动创建或连接。当前的 memory sink 只记录发送
的 16 维 action，供虚拟测试使用。

```bash
dexterous-hand-data replay \
  --input recordings/demo/episodes/episode_0000.h5 \
  --backend memory
```

## LeRobot 导出

```bash
dexterous-hand-data export-lerobot \
  --input recordings/demo \
  --output datasets/demo \
  --fps 30
```

导出 feature：

- `action`：16 维目标角度；
- `observation.state`：16 维实际角度；
- `observation.velocity`：16 维实际速度；
- `task`：episode 任务文本。

导出器会跳过 `valid=false` 帧，并在结束时调用 `finalize()`。真实硬件回放需要
后续 P3 的串口、方向和零位标定，不在本任务内开启。

## 验收标准

- HDF5 写入、读取和 metadata round-trip 一致；
- 非法形状、NaN、倒序时间戳和未 finalize 文件被拒绝；
- 回放顺序、速度模式和 invalid frame 策略可测试；
- memory sink 收到的 command 与 HDF5 action 一致；
- LeRobot 未安装时给出明确可操作的 optional dependency 错误；
- 不需要摄像头、ROS 2 或真实 STS3215 即可通过完整测试。
