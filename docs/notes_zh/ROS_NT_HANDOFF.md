# ROS ↔ NetworkTables 交接文档（Jetson/ROS 端应如何写）

## 你应该用什么库

- **必须用 NT4 客户端**：Python 推荐 `pyntcore`（提供 `ntcore` 模块）
- 不要用 `pynetworktables`（NT3，默认速率低且行为不同，常见 ~10Hz）

安装：

```bash
python3 -m pip install -U pyntcore
```

## Topic/Key 约定

根表：`Arm/`  
关节子表：`Arm/J1/`、`Arm/J2/`、`Arm/J3/`

### Jetson 写入（必须）

在 `Arm/Jx/`：

- `RosEnable`（bool）：请求 ROS 控制（true）
- `RosPosRad`（double）：位置目标（rad）
- `RosVelRadPerS`（double）：速度目标（rad/s）
- `RosAccRadPerS2`（double）：加速度目标（rad/s^2）
- `RosSeq`（int64 推荐）：每帧递增
- `RosTimeoutS`（double，可选）：建议 0.25～0.40

### Jetson 读取（建议）

在 `Arm/Jx/`：

- `RosValid`、`RosAgeS`、`RosSeqEcho`
- `ControlMode`、`InhibitReason`
- `AngleDeg`、`GoalDeg`
- `OutVoltsCmd`、`OutVoltsRaw`、`Saturated`、`SoftLimitClamped`

在 `Arm/`：

- `Enabled`（全局 DS enabled）
- `MonitorOnly`（安全监控模式）
- `ActiveJoint`（UI 用；ROS 开启时 roboRIO 会强制 `Both`）

## 50Hz 的关键点（Jetson 侧）

- 用 `PubSubOptions(periodic=0.02)`（订阅）与稳定的 50Hz loop（发布）
- 每一帧必须同时写入 `pos/vel/acc/seq`
- `RosSeq` 必须每帧递增（watchdog 只看 seq 变化）

## roboRIO 侧控制方式（当前：轨迹追踪模式）

- `TunableJoint` 轨迹追踪：
  - `RosPosRad` 用作参考位置（PI 位置误差）
  - `RosVelRadPerS` 用作速度误差反馈 + Kv/Ks
  - `RosAccRadPerS2` 用作 Ka 前馈
  - watchdog：`timeout = max(RosTimeoutS, 0.20)`
- 高频遥测使用 NT4 Publisher periodic=0.02：
  - `NowSec/AngleDeg/GoalDeg/OutVoltsCmd/Enabled`
- 自洽遥测：
  - 所有强制 0V 的 return 都写 `InhibitReason` 并发布 `OutVoltsCmd=0`


