# ROS 控制算法（Scheme B：外部轨迹跟踪）— 仅控制算法说明

本文只描述 **roboRIO 侧 TunableJoint 的 ROS 控制算法**，不包含 UI/调参/机械结构细节。

## 目标

- Jetson/ROS 以 **50Hz（20ms）** 发送每帧 setpoint：位置/速度/加速度 + 序号（RosSeq）
- roboRIO 每 20ms：
  - 读 ROS setpoint
  - 做 watchdog（超时强制 0V）
  - 用 **PID(位置误差)** + **FF(vel/acc/ks/kG)** 计算电压并输出

## 输入（Jetson -> roboRIO，NetworkTables）

均在 `Arm/Jx/` 下：

- `RosEnable`（bool）：是否请求 ROS 控制
- `RosPosRad`（double）：期望关节角（rad）。**这是 PID 的主目标**
- `RosVelRadPerS`（double）：期望角速度（rad/s）。**只用于 FF 的 Kv 与 Ks 方向选择**
- `RosAccRadPerS2`（double）：期望角加速度（rad/s^2）。**只用于 FF 的 Ka**
- `RosSeq`（int64 优先；double 兼容）：每帧递增的序号，用于 watchdog 判断“是否有新帧”
- `RosTimeoutS`（double）：允许的最大“无新帧时间”。建议 0.25～0.40s

### 发布契约（强烈建议）

每一帧必须 **同时** 写入：

- `RosPosRad`
- `RosVelRadPerS`
- `RosAccRadPerS2`
- `RosSeq`（递增）

如果你“只更新 pos，不更新 vel/acc/seq”，roboRIO 侧可能：

- FF 用到旧的 vel/acc（表现为拖拽/发黏）
- watchdog 认为 seq 没变（表现为“停一下再走”）

## 输出（roboRIO -> Jetson/Web，NetworkTables）

均在 `Arm/Jx/` 下：

- `RosValid`（bool）：`Enabled && age<=timeout`
- `RosAgeS`（double）：距离上次 RosSeq 更新的时间（秒）
- `RosSeqEcho`（double）：roboRIO 侧看到的 seq（用于对齐 Jetson 侧）
- `ControlMode`（string）：`ros`/`manual`/`sine`/`inactive`/`monitor`
- `InhibitReason`（string）：为什么输出被强制 0V（如 `disabled`/`rosTimeout`/`maxVolts=0`/`monitorOnly`/`inactive`）
- `OutVoltsCmd`（double）：**最终 applied** 电压（已过所有门控/限幅）
- `OutVoltsRaw`（double）：限幅前电压（用于判断饱和）
- `Saturated`（bool）：`abs(OutVoltsRaw) > MaxVolts`
- `SoftLimitClamped`（bool）：RosPos 被软限位 clamp

## 核心算法

### 1) Watchdog（基于 RosSeq）

- roboRIO 每周期读取 `RosSeq`（优先 int64 topic；否则回退 double）
- 若 `seq != lastSeq`：更新 `lastRosUpdateSec = nowSec`
- 计算 `age = nowSec - lastRosUpdateSec`
- `timeout = max(RosTimeoutS, 0.20)`  
  - 0.20s ≈ 10 个 20ms 周期：减少网络/调度抖动误触发
- `RosValid = Enabled && (age <= timeout)`

若 `RosValid=false`：**强制 0V**，`InhibitReason="rosTimeout"`（或 `disabled`）

### 2) 位置软限位（Soft Limits）

将 ROS 位置目标 clamp 到 `MinDeg/MaxDeg`：

- `posClamped = clamp(RosPosRad, minRad, maxRad)`
- 若发生 clamp：`SoftLimitClamped=true`

并做一个“防顶死”处理：

- 如果已经 clamp 到最小角，但 ROS 仍给负速度（继续往小角推），则将 vel/acc FF 置 0
- clamp 到最大角同理


