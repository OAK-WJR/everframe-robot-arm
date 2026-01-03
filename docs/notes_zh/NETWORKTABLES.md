# NetworkTables 键表（Arm 项目）

## 根表：`Arm/`

- `Enabled`（bool）：DriverStation.isEnabled()（全局）
- `MonitorOnly`（bool）：true 时所有关节强制 0V（只发布遥测）
- `ActiveJoint`（string）：`J1`/`J2`/`J3`/`Both`（ROS 开启时 roboRIO 会强制 `Both`）

## 关节表：`Arm/Jx/`

### 输入（UI/调参）

- `TargetDeg`（double）：手动目标（deg）
- `TestEnable`（bool）：正弦测试
- `TestAmpDeg` / `TestPeriodS` / `TestOffsetDeg`
- `TuneLive`（bool）：允许 enabled 时改 PID/FF
- `kP` `kI` `kD`
- `Ks` `Kv` `Ka` `kG`
- `kG2` `kG2OtherScale`（耦合重力补偿项）
- `GlobalOtherScale`（global = self + scale*other）
- `GoalMode` `GoalOtherScale`（耦合/全局目标模式）
- `MaxVolts`（double）：输出限幅
- `MinDeg` `MaxDeg`（soft limits）
- `IZoneDeg` `IMaxVolts`（抗积分饱和）
- `Zero`（bool）：回零
- `SetPosDeg`（double）+ `SetPos`（bool）：任意角度标定

### 输入（ROS Scheme B）

- `RosEnable`（bool）
- `RosPosRad`（double）
- `RosVelRadPerS`（double）
- `RosAccRadPerS2`（double）
- `RosSeq`（int64 推荐；double 兼容）
- `RosTimeoutS`（double）

### 输出（高频遥测）

- `NowSec`（double）：时间戳心跳
- `Enabled`（bool）：DriverStation.isEnabled()
- `Active`（bool）：是否处于 activeJoint 选中（inactive 时 0V）
- `AngleDeg`（double）
- `GoalDeg`（double）：本地关节目标
- `AngleDegGlobal` / `GoalDegGlobal`（double）：耦合下的全局角度/目标
- `OtherAngleDeg`（double）：耦合 other joint 角度
- `GoalSource`（string）：`manual`/`sine`/`ros`
- `OutVoltsCmd`（double）：最终 applied 电压（含门控/限幅）

### 输出（诊断/门控）

- `ControlMode`（string）：`manual`/`sine`/`ros`/`inactive`/`monitor`
- `InhibitReason`（string）：输出被强制 0V 的原因（空串表示未抑制）
- `RosValid`（bool）
- `RosAgeS`（double）
- `RosSeqEcho`（double）
- `OutVoltsRaw`（double）：限幅前
- `Saturated`（bool）：是否饱和
- `SoftLimitClamped`（bool）：ROS pos 软限位 clamp


