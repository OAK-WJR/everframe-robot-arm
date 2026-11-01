# ROS 控制算法总览（Scheme B：Jetson 直接下发 pos/vel/acc）

本文档只描述 **ROS 控制模式（Scheme B）**：Jetson/ROS 通过 NT4 直接向 roboRIO 下发 **位置/速度/加速度** setpoint，roboRIO 端做 **位置跟踪（PID）+ 前馈（Ks/Kv/Ka/kG/kG2）** 输出电压，不使用内部梯形轨迹生成（也不在 rio 端“再规划一遍轨迹”）。

实现位置：`src/main/java/frc/robot/subsystems/TunableJoint.java`（ROS 分支）

---

## 1) 输入（Jetson → roboRIO / 每关节 `Arm/Jx/`）

Jetson 每帧（建议 20ms/50Hz）必须写：

- `RosEnable`（bool）：是否启用 ROS 控制
- `RosPosRad`（double, rad）：目标位置
- `RosVelRadPerS`（double, rad/s）：目标速度
- `RosAccRadPerS2`（double, rad/s²）：目标加速度
- `RosSeq`（int 语义）：每帧递增（用于 watchdog / 新数据判定）
- `RosTimeoutS`（double, s）：超时阈值

推荐：`RosTimeoutS` 起步 **0.25–0.40s**。设置过小（0.02–0.15s）在实际系统抖动下很容易误判超时，导致 `RosValid` 掉线、输出间歇归零。

**发布契约**：不要只发 pos；每帧必须同时发 pos/vel/acc/seq，否则 `Kv/Ka` 前馈会失效或表现异常。

### 1.1 pos/vel/acc 在算法中的“角色”

- **`RosPosRad` 是主 setpoint**：PID 的目标就是追踪这个位置。
- **`RosVelRadPerS` / `RosAccRadPerS2` 只用于前馈（FF）**：
  - 它们不会被积分成位置，也不会被用来“预测未来位置”
  - 它们不会改变 PID 的目标（PID 目标始终是 `RosPosRad`）

因此：如果你希望“在某处停住/继续走/减速”，必须通过 Jetson 侧改变 `RosPosRad`（以及与之匹配的 vel/acc），而不是只把 vel/acc 置 0。

### 1.2 一致性要求（非常重要）

Jetson 每帧输出的三元组 \((pos, vel, acc)\) 应来自同一条轨迹在同一时刻的采样点，否则可能出现：

- `pos` 已到目标但 `acc` 仍很大（或 `vel=0 但 acc≠0`）：会让 Ka 前馈产生不必要的电压脉冲/饱和
- `vel/acc` 与 `pos` 的变化方向不一致：会增加 PID 抵消前馈的工作量，体感“顿挫/台阶”

---

## 2) 有效性（Watchdog / 门控）

roboRIO 每 20ms 周期读取 `RosSeq`：

- 若本周期 `RosSeq != lastRosSeq`：
  - `lastRosSeq = RosSeq`
  - `lastRosUpdateSec = nowSec`

定义（本仓库实现细节）：

- \(age = nowSec - lastRosUpdateSec\)
- \(timeout = \max(RosTimeoutS, 0.06)\)（至少约 3 个 20ms 周期，降低抖动误触发）
- `RosValid = (Enabled == true) && (age <= timeout)`

若满足任一条件，则**强制输出 0V**：

- `Enabled == false` → `InhibitReason="disabled"`
- `MaxVolts <= 0` → `InhibitReason="maxVolts=0"`
- `RosEnable == true` 且 `RosValid == false` → `InhibitReason="rosTimeout"`
- `Arm/MonitorOnly == true` → `InhibitReason="monitorOnly"`
- 关节处于 inactive（非 ActiveJoint 且非 Both）→ `InhibitReason="inactive"`

> 你要排查“不动”，优先看 `ControlMode` 和 `InhibitReason`。

### 2.1 是否会“到达后停止/保持上一次 setpoint”？

- **正常情况下（持续收到 RosSeq）**：rio 每 20ms 都使用“最新一次收到的 pos/vel/acc”，也就是“保持最后一次 setpoint，直到下一次 setpoint 到来”。
- **若 Jetson 停止更新（RosSeq 不再变化）**：一旦 `age > timeout`，rio 判定 `RosValid=false`，**输出强制 0V**（安全），而不是继续保持力矩。

这就是为什么 `RosTimeoutS` 不要太小：太小会导致短暂抖动就误触发 0V，体感“停一下再走”。

---

## 3) 目标与限位（Soft limits）

读取：

- `MinDeg/MaxDeg`（deg，软限位）

将 ROS 位置目标 clamp 到软限位：

\[
posCmd = clamp(RosPosRad,\; rad(MinDeg),\; rad(MaxDeg))
\]

> 注意：当前实现只对 **位置**做限位 clamp；`RosVel/RosAcc` 不会自动根据限位被重算，所以 Jetson 端最好保证限位附近的三元组仍然一致。

---

## 4) 控制器（Scheme B：PIDController）

使用标准 `PIDController`（不是 ProfiledPIDController）：

- 误差：\(e = posCmd - posMeas\)
- 测量：\(posMeas = getAngleRad()\)（已包含 INVERTED 的符号统一）

PID 输出（概念上）：

\[
u_{pid} = PID(posMeas,\; posCmd)
\]

> 代码中 `kP/kI/kD` 与 Manual/Sine 共用同一套 NT 参数（`kP/kI/kD`），并同样受 `IZoneDeg/IMaxVolts` 约束。

### 4.1 到达目标后会发生什么？（不会“自动停规划”，只会继续保持）

当 `posMeas ≈ posCmd`：

- 位置误差趋近 0，`u_pid` 会趋近 0（I 项为消除静态偏差可能保留小输出）
- 控制输出主要由重力项 `kG*cos(angle)`（以及可能的 `kG2`）承担
- 若 Jetson 后续继续沿轨迹更新 `posCmd`，rio 会继续跟踪；若 Jetson 把 `posCmd` 固定在某个值，rio 就会保持该目标

---

## 5) 前馈（FF：Ks + Kv + Ka + kG + kG2）

### 5.1 方向 sign（带滞回，抑制零速抖动/“Ks 台阶感”）

优先级与阈值（实现值）：

1. 若 \(|RosVelRadPerS| > 1e-4\)：`sign = sign(RosVelRadPerS)`
2. 否则使用误差滞回：
   - 死区：`|error| < 0.25°` 保持 `lastDir`
   - 反向翻转：只有当 `|error| >= 0.75°` 才允许从 `lastDir` 翻到相反方向
   - 初次建立方向：`lastDir=0` 时也要求 `|error| >= 0.75°`

### 5.2 重力项（用实际角度）

记：

- \(self = getAngleRad()\)
- 若启用全局角组合（由 `GoalRelativeToOther` + `GlobalOtherScale` 影响）并且存在 `otherAngleRad`：
  - \(gravAngle = self + GlobalOtherScale \cdot other\)
  - 否则 \(gravAngle = self\)

\[
gVolts = kG \cdot \cos(gravAngle)
\]

可选耦合重力项（若有 other）：

\[
gVolts += kG2 \cdot \cos(self + kG2OtherScale \cdot other)
\]

### 5.3 速度/加速度前馈

在 ROS 模式下：

- \(velCmd = RosVelRadPerS\)
- \(accCmd = RosAccRadPerS2\)

前馈总项（概念上）：

\[
u_{ff} = Ks \cdot sign + gVolts + Kv \cdot velCmd + Ka \cdot |accCmd| \cdot sign
\]

> `Ka` 用 `|acc|*sign` 是为了减少 acc 噪声导致的正反抖动。

### 5.4 “提前到达然后停止”的可能性（常见误解澄清）

严格从算法定义上说：**rio 不会因为 vel/acc=0 就“提前停止”**，因为 PID 仍在追踪 `posCmd`。

你可能观察到“看起来提前停住/走不动”，通常来自下面几类物理/约束原因：

- **饱和**：`MaxVolts` 太小，PID+FF 被 clamp，无法克服重力/摩擦 → 误差消不掉 → 看起来停住
- **静摩擦/间隙**：`Ks` 与 sign 选择在小误差区间导致电压不够“突破” → 停在目标附近某个偏差
- **Jetson setpoint 自己提前变成常值**：Jetson 把 `RosPosRad` 提前固定（而不是继续沿轨迹更新），rio 当然会保持当前目标

建议排查顺序（只看控制算法相关）：

- `InhibitReason` 是否为空（否则先解决门控）
- 是否长期饱和（`OutVoltsCmd` 是否常年贴着 ±MaxVolts）
- `AngleDeg` 与 `GoalDeg` 是否仍存在显著误差（例如 >0.5°）
- `RosVel/RosAcc` 是否与 `RosPos` 同帧一致（避免 `vel=0 acc≠0`）

---

## 6) 电压合成与输出

合成：

\[
u = u_{pid} + u_{ff}
\]

限幅：

\[
u = clamp(u,\; -MaxVolts,\; +MaxVolts)
\]

应用电机方向（INVERTED）：

\[
applied = u \cdot ioSign
\]

最终：

- `motor.setVoltage(applied)`
- `OutVoltsCmd` 发布 `applied`

---

## 7) ROS 模式下的关键遥测（roboRIO → Jetson/Web）

每关节：

- `ControlMode="ros"`
- `InhibitReason`：空串表示未被门控；否则给出原因
- `RosValid` / `RosAgeS` / `RosSeqEcho`
- 高频（NT4 Topic Publisher periodic=0.02）：
  - `NowSec` / `AngleDeg` / `GoalDeg` / `OutVoltsCmd`

---

## 8) 最小验收（你怎么判断“ROS 控制算法跑起来了”）

在机器人 **Enabled** 状态下：

- `RosEnable=true`
- `ControlMode="ros"`
- `InhibitReason=""`
- `RosValid=true` 且 `RosAgeS < RosTimeoutS`
- `OutVoltsCmd` 随目标变化（不再长期为 0）

---

## 9) vel/acc 是否“真的被用上”的判据（只针对算法）

因为在 Scheme B 中 vel/acc 只走前馈，所以判断它们是否生效的最直接方式是：

- 当 `RosVel/RosAcc` 明显非 0 且 `Kv/Ka` 也非 0 时，`OutVoltsCmd` 应明显随其变化（尤其在位置误差很小、但你仍在运动时）
- 若你经常看到 `RosVel=0` 同时 `RosAcc` 很大，那不是 rio 算法的问题，而是 Jetson 发布侧三元组不一致；先修发布侧


