# 控制算法总览（ArmSubsystem / TunableJoint）

本文件给出“模式总览”，ROS Scheme B 的详细细节请看 `docs/ROS_CONTROL_ALGORITHM.md`。

## 控制模式优先级（高 → 低）

1. `Arm/MonitorOnly == true`  
   - 所有关节强制 0V  
   - 只发布遥测  
   - `ControlMode="monitor"`，`InhibitReason="monitorOnly"`

2. 关节不在 `Arm/ActiveJoint` 选择范围内（inactive）  
   - 该关节强制 0V  
   - `ControlMode="inactive"`，`InhibitReason="inactive"`

3. `RosEnable == true`（ROS Scheme B）  
   - watchdog 生效：`timeout = max(RosTimeoutS, 0.20)`  
   - `RosValid=false` 时强制 0V，`InhibitReason="rosTimeout"`

4. `TestEnable == true`（正弦测试）

5. 手动 `TargetDeg`

## 输出电压的“自洽”原则

- 任何导致强制 0V 的 return 路径必须：
  - `motor.setVoltage(0.0)`
  - `OutVoltsCmd=0.0`（发布最终 applied 电压）
  - `InhibitReason` 写入明确原因

这样 Jetson/Web 才能通过 `InhibitReason` 准确定位“不动”的根因（disabled/monitorOnly/inactive/rosTimeout/maxVolts=0）。


