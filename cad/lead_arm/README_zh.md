# ARM — 微缩 lead arm 设计基准

从 `arm_description/`(fusion2urdf 从 Fusion 导出的 4-DOF 机械臂 URDF)提取关节骨架,
给微缩主手(lead arm)建模当参考。

## 内容

- `arm_description/` — URDF 包(ROS 1 格式;本快照只保留 `urdf/arm.xacro`,STL mesh 未收录)
- `gen_joint_axes_step.py` — 解析 `urdf/arm.xacro`,对每个转动关节生成一个
  「圆面 + 圆周边线」标记(圆心 = 关节原点,法向 = 旋转轴),导出 STEP
- `step/arm_joint_axes_full.step` — 1:1(mm)
- `step/arm_joint_axes_1to4.step` — 1:4 微缩
- `onshape/arm_joint_axes.fs` — Onshape FeatureScript 参数化版:ratio / 半径 /
  各段长度全在特征对话框里改,改完 4 个草图圆即时重算(推荐用这个)

## 关节世界坐标(1:1, mm, 零位姿)

| 标记 | URDF 关节 | 圆心 (X, Y, Z) | 轴向 |
|---|---|---|---|
| J_yaw_base | Revolute 6 | −0.14, 6.362, 259.291 | Z |
| J_shoulder | Revolute 2 | 14.86, 6.362, 331.291 | X |
| J_elbow | Revolute 3 | 51.86, −493.638, 331.291 | X |
| J_wrist | Revolute 4 | 3.859, −798.113, 331.291 | X |

摆动长度:大臂(肩→肘)500 mm、小臂(肘→腕)304.475 mm;侧向错位 +37 / −48 mm。

## 用法

```bash
uv run gen_joint_axes_step.py   # 重新生成 step/(改半径/比例改脚本顶部常量)
```

Fusion 导入 STEP 后每个关节是一个带名字的组件;圆以**平面圆盘面**承载
(纯线框 STEP Fusion 会丢),在草图里对圆盘边线 **Project/Include** 即得草图圆。
