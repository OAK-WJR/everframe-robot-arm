#!/usr/bin/env python3
"""
NT4 持久在线监控工具（一次连接，持续打印关键字段，避免“新建连接没同步到值就退出”的空值问题）

依赖：
  python3 -m pip install -U pyntcore

示例：
  python tools/nt/nt4_watch_arm.py --server 10.1.14.2 --dur 20 --hz 50
  python tools/nt/nt4_watch_arm.py --server 10.1.14.2 --dur 30 --joint J2 --hz 50 --csv tools/nt/j2_trace.csv
"""

from __future__ import annotations

import argparse
import csv
import time
from dataclasses import dataclass

from ntcore import NetworkTableInstance, PubSubOptions


@dataclass
class Row:
    t: float
    joint: str
    enabled: bool | None
    monitor_only: bool | None
    active_joint: str | None
    ros_enable: bool | None
    ros_valid: bool | None
    ros_age_s: float | None
    ros_seq: float | None
    control_mode: str | None
    inhibit_reason: str | None
    angle_deg: float | None
    goal_deg: float | None
    out_volts_cmd: float | None
    out_volts_raw: float | None
    saturated: bool | None
    soft_limit_clamped: bool | None


def _fmt(v) -> str:
    if v is None:
        return "None"
    if isinstance(v, float):
        if v != v:
            return "nan"
        return f"{v:.6g}"
    return str(v)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", required=True, help="roboRIO IP, e.g. 10.TE.AM.2")
    ap.add_argument("--dur", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--joint", default="J2", help="J1/J2/J3")
    ap.add_argument("--warmup", type=float, default=1.0, help="连接后等待同步秒数")
    ap.add_argument("--csv", default="", help="可选：输出 CSV 路径")
    args = ap.parse_args()

    inst = NetworkTableInstance.getDefault()
    inst.startClient4("nt4_watch_arm")
    inst.setServer(args.server)

    # 订阅选项：明确 periodic=0.02，避免客户端默认 0.1s。
    opt = PubSubOptions(periodic=1.0 / max(args.hz, 1.0), sendAll=True)

    arm = inst.getTable("Arm")
    jt = arm.getSubTable(args.joint)

    # root keys
    sub_monitor_only = arm.getBooleanTopic("MonitorOnly").subscribe(False, opt)
    sub_active_joint = arm.getStringTopic("ActiveJoint").subscribe("", opt)
    sub_arm_enabled = arm.getBooleanTopic("Enabled").subscribe(False, opt)

    # joint keys
    sub_enabled = jt.getBooleanTopic("Enabled").subscribe(False, opt)
    sub_now = jt.getDoubleTopic("NowSec").subscribe(float("nan"), opt)
    sub_ros_enable = jt.getBooleanTopic("RosEnable").subscribe(False, opt)
    sub_ros_valid = jt.getBooleanTopic("RosValid").subscribe(False, opt)
    sub_ros_age = jt.getDoubleTopic("RosAgeS").subscribe(float("nan"), opt)
    sub_ros_seq = jt.getDoubleTopic("RosSeq").subscribe(float("nan"), opt)
    sub_ros_seq_echo = jt.getDoubleTopic("RosSeqEcho").subscribe(float("nan"), opt)
    sub_control_mode = jt.getStringTopic("ControlMode").subscribe("", opt)
    sub_inhibit = jt.getStringTopic("InhibitReason").subscribe("", opt)
    sub_angle = jt.getDoubleTopic("AngleDeg").subscribe(float("nan"), opt)
    sub_goal = jt.getDoubleTopic("GoalDeg").subscribe(float("nan"), opt)
    sub_out = jt.getDoubleTopic("OutVoltsCmd").subscribe(float("nan"), opt)
    sub_raw = jt.getDoubleTopic("OutVoltsRaw").subscribe(float("nan"), opt)
    sub_sat = jt.getBooleanTopic("Saturated").subscribe(False, opt)
    sub_clamp = jt.getBooleanTopic("SoftLimitClamped").subscribe(False, opt)

    # warmup: 等待首批 topic/value 同步
    t0 = time.time()
    while time.time() - t0 < args.warmup:
        time.sleep(0.02)

    period = 1.0 / max(args.hz, 1.0)
    end = time.time() + args.dur

    writer = None
    f = None
    if args.csv:
        f = open(args.csv, "w", newline="")
        writer = csv.writer(f)
        writer.writerow([field.name for field in Row.__dataclass_fields__.values()])

    last_print = 0.0
    while time.time() < end:
        now = time.time()
        row = Row(
            t=now,
            joint=args.joint,
            enabled=bool(sub_enabled.get()),
            monitor_only=bool(sub_monitor_only.get()),
            active_joint=sub_active_joint.get() or "",
            ros_enable=bool(sub_ros_enable.get()),
            ros_valid=bool(sub_ros_valid.get()),
            ros_age_s=float(sub_ros_age.get()),
            ros_seq=float(sub_ros_seq.get()),
            control_mode=sub_control_mode.get() or "",
            inhibit_reason=sub_inhibit.get() or "",
            angle_deg=float(sub_angle.get()),
            goal_deg=float(sub_goal.get()),
            out_volts_cmd=float(sub_out.get()),
            out_volts_raw=float(sub_raw.get()),
            saturated=bool(sub_sat.get()),
            soft_limit_clamped=bool(sub_clamp.get()),
        )

        if writer:
            writer.writerow(
                [
                    row.t,
                    row.joint,
                    row.enabled,
                    row.monitor_only,
                    row.active_joint,
                    row.ros_enable,
                    row.ros_valid,
                    row.ros_age_s,
                    row.ros_seq,
                    row.control_mode,
                    row.inhibit_reason,
                    row.angle_deg,
                    row.goal_deg,
                    row.out_volts_cmd,
                    row.out_volts_raw,
                    row.saturated,
                    row.soft_limit_clamped,
                ]
            )

        # 控制台输出：最多 20Hz，避免刷屏
        if now - last_print >= 0.05:
            last_print = now
            print(
                f"[{args.joint}] Enabled={row.enabled} ArmEnabled={_fmt(bool(sub_arm_enabled.get()))} "
                f"MonitorOnly={row.monitor_only} ActiveJoint={row.active_joint} "
                f"RosEnable={row.ros_enable} RosValid={row.ros_valid} AgeS={_fmt(row.ros_age_s)} "
                f"Seq={_fmt(row.ros_seq)} Echo={_fmt(float(sub_ros_seq_echo.get()))} "
                f"Mode={row.control_mode} Inhibit={row.inhibit_reason} "
                f"AngleDeg={_fmt(row.angle_deg)} GoalDeg={_fmt(row.goal_deg)} "
                f"OutV={_fmt(row.out_volts_cmd)} Raw={_fmt(row.out_volts_raw)} "
                f"Sat={row.saturated} Clamp={row.soft_limit_clamped}"
            )

        time.sleep(period)

    if f:
        f.close()
        print(f"CSV saved: {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


