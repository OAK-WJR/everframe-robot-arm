#!/usr/bin/env python3
"""
非 ROS 模式：通过 NetworkTables 写入“手动目标”并监控 AngleDeg 收敛。

按你的约定会写入这些键（一次性/低频）：
  /Arm/J2/RosEnable   = false
  /Arm/J2/TestEnable  = false
  /Arm/J2/TargetDeg   = <target>
  /Arm/J2/MaxVolts    = <maxvolts>
  /Arm/ActiveJoint    = "J2" 或 "Both"

注意：
- 本脚本使用 pynetworktables（networktables 包），它通常是 NetworkTables v3 客户端。
  如果你的 RoboRIO 只提供 NT4(5810) 而不兼容 NT3(常见 1735)，本脚本可能无法连接。
  这种情况下只能换成 WPILib 的 ntcore/pyntcore（你当前说不要用，所以这里只给提示，不自动切换）。

用法示例（Jetson 或任意装了 pynetworktables 的机器）：
  python3 tools/nt/nt4_goto.py --server 10.1.14.2 --joint J2 --target 0.0 --maxvolts 2.0
"""

from __future__ import annotations

import argparse
import math
import os
import time
from datetime import datetime
from pathlib import Path


def _now_s() -> float:
    return time.time()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="10.1.14.2", help="RoboRIO/NT server IP")
    ap.add_argument("--joint", default="J2", choices=["J1", "J2", "J3"], help="Active joint")
    ap.add_argument("--active", default=None, choices=[None, "J1", "J2", "J3", "Both"], help="Arm/ActiveJoint (default: same as --joint)")
    ap.add_argument("--target", type=float, default=0.0, help="TargetDeg")
    ap.add_argument("--maxvolts", type=float, default=2.0, help="MaxVolts")
    ap.add_argument("--eps", type=float, default=0.5, help="Converge threshold in degrees")
    ap.add_argument("--timeout", type=float, default=15.0, help="Max seconds to wait for reaching target")
    ap.add_argument("--rate", type=float, default=10.0, help="Monitor print rate (Hz)")
    ap.add_argument("--log", action="store_true", help="Save a small log to catkin_ws/arm_data/")
    args = ap.parse_args()

    try:
        from networktables import NetworkTables  # type: ignore
    except Exception as e:
        print("[nt_goto] 缺少依赖：pynetworktables（pip3 install pynetworktables）")
        print(f"[nt_goto] import error: {e}")
        return 2

    server = str(args.server)
    joint = str(args.joint)
    active = str(args.active) if args.active else joint

    # Optional log file
    log_path: Path | None = None
    if args.log:
        ws = Path(os.environ.get("CATKIN_WS", "/home/jetson/catkin_ws"))
        out_dir = ws / "arm_data"
        out_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = out_dir / f"nt_manual_goto_{joint}_{args.target:+.1f}deg_{ts}.txt"

    def log(line: str) -> None:
        print(line)
        if log_path:
            log_path.write_text((log_path.read_text() if log_path.exists() else "") + line + "\n")

    log(f"[nt_goto] server={server} joint={joint} active={active} target={args.target:.3f}deg maxvolts={args.maxvolts:.2f}V")

    # Connect
    # NOTE: pynetworktables doesn't let you pick port; it uses NT3 defaults.
    NetworkTables.initialize(server=server)

    # Wait for connection
    t0 = _now_s()
    while not NetworkTables.isConnected() and (_now_s() - t0) < 3.0:
        time.sleep(0.05)
    if not NetworkTables.isConnected():
        log("[nt_goto] 连接失败：pynetworktables 可能无法连到你的 NT 服务器。")
        log("[nt_goto] 可能原因：RoboRIO 只开放 NT4(5810)；pynetworktables 是 NT3 客户端。")
        log("[nt_goto] 解决：需要用 ntcore/pyntcore（你当前要求不使用，所以这里只提示）。")
        return 1

    arm = NetworkTables.getTable("Arm")
    jt = arm.getSubTable(joint)

    # One-shot writes for non-ROS manual mode
    jt.putBoolean("RosEnable", False)
    jt.putBoolean("TestEnable", False)
    jt.putNumber("TargetDeg", float(args.target))
    jt.putNumber("MaxVolts", float(args.maxvolts))
    arm.putString("ActiveJoint", active)

    # Monitoring loop
    hz = max(1.0, float(args.rate))
    dt = 1.0 / hz
    deadline = _now_s() + max(0.0, float(args.timeout))

    last_print = 0.0
    while _now_s() < deadline:
        angle = float(jt.getNumber("AngleDeg", float("nan")))
        enabled_arm = bool(arm.getBoolean("Enabled", False))
        enabled_j = bool(jt.getBoolean("Enabled", False))
        inhibit = str(jt.getString("InhibitReason", ""))
        volts = float(jt.getNumber("OutVoltsCmd", float("nan")))

        err = float("nan") if math.isnan(angle) else (float(args.target) - angle)
        now = _now_s()
        if now - last_print > 0.2:
            log(f"[nt_goto] Enabled={enabled_arm}/{enabled_j} Inhibit='{inhibit}' AngleDeg={angle:.3f} err={err:+.3f} V={volts:.3f}")
            last_print = now

        if (not math.isnan(err)) and abs(err) <= float(args.eps):
            log("[nt_goto] OK: 已到达目标阈值内")
            if log_path:
                log(f"[nt_goto] log saved: {log_path}")
            return 0

        time.sleep(dt)

    log("[nt_goto] 超时：未到达目标（请确认机器人已 Enabled、ActiveJoint 正确、且 RosEnable=false）")
    if log_path:
        log(f"[nt_goto] log saved: {log_path}")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())


