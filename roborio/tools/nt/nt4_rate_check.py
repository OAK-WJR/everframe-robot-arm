#!/usr/bin/env python3
"""
NT4 订阅速率检查（用于确认是不是 NT3/默认 periodic 导致 ~10Hz）

依赖：
  python3 -m pip install -U pyntcore

示例：
  python tools/nt/nt4_rate_check.py --server 10.1.14.2 --joint J2 --dur 5 --periodic 0.02
"""

from __future__ import annotations

import argparse
import time

from ntcore import NetworkTableInstance, PubSubOptions


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", required=True)
    ap.add_argument("--joint", default="J2")
    ap.add_argument("--dur", type=float, default=5.0)
    ap.add_argument("--periodic", type=float, default=0.02)
    args = ap.parse_args()

    inst = NetworkTableInstance.getDefault()
    inst.startClient4("nt4_rate_check")
    inst.setServer(args.server)

    opt = PubSubOptions(periodic=args.periodic, sendAll=True)
    jt = inst.getTable("Arm").getSubTable(args.joint)

    keys = [
        ("NowSec", jt.getDoubleTopic("NowSec").subscribe(float("nan"), opt)),
        ("AngleDeg", jt.getDoubleTopic("AngleDeg").subscribe(float("nan"), opt)),
        ("GoalDeg", jt.getDoubleTopic("GoalDeg").subscribe(float("nan"), opt)),
        ("OutVoltsCmd", jt.getDoubleTopic("OutVoltsCmd").subscribe(float("nan"), opt)),
        ("RosSeq", jt.getDoubleTopic("RosSeq").subscribe(float("nan"), opt)),
    ]

    # warmup
    t0 = time.time()
    while time.time() - t0 < 1.0:
        time.sleep(0.02)

    start = time.time()
    last = {k: None for k, _ in keys}
    changes = {k: 0 for k, _ in keys}

    while time.time() - start < args.dur:
        for k, sub in keys:
            v = sub.get()
            if last[k] is None:
                last[k] = v
            else:
                if v != last[k]:
                    changes[k] += 1
                    last[k] = v
        time.sleep(0.001)

    print(f"=== NT4 rate check Arm/{args.joint}/ periodic={args.periodic:.3f}s dur={args.dur:.1f}s ===")
    for k, _ in keys:
        hz = changes[k] / args.dur
        print(f"{k:10s} changes={changes[k]:4d} ~{hz:6.1f} Hz last={last[k]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


