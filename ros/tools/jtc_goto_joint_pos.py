#!/usr/bin/env python3
"""
用 ROS 直接给 JointTrajectoryController 发布“去某个绝对位置”的命令（用于回零）。

不需要 NT4/pyntcore：你只需要在容器里（已有 ROS 环境）运行本脚本。

默认：让 joint2 在 2.5s 内运动到 0 rad，并等待误差收敛到阈值以内。

示例（容器内）：
  python3 /root/catkin_ws/tools/jtc_goto_joint_pos.py --joint joint2 --pos 0.0 --dur 3.0
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, Optional

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class StateCache:
    def __init__(self) -> None:
        self.actual: Dict[str, float] = {}
        self.desired: Dict[str, float] = {}
        self.joint_names = []
        self._ready = False

    def cb(self, msg: JointTrajectoryControllerState) -> None:
        self.joint_names = list(msg.joint_names)
        for i, name in enumerate(msg.joint_names):
            if i < len(msg.actual.positions):
                self.actual[name] = float(msg.actual.positions[i])
            if i < len(msg.desired.positions):
                self.desired[name] = float(msg.desired.positions[i])
        self._ready = True

    @property
    def ready(self) -> bool:
        return self._ready and bool(self.actual) and bool(self.joint_names)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/arm_controller/command")
    ap.add_argument("--state", default="/arm_controller/state")
    ap.add_argument("--joint", default="joint2")
    ap.add_argument("--pos", type=float, default=0.0, help="target position (rad)")
    ap.add_argument("--dur", type=float, default=0.0, help="trajectory duration (s). 0=auto by max_vel")
    ap.add_argument("--max_vel", type=float, default=0.3, help="used when --dur=0, rad/s")
    ap.add_argument("--hz", type=float, default=20.0, help="how often to re-publish the command (Hz)")
    ap.add_argument("--wait", type=float, default=6.0, help="max wait for convergence (s)")
    ap.add_argument("--eps", type=float, default=0.01, help="converge threshold (rad)")
    args = ap.parse_args()

    rospy.init_node("jtc_goto_joint_pos", anonymous=True)

    cache = StateCache()
    rospy.Subscriber(args.state, JointTrajectoryControllerState, cache.cb, queue_size=1)

    pub = rospy.Publisher(args.topic, JointTrajectory, queue_size=1)
    rate = rospy.Rate(max(args.hz, 1.0))

    # Wait for state
    t0 = time.time()
    while not rospy.is_shutdown() and not cache.ready and (time.time() - t0) < 3.0:
        rate.sleep()
    if not cache.ready:
        print(f"[home] 等待 {args.state} 超时：没有收到 controller state")
        return 2

    cur = cache.actual.get(args.joint, None)
    if cur is None:
        print(f"[home] controller state 里找不到关节 {args.joint}")
        return 2

    # Auto duration if needed
    dur = float(args.dur)
    if dur <= 0.0:
        mv = max(1e-3, float(args.max_vel))
        dur = abs(float(args.pos) - float(cur)) / mv + 1.0

    print(f"[home] {args.joint} 当前={cur:.4f} rad 目标={args.pos:.4f} rad dur={dur:.2f}s")

    # IMPORTANT:
    # controllers.yaml has allow_partial_joints_goal: false
    # so we MUST send a full joint list, otherwise the controller ignores the command.
    full_names = list(cache.joint_names)
    full_positions = []
    for name in full_names:
        v = cache.actual.get(name, 0.0)
        if name == args.joint:
            v = float(args.pos)
        full_positions.append(float(v))

    # Build trajectory (single point at t=dur is enough for JTC topic interface)
    traj = JointTrajectory()
    traj.joint_names = full_names
    pt = JointTrajectoryPoint()
    pt.positions = full_positions
    pt.velocities = [0.0] * len(full_names)
    pt.time_from_start = rospy.Duration.from_sec(max(dur, 0.1))
    traj.points = [pt]

    # Publish for a short burst (ensures it wins any timing / latched issues)
    burst_s = min(0.8, max(0.2, 3.0 / max(args.hz, 1.0)))
    burst_end = time.time() + burst_s
    while not rospy.is_shutdown() and time.time() < burst_end:
        traj.header.stamp = rospy.Time.now()
        pub.publish(traj)
        rate.sleep()

    # Wait until error small
    deadline = time.time() + max(args.wait, 0.0)
    last_print = 0.0
    while not rospy.is_shutdown() and time.time() < deadline:
        cur2: Optional[float] = cache.actual.get(args.joint, None)
        if cur2 is None:
            rate.sleep()
            continue
        err = float(args.pos) - float(cur2)
        now = time.time()
        if now - last_print > 0.2:
            print(f"[home] err={err:+.4f} rad (cur={cur2:.4f})")
            last_print = now
        if abs(err) <= float(args.eps):
            print("[home] OK: 已收敛到阈值内")
            return 0
        rate.sleep()

    print("[home] 超时：未收敛（可能机械限位/电机未 enable/或其它节点覆盖了命令）")
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(130)


