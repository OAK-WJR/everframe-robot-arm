#!/usr/bin/env python3
"""
Bridge MoveIt Servo output JointTrajectory to ros_control JointTrajectoryController input.

Why:
- Servo often publishes a 1-point JointTrajectory with a very short horizon.
- JTC behaves much better when it receives a small rolling window (e.g., 2-5 points / 50-150ms).

This node:
- Subscribes:  ~in_topic  (default: /servo_server/joint_trajectory_cmd)
- Publishes:   ~out_topic (default: /arm_controller/command)
- Produces a rolling horizon trajectory with N points spaced by dt_s, assuming constant accel.

Notes:
- If incoming point lacks velocities/accelerations, we estimate vel from finite difference and set acc=0.
"""

import math
from typing import Dict, Optional, Tuple

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ServoTrajToJTC:
    def __init__(self):
        self.in_topic = rospy.get_param("~in_topic", "/servo_server/joint_trajectory_cmd")
        self.out_topic = rospy.get_param("~out_topic", "/arm_controller/command")

        self.dt_s = float(rospy.get_param("~dt_s", 0.02))
        self.points = int(rospy.get_param("~points", 5))
        # Lead time so the first point is still in the future when JTC processes the message.
        # This avoids the "first point already in the past" problem without needing stamp=0,
        # and it gives JTC a rolling window it can actually interpolate over.
        self.lead_s = float(rospy.get_param("~lead_s", 0.04))

        self.dt_s = max(self.dt_s, 0.001)
        self.points = max(self.points, 1)
        self.lead_s = max(self.lead_s, 0.0)

        self._last_pos: Dict[str, float] = {}
        self._last_t: Optional[float] = None

        self.pub = rospy.Publisher(self.out_topic, JointTrajectory, queue_size=1)
        self.sub = rospy.Subscriber(self.in_topic, JointTrajectory, self._cb, queue_size=1)

        rospy.loginfo(
            "servo_traj_to_jtc ready. in=%s out=%s points=%d dt_s=%.3f",
            self.in_topic,
            self.out_topic,
            self.points,
            self.dt_s,
        )

    def _estimate_vel(self, joint_names, pos_list, now_s) -> Dict[str, float]:
        vel = {}
        if self._last_t is None:
            return {n: 0.0 for n in joint_names}
        dt = now_s - float(self._last_t)
        if dt <= 1e-6:
            return {n: 0.0 for n in joint_names}
        for i, name in enumerate(joint_names):
            try:
                p = float(pos_list[i])
            except Exception:
                p = 0.0
            p0 = float(self._last_pos.get(name, p))
            vel[name] = (p - p0) / dt
        return vel

    def _cb(self, msg: JointTrajectory):
        if not msg.joint_names or not msg.points:
            return

        now = rospy.Time.now()
        now_s = now.to_sec()

        p0 = msg.points[0]
        joint_names = list(msg.joint_names)

        # Extract pos/vel/acc (fallbacks if missing)
        pos0 = [0.0] * len(joint_names)
        for i in range(len(joint_names)):
            if i < len(p0.positions):
                try:
                    pos0[i] = float(p0.positions[i])
                except Exception:
                    pos0[i] = 0.0

        has_vel = len(p0.velocities) == len(joint_names)
        has_acc = len(p0.accelerations) == len(joint_names)

        if has_vel:
            vel0 = [float(v) for v in p0.velocities]
        else:
            est = self._estimate_vel(joint_names, pos0, now_s)
            vel0 = [float(est.get(n, 0.0)) for n in joint_names]

        if has_acc:
            acc0 = [float(a) for a in p0.accelerations]
        else:
            acc0 = [0.0] * len(joint_names)

        # Update last sample for next estimation
        for i, name in enumerate(joint_names):
            self._last_pos[name] = float(pos0[i])
        self._last_t = now_s

        out = JointTrajectory()
        # Use an explicit start time and shift all points into the future by lead_s.
        # This prevents the controller from constantly "resetting to t=0" every 20ms.
        out.header.stamp = now
        out.joint_names = joint_names

        # Rolling horizon forward from current sample, shifted by lead_s so points are future.
        for k in range(self.points):
            t = self.lead_s + self.dt_s * k
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration.from_sec(t)

            pt.positions = []
            pt.velocities = []
            pt.accelerations = []
            for i in range(len(joint_names)):
                p = pos0[i] + vel0[i] * t + 0.5 * acc0[i] * t * t
                v = vel0[i] + acc0[i] * t
                a = acc0[i]
                pt.positions.append(p)
                pt.velocities.append(v)
                pt.accelerations.append(a)
            out.points.append(pt)

        self.pub.publish(out)


def main():
    rospy.init_node("servo_traj_to_jtc", anonymous=False)
    ServoTrajToJTC()
    rospy.spin()


if __name__ == "__main__":
    main()


