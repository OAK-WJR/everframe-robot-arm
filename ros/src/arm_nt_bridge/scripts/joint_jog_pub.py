#!/usr/bin/env python3
"""
Publish control_msgs/JointJog at a fixed rate with header.stamp refreshed every cycle.

Why: `rostopic pub -f xxx.yaml` keeps a fixed stamp, which can make MoveIt Servo treat
the command as stale and periodically halt ("走走停停"/卡顿).
"""

from __future__ import annotations

import ast
from typing import List

import rospy
from control_msgs.msg import JointJog


def _as_list(x) -> List[float]:
    if x is None:
        return []
    if isinstance(x, list):
        return x
    return [x]

def _maybe_parse_list(x):
    """
    Allow params to come in as:
    - list (already parsed)
    - scalar (float/int)
    - string like "[0.2]" or "['joint2']"
    """
    if isinstance(x, list):
        return x
    if isinstance(x, (int, float)):
        return [x]
    if isinstance(x, str):
        s = x.strip()
        # Accept single token too
        if not s:
            return []
        if s[0] in "[(":
            try:
                v = ast.literal_eval(s)
                if isinstance(v, list):
                    return v
                if isinstance(v, tuple):
                    return list(v)
                return [v]
            except Exception:
                return [s]
        return [s]
    return [x]


def _broadcast(values: List[float], n: int, name: str) -> List[float]:
    if n <= 0:
        return []
    if len(values) == 0:
        return []
    if len(values) == 1 and n > 1:
        return [float(values[0])] * n
    if len(values) != n:
        raise ValueError(f"{name} length must be 1 or equal to joint_names length (got {len(values)} vs {n})")
    return [float(v) for v in values]


def main() -> None:
    rospy.init_node("joint_jog_pub", anonymous=False)

    topic = rospy.get_param("~topic", "/servo_server/delta_joint_cmds")
    rate_hz = float(rospy.get_param("~rate", 50.0))
    frame_id = str(rospy.get_param("~frame_id", "base_link"))

    # Prefer list param (for advanced use), else fall back to scalar ~joint (for roslaunch arg friendliness)
    joint_names = rospy.get_param("~joint_names", None)
    if joint_names is None:
        joint = rospy.get_param("~joint", "joint2")
        joint_names = [str(joint)]
    else:
        joint_names = _maybe_parse_list(joint_names)
        if not joint_names:
            raise ValueError("~joint_names must be a non-empty list (or set ~joint)")
        joint_names = [str(j) for j in joint_names]

    # Vel/disp: prefer array params, else scalar vel for simple launch usage
    velocities_param = rospy.get_param("~velocities", None)
    if velocities_param is None:
        vel = float(rospy.get_param("~vel", 0.2))
        velocities = _broadcast([vel], len(joint_names), "velocities")
    else:
        velocities = _broadcast(_as_list(_maybe_parse_list(velocities_param)), len(joint_names), "velocities")

    displacements_param = rospy.get_param("~displacements", None)
    if displacements_param is None:
        displacements = []
    else:
        displacements = _broadcast(_as_list(_maybe_parse_list(displacements_param)), len(joint_names), "displacements")

    # JointJog.duration is the intended command duration for Servo (not the runtime of this node).
    msg_duration_s = float(rospy.get_param("~msg_duration_s", 0.0))
    # How long this node should run; 0 means forever.
    run_for_s = float(rospy.get_param("~run_for_s", 0.0))
    publish_zero_on_exit = bool(rospy.get_param("~publish_zero_on_exit", True))

    pub = rospy.Publisher(topic, JointJog, queue_size=1)
    rate = rospy.Rate(max(rate_hz, 1.0))

    rospy.loginfo(
        "joint_jog_pub: publishing %s @ %.1fHz joints=%s vel=%s disp=%s frame_id=%s run_for_s=%.1f",
        topic,
        rate_hz,
        joint_names,
        velocities,
        displacements,
        frame_id,
        run_for_s,
    )

    start = rospy.Time.now()
    while not rospy.is_shutdown():
        if run_for_s > 0.0 and (rospy.Time.now() - start).to_sec() >= run_for_s:
            break

        msg = JointJog()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.joint_names = joint_names
        msg.velocities = velocities
        msg.displacements = displacements
        # control_msgs/JointJog.duration is a float (seconds) in ROS1 control_msgs
        msg.duration = float(max(msg_duration_s, 0.0))

        pub.publish(msg)
        rate.sleep()

    if publish_zero_on_exit and not rospy.is_shutdown():
        try:
            zero = JointJog()
            zero.header.stamp = rospy.Time.now()
            zero.header.frame_id = frame_id
            zero.joint_names = joint_names
            zero.velocities = [0.0] * len(joint_names)
            zero.displacements = [0.0] * len(joint_names) if displacements else []
            zero.duration = 0.0
            pub.publish(zero)
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        rospy.logerr("joint_jog_pub failed: %s", e)
        raise


