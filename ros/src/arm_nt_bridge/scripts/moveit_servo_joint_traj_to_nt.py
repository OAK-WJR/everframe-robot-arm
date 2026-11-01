#!/usr/bin/env python3
"""
MoveIt Servo -> NetworkTables (RoboRIO ROS trajectory mode)

This node bridges MoveIt Servo's outgoing joint setpoints to your RoboRIO NetworkTables
interface described in docs/NETWORKTABLES.md (方案 B: RosEnable + RosPos/Vel/Acc + RosSeq watchdog).

Input:
  - trajectory_msgs/JointTrajectory (default topic: /servo/joint_trajectory)
    Uses the LAST point in the message as the current setpoint.

Output (per joint Arm/Jx/*):
  - RosEnable (bool) = true
  - RosPosRad (double)
  - RosVelRadPerS (double)
  - RosAccRadPerS2 (double)
  - RosSeq (double, counter) += 1   <-- watchdog refresh (critical)
  - RosTimeoutS (double, optional, keepalive)

Root keys Arm/*:
  - ActiveJoint (string) keepalive (default: "Both")
  - MonitorOnly (bool) keepalive (default: False)  NOTE: true disables motor output (0V)
"""

import math
import time
from typing import Dict, Optional

import rospy
from trajectory_msgs.msg import JointTrajectory

try:
    from networktables import NetworkTables
except Exception as e:  # pragma: no cover
    NetworkTables = None
    _IMPORT_ERR = e


class ServoJointTrajToNT:
    def __init__(self):
        if NetworkTables is None:
            raise RuntimeError(
                "Import networktables failed. Install pynetworktables in your ROS container, e.g.:\n"
                "  python3 -m pip install --user pynetworktables\n"
                f"Original error: {_IMPORT_ERR}"
            )

        # NT connection
        self.server_ip = rospy.get_param("~server_ip", "10.1.14.2")
        self.nt_table = rospy.get_param("~table", "Arm")

        # Servo output topic
        self.servo_topic = rospy.get_param("~servo_topic", "/servo/joint_trajectory")

        # ROS joint name -> NT subtable name (J1/J2/J3)
        self.joint_map: Dict[str, str] = rospy.get_param(
            "~joint_map",
            {"joint1": "J1", "joint2": "J2", "joint3": "J3"},
        )

        # Root keys
        self.active_joint = rospy.get_param("~active_joint", "Both")
        self.monitor_only = bool(rospy.get_param("~monitor_only", False))
        self.root_keepalive_s = float(rospy.get_param("~root_keepalive_s", 1.0))
        self._last_root_keepalive_t = 0.0

        # Per-joint keepalive
        self.ros_timeout_s = float(rospy.get_param("~ros_timeout_s", 0.15))
        self.joint_keepalive_s = float(rospy.get_param("~joint_keepalive_s", 1.0))
        self._last_joint_keepalive_t = 0.0

        # Logging / safety
        self.warn_no_point = bool(rospy.get_param("~warn_no_point", True))
        self._last_warn_t = 0.0

        NetworkTables.initialize(server=self.server_ip)
        self.nt = NetworkTables.getTable(str(self.nt_table))

        # Seq counters (per joint)
        self._seq: Dict[str, float] = {}
        for ros_joint, nt_joint in self.joint_map.items():
            st = self.nt.getSubTable(str(nt_joint))
            try:
                self._seq[ros_joint] = float(st.getNumber("RosSeq", 0.0))
            except Exception:
                self._seq[ros_joint] = 0.0

        rospy.Subscriber(self.servo_topic, JointTrajectory, self._cb, queue_size=1)

        rospy.loginfo(
            "servo_joint_traj_to_nt ready. server_ip=%s table=%s servo_topic=%s joints=%s",
            self.server_ip,
            self.nt_table,
            self.servo_topic,
            ",".join(self.joint_map.keys()),
        )
        rospy.logwarn(
            "Will set Arm/ActiveJoint=%s Arm/MonitorOnly=%s. (MonitorOnly=true => 0V output)",
            self.active_joint,
            self.monitor_only,
        )
        rospy.on_shutdown(self._on_shutdown)

    def _set_root_keepalive(self):
        now = time.time()
        if (now - self._last_root_keepalive_t) < self.root_keepalive_s:
            return
        self._last_root_keepalive_t = now
        self.nt.putString("ActiveJoint", str(self.active_joint))
        self.nt.putBoolean("MonitorOnly", bool(self.monitor_only))

    def _set_joint_keepalive(self):
        now = time.time()
        if (now - self._last_joint_keepalive_t) < self.joint_keepalive_s:
            return
        self._last_joint_keepalive_t = now
        for ros_joint, nt_joint in self.joint_map.items():
            st = self.nt.getSubTable(str(nt_joint))
            st.putBoolean("RosEnable", True)
            st.putNumber("RosTimeoutS", float(self.ros_timeout_s))

    def _on_shutdown(self):
        # Fail-safe: disable ROS mode so RoboRIO doesn't keep expecting ROS seq updates
        try:
            for ros_joint, nt_joint in self.joint_map.items():
                st = self.nt.getSubTable(str(nt_joint))
                st.putBoolean("RosEnable", False)
        except Exception:
            pass

    def _cb(self, msg: JointTrajectory):
        self._set_root_keepalive()
        self._set_joint_keepalive()

        if not msg.points:
            if self.warn_no_point:
                now = time.time()
                if (now - self._last_warn_t) > 1.0:
                    self._last_warn_t = now
                    rospy.logwarn("Received JointTrajectory with no points on %s", self.servo_topic)
            return

        pt = msg.points[-1]
        names = list(msg.joint_names)

        # Build index lookup
        idx = {n: i for i, n in enumerate(names)}

        for ros_joint, nt_joint in self.joint_map.items():
            if ros_joint not in idx:
                continue
            i = idx[ros_joint]

            if i >= len(pt.positions):
                continue
            pos = float(pt.positions[i])
            vel = float(pt.velocities[i]) if i < len(pt.velocities) else 0.0
            acc = float(pt.accelerations[i]) if i < len(pt.accelerations) else 0.0

            st = self.nt.getSubTable(str(nt_joint))
            st.putBoolean("RosEnable", True)
            st.putNumber("RosPosRad", pos)
            st.putNumber("RosVelRadPerS", vel)
            st.putNumber("RosAccRadPerS2", acc)

            self._seq[ros_joint] = float(self._seq.get(ros_joint, 0.0) + 1.0)
            st.putNumber("RosSeq", self._seq[ros_joint])


def main():
    rospy.init_node("servo_joint_traj_to_nt", anonymous=False)
    node = ServoJointTrajToNT()
    rospy.spin()


if __name__ == "__main__":
    main()


