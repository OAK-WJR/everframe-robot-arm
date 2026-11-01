#!/usr/bin/env python3
"""
MoveIt Servo (Float64MultiArray output) -> NetworkTables (RoboRIO ROS setpoint mode)

Servo output:
  - std_msgs/Float64MultiArray on ~servo_topic (default: /servo/joint_position_cmd)
    Values are joint positions in radians, in the order of ~joint_order.

RoboRIO NetworkTables keys (per docs/NETWORKTABLES.md):
  - Arm/Jx/RosEnable = true
  - Arm/Jx/RosPosRad = pos
  - Arm/Jx/RosVelRadPerS = 0
  - Arm/Jx/RosAccRadPerS2 = 0
  - Arm/Jx/RosSeq += 1   (watchdog refresh)
  - Arm/Jx/RosTimeoutS keepalive
  - Arm/ActiveJoint, Arm/MonitorOnly keepalive
"""

import time
from typing import Dict, List, Optional

import rospy
from std_msgs.msg import Float64MultiArray

try:
    import ntcore  # NT4 client (pyntcore)
except Exception as e:  # pragma: no cover
    ntcore = None
    _IMPORT_ERR = e


class ServoArrayToNT:
    def __init__(self):
        if ntcore is None:
            raise RuntimeError(
                "Import ntcore failed. WPILib 2025 uses NT4; do NOT use pynetworktables (NT3).\n"
                "Install NT4 python client in your ROS container:\n"
                "  python3 -m pip install -U pyntcore\n"
                f"Original error: {_IMPORT_ERR}"
            )

        self.server_ip = rospy.get_param("~server_ip", "10.1.14.2")
        self.nt_table = rospy.get_param("~table", "Arm")

        self.servo_topic = rospy.get_param("~servo_topic", "/servo/joint_position_cmd")
        self.joint_order = rospy.get_param("~joint_order", ["joint1", "joint2", "joint3", "joint4"])

        self.joint_map: Dict[str, str] = rospy.get_param(
            "~joint_map",
            {"joint1": "J1", "joint2": "J2", "joint3": "J3"},
        )

        self.active_joint = rospy.get_param("~active_joint", "Both")
        self.monitor_only = bool(rospy.get_param("~monitor_only", False))
        self.root_keepalive_s = float(rospy.get_param("~root_keepalive_s", 1.0))
        self._last_root_keepalive_t = 0.0

        self.ros_timeout_s = float(rospy.get_param("~ros_timeout_s", 0.15))
        self.joint_keepalive_s = float(rospy.get_param("~joint_keepalive_s", 1.0))
        self._last_joint_keepalive_t = 0.0

        # Critical: watchdog requires continuous updates.
        self.nt_write_rate = float(rospy.get_param("~nt_write_rate", 50.0))
        # NT4 client (pyntcore) - no Timer/flush hacks; NT4 publishers handle high-rate updates correctly.
        self._nt_inst = ntcore.NetworkTableInstance.getDefault()
        self._nt_inst.setServer(str(self.server_ip))
        self._nt_inst.startClient4("jetson_ros_servo_setpoint")
        self.nt = self._nt_inst.getTable(str(self.nt_table))

        # Pre-create publishers (NT4 topics) for low jitter.
        self._pub_root_active = self.nt.getStringTopic("ActiveJoint").publish()
        self._pub_root_monitor = self.nt.getBooleanTopic("MonitorOnly").publish()

        self._pub = {}  # ros_joint -> dict of publishers
        self._seq: Dict[str, int] = {}
        for ros_joint, nt_joint in self.joint_map.items():
            st = self.nt.getSubTable(str(nt_joint))
            self._pub[ros_joint] = {
                "enable": st.getBooleanTopic("RosEnable").publish(),
                "timeout": st.getDoubleTopic("RosTimeoutS").publish(),
                "pos": st.getDoubleTopic("RosPosRad").publish(),
                "vel": st.getDoubleTopic("RosVelRadPerS").publish(),
                "acc": st.getDoubleTopic("RosAccRadPerS2").publish(),
                "seq": st.getIntegerTopic("RosSeq").publish(),
            }
            # Start seq from 0 (RoboRIO can echo back if needed)
            self._seq[ros_joint] = 0

        self._latest_positions: Optional[List[float]] = None
        self._latest_positions_t = 0.0
        rospy.Subscriber(self.servo_topic, Float64MultiArray, self._cb_servo, queue_size=1)
        rospy.on_shutdown(self._on_shutdown)

        # Simple runtime stats to debug effective loop rate / seq rate
        self._stat_t = time.time()
        self._stat_loops = 0
        self._stat_seq_incs = 0

        rospy.loginfo(
            "servo_array_to_nt ready. servo_topic=%s joints=%s",
            self.servo_topic,
            ",".join(self.joint_order),
        )

        # IMPORTANT:
        # Do NOT rely on rospy.Timer for this watchdog loop on embedded/Docker;
        # we've observed severe jitter (effective ~10Hz). We'll run an explicit Rate() loop in spin().

    def _set_root_keepalive(self):
        now = time.time()
        if (now - self._last_root_keepalive_t) < self.root_keepalive_s:
            return
        self._last_root_keepalive_t = now
        self._pub_root_active.set(str(self.active_joint))
        self._pub_root_monitor.set(bool(self.monitor_only))

    def _set_joint_keepalive(self):
        now = time.time()
        if (now - self._last_joint_keepalive_t) < self.joint_keepalive_s:
            return
        self._last_joint_keepalive_t = now
        for ros_joint, nt_joint in self.joint_map.items():
            p = self._pub.get(ros_joint)
            if not p:
                continue
            p["enable"].set(True)
            p["timeout"].set(float(self.ros_timeout_s))

    def _on_shutdown(self):
        try:
            for ros_joint, nt_joint in self.joint_map.items():
                p = self._pub.get(ros_joint)
                if not p:
                    continue
                p["enable"].set(False)
            # Best-effort final flush (NT4)
            try:
                self._nt_inst.flush()
            except Exception:
                pass
        except Exception:
            pass

    def _cb_servo(self, msg: Float64MultiArray):
        # Store latest setpoint from Servo; actual NT writing happens in timer for watchdog continuity.
        self._latest_positions = list(msg.data)
        self._latest_positions_t = time.time()

    def write_once(self):
        self._set_root_keepalive()
        # Always enforce enable + timeout at least once per keepalive interval,
        # and also refresh on every write for clarity/safety.
        self._set_joint_keepalive()

        if self._latest_positions is None:
            return

        data = self._latest_positions
        for i, ros_joint in enumerate(self.joint_order):
            if ros_joint not in self.joint_map:
                continue
            if i >= len(data):
                continue

            pos = float(data[i])
            p = self._pub.get(ros_joint)
            if not p:
                continue

            # Required watchdog fields: enable + seq increments + pos/vel/acc update
            p["enable"].set(True)
            p["timeout"].set(float(self.ros_timeout_s))
            p["pos"].set(pos)
            p["vel"].set(0.0)
            p["acc"].set(0.0)

            self._seq[ros_joint] = int(self._seq.get(ros_joint, 0) + 1)
            p["seq"].set(int(self._seq[ros_joint]))
            self._stat_seq_incs += 1

    def spin(self):
        rate_hz = max(1.0, float(self.nt_write_rate))
        rate = rospy.Rate(rate_hz)
        rospy.loginfo(
            "servo_array_to_nt NT4 loop started. nt_write_rate=%.3fHz ros_timeout_s=%.3fs",
            self.nt_write_rate,
            self.ros_timeout_s,
        )
        while not rospy.is_shutdown():
            self._stat_loops += 1
            self.write_once()
            now = time.time()
            if (now - self._stat_t) >= 1.0:
                dt = now - self._stat_t
                loops_hz = self._stat_loops / dt
                seq_hz = self._stat_seq_incs / dt
                rospy.loginfo("servo_array_to_nt stats: loop=%.1fHz seq_incs=%.1fHz (over %.2fs)", loops_hz, seq_hz, dt)
                self._stat_t = now
                self._stat_loops = 0
                self._stat_seq_incs = 0
            rate.sleep()


def main():
    rospy.init_node("servo_array_to_nt", anonymous=False)
    node = ServoArrayToNT()
    node.spin()


if __name__ == "__main__":
    main()


