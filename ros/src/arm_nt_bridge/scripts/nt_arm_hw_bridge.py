#!/usr/bin/env python3
"""
NetworkTables <-> ros_control hardware bridge

This node is meant to be used with arm_control/src/arm_hw_node.cpp when ~sim_mode:=false.

- Publishes:  sensor_msgs/JointState on ~state_topic (rad) from Arm/Jx/AngleDeg (deg)
- Writes setpoints to RoboRIO via NT4.

Modes:
- mode=target_deg (legacy):
- Subscribes: sensor_msgs/JointState on ~command_topic (rad) and writes Arm/Jx/TargetDeg (deg)
- mode=scheme_b (recommended):
  - Publishes per joint (Arm/Jx/):
    - RosEnable (bool)
    - RosTimeoutS (double)
    - RosPosRad (double)
    - RosVelRadPerS (double)
    - RosAccRadPerS2 (double)
    - RosSeq (int preferred; falls back to double)
  - Input source:
    - cmd_source=jtc_state: subscribes /arm_controller/state (control_msgs/JointTrajectoryControllerState) and uses desired pos/vel/acc
    - cmd_source=joint_state_cmd: subscribes ~command_topic (JointState) as positions-only (vel/acc=0)
- Also sets root keys:
  - Arm/ActiveJoint (default "Both")
  - Arm/MonitorOnly (default False)  NOTE: if True, RoboRIO will output 0V and TargetDeg won't move the arm.

Keys used (per your doc):
  Arm/Jx/AngleDeg
  Arm/Jx/TargetDeg (mode=target_deg)
  Arm/Jx/TestEnable (mode=target_deg, forced False when command arrives)
  Arm/Jx/RosEnable (mode=scheme_b)
  Arm/Jx/RosTimeoutS (mode=scheme_b)
  Arm/Jx/RosPosRad (mode=scheme_b)
  Arm/Jx/RosVelRadPerS (mode=scheme_b)
  Arm/Jx/RosAccRadPerS2 (mode=scheme_b)
  Arm/Jx/RosSeq (mode=scheme_b)
  Arm/ActiveJoint
  Arm/MonitorOnly
"""

import math
import time
from typing import Dict, Optional

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

try:
    import ntcore  # NT4 client (pyntcore)
except Exception as e:  # pragma: no cover
    ntcore = None
    _IMPORT_ERR = e


class NTArmHWBridge:
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

        self.state_topic = rospy.get_param("~state_topic", "/arm_hw/nt_state")
        self.command_topic = rospy.get_param("~command_topic", "/arm_hw/nt_command")

        # ROS joint name -> NT subtable name (J1/J2/J3)
        self.joint_map: Dict[str, str] = rospy.get_param(
            "~joint_map",
            {"joint1": "J1", "joint2": "J2", "joint3": "J3"},
        )

        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.warn_grace_period_s = float(rospy.get_param("~warn_grace_period_s", 2.0))
        self._start_t = time.time()
        self._last_warn_t = 0.0

        # Behavior mode
        self.mode = str(rospy.get_param("~mode", "scheme_b")).strip().lower()
        if self.mode not in ("scheme_b", "target_deg"):
            rospy.logwarn("~mode=%s not recognized; falling back to scheme_b", self.mode)
            self.mode = "scheme_b"

        # Command write throttling (to reduce jitter / NT traffic)
        # RoboRIO 侧周期通常 20ms；默认最多 50Hz 写 TargetDeg
        self.min_command_period_s = float(rospy.get_param("~min_command_period_s", 0.02))
        # 小于该阈值的目标变化不写（单位：deg）
        self.command_deadband_deg = float(rospy.get_param("~command_deadband_deg", 0.05))
        self._last_cmd_write_t = 0.0
        self._last_written_deg: Dict[str, Optional[float]] = {j: None for j in self.joint_map.keys()}
        self._cmd_write_count = 0
        self._last_cmd_stat_t = time.time()
        self.log_cmd_stats = bool(rospy.get_param("~log_cmd_stats", True))

        # Root keys behavior
        self.active_joint = rospy.get_param("~active_joint", "Both")
        self.monitor_only = bool(rospy.get_param("~monitor_only", False))
        self.keepalive_s = float(rospy.get_param("~root_keepalive_s", 1.0))
        self._last_keepalive_t = 0.0

        # Scheme B parameters
        self.ros_timeout_s = float(rospy.get_param("~ros_timeout_s", 0.25))
        self.cmd_source = str(rospy.get_param("~cmd_source", "jtc_state")).strip().lower()
        self.jtc_state_topic = str(rospy.get_param("~jtc_state_topic", "/arm_controller/state"))

        # Last desired setpoint (rad / rad/s / rad/s^2)
        self._des_pos_rad: Dict[str, float] = {j: 0.0 for j in self.joint_map.keys()}
        self._des_vel_rad_s: Dict[str, float] = {j: 0.0 for j in self.joint_map.keys()}
        self._des_acc_rad_s2: Dict[str, float] = {j: 0.0 for j in self.joint_map.keys()}
        self._seq = 0

        # NT4 client
        self._nt_inst = ntcore.NetworkTableInstance.getDefault()
        self._nt_inst.setServer(str(self.server_ip))
        self._nt_inst.startClient4("jetson_ros_control_bridge")
        self.nt = self._nt_inst.getTable(str(self.nt_table))

        # Root publishers
        self._pub_root_active = self.nt.getStringTopic("ActiveJoint").publish()
        self._pub_root_monitor = self.nt.getBooleanTopic("MonitorOnly").publish()

        # Per-joint publishers/subscribers
        self._pub_target_deg = {}       # mode=target_deg
        self._pub_test_enable = {}      # mode=target_deg
        self._pub_ros_enable = {}       # mode=scheme_b
        self._pub_ros_timeout = {}      # mode=scheme_b
        self._pub_ros_pos = {}          # mode=scheme_b
        self._pub_ros_vel = {}          # mode=scheme_b
        self._pub_ros_acc = {}          # mode=scheme_b
        self._pub_ros_seq = {}          # mode=scheme_b (int preferred)
        self._sub_angle_deg = {}
        for ros_joint, nt_joint in self.joint_map.items():
            st = self.nt.getSubTable(str(nt_joint))
            self._pub_target_deg[ros_joint] = st.getDoubleTopic("TargetDeg").publish()
            self._pub_test_enable[ros_joint] = st.getBooleanTopic("TestEnable").publish()

            self._pub_ros_enable[ros_joint] = st.getBooleanTopic("RosEnable").publish()
            self._pub_ros_timeout[ros_joint] = st.getDoubleTopic("RosTimeoutS").publish()
            self._pub_ros_pos[ros_joint] = st.getDoubleTopic("RosPosRad").publish()
            self._pub_ros_vel[ros_joint] = st.getDoubleTopic("RosVelRadPerS").publish()
            self._pub_ros_acc[ros_joint] = st.getDoubleTopic("RosAccRadPerS2").publish()
            try:
                self._pub_ros_seq[ros_joint] = st.getIntegerTopic("RosSeq").publish()
            except Exception:
                self._pub_ros_seq[ros_joint] = st.getDoubleTopic("RosSeq").publish()
            self._sub_angle_deg[ros_joint] = st.getDoubleTopic("AngleDeg").subscribe(float("nan"))

        self.pub = rospy.Publisher(self.state_topic, JointState, queue_size=1)
        self.sub = None
        self.sub_jtc = None
        if self.mode == "target_deg":
        self.sub = rospy.Subscriber(self.command_topic, JointState, self._cmd_cb, queue_size=1)
        else:
            if self.cmd_source == "jtc_state":
                self.sub_jtc = rospy.Subscriber(self.jtc_state_topic, JointTrajectoryControllerState, self._jtc_state_cb, queue_size=1)
            else:
                self.sub = rospy.Subscriber(self.command_topic, JointState, self._cmd_pos_only_cb, queue_size=1)

        rospy.loginfo(
            "nt_arm_hw_bridge ready. server_ip=%s table=%s state=%s cmd=%s joints=%s",
            self.server_ip,
            self.nt_table,
            self.state_topic,
            self.command_topic,
            ",".join(self.joint_map.keys()),
        )
        rospy.logwarn(
            "Root keys will be set: Arm/ActiveJoint=%s Arm/MonitorOnly=%s (MonitorOnly=true will disable motor output).",
            self.active_joint,
            self.monitor_only,
        )
        rospy.loginfo("nt_arm_hw_bridge mode=%s cmd_source=%s rate=%.1fHz ros_timeout_s=%.3f",
                      self.mode, self.cmd_source, self.rate_hz, self.ros_timeout_s)

    def _set_root_keys(self):
        now = time.time()
        if (now - self._last_keepalive_t) < self.keepalive_s:
            return
        self._last_keepalive_t = now
        # Root keys
        self._pub_root_active.set(str(self.active_joint))
        self._pub_root_monitor.set(bool(self.monitor_only))

    def _cmd_cb(self, msg: JointState):
        # Update cached commands
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                continue
            try:
                # store in desired cache so we can re-use in scheme_b fallback if needed
                if name in self._des_pos_rad:
                    self._des_pos_rad[name] = float(msg.position[i])
            except Exception:
                continue

        # Throttle + deadband before pushing to NT
        now = time.time()
        if self.min_command_period_s > 0 and (now - self._last_cmd_write_t) < self.min_command_period_s:
            return
        self._last_cmd_write_t = now

        for ros_joint, nt_joint in self.joint_map.items():
            rad = self._des_pos_rad.get(ros_joint, None)
            if rad is None:
                continue
            deg = float(math.degrees(float(rad)))
            last_deg = self._last_written_deg.get(ros_joint, None)
            if last_deg is not None and abs(deg - last_deg) < self.command_deadband_deg:
                continue

            pub_t = self._pub_target_deg.get(ros_joint)
            pub_te = self._pub_test_enable.get(ros_joint)
            if pub_t is None or pub_te is None:
                continue
            pub_t.set(float(deg))
            # ensure manual goal is used
            pub_te.set(False)
            self._last_written_deg[ros_joint] = deg
            self._cmd_write_count += 1

        # Optional stats
        if self.log_cmd_stats:
            now2 = time.time()
            if (now2 - self._last_cmd_stat_t) >= 1.0:
                rospy.loginfo("NT cmd writes last 1s: %d", self._cmd_write_count)
                self._cmd_write_count = 0
                self._last_cmd_stat_t = now2

    def _cmd_pos_only_cb(self, msg: JointState):
        # positions-only input for scheme_b fallback (vel/acc=0)
        for i, name in enumerate(msg.name):
            if name not in self._des_pos_rad:
                continue
            if i < len(msg.position):
                try:
                    self._des_pos_rad[name] = float(msg.position[i])
                    self._des_vel_rad_s[name] = 0.0
                    self._des_acc_rad_s2[name] = 0.0
                except Exception:
                    pass

    def _jtc_state_cb(self, msg: JointTrajectoryControllerState):
        # Use desired values from JTC
        for i, name in enumerate(msg.joint_names):
            if name not in self._des_pos_rad:
                continue
            if i < len(msg.desired.positions):
                try:
                    self._des_pos_rad[name] = float(msg.desired.positions[i])
                except Exception:
                    pass
            if i < len(msg.desired.velocities):
                try:
                    self._des_vel_rad_s[name] = float(msg.desired.velocities[i])
                except Exception:
                    pass
            if i < len(msg.desired.accelerations):
                try:
                    self._des_acc_rad_s2[name] = float(msg.desired.accelerations[i])
                except Exception:
                    pass

    def _publish_scheme_b(self):
        # Always publish at rate_hz and bump RosSeq each cycle (watchdog contract)
        self._seq += 1
        for ros_joint in self.joint_map.keys():
            self._pub_ros_enable[ros_joint].set(True)
            self._pub_ros_timeout[ros_joint].set(float(self.ros_timeout_s))
            self._pub_ros_pos[ros_joint].set(float(self._des_pos_rad.get(ros_joint, 0.0)))
            self._pub_ros_vel[ros_joint].set(float(self._des_vel_rad_s.get(ros_joint, 0.0)))
            self._pub_ros_acc[ros_joint].set(float(self._des_acc_rad_s2.get(ros_joint, 0.0)))
            pub_seq = self._pub_ros_seq.get(ros_joint)
            try:
                pub_seq.set(int(self._seq))
            except Exception:
                pub_seq.set(float(self._seq))

    def spin(self):
        rate = rospy.Rate(max(self.rate_hz, 1.0))
        while not rospy.is_shutdown():
            self._set_root_keys()
            if self.mode == "scheme_b":
                self._publish_scheme_b()

            names = []
            pos = []
            missing = []

            for ros_joint, nt_joint in self.joint_map.items():
                sub = self._sub_angle_deg.get(ros_joint)
                if sub is None:
                    missing.append(ros_joint)
                    continue
                try:
                    deg = float(sub.get())
                    if math.isnan(deg):
                        raise ValueError("AngleDeg is NaN")
                    rad = math.radians(deg)
                except Exception:
                    missing.append(ros_joint)
                    continue
                names.append(ros_joint)
                pos.append(rad)

            if missing:
                now = time.time()
                if (now - self._start_t) >= self.warn_grace_period_s and (now - self._last_warn_t) > 1.0:
                    self._last_warn_t = now
                    rospy.logwarn("Missing AngleDeg for joints: %s", ",".join(missing))

            if names:
                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name = names
                js.position = pos
                self.pub.publish(js)

            rate.sleep()


def main():
    rospy.init_node("nt_arm_hw_bridge", anonymous=False)
    node = NTArmHWBridge()
    node.spin()


if __name__ == "__main__":
    main()



