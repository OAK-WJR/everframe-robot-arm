#!/usr/bin/env python3
import math
import time
from typing import Dict, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from visualization_msgs.msg import InteractiveMarkerFeedback

from networktables import NetworkTables


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MoveItMarkerFeedbackToArmNT:
    """
    订阅 MoveIt RViz 插件自带的 InteractiveMarker feedback（无需按钮，拖动即触发），
    提取 xyz -> 调 /compute_ik -> 写 NetworkTables: Arm/Jx/TargetDeg
    """

    def __init__(self):
        self.server_ip = rospy.get_param("~server_ip", "10.1.14.2")
        self.feedback_topic = rospy.get_param(
            "~feedback_topic",
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
        )

        self.group_name = rospy.get_param("~group_name", "arm")
        self.ik_link = rospy.get_param("~ik_link", "J5_1")
        self.planning_frame = rospy.get_param("~planning_frame", "world")
        self.active_joint = rospy.get_param("~active_joint", "Both")

        rpy = rospy.get_param("~target_rpy", [0.0, 0.0, 0.0])
        self.target_quat = quat_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        self.joint_map: Dict[str, str] = rospy.get_param(
            "~joint_map",
            {"joint1": "J1", "joint2": "J2", "joint3": "J3"},
        )

        # 角度标定：out_deg = sign * in_deg + offset_deg
        self.joint_sign: Dict[str, float] = rospy.get_param("~joint_sign", {})
        self.joint_offset_deg: Dict[str, float] = rospy.get_param("~joint_offset_deg", {})
        self.wrap_deg: str = str(rospy.get_param("~wrap_deg", "none")).lower()

        self.avoid_collisions = bool(rospy.get_param("~avoid_collisions", False))
        self.ik_timeout = float(rospy.get_param("~ik_timeout", 0.05))
        self.deg_round = int(rospy.get_param("~deg_round", 3))

        # 限流：拖动 feedback 很频繁，默认 20Hz
        self.max_rate = float(rospy.get_param("~max_rate", 20.0))
        self._last_send_t = 0.0

        # 触发方式：
        # - mouse_up：松开鼠标才下发一次（相当于“确认”）
        # - pose_update：拖动过程中持续下发（实时）
        trigger = str(rospy.get_param("~trigger_event", "mouse_up")).lower()
        self.trigger_on_pose_update = trigger in ("pose_update", "continuous", "stream")

        # 平滑：把“目标角突变”变成“匀速跟随”，避免电机一卡一卡
        # 0 表示关闭（保持原行为：算到啥就立刻下发啥）
        self.slew_rate_deg_per_s = float(rospy.get_param("~slew_rate_deg_per_s", 0.0))
        self.send_rate = float(rospy.get_param("~send_rate", 50.0))  # Hz
        self._desired_deg: Dict[str, float] = {}
        self._cmd_deg: Dict[str, float] = {}
        self._last_step_t = time.time()

        # Seed from joint_states
        self.latest_js: Optional[JointState] = None
        rospy.Subscriber("/joint_states", JointState, self.cb_joint_state, queue_size=1)

        rospy.wait_for_service("/compute_ik")
        self.compute_ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        NetworkTables.initialize(server=self.server_ip)
        self.nt_arm = NetworkTables.getTable("Arm")

        rospy.Subscriber(self.feedback_topic, InteractiveMarkerFeedback, self.cb_feedback, queue_size=1)

        if self.slew_rate_deg_per_s > 0.0:
            rospy.Timer(rospy.Duration(1.0 / max(self.send_rate, 1.0)), self._timer_send)

        rospy.loginfo(
            "moveit_marker_feedback_to_arm_nt ready. feedback_topic=%s server_ip=%s",
            self.feedback_topic,
            self.server_ip,
        )

    def cb_joint_state(self, msg: JointState):
        self.latest_js = msg

    def _build_seed_state(self) -> RobotState:
        rs = RobotState()
        if self.latest_js:
            rs.joint_state = self.latest_js
        return rs

    def _compute_ik(self, pose: PoseStamped) -> Optional[Dict[str, float]]:
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link
        req.ik_request.pose_stamped = pose
        req.ik_request.robot_state = self._build_seed_state()
        req.ik_request.avoid_collisions = self.avoid_collisions
        req.ik_request.timeout = rospy.Duration.from_sec(self.ik_timeout)

        try:
            res = self.compute_ik(req)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(1.0, "compute_ik 调用失败: %s", e)
            return None

        if int(res.error_code.val) != 1:
            return None

        sol = {}
        for n, p in zip(res.solution.joint_state.name, res.solution.joint_state.position):
            sol[str(n)] = float(p)
        return sol

    def _write_targets(self, joint_rad: Dict[str, float]):
        self.nt_arm.putString("ActiveJoint", str(self.active_joint))
        for ros_joint, nt_joint in self.joint_map.items():
            if ros_joint not in joint_rad:
                continue
            deg = math.degrees(joint_rad[ros_joint])
            deg = float(self.joint_sign.get(ros_joint, 1.0)) * deg + float(self.joint_offset_deg.get(ros_joint, 0.0))
            if self.wrap_deg == "360":
                deg = deg % 360.0
            elif self.wrap_deg == "180":
                deg = (deg + 180.0) % 360.0 - 180.0
            deg = round(deg, self.deg_round) if self.deg_round >= 0 else deg
            self.nt_arm.getSubTable(nt_joint).putNumber("TargetDeg", deg)

    def _write_targets_deg(self, joint_deg: Dict[str, float]):
        self.nt_arm.putString("ActiveJoint", str(self.active_joint))
        for ros_joint, nt_joint in self.joint_map.items():
            if ros_joint not in joint_deg:
                continue
            deg = joint_deg[ros_joint]
            deg = round(deg, self.deg_round) if self.deg_round >= 0 else deg
            self.nt_arm.getSubTable(nt_joint).putNumber("TargetDeg", deg)

    def _timer_send(self, _evt):
        if not self._desired_deg:
            return
        now = time.time()
        dt = max(1e-3, now - self._last_step_t)
        self._last_step_t = now

        max_step = self.slew_rate_deg_per_s * dt
        updated = {}
        for j, target in self._desired_deg.items():
            cur = self._cmd_deg.get(j, target)
            err = target - cur
            if abs(err) <= max_step:
                cur2 = target
            else:
                cur2 = cur + (max_step if err > 0 else -max_step)
            self._cmd_deg[j] = cur2
            updated[j] = cur2

        self._write_targets_deg(updated)

    def cb_feedback(self, fb: InteractiveMarkerFeedback):
        # “确认式”默认只在松开鼠标时处理；如需实时则允许 POSE_UPDATE
        if self.trigger_on_pose_update:
            if fb.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP):
                return
        else:
            if fb.event_type != InteractiveMarkerFeedback.MOUSE_UP:
                return

        now = time.time()
        if self.trigger_on_pose_update and self.max_rate > 0.0 and (now - self._last_send_t) < (1.0 / self.max_rate):
            return
        self._last_send_t = now

        # fb.pose 是 marker 在 fb.header.frame_id 下的位姿
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = fb.pose.position.x
        pose.pose.position.y = fb.pose.position.y
        pose.pose.position.z = fb.pose.position.z
        pose.pose.orientation = self.target_quat

        sol = self._compute_ik(pose)
        if sol is None:
            rospy.logwarn_throttle(0.5, "IK 失败（目标可能不可达/超时太短）")
            return

        if self.slew_rate_deg_per_s > 0.0:
            # 只更新“期望角”，由定时器平滑下发
            desired = {}
            for j in self.joint_map.keys():
                if j not in sol:
                    continue
                d = math.degrees(sol[j])
                d = float(self.joint_sign.get(j, 1.0)) * d + float(self.joint_offset_deg.get(j, 0.0))
                if self.wrap_deg == "360":
                    d = d % 360.0
                elif self.wrap_deg == "180":
                    d = (d + 180.0) % 360.0 - 180.0
                desired[j] = d
            self._desired_deg = desired
        else:
            self._write_targets(sol)
        mapped = {}
        for j in self.joint_map.keys():
            if j not in sol:
                continue
            d = math.degrees(sol[j])
            d = float(self.joint_sign.get(j, 1.0)) * d + float(self.joint_offset_deg.get(j, 0.0))
            if self.wrap_deg == "360":
                d = d % 360.0
            elif self.wrap_deg == "180":
                d = (d + 180.0) % 360.0 - 180.0
            mapped[j] = round(d, 2)
        rospy.loginfo_throttle(0.5, "IM xyz=(%.3f, %.3f, %.3f) -> deg %s", fb.pose.position.x, fb.pose.position.y, fb.pose.position.z, mapped)


def main():
    rospy.init_node("moveit_marker_feedback_to_arm_nt", anonymous=False)
    MoveItMarkerFeedbackToArmNT()
    rospy.spin()


if __name__ == "__main__":
    main()



