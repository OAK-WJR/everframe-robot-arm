#!/usr/bin/env python3
import math
from typing import Dict, Optional, Tuple

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (enables tf2 conversions)

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


class XyzToArmNT:
    def __init__(self):
        # -------- ROS params --------
        self.server_ip = rospy.get_param("~server_ip", "10.1.14.2")
        self.xyz_topic = rospy.get_param("~xyz_topic", "/arm_xyz_goal")
        self.group_name = rospy.get_param("~group_name", "arm")
        self.ik_link = rospy.get_param("~ik_link", "J5_1")
        self.planning_frame = rospy.get_param("~planning_frame", "world")
        self.active_joint = rospy.get_param("~active_joint", "Both")

        # fixed orientation (rpy in rad). If you truly only want xyz, keep (0,0,0).
        rpy = rospy.get_param("~target_rpy", [0.0, 0.0, 0.0])
        self.target_quat = quat_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        # joint mapping: MoveIt joint -> NT joint table name
        self.joint_map: Dict[str, str] = rospy.get_param(
            "~joint_map",
            {
                "joint1": "J1",
                "joint2": "J2",
                "joint3": "J3",
                # joint4 默认不下发（真机 J4 未接时更安全）
                # "joint4": "J4",
            },
        )

        # 角度标定：解决“方向反了/零位不一致/想全用正数”等问题
        # out_deg = sign * in_deg + offset_deg
        # 例：{"joint2": -1} 表示把 joint2 角度取反
        self.joint_sign: Dict[str, float] = rospy.get_param("~joint_sign", {})
        self.joint_offset_deg: Dict[str, float] = rospy.get_param("~joint_offset_deg", {})
        # wrap: "none" | "180" ([-180,180)) | "360" ([0,360))
        self.wrap_deg: str = str(rospy.get_param("~wrap_deg", "none")).lower()

        self.deg_round = int(rospy.get_param("~deg_round", 3))
        self.write_all_targets = bool(rospy.get_param("~write_all_targets", True))
        self.avoid_collisions = bool(rospy.get_param("~avoid_collisions", False))
        self.ik_timeout = float(rospy.get_param("~ik_timeout", 0.05))

        # -------- TF --------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # -------- JointState seed --------
        self.latest_js: Optional[JointState] = None
        rospy.Subscriber("/joint_states", JointState, self.cb_joint_state, queue_size=1)

        # -------- MoveIt IK service --------
        rospy.wait_for_service("/compute_ik")
        self.compute_ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        # -------- NetworkTables --------
        NetworkTables.initialize(server=self.server_ip)
        NetworkTables.addConnectionListener(self._nt_conn_listener, immediateNotify=True)
        self.nt_arm = NetworkTables.getTable("Arm")

        rospy.Subscriber(self.xyz_topic, PointStamped, self.cb_xyz, queue_size=1)
        rospy.loginfo(
            "xyz_to_arm_nt ready. xyz_topic=%s server_ip=%s group=%s ik_link=%s planning_frame=%s",
            self.xyz_topic,
            self.server_ip,
            self.group_name,
            self.ik_link,
            self.planning_frame,
        )

    def _nt_conn_listener(self, connected, info):
        if connected:
            rospy.loginfo("NetworkTables 已连接: %s", getattr(info, "remote_id", "unknown"))
        else:
            rospy.logwarn("NetworkTables 已断开: %s", getattr(info, "remote_id", "unknown"))

    def cb_joint_state(self, msg: JointState):
        self.latest_js = msg

    def _transform_point(self, pt: PointStamped) -> Optional[PointStamped]:
        if not pt.header.frame_id or pt.header.frame_id == self.planning_frame:
            return pt
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.planning_frame, pt.header.frame_id, rospy.Time(0), rospy.Duration(0.2)
            )
            return tf2_geometry_msgs.do_transform_point(pt, tfm)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF 变换失败 (%s -> %s): %s", pt.header.frame_id, self.planning_frame, e)
            return None

    def _build_seed_state(self) -> RobotState:
        rs = RobotState()
        if self.latest_js:
            # Use the latest joint_states as seed (best for convergence / picking the "near" IK solution).
            rs.joint_state = self.latest_js
        return rs

    def _compute_ik(self, pose: PoseStamped) -> Optional[Tuple[Dict[str, float], int]]:
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
            rospy.logwarn_throttle(1.0, "compute_ik 服务调用失败: %s", e)
            return None

        code = int(res.error_code.val)
        if code != 1:
            rospy.logwarn_throttle(0.5, "IK 失败 error_code=%d (目标可能不可达/姿态约束太强/超时太短)", code)
            return None

        sol = {}
        names = list(res.solution.joint_state.name)
        pos = list(res.solution.joint_state.position)
        for n, p in zip(names, pos):
            sol[n] = float(p)
        return sol, code

    def _write_targets_to_nt(self, joint_rad: Dict[str, float]):
        # global switch: allow robot side decide outputs
        self.nt_arm.putString("ActiveJoint", str(self.active_joint))

        for ros_joint, nt_joint in self.joint_map.items():
            if ros_joint not in joint_rad:
                continue
            deg = math.degrees(joint_rad[ros_joint])
            sign = float(self.joint_sign.get(ros_joint, 1.0))
            offset = float(self.joint_offset_deg.get(ros_joint, 0.0))
            deg = sign * deg + offset
            if self.wrap_deg == "360":
                deg = deg % 360.0
            elif self.wrap_deg == "180":
                deg = (deg + 180.0) % 360.0 - 180.0
            if self.deg_round >= 0:
                deg = round(deg, self.deg_round)
            self.nt_arm.getSubTable(nt_joint).putNumber("TargetDeg", deg)

    def cb_xyz(self, msg: PointStamped):
        pt = self._transform_point(msg)
        if pt is None:
            return

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = pt.point.x
        pose.pose.position.y = pt.point.y
        pose.pose.position.z = pt.point.z
        pose.pose.orientation = self.target_quat

        out = self._compute_ik(pose)
        if out is None:
            return

        joint_rad, _ = out
        self._write_targets_to_nt(joint_rad)

        # short log with mapped joints
        mapped = {}
        for j in self.joint_map.keys():
            if j not in joint_rad:
                continue
            d = math.degrees(joint_rad[j])
            d = float(self.joint_sign.get(j, 1.0)) * d + float(self.joint_offset_deg.get(j, 0.0))
            if self.wrap_deg == "360":
                d = d % 360.0
            elif self.wrap_deg == "180":
                d = (d + 180.0) % 360.0 - 180.0
            mapped[j] = round(d, 2)
        rospy.loginfo_throttle(0.5, "IK OK xyz=(%.3f, %.3f, %.3f) -> deg %s", pt.point.x, pt.point.y, pt.point.z, mapped)


def main():
    rospy.init_node("xyz_to_arm_nt", anonymous=False)
    XyzToArmNT()
    rospy.spin()


if __name__ == "__main__":
    main()



