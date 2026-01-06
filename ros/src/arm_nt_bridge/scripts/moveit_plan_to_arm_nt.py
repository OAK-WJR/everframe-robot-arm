#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
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


def interp(a: float, b: float, u: float) -> float:
    return a + (b - a) * u


def wrap_deg(val: float, mode: str) -> float:
    if mode == "360":
        return val % 360.0
    if mode == "180":
        return (val + 180.0) % 360.0 - 180.0
    return val


@dataclass
class TrajSample:
    t: float
    pos_rad: Dict[str, float]


class MoveItPlanToArmNT:
    """
    用 MoveIt 规划轨迹（JointTrajectory），并按固定频率采样后写入 NT 的 TargetDeg。
    相当于把“优化过的轨迹”转换成 roboRIO 能用的 deg setpoint 流。
    """

    def __init__(self):
        # NT
        self.server_ip = rospy.get_param("~server_ip", "10.1.14.2")
        self.active_joint = rospy.get_param("~active_joint", "Both")

        # 从 demo 的交互 marker 取目标
        self.feedback_topic = rospy.get_param(
            "~feedback_topic",
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
        )
        self.planning_frame = rospy.get_param("~planning_frame", "world")
        self.group_name = rospy.get_param("~group_name", "arm")
        self.ik_link = rospy.get_param("~ik_link", "J5_1")

        # 姿态策略：默认用“当前末端姿态”，减少 4DOF 姿态不可达导致的失败
        self.use_current_orientation = bool(rospy.get_param("~use_current_orientation", True))
        rpy = rospy.get_param("~target_rpy", [0.0, 0.0, 0.0])
        self.fixed_quat = quat_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        # MoveIt plan 参数
        self.planning_time = float(rospy.get_param("~planning_time", 2.0))
        self.planning_attempts = int(rospy.get_param("~planning_attempts", 5))
        self.vel_scale = float(rospy.get_param("~velocity_scaling", 0.2))
        self.acc_scale = float(rospy.get_param("~acceleration_scaling", 0.2))

        # 执行（对 roboRIO 输出）
        self.sample_rate = float(rospy.get_param("~sample_rate", 50.0))  # Hz
        self.trigger_event = str(rospy.get_param("~trigger_event", "mouse_up")).lower()  # mouse_up | pose_update
        self.max_rate = float(rospy.get_param("~max_rate", 10.0))  # 当 pose_update 时对重规划限流
        self._last_trigger_t = 0.0

        # 关节映射与标定
        self.joint_map: Dict[str, str] = rospy.get_param("~joint_map", {"joint1": "J1", "joint2": "J2", "joint3": "J3"})
        self.joint_sign: Dict[str, float] = rospy.get_param("~joint_sign", {})
        self.joint_offset_deg: Dict[str, float] = rospy.get_param("~joint_offset_deg", {})
        self.wrap_mode: str = str(rospy.get_param("~wrap_deg", "none")).lower()

        # 软限位（在下发前裁剪，单位 deg）
        self.joint_min_deg: Dict[str, float] = rospy.get_param("~joint_min_deg", {})
        self.joint_max_deg: Dict[str, float] = rospy.get_param("~joint_max_deg", {})

        # 内部状态：用于“打断当前播放并重规划”
        self._exec_id = 0

        # Init NT
        NetworkTables.initialize(server=self.server_ip)
        self.nt_arm = NetworkTables.getTable("Arm")

        # Init MoveIt (python)
        import moveit_commander  # 延迟 import，避免未安装时直接崩

        moveit_commander.roscpp_initialize([])
        self._moveit_commander = moveit_commander
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.planning_attempts)
        self.group.set_max_velocity_scaling_factor(self.vel_scale)
        self.group.set_max_acceleration_scaling_factor(self.acc_scale)

        rospy.Subscriber(self.feedback_topic, InteractiveMarkerFeedback, self.cb_feedback, queue_size=1)
        rospy.loginfo(
            "moveit_plan_to_arm_nt ready. feedback_topic=%s server_ip=%s sample_rate=%.1fHz trigger=%s",
            self.feedback_topic,
            self.server_ip,
            self.sample_rate,
            self.trigger_event,
        )

    def _event_ok(self, event_type: int) -> bool:
        if self.trigger_event == "mouse_up":
            return event_type == InteractiveMarkerFeedback.MOUSE_UP
        # pose_update / continuous
        return event_type in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP)

    def _rate_limit_ok(self) -> bool:
        if self.trigger_event == "mouse_up":
            return True
        now = time.time()
        if self.max_rate <= 0:
            return True
        if (now - self._last_trigger_t) < (1.0 / self.max_rate):
            return False
        self._last_trigger_t = now
        return True

    def _convert_deg(self, ros_joint: str, rad: float) -> float:
        deg = math.degrees(rad)
        deg = float(self.joint_sign.get(ros_joint, 1.0)) * deg + float(self.joint_offset_deg.get(ros_joint, 0.0))
        deg = wrap_deg(deg, self.wrap_mode)
        if ros_joint in self.joint_min_deg:
            deg = max(float(self.joint_min_deg[ros_joint]), deg)
        if ros_joint in self.joint_max_deg:
            deg = min(float(self.joint_max_deg[ros_joint]), deg)
        return deg

    def _write_targets_deg(self, joint_deg: Dict[str, float]):
        self.nt_arm.putString("ActiveJoint", str(self.active_joint))
        for ros_joint, nt_joint in self.joint_map.items():
            if ros_joint not in joint_deg:
                continue
            self.nt_arm.getSubTable(nt_joint).putNumber("TargetDeg", joint_deg[ros_joint])

    def _extract_plan(self, plan_obj) -> Optional[Tuple[List[str], List[TrajSample]]]:
        """
        moveit_commander 不同版本的 plan() 返回结构可能不同：
        - 可能直接返回 RobotTrajectory
        - 或 (success, RobotTrajectory, planning_time, error_code)
        """
        traj = None
        if hasattr(plan_obj, "joint_trajectory"):
            traj = plan_obj.joint_trajectory
        elif isinstance(plan_obj, tuple) and len(plan_obj) >= 2 and hasattr(plan_obj[1], "joint_trajectory"):
            traj = plan_obj[1].joint_trajectory
        if traj is None or not traj.points:
            return None
        names = list(traj.joint_names)
        samples: List[TrajSample] = []
        for pt in traj.points:
            t = float(pt.time_from_start.to_sec())
            pos = {n: float(v) for n, v in zip(names, pt.positions)}
            samples.append(TrajSample(t=t, pos_rad=pos))
        return names, samples

    def _resample(self, names: List[str], samples: List[TrajSample], rate_hz: float) -> List[Dict[str, float]]:
        if not samples:
            return []
        T = samples[-1].t
        if T <= 0:
            return [samples[-1].pos_rad]
        dt = 1.0 / max(rate_hz, 1.0)
        out: List[Dict[str, float]] = []
        i = 0
        t = 0.0
        while t <= T + 1e-6:
            while i + 1 < len(samples) and samples[i + 1].t < t:
                i += 1
            if i + 1 >= len(samples):
                out.append(samples[-1].pos_rad)
                break
            a = samples[i]
            b = samples[i + 1]
            if b.t <= a.t:
                u = 1.0
            else:
                u = (t - a.t) / (b.t - a.t)
                u = max(0.0, min(1.0, u))
            pos = {n: interp(a.pos_rad[n], b.pos_rad[n], u) for n in names}
            out.append(pos)
            t += dt
        return out

    def _play_trajectory(self, exec_id: int, names: List[str], rad_seq: List[Dict[str, float]]):
        dt = 1.0 / max(self.sample_rate, 1.0)
        rate = rospy.Rate(max(self.sample_rate, 1.0))
        for pos_rad in rad_seq:
            if exec_id != self._exec_id or rospy.is_shutdown():
                return
            deg_cmd = {}
            for ros_joint in self.joint_map.keys():
                if ros_joint not in pos_rad:
                    continue
                deg_cmd[ros_joint] = self._convert_deg(ros_joint, pos_rad[ros_joint])
            self._write_targets_deg(deg_cmd)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return
        rospy.loginfo("trajectory playback done.")

    def cb_feedback(self, fb: InteractiveMarkerFeedback):
        if not self._event_ok(fb.event_type):
            return
        if not self._rate_limit_ok():
            return

        # 构造 pose target
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = fb.pose.position.x
        pose.pose.position.y = fb.pose.position.y
        pose.pose.position.z = fb.pose.position.z

        if self.use_current_orientation:
            pose.pose.orientation = self.group.get_current_pose(self.ik_link).pose.orientation
        else:
            pose.pose.orientation = self.fixed_quat

        # 规划
        self.group.set_pose_target(pose, self.ik_link)
        plan = self.group.plan()
        parsed = self._extract_plan(plan)
        self.group.clear_pose_targets()
        if parsed is None:
            rospy.logwarn_throttle(0.5, "MoveIt 规划失败（无轨迹点）。可尝试增大 planning_time/attempts 或放宽约束")
            return

        names, samples = parsed
        rad_seq = self._resample(names, samples, self.sample_rate)

        # 打断上一次执行
        self._exec_id += 1
        exec_id = self._exec_id
        rospy.loginfo("plan OK: %d points (resampled). start playback...", len(rad_seq))
        self._play_trajectory(exec_id, names, rad_seq)


def main():
    rospy.init_node("moveit_plan_to_arm_nt", anonymous=False)
    MoveItPlanToArmNT()
    rospy.spin()


if __name__ == "__main__":
    main()



