#!/usr/bin/env python3
"""
在 XY 平面画一个正方形（默认 10cm×10cm）。

特点：
- 使用 MoveIt MoveGroupCommander(group=arm)
- 自动在当前末端附近搜索一个“4 个角都可达”的起点（保证能画完整正方形）
- 可选锁住 joint4（尽量只动 J1/J2/J3）
"""

import copy
import math
import sys
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest


def _pose_with_xy(p0: Pose, x: float, y: float, z: float) -> Pose:
    p = copy.deepcopy(p0)
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    return p


def _make_joint4_lock(joint4_name: str, joint4_val: float, tol: float = 1e-3) -> Constraints:
    c = Constraints()
    jc = JointConstraint()
    jc.joint_name = joint4_name
    jc.position = float(joint4_val)
    jc.tolerance_above = float(tol)
    jc.tolerance_below = float(tol)
    jc.weight = 1.0
    c.joint_constraints = [jc]
    return c


class SquareDrawer:
    def __init__(self):
        self.group_name = rospy.get_param("~group_name", "arm")
        self.eef_link = rospy.get_param("~eef_link", "J5_1")
        self.planning_frame = rospy.get_param("~planning_frame", "base_link")

        self.side_m = float(rospy.get_param("~side_m", 0.10))
        self.eef_step = float(rospy.get_param("~eef_step", 0.005))
        self.search_step_m = float(rospy.get_param("~search_step_m", 0.02))
        self.search_range_m = float(rospy.get_param("~search_range_m", 0.20))
        self.z_mode = str(rospy.get_param("~z_mode", "keep")).lower()  # keep | set
        self.z_m = float(rospy.get_param("~z_m", 0.25))

        self.lock_joint4 = bool(rospy.get_param("~lock_joint4", True))
        self.joint4_name = rospy.get_param("~joint4_name", "joint4")
        self.joint4_tol = float(rospy.get_param("~joint4_tol", 1e-3))

        self.vel_scale = float(rospy.get_param("~velocity_scaling", 0.2))
        self.acc_scale = float(rospy.get_param("~acceleration_scaling", 0.2))
        self.planning_time = float(rospy.get_param("~planning_time", 3.0))
        self.planning_attempts = int(rospy.get_param("~planning_attempts", 10))
        self.ik_timeout_s = float(rospy.get_param("~ik_timeout_s", 0.05))

        import moveit_commander

        moveit_commander.roscpp_initialize(sys.argv)
        self._moveit = moveit_commander
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.planning_attempts)
        self.group.set_max_velocity_scaling_factor(self.vel_scale)
        self.group.set_max_acceleration_scaling_factor(self.acc_scale)

        rospy.wait_for_service("/compute_ik", timeout=10.0)
        self.ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

    def _ik_ok(self, pose: Pose) -> bool:
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = self.robot.get_current_state()
        req.ik_request.ik_link_name = self.eef_link
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.planning_frame
        ps.pose = pose
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout = rospy.Duration(self.ik_timeout_s)
        req.ik_request.avoid_collisions = False
        res = self.ik_srv(req)
        return res.error_code.val == MoveItErrorCodes.SUCCESS

    def _corners(self, center_xy: Tuple[float, float], pose_ref: Pose, z: float) -> List[Pose]:
        cx, cy = center_xy
        h = self.side_m * 0.5
        # 顺序：左下 -> 右下 -> 右上 -> 左上 -> 回到左下
        pts = [
            (cx - h, cy - h),
            (cx + h, cy - h),
            (cx + h, cy + h),
            (cx - h, cy + h),
            (cx - h, cy - h),
        ]
        return [_pose_with_xy(pose_ref, x, y, z) for (x, y) in pts]

    def find_reachable_center(self, pose_ref: Pose, z: float) -> Optional[Tuple[float, float]]:
        cur = pose_ref.position
        h = self.side_m * 0.5
        # 以当前点为中心，做网格搜索；优先尝试靠近当前的点
        steps = int(max(1, math.ceil(self.search_range_m / max(1e-6, self.search_step_m))))
        candidates = [0]
        for i in range(1, steps + 1):
            candidates.extend([i, -i])

        for ix in candidates:
            dx = ix * self.search_step_m
            for iy in candidates:
                dy = iy * self.search_step_m
                cx = float(cur.x + dx + h)  # 让“当前点”更像落在正方形内部，而不是边界
                cy = float(cur.y + dy)
                corners = self._corners((cx, cy), pose_ref, z)
                ok = True
                for p in corners[:-1]:
                    if not self._ik_ok(p):
                        ok = False
                        break
                if ok:
                    return (cx, cy)
        return None

    def go_pose(self, pose: Pose) -> None:
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose, self.eef_link)
        plan_obj = self.group.plan()
        self.group.clear_pose_targets()

        # moveit_commander plan() 可能返回 RobotTrajectory 或 tuple(success, RobotTrajectory, ...)
        plan = None
        if hasattr(plan_obj, "joint_trajectory"):
            plan = plan_obj
        elif isinstance(plan_obj, tuple) and len(plan_obj) >= 2 and hasattr(plan_obj[1], "joint_trajectory"):
            plan = plan_obj[1]

        if plan is None or (not plan.joint_trajectory.points):
            raise RuntimeError("移动到起点失败：规划无轨迹点（可能 IK/规划失败）")

        # 必须 retime：否则某些情况下轨迹时间戳过小/为 0，JTC 可能直接 CONTROL_FAILED
        try:
            plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, self.vel_scale, self.acc_scale)
        except Exception:
            pass

        ok = self.group.execute(plan, wait=True)
        self.group.stop()
        if not ok:
            raise RuntimeError("移动到起点失败（execute() 返回 false）")

    def draw_square(self, corners: List[Pose]) -> float:
        self.group.set_start_state_to_current_state()
        waypoints = corners
        # ROS Noetic 签名: compute_cartesian_path(waypoints, eef_step, avoid_collisions=True, path_constraints=None)
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, self.eef_step, avoid_collisions=False
        )
        # 重新定时，速度无所谓但要有时间戳
        try:
            plan = self.group.retime_trajectory(
                self.robot.get_current_state(), plan, self.vel_scale, self.acc_scale
            )
        except Exception:
            pass
        if fraction < 0.99:
            rospy.logwarn("笛卡尔路径覆盖率不足: fraction=%.3f（可尝试减小 eef_step 或放宽约束）", fraction)
        ok = self.group.execute(plan, wait=True)
        self.group.stop()
        if not ok:
            raise RuntimeError("执行笛卡尔路径失败（execute() 返回 false）")
        return float(fraction)


def main():
    rospy.init_node("draw_square_xy", anonymous=True)
    d = SquareDrawer()

    # 当前末端姿态作为参考（kinematics.yaml 已开启 position_only_ik，姿态不会强约束）
    pose_ref = d.group.get_current_pose(d.eef_link).pose
    z = float(pose_ref.position.z) if d.z_mode == "keep" else float(d.z_m)

    # 可选：锁 joint4，尽量只用 J1/J2/J3 来完成
    if d.lock_joint4:
        jv = d.group.get_current_joint_values()
        if len(jv) >= 4:
            d.group.set_path_constraints(_make_joint4_lock(d.joint4_name, jv[3], d.joint4_tol))
            rospy.loginfo("lock_joint4 enabled: %s=%.4f ±%.4f", d.joint4_name, jv[3], d.joint4_tol)
        else:
            rospy.logwarn("lock_joint4 requested but joint_values length=%d", len(jv))

    center = d.find_reachable_center(pose_ref, z)
    if center is None and d.lock_joint4:
        rospy.logwarn("锁 joint4 未找到可行正方形起点，尝试解锁 joint4 再搜一次")
        d.group.clear_path_constraints()
        center = d.find_reachable_center(pose_ref, z)

    if center is None:
        raise RuntimeError("在搜索范围内未找到能画完整 10cm 正方形的起点（四角 IK 失败）")

    corners = d._corners(center, pose_ref, z)
    rospy.loginfo("found center: (%.3f, %.3f), z=%.3f, side=%.3f", center[0], center[1], z, d.side_m)

    # 先移动到起点（左下角）
    rospy.loginfo("go to start corner...")
    d.go_pose(corners[0])

    # 再画正方形
    rospy.loginfo("draw square...")
    frac = d.draw_square(corners)
    rospy.loginfo("done. cartesian fraction=%.3f", frac)


if __name__ == "__main__":
    main()


