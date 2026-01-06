#!/usr/bin/env python3
"""
在 base_link 的 XY 平面画一条直线（默认 5cm）。

特点：
- 完全使用 MoveIt 标准方法：compute_ik 检查可达性 + compute_cartesian_path + execute
- 自动在当前末端附近搜索一个“整条线都可达”的起点（端点 + 采样点都 IK 成功）
- 不使用自定义 PID，不依赖 Servo Twist（避免奇异点 halt / stale 等实时问题）
"""

import copy
import math
import sys
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest


def _pose_xyz(p0: Pose, x: float, y: float, z: float) -> Pose:
    p = copy.deepcopy(p0)
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    return p


class LineDrawerXY:
    def __init__(self):
        self.group_name = rospy.get_param("~group_name", "arm")
        self.eef_link = rospy.get_param("~eef_link", "J5_1")
        self.planning_frame = rospy.get_param("~planning_frame", "base_link")

        # Line geometry
        self.length_m = float(rospy.get_param("~length_m", 0.05))
        self.axis = str(rospy.get_param("~axis", "x")).lower()  # x | y
        self.z_mode = str(rospy.get_param("~z_mode", "keep")).lower()  # keep | set
        self.z_m = float(rospy.get_param("~z_m", 0.25))

        # Search for start point
        self.search_range_m = float(rospy.get_param("~search_range_m", 0.40))
        self.search_step_m = float(rospy.get_param("~search_step_m", 0.02))
        self.sample_step_m = float(rospy.get_param("~sample_step_m", 0.01))

        # Cartesian planning
        self.eef_step = float(rospy.get_param("~eef_step", 0.005))
        self.vel_scale = float(rospy.get_param("~velocity_scaling", 0.3))
        self.acc_scale = float(rospy.get_param("~acceleration_scaling", 0.3))
        self.ik_timeout_s = float(rospy.get_param("~ik_timeout_s", 0.05))
        self.min_fraction = float(rospy.get_param("~min_fraction", 0.95))

        import moveit_commander

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
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

    def _line_endpoints(self, start_xy: Tuple[float, float], pose_ref: Pose, z: float) -> Tuple[Pose, Pose]:
        sx, sy = start_xy
        if self.axis == "y":
            ex, ey = sx, sy + self.length_m
        else:
            ex, ey = sx + self.length_m, sy
        a = _pose_xyz(pose_ref, sx, sy, z)
        b = _pose_xyz(pose_ref, ex, ey, z)
        return a, b

    def _sample_line(self, a: Pose, b: Pose) -> List[Pose]:
        ax, ay, az = a.position.x, a.position.y, a.position.z
        bx, by, bz = b.position.x, b.position.y, b.position.z
        dist = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2 + (bz - az) ** 2)
        n = int(max(1, math.ceil(dist / max(1e-6, self.sample_step_m))))
        out: List[Pose] = []
        for i in range(n + 1):
            u = i / float(n)
            x = ax + (bx - ax) * u
            y = ay + (by - ay) * u
            z = az + (bz - az) * u
            out.append(_pose_xyz(a, x, y, z))
        return out

    def find_reachable_start(self, pose_ref: Pose, z: float) -> Optional[Tuple[float, float]]:
        cur = pose_ref.position
        steps = int(max(1, math.ceil(self.search_range_m / max(1e-6, self.search_step_m))))
        offsets = [0]
        for i in range(1, steps + 1):
            offsets.extend([i, -i])  # 0, +1, -1, +2, -2...

        for ix in offsets:
            dx = ix * self.search_step_m
            for iy in offsets:
                dy = iy * self.search_step_m
                sx = float(cur.x + dx)
                sy = float(cur.y + dy)
                a, b = self._line_endpoints((sx, sy), pose_ref, z)

                # endpoints + samples must be IK-feasible
                ok = True
                for p in self._sample_line(a, b):
                    if not self._ik_ok(p):
                        ok = False
                        break
                if ok:
                    return (sx, sy)
        return None

    def go_pose(self, pose: Pose) -> None:
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose, self.eef_link)
        plan_obj = self.group.plan()
        self.group.clear_pose_targets()

        plan = None
        if hasattr(plan_obj, "joint_trajectory"):
            plan = plan_obj
        elif isinstance(plan_obj, tuple) and len(plan_obj) >= 2 and hasattr(plan_obj[1], "joint_trajectory"):
            plan = plan_obj[1]

        if plan is None or (not plan.joint_trajectory.points):
            raise RuntimeError("移动到直线起点失败：规划无轨迹点（可能 IK/规划失败）")

        try:
            plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, self.vel_scale, self.acc_scale)
        except Exception:
            pass

        ok = self.group.execute(plan, wait=True)
        self.group.stop()
        if not ok:
            raise RuntimeError("移动到直线起点失败（execute() 返回 false）")

    def draw_line(self, a: Pose, b: Pose) -> float:
        self.group.set_start_state_to_current_state()
        waypoints = [a, b]
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, self.eef_step, avoid_collisions=False)
        try:
            plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, self.vel_scale, self.acc_scale)
        except Exception:
            pass
        if fraction < self.min_fraction:
            rospy.logwarn("直线笛卡尔覆盖率不足: fraction=%.3f", fraction)
        ok = self.group.execute(plan, wait=True)
        self.group.stop()
        if not ok:
            raise RuntimeError("执行直线笛卡尔路径失败（execute() 返回 false）")
        return float(fraction)


def main():
    rospy.init_node("draw_line_xy", anonymous=True)
    d = LineDrawerXY()

    pose_ref = d.group.get_current_pose(d.eef_link).pose
    z = float(pose_ref.position.z) if d.z_mode == "keep" else float(d.z_m)

    start_xy = d.find_reachable_start(pose_ref, z)
    if start_xy is None:
        raise RuntimeError("未找到可行直线起点（建议：把机械臂移到更开阔姿态，或增大 search_range_m）")

    a, b = d._line_endpoints(start_xy, pose_ref, z)
    rospy.loginfo("line axis=%s length=%.3f start=(%.3f, %.3f) z=%.3f", d.axis, d.length_m, start_xy[0], start_xy[1], z)
    rospy.loginfo("go to line start...")
    d.go_pose(a)
    rospy.loginfo("draw line...")
    frac = d.draw_line(a, b)
    rospy.loginfo("done. cartesian fraction=%.3f", frac)


if __name__ == "__main__":
    main()



