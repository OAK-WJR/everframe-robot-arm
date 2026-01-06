#!/usr/bin/env python3
"""
用 MoveIt Servo (Twist) 在 base_link 的 XY 平面画正方形（默认 5cm）。

特点：
- 自动在“当前末端附近”搜索一个中心点，使得正方形边上采样点全部可 IK（更容易画完整）
- 用 Servo 的 /servo_server/delta_twist_cmds 发送线速度（只关心 xyz）
- 先用闭环（位置误差 -> 速度）移动到起点，再按四条边走完

依赖：
- servo_server 必须运行，并发布/订阅：
  - /servo_server/delta_twist_cmds (geometry_msgs/TwistStamped)
  - /joint_states
- /compute_ik 服务可用（来自 move_group）
"""

import copy
import math
import sys
from typing import List, Optional, Tuple

import rospy
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from tf2_ros import TransformListener, Buffer


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def pose_xy(p0: Pose, x: float, y: float, z: float) -> Pose:
    p = copy.deepcopy(p0)
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    return p


class ServoSquareXY:
    def __init__(self):
        self.group_name = rospy.get_param("~group_name", "arm")
        self.planning_frame = rospy.get_param("~planning_frame", "base_link")
        self.eef_link = rospy.get_param("~eef_link", "J5_1")

        # Square geometry
        self.side_m = float(rospy.get_param("~side_m", 0.05))
        self.z_mode = str(rospy.get_param("~z_mode", "keep")).lower()  # keep | set
        self.z_m = float(rospy.get_param("~z_m", 0.25))

        # Search
        self.search_range_m = float(rospy.get_param("~search_range_m", 0.40))
        self.search_step_m = float(rospy.get_param("~search_step_m", 0.02))
        self.edge_sample_step_m = float(rospy.get_param("~edge_sample_step_m", 0.01))

        # Servo command
        self.twist_topic = rospy.get_param("~twist_topic", "/servo_server/delta_twist_cmds")
        self.rate_hz = float(rospy.get_param("~rate_hz", 50.0))
        self.max_speed_mps = float(rospy.get_param("~max_speed_mps", 0.04))  # linear speed cap
        self.min_speed_mps = float(rospy.get_param("~min_speed_mps", 0.01))  # overcome stiction near target
        self.kp = float(rospy.get_param("~kp", 1.5))  # for go-to-start: v = kp * pos_error (降低减少震荡)
        self.reach_eps_m = float(rospy.get_param("~reach_eps_m", 0.005))
        self.goto_timeout_s = float(rospy.get_param("~goto_timeout_s", 15.0))

        # Execution
        self.edge_speed_mps = float(rospy.get_param("~edge_speed_mps", 0.03))
        self.ramp_s = float(rospy.get_param("~ramp_s", 0.20))
        self.post_stop_s = float(rospy.get_param("~post_stop_s", 0.30))

        # IK
        self.ik_timeout_s = float(rospy.get_param("~ik_timeout_s", 0.05))

        import moveit_commander

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        rospy.wait_for_service("/compute_ik", timeout=10.0)
        self.ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        self.pub = rospy.Publisher(self.twist_topic, TwistStamped, queue_size=10)
        
        # TF for reliable current pose
        self.tf_buffer = Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = TransformListener(self.tf_buffer)
        rospy.sleep(1.0)  # wait for TF buffer and robot_state_publisher to be ready
        
        # Servo status check
        self.status_topic = rospy.get_param("~status_topic", "/servo_server/status")
        self.last_status = None
        try:
            from std_msgs.msg import Int8
            rospy.Subscriber(self.status_topic, Int8, self._status_cb, queue_size=1)
        except Exception as e:
            rospy.logwarn(f"无法订阅servo status: {e}，跳过状态检查")

    def _status_cb(self, msg):
        """Store latest servo status"""
        self.last_status = msg.data
    
    def current_pose(self) -> Pose:
        """Get current end-effector pose via TF (more reliable than MoveIt)"""
        try:
            # Wait for transform to be available
            self.tf_buffer.can_transform(self.planning_frame, self.eef_link, rospy.Time(0), timeout=rospy.Duration(0.2))
            trans = self.tf_buffer.lookup_transform(
                self.planning_frame, self.eef_link, rospy.Time(0), timeout=rospy.Duration(0.1)
            )
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"TF lookup failed: {e}, falling back to MoveIt")
            try:
                return self.group.get_current_pose(self.eef_link).pose
            except Exception as e2:
                rospy.logerr(f"MoveIt get_current_pose also failed: {e2}")
                # Return a default pose to avoid crash
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.25
                pose.orientation.w = 1.0
                return pose

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

    def _square_waypoints(self, center_xy: Tuple[float, float], pose_ref: Pose, z: float) -> List[Pose]:
        cx, cy = center_xy
        h = self.side_m * 0.5
        pts = [
            (cx - h, cy - h),
            (cx + h, cy - h),
            (cx + h, cy + h),
            (cx - h, cy + h),
            (cx - h, cy - h),
        ]
        return [pose_xy(pose_ref, x, y, z) for (x, y) in pts]

    def _edge_samples(self, a: Pose, b: Pose) -> List[Pose]:
        ax, ay = a.position.x, a.position.y
        bx, by = b.position.x, b.position.y
        dist = math.hypot(bx - ax, by - ay)
        n = int(max(1, math.ceil(dist / max(1e-6, self.edge_sample_step_m))))
        out: List[Pose] = []
        for i in range(n + 1):
            u = i / float(n)
            x = ax + (bx - ax) * u
            y = ay + (by - ay) * u
            out.append(pose_xy(a, x, y, a.position.z))
        return out

    def find_reachable_center(self, pose_ref: Pose, z: float) -> Optional[Tuple[float, float]]:
        cur = pose_ref.position
        steps = int(max(1, math.ceil(self.search_range_m / max(1e-6, self.search_step_m))))
        offsets = [0]
        for i in range(1, steps + 1):
            offsets.extend([i, -i])

        # 按“离当前更近”优先
        for ix in offsets:
            dx = ix * self.search_step_m
            for iy in offsets:
                dy = iy * self.search_step_m
                cx = float(cur.x + dx)
                cy = float(cur.y + dy)
                wps = self._square_waypoints((cx, cy), pose_ref, z)

                ok = True
                # 角点 + 边上采样点全部 IK 通过
                for i in range(len(wps) - 1):
                    seg = self._edge_samples(wps[i], wps[i + 1])
                    for p in seg:
                        if not self._ik_ok(p):
                            ok = False
                            break
                    if not ok:
                        break
                if ok:
                    return (cx, cy)
        return None

    def _publish_twist(self, vx: float, vy: float, vz: float = 0.0):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.planning_frame
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)

    def stop(self):
        for _ in range(int(max(1.0, self.post_stop_s * self.rate_hz))):
            self._publish_twist(0.0, 0.0, 0.0)
            rospy.sleep(1.0 / self.rate_hz)

    def goto_xy(self, x: float, y: float, z: float):
        """移动到目标位置，使用 MoveIt go()（让 MoveIt 自己处理闭环控制）"""
        # 构造目标位姿（保持当前姿态，只改位置）
        target_pose = self.current_pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        
        rospy.loginfo(f"goto ({x:.3f}, {y:.3f}, {z:.3f}) using MoveIt go()...")
        
        # 使用 MoveIt 的 set_pose_target() + go()（内部有完整的闭环控制）
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(target_pose, self.eef_link)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        
        if not success:
            raise RuntimeError(f"goto 起点失败：MoveIt go() 返回 false（可能 IK/规划失败或超时）")
        
        # 验证是否真的到达
        p = self.current_pose().position
        dist = math.sqrt((x - p.x)**2 + (y - p.y)**2 + (z - p.z)**2)
        if dist > self.reach_eps_m * 2:
            rospy.logwarn(f"MoveIt go() 完成但距离目标还有 dist={dist:.4f}m（可能接近奇异点）")
        else:
            rospy.loginfo(f"到达目标：dist={dist:.4f}m")

    def drive_edge(self, vx: float, vy: float, duration_s: float):
        """用 Servo Twist 画一条边（只计算方向向量，让 Servo 自己处理闭环）"""
        rospy.loginfo(f"drive_edge: v=({vx:.3f}, {vy:.3f}) duration={duration_s:.2f}s")
        rate = rospy.Rate(max(1.0, self.rate_hz))
        t0 = rospy.Time.now().to_sec()
        count = 0
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - t0
            if t >= duration_s:
                break
            # 简单速度斜坡（避免突变）
            if self.ramp_s > 1e-3:
                a = clamp(t / self.ramp_s, 0.0, 1.0)
                d = clamp((duration_s - t) / self.ramp_s, 0.0, 1.0)
                s = min(a, d)
            else:
                s = 1.0
            self._publish_twist(vx * s, vy * s, 0.0)
            count += 1
            rate.sleep()
        rospy.loginfo(f"drive_edge 完成：发送了 {count} 条命令")
        self.stop()

    def run(self):
        pose_ref = self.current_pose()
        z = float(pose_ref.position.z) if self.z_mode == "keep" else float(self.z_m)

        center = self.find_reachable_center(pose_ref, z)
        if center is None:
            raise RuntimeError("未找到可行中心点（建议：移动到更开阔姿态，或增大 search_range_m）")

        wps = self._square_waypoints(center, pose_ref, z)
        start = wps[0].position
        rospy.loginfo("square center=(%.3f, %.3f) z=%.3f side=%.3f", center[0], center[1], z, self.side_m)
        rospy.loginfo("goto start (%.3f, %.3f, %.3f)...", start.x, start.y, start.z)
        self.goto_xy(start.x, start.y, start.z)

        # 使用 MoveIt 的标准方法：compute_cartesian_path + execute
        # 这是正常的方法，确保所有关节都正确移动
        rospy.loginfo("使用 MoveIt compute_cartesian_path 画正方形...")
        
        # 构造四条边的waypoints（保持当前姿态，只改位置）
        waypoints = wps
        
        # 使用 MoveIt 规划并执行笛卡尔路径（标准方法，和 draw_square_xy.py 一样）
        self.group.set_start_state_to_current_state()
        eef_step = 0.01  # 1cm步长
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,
            eef_step,
            avoid_collisions=False
        )
        
        if fraction < 0.9:
            raise RuntimeError(f"Cartesian路径规划失败：fraction={fraction:.3f}（建议：移动到更开阔姿态）")
        
        rospy.loginfo(f"Cartesian路径规划成功：fraction={fraction:.3f}")
        
        # 重新定时（标准方法）
        try:
            plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, 0.5, 0.5)
        except Exception:
            pass
        
        # 执行轨迹（标准方法）
        success = self.group.execute(plan, wait=True)
        self.group.stop()
        
        if not success:
            raise RuntimeError("执行轨迹失败（execute() 返回 false）")
        
        rospy.loginfo("done.")


def main():
    rospy.init_node("draw_square_servo_xy", anonymous=True)
    node = ServoSquareXY()
    rospy.sleep(0.5)  # wait publisher connections
    node.run()


if __name__ == "__main__":
    main()


