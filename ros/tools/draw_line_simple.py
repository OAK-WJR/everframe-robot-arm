#!/usr/bin/env python3
"""
用 MoveIt compute_cartesian_path() 进行笛卡尔空间规划，画一条直线。

特点：
- 获取当前位姿作为起点
- 在指定轴（x/y/z）移动指定长度（默认 5cm）
- 使用 compute_cartesian_path() 确保末端执行器沿直线移动（笛卡尔空间规划）
"""

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
from copy import deepcopy


class MoveItLineDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化节点
        rospy.init_node('draw_line_simple', anonymous=True)
        
        # 参数
        self.length_m = float(rospy.get_param("~length_m", 0.05))  # 直线长度（米）
        self.axis = str(rospy.get_param("~axis", "x")).lower()  # x、y 或 z
        self.eef_step = float(rospy.get_param("~eef_step", 0.005))  # 笛卡尔路径步长（米，默认5mm）
        self.vel_scale = float(rospy.get_param("~velocity_scaling", 0.3))  # 速度缩放
        self.acc_scale = float(rospy.get_param("~acceleration_scaling", 0.3))  # 加速度缩放
        self.min_fraction = float(rospy.get_param("~min_fraction", 0.8))  # 最小路径覆盖率（降低要求，4-DOF可能无法完全覆盖）
        
        # 初始化需要使用move group控制的机械臂中的arm group
        # MoveGroupCommander 内部会等待 action server
        rospy.loginfo("初始化 MoveGroupCommander...")
        self.arm = moveit_commander.MoveGroupCommander('arm')
        rospy.loginfo("MoveGroupCommander 初始化完成")
        
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        self.arm.set_pose_reference_frame('base_link')
        
        # 设置速度/加速度缩放
        self.arm.set_max_velocity_scaling_factor(self.vel_scale)
        self.arm.set_max_acceleration_scaling_factor(self.acc_scale)
        
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        rospy.loginfo(f"end_effector_link: {self.end_effector_link}")
        
    def draw_line(self):
        """使用笛卡尔空间规划画一条直线（参考 draw_square_xy.py 和 draw_line_xy.py）"""
        # 先移动到一个更灵活的位置（避免在边界或奇异点）
        rospy.loginfo("先移动到灵活位置（水平姿态）...")
        try:
            current_joints = self.arm.get_current_joint_values()
            rospy.loginfo(f"当前关节角度: {[f'{x:.3f}' for x in current_joints]}")
            
            # 设置一个水平姿态（参考之前成功的配置）
            # joint2=0.5 rad 可以让机械臂处于水平位置，更容易进行XY平面移动
            if len(current_joints) >= 2:
                target_joints = list(current_joints)
                # 设置joint2到0.5 rad（水平位置）
                target_joints[1] = 0.5
                # 设置joint3到合适角度（比如-0.3 rad）
                if len(current_joints) >= 3:
                    target_joints[2] = -0.3
                
                rospy.loginfo(f"目标关节角度: {[f'{x:.3f}' for x in target_joints]}")
                
                # 使用 go() 方法（更可靠，参考 draw_square_xy.py）
                self.arm.set_start_state_to_current_state()
                self.arm.set_joint_value_target(target_joints)
                
                # 尝试多次规划
                plan_obj = None
                for attempt in range(5):
                    plan_obj = self.arm.plan()
                    if isinstance(plan_obj, tuple) and len(plan_obj) > 0:
                        if plan_obj[0]:  # success=True
                            break
                    elif hasattr(plan_obj, "joint_trajectory") and plan_obj.joint_trajectory.points:
                        break
                    if attempt < 4:
                        rospy.logwarn(f"移动到灵活位置规划失败，尝试 {attempt + 1}/5...")
                        rospy.sleep(0.1)
                
                # 提取plan
                plan = None
                if hasattr(plan_obj, "joint_trajectory"):
                    plan = plan_obj
                elif isinstance(plan_obj, tuple) and len(plan_obj) >= 2:
                    plan = plan_obj[1] if hasattr(plan_obj[1], "joint_trajectory") else None
                
                if plan and plan.joint_trajectory.points:
                    # 重定时
                    try:
                        plan = self.arm.retime_trajectory(self.arm.get_current_state(), plan, self.vel_scale, self.acc_scale)
                    except:
                        pass
                    # 执行
                    ok = self.arm.execute(plan, wait=True)
                    self.arm.stop()
                    if ok:
                        rospy.loginfo("已移动到灵活位置")
                        rospy.sleep(1.5)  # 等待稳定
                    else:
                        rospy.logwarn("移动到灵活位置执行失败，继续尝试笛卡尔规划...")
                else:
                    rospy.logwarn("移动到灵活位置的规划失败，继续尝试笛卡尔规划...")
        except Exception as e:
            rospy.logwarn(f"移动到灵活位置时出错: {e}，继续尝试笛卡尔规划...")
            import traceback
            rospy.logwarn(traceback.format_exc())
        
        rospy.loginfo("开始获取当前位姿...")
        # 获取当前位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        rospy.loginfo(f"起点: x={start_pose.position.x:.3f}, y={start_pose.position.y:.3f}, z={start_pose.position.z:.3f}")
        
        # 设置终点位姿（在指定轴上移动指定长度）
        end_pose = deepcopy(start_pose)
        if self.axis == 'x':
            end_pose.position.x += self.length_m
            rospy.loginfo(f"终点: x={end_pose.position.x:.3f} (X方向移动 {self.length_m:.3f}m)")
        elif self.axis == 'y':
            end_pose.position.y += self.length_m
            rospy.loginfo(f"终点: y={end_pose.position.y:.3f} (Y方向移动 {self.length_m:.3f}m)")
        elif self.axis == 'z':
            end_pose.position.z += self.length_m
            rospy.loginfo(f"终点: z={end_pose.position.z:.3f} (Z方向移动 {self.length_m:.3f}m)")
        else:
            raise ValueError(f"axis 必须是 'x'、'y' 或 'z'，当前是 '{self.axis}'")
        
        # 使用 compute_cartesian_path() 进行笛卡尔空间规划（确保末端执行器沿直线移动）
        rospy.loginfo(f"使用 compute_cartesian_path() 进行笛卡尔空间规划（步长={self.eef_step:.3f}m）...")
        self.arm.set_start_state_to_current_state()
        
        # 构造waypoints（起点和终点）
        waypoints = [start_pose, end_pose]
        
        # 调用 compute_cartesian_path（ROS Noetic 签名）
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,
            self.eef_step,
            avoid_collisions=False
        )
        
        rospy.loginfo(f"笛卡尔路径规划完成：fraction={fraction:.3f}")
        
        if fraction < self.min_fraction:
            rospy.logwarn(f"笛卡尔路径覆盖率不足: fraction={fraction:.3f}（最小要求 {self.min_fraction:.3f}）")
            raise RuntimeError(f"笛卡尔路径规划失败：fraction={fraction:.3f}（可能目标位置不可达，或步长太大）")
        
        # 重新定时（必须，否则轨迹时间戳可能为0，JTC会CONTROL_FAILED）
        try:
            plan = self.arm.retime_trajectory(
                self.arm.get_current_state(), 
                plan, 
                self.vel_scale, 
                self.acc_scale
            )
            rospy.loginfo("轨迹重定时完成")
        except Exception as e:
            rospy.logwarn(f"轨迹重定时失败: {e}，继续执行...")
        
        # 执行轨迹
        rospy.loginfo("开始执行笛卡尔路径...")
        ok = self.arm.execute(plan, wait=True)
        self.arm.stop()
        
        if not ok:
            raise RuntimeError("执行笛卡尔路径失败（execute() 返回 false）")
        
        rospy.loginfo("笛卡尔路径执行成功！")
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        rospy.loginfo("=== 开始运行 draw_line_simple ===")
        demo = MoveItLineDemo()
        rospy.loginfo("=== MoveItLineDemo 初始化完成，开始画直线 ===")
        demo.draw_line()
        rospy.loginfo("=== 画直线完成 ===")
    except Exception as e:
        rospy.logerr(f"=== 错误: {e} ===")
        import traceback
        rospy.logerr(traceback.format_exc())
        raise
    except rospy.ROSInterruptException:
        rospy.loginfo("=== 被中断 ===")
        pass
