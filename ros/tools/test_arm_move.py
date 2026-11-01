#!/usr/bin/env python3
"""
测试机械臂移动到指定位置

支持两种模式：
1. 笛卡尔坐标模式：移动到指定的 (x, y, z) 位置
2. 关节角度模式：移动到指定的关节角度

使用方法：
  # 移动到笛卡尔坐标 (0.2, 0.1, 0.25)
  python3 tools/test_arm_move.py --mode pose --x 0.2 --y 0.1 --z 0.25

  # 移动到关节角度
  python3 tools/test_arm_move.py --mode joint --joint1 0.5 --joint2 0.3 --joint3 0.2

  # 查看当前位姿和关节状态
  python3 tools/test_arm_move.py --mode status
"""

import sys
import argparse
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from copy import deepcopy


class ArmMoveTest:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('test_arm_move', anonymous=True)
        
        # 参数
        self.group_name = rospy.get_param("~group_name", "arm")
        self.eef_link = rospy.get_param("~eef_link", "J5_1")
        self.planning_frame = rospy.get_param("~planning_frame", "base_link")
        
        # 初始化 MoveIt
        rospy.loginfo("正在初始化 MoveIt...")
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # 设置规划参数
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3)
        
        rospy.loginfo("MoveIt 初始化完成")
    
    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        try:
            pose = self.group.get_current_pose(self.eef_link)
            return pose.pose
        except Exception as e:
            rospy.logerr(f"获取当前位姿失败: {e}")
            return None
    
    def get_current_joint_values(self):
        """获取当前关节角度"""
        try:
            return self.group.get_current_joint_values()
        except Exception as e:
            rospy.logerr(f"获取当前关节角度失败: {e}")
            return None
    
    def print_status(self):
        """打印当前状态"""
        print("\n=== 当前机械臂状态 ===")
        
        # 打印关节角度
        joint_values = self.get_current_joint_values()
        if joint_values:
            joint_names = self.group.get_active_joints()
            print("\n关节角度 (rad):")
            for name, value in zip(joint_names, joint_values):
                print(f"  {name}: {value:.4f} ({value * 180 / 3.14159:.2f}°)")
        
        # 打印末端执行器位姿
        pose = self.get_current_pose()
        if pose:
            print("\n末端执行器位姿 (base_link 坐标系):")
            print(f"  位置: x={pose.position.x:.4f}, y={pose.position.y:.4f}, z={pose.position.z:.4f}")
            print(f"  姿态: qx={pose.orientation.x:.4f}, qy={pose.orientation.y:.4f}, "
                  f"qz={pose.orientation.z:.4f}, qw={pose.orientation.w:.4f}")
        
        print("=" * 30 + "\n")
    
    def move_to_pose(self, x, y, z, keep_orientation=True):
        """移动到指定的笛卡尔坐标位置"""
        rospy.loginfo(f"准备移动到位置: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # 获取当前位姿
        current_pose = self.get_current_pose()
        if current_pose is None:
            rospy.logerr("无法获取当前位姿，退出")
            return False
        
        # 构造目标位姿
        target_pose = deepcopy(current_pose)
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        # 如果 keep_orientation=True，保持当前姿态
        
        rospy.loginfo(f"目标位姿: 位置=({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
        
        # 设置目标并执行
        try:
            self.group.set_start_state_to_current_state()
            self.group.set_pose_target(target_pose, self.eef_link)
            
            rospy.loginfo("开始规划并执行...")
            success = self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            
            if success:
                rospy.loginfo("✓ 移动成功！")
                # 验证实际到达位置
                final_pose = self.get_current_pose()
                if final_pose:
                    dist = ((x - final_pose.position.x)**2 + 
                           (y - final_pose.position.y)**2 + 
                           (z - final_pose.position.z)**2)**0.5
                    rospy.loginfo(f"实际位置误差: {dist:.4f} m")
                return True
            else:
                rospy.logwarn("✗ 移动失败：规划或执行失败")
                return False
                
        except Exception as e:
            rospy.logerr(f"移动过程中出错: {e}")
            self.group.stop()
            self.group.clear_pose_targets()
            return False
    
    def move_to_joint_positions(self, joint_positions):
        """移动到指定的关节角度"""
        rospy.loginfo(f"准备移动到关节角度: {joint_positions}")
        
        # 获取当前关节值
        current_values = self.get_current_joint_values()
        if current_values is None:
            rospy.logerr("无法获取当前关节角度，退出")
            return False
        
        joint_names = self.group.get_active_joints()
        
        # 构建目标关节值（只更新指定的关节，其他保持当前值）
        target_values = list(current_values)
        for i, name in enumerate(joint_names):
            if name in joint_positions:
                target_values[i] = float(joint_positions[name])
                rospy.loginfo(f"  {name}: {current_values[i]:.4f} -> {target_values[i]:.4f}")
        
        # 设置目标并执行
        try:
            self.group.set_start_state_to_current_state()
            self.group.set_joint_value_target(target_values)
            
            rospy.loginfo("开始规划并执行...")
            success = self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            
            if success:
                rospy.loginfo("✓ 移动成功！")
                return True
            else:
                rospy.logwarn("✗ 移动失败：规划或执行失败")
                return False
                
        except Exception as e:
            rospy.logerr(f"移动过程中出错: {e}")
            self.group.stop()
            self.group.clear_pose_targets()
            return False


def main():
    parser = argparse.ArgumentParser(description='测试机械臂移动到指定位置')
    parser.add_argument('--mode', choices=['pose', 'joint', 'status'], default='status',
                       help='模式: pose=笛卡尔坐标, joint=关节角度, status=查看状态')
    
    # 笛卡尔坐标参数
    parser.add_argument('--x', type=float, help='目标 X 坐标 (m)')
    parser.add_argument('--y', type=float, help='目标 Y 坐标 (m)')
    parser.add_argument('--z', type=float, help='目标 Z 坐标 (m)')
    
    # 关节角度参数
    parser.add_argument('--joint1', type=float, help='joint1 目标角度 (rad)')
    parser.add_argument('--joint2', type=float, help='joint2 目标角度 (rad)')
    parser.add_argument('--joint3', type=float, help='joint3 目标角度 (rad)')
    parser.add_argument('--joint4', type=float, help='joint4 目标角度 (rad)')
    
    args = parser.parse_args()
    
    try:
        test = ArmMoveTest()
        
        if args.mode == 'status':
            # 只查看状态
            test.print_status()
            return 0
        
        elif args.mode == 'pose':
            # 笛卡尔坐标模式
            if args.x is None or args.y is None or args.z is None:
                rospy.logerr("笛卡尔坐标模式需要指定 --x, --y, --z 参数")
                return 1
            
            # 先打印当前状态
            test.print_status()
            
            # 执行移动
            success = test.move_to_pose(args.x, args.y, args.z)
            
            # 移动后再次打印状态
            if success:
                rospy.sleep(0.5)  # 等待一下让状态更新
                test.print_status()
            
            return 0 if success else 1
        
        elif args.mode == 'joint':
            # 关节角度模式
            joint_positions = {}
            if args.joint1 is not None:
                joint_positions['joint1'] = args.joint1
            if args.joint2 is not None:
                joint_positions['joint2'] = args.joint2
            if args.joint3 is not None:
                joint_positions['joint3'] = args.joint3
            if args.joint4 is not None:
                joint_positions['joint4'] = args.joint4
            
            if not joint_positions:
                rospy.logerr("关节角度模式需要至少指定一个关节角度 (--joint1, --joint2, etc.)")
                return 1
            
            # 先打印当前状态
            test.print_status()
            
            # 执行移动
            success = test.move_to_joint_positions(joint_positions)
            
            # 移动后再次打印状态
            if success:
                rospy.sleep(0.5)  # 等待一下让状态更新
                test.print_status()
            
            return 0 if success else 1
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
        return 130
    except Exception as e:
        rospy.logerr(f"程序出错: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())


