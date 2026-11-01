#!/usr/bin/env python3
"""
用 MoveIt Servo 测试 J2 关节，速度 0.3 rad/s
"""

import rospy
from control_msgs.msg import JointJog
from std_msgs.msg import Header
import time

class ServoJ2Test:
    def __init__(self):
        rospy.init_node('servo_j2_test', anonymous=True)
        
        # 参数
        self.vel_rad_s = float(rospy.get_param("~vel_rad_s", 0.3))  # 速度 rad/s
        self.duration_s = float(rospy.get_param("~duration_s", 10.0))  # 持续时间（秒）
        
        # 发布到 Servo
        self.servo_pub = rospy.Publisher('/servo_server/delta_joint_cmds', JointJog, queue_size=1)
        
        # 等待 Servo 启动
        rospy.loginfo("等待 Servo 启动...")
        rospy.sleep(2.0)
        
        rospy.loginfo(f"Servo J2 测试：速度={self.vel_rad_s:.3f} rad/s，持续时间={self.duration_s:.1f}秒")
    
    def run(self):
        rate = rospy.Rate(50)  # 50Hz
        start_time = rospy.Time.now().to_sec()
        
        rospy.loginfo("开始发送 Servo 命令...")
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            elapsed = current_time - start_time
            
            if elapsed >= self.duration_s:
                rospy.loginfo("测试时间到，停止发送命令")
                break
            
            # 创建 JointJog 消息
            msg = JointJog()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"
            
            # 只设置 J2（joint2）
            msg.joint_names = ["joint2"]
            msg.velocities = [self.vel_rad_s]  # 正速度，向上移动
            
            # 发布
            self.servo_pub.publish(msg)
            
            if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
                rospy.loginfo(f"已运行 {elapsed:.1f} 秒...")
            
            rate.sleep()
        
        # 发送停止命令
        rospy.loginfo("发送停止命令...")
        stop_msg = JointJog()
        stop_msg.header = Header()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.header.frame_id = "base_link"
        stop_msg.joint_names = ["joint2"]
        stop_msg.velocities = [0.0]
        
        for _ in range(10):
            self.servo_pub.publish(stop_msg)
            rospy.sleep(0.1)
        
        rospy.loginfo("测试完成")


if __name__ == "__main__":
    try:
        test = ServoJ2Test()
        test.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())

