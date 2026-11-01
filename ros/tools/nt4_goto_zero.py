#!/usr/bin/env python3
"""
直接通过 NT4 发送位置命令，让电机回到 0 位置
不经过 ROS 平滑算法，直接控制
"""
import sys
import time

try:
    from ntcore import NetworkTableInstance
except ImportError:
    print("需要安装 pyntcore: pip3 install pyntcore")
    sys.exit(1)

def main():
    server_ip = sys.argv[1] if len(sys.argv) > 1 else "10.1.14.2"
    joint = sys.argv[2] if len(sys.argv) > 2 else "J2"
    target_pos = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    speed = float(sys.argv[4]) if len(sys.argv) > 4 else 0.3  # rad/s
    
    print(f"NT4 直接回零: {server_ip} {joint} -> {target_pos} rad @ {speed} rad/s")
    
    inst = NetworkTableInstance.getDefault()
    inst.startClient4("nt4_goto_zero")
    inst.setServer(server_ip, 5810)
    
    # 等待连接
    for i in range(50):
        if inst.isConnected():
            break
        time.sleep(0.1)
    
    if not inst.isConnected():
        print("无法连接到 NT4 服务器")
        return 1
    
    print("已连接")
    
    st = inst.getTable(f"Arm/{joint}")
    
    # 订阅当前位置
    angle_sub = st.getDoubleTopic("AngleRad").subscribe(0.0)
    
    # 发布控制命令
    ros_enable = st.getBooleanTopic("RosEnable").publish()
    ros_timeout = st.getDoubleTopic("RosTimeoutS").publish()
    ros_pos = st.getDoubleTopic("RosPosRad").publish()
    ros_vel = st.getDoubleTopic("RosVelRadPerS").publish()
    ros_acc = st.getDoubleTopic("RosAccRadPerS2").publish()
    ros_seq = st.getIntegerTopic("RosSeq").publish()
    
    time.sleep(0.5)  # 等待订阅生效
    
    current_pos = angle_sub.get()
    print(f"当前位置: {current_pos:.4f} rad")
    
    if abs(current_pos - target_pos) < 0.01:
        print("已在目标位置")
        inst.stopClient()
        return 0
    
    # 计算方向和时间
    direction = 1.0 if target_pos > current_pos else -1.0
    distance = abs(target_pos - current_pos)
    duration = distance / speed + 0.5  # 额外 0.5s 缓冲
    
    print(f"移动距离: {distance:.4f} rad, 预计时间: {duration:.2f} s")
    
    # 启用 ROS 控制
    ros_enable.set(True)
    ros_timeout.set(1.0)
    inst.flush()
    
    seq = 0
    rate = 100  # 100 Hz
    dt = 1.0 / rate
    
    pos = current_pos
    start_time = time.time()
    
    # 线性移动到目标
    while True:
        elapsed = time.time() - start_time
        
        # 计算当前目标位置（线性插值）
        progress = min(elapsed * speed / distance, 1.0)
        pos = current_pos + direction * distance * progress
        
        # 计算速度
        if progress < 1.0:
            vel = direction * speed
        else:
            vel = 0.0
            pos = target_pos  # 确保最终位置精确
        
        ros_pos.set(pos)
        ros_vel.set(vel)
        ros_acc.set(0.0)
        ros_seq.set(seq)
        inst.flush()
        
        seq += 1
        
        if progress >= 1.0 and elapsed > duration:
            break
        
        time.sleep(dt)
    
    # 保持在目标位置一段时间
    print(f"到达目标位置: {target_pos:.4f} rad")
    for _ in range(50):
        ros_pos.set(target_pos)
        ros_vel.set(0.0)
        ros_acc.set(0.0)
        ros_seq.set(seq)
        inst.flush()
        seq += 1
        time.sleep(dt)
    
    # 禁用 ROS 控制
    ros_enable.set(False)
    inst.flush()
    
    final_pos = angle_sub.get()
    print(f"最终位置: {final_pos:.4f} rad, 误差: {abs(final_pos - target_pos):.4f} rad")
    
    inst.stopClient()
    return 0

if __name__ == "__main__":
    sys.exit(main())


