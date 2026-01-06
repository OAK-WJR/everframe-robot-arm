#!/usr/bin/env python3
"""
发送手动目标位置（非 ROS 模式）。

用法:
  python nt4_goto.py --server 10.1.14.2 --joint J2 --target 0.0
"""
import argparse
import time
from ntcore import NetworkTableInstance, PubSubOptions

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', default='10.1.14.2')
    parser.add_argument('--joint', default='J2')
    parser.add_argument('--target', type=float, default=0.0, help='Target angle in degrees')
    parser.add_argument('--maxvolts', type=float, default=2.0, help='Max voltage')
    args = parser.parse_args()

    inst = NetworkTableInstance.getDefault()
    inst.startClient4('nt4_goto')
    inst.setServer(args.server)

    for _ in range(200):
        if inst.isConnected():
            break
        time.sleep(0.02)
    
    if not inst.isConnected():
        print('Failed to connect')
        return

    print(f'Connected to {args.server}')
    
    arm = inst.getTable('Arm')
    jt = arm.getSubTable(args.joint)
    
    # 设置非 ROS 模式
    jt.getEntry('RosEnable').setBoolean(False)
    jt.getEntry('TestEnable').setBoolean(False)
    jt.getEntry('MaxVolts').setDouble(args.maxvolts)
    jt.getEntry('TargetDeg').setDouble(args.target)
    
    # 设置 ActiveJoint
    arm.getEntry('ActiveJoint').setString(args.joint)
    
    inst.flush()
    
    print(f'Set {args.joint} target to {args.target}°')
    print(f'MaxVolts = {args.maxvolts}')
    print('Make sure robot is ENABLED in DriverStation!')
    print()
    print('Monitoring... (Ctrl+C to stop)')
    
    sub_angle = jt.getDoubleTopic('AngleDeg').subscribe(float('nan'), PubSubOptions(periodic=0.1))
    sub_goal = jt.getDoubleTopic('GoalDeg').subscribe(float('nan'), PubSubOptions(periodic=0.1))
    sub_mode = jt.getStringTopic('ControlMode').subscribe('', PubSubOptions(periodic=0.1))
    sub_inhibit = jt.getStringTopic('InhibitReason').subscribe('', PubSubOptions(periodic=0.1))
    
    try:
        while True:
            time.sleep(0.5)
            angle = sub_angle.get()
            goal = sub_goal.get()
            mode = sub_mode.get()
            inhibit = sub_inhibit.get()
            print(f'Angle={angle:.2f}° Goal={goal:.2f}° Mode={mode} Inhibit={inhibit}')
    except KeyboardInterrupt:
        print('\nDone')

if __name__ == '__main__':
    main()

