#!/usr/bin/env python3
"""
发送 sine 轨迹作为 RosPosRad，模拟"连续的 goal"。
使用 DoublePublisher + flush 保证高频发送（而不是被 NT4 合并）。

用法:
  python nt4_sine_ros.py --server 10.1.14.2 --joint J2 --amp 0.3 --period 4.0 --offset 0.5 --dur 10
"""
import argparse
import math
import time
from ntcore import NetworkTableInstance, PubSubOptions

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', default='10.1.14.2')
    parser.add_argument('--joint', default='J2')
    parser.add_argument('--amp', type=float, default=0.3, help='Amplitude in radians')
    parser.add_argument('--period', type=float, default=4.0, help='Period in seconds')
    parser.add_argument('--offset', type=float, default=0.5, help='Offset in radians')
    parser.add_argument('--dur', type=float, default=10.0, help='Duration in seconds')
    parser.add_argument('--hz', type=float, default=50.0, help='Update rate in Hz')
    args = parser.parse_args()

    inst = NetworkTableInstance.getDefault()
    inst.startClient4('nt4_sine_ros')
    inst.setServer(args.server)

    for _ in range(200):
        if inst.isConnected():
            break
        time.sleep(0.02)
    
    if not inst.isConnected():
        print('Failed to connect')
        return

    print(f'Connected to {args.server}')
    
    jt = inst.getTable('Arm').getSubTable(args.joint)
    
    # 使用 DoublePublisher 而不是 Entry，保证高频发送
    period_s = 1.0 / args.hz
    opts = PubSubOptions(periodic=period_s, sendAll=True)
    
    pubPos = jt.getDoubleTopic('RosPosRad').publish(opts)
    pubVel = jt.getDoubleTopic('RosVelRadPerS').publish(opts)
    pubAcc = jt.getDoubleTopic('RosAccRadPerS2').publish(opts)
    pubSeq = jt.getDoubleTopic('RosSeq').publish(opts)
    pubEnable = jt.getBooleanTopic('RosEnable').publish()
    
    # 启用 ROS 模式
    pubEnable.set(True)
    inst.flush()
    time.sleep(0.1)  # 等待 RosEnable 生效
    
    dt = period_s
    seq = 1
    t0 = time.time()
    
    print(f'Sending sine: amp={args.amp} rad, period={args.period}s, offset={args.offset} rad')
    print(f'Duration: {args.dur}s, Rate: {args.hz} Hz (using DoublePublisher + flush)')
    print('Press Ctrl+C to stop')
    
    try:
        while True:
            t = time.time() - t0
            if t > args.dur:
                break
            
            # 计算 sine 轨迹（和 roboRIO 的 SineGoalGenerator 一样）
            omega = 2.0 * math.pi / args.period
            pos = args.offset + args.amp * math.sin(omega * t)
            vel = args.amp * omega * math.cos(omega * t)
            acc = -args.amp * omega * omega * math.sin(omega * t)
            
            # 发送到 NT（使用 Publisher）
            pubPos.set(pos)
            pubVel.set(vel)
            pubAcc.set(acc)
            pubSeq.set(float(seq))
            
            # 强制立即发送（不等待 NT4 合并）
            inst.flush()
            
            seq += 1
            
            # 等待下一个周期
            next_t = t0 + seq * dt
            sleep_time = next_t - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print('\nInterrupted')
    
    finally:
        # 停止
        pubEnable.set(False)
        inst.flush()
        print(f'Sent {seq} frames in {time.time() - t0:.2f}s')

if __name__ == '__main__':
    main()
