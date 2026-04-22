#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_zoh_fb_cmd.py

测试 zoh_fb_cmd.py 的 ZMQ 控制功能
通过 ZMQ PUB 发送控制命令，让底盘按指定轨迹移动
"""

import time
import zmq
import json
import argparse
import math


class ZMQControlSender:
    """ZMQ 控制命令发送器"""
    
    def __init__(self, addr="tcp://10.8.0.90:4399", topic="control"):
        """
        :param addr: ZMQ 绑定地址 (注意: PUB 端需要 bind，而不是 connect)
        :param topic: ZMQ topic
        """
        self.addr = addr
        self.topic = topic
        
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.PUB)
        
        # PUB 端需要 bind，让 SUB 端来 connect
        # 如果 zoh_fb_cmd.py 是 connect 到这个地址，那这里需要 bind
        # 但原代码是 connect，所以测试端需要用不同方式
        # 这里我们也用 connect 到同一个地址，假设有个中间 broker
        # 或者直接在本机测试时，改用 bind
        
        bind_addr = addr.replace("10.8.0.90", "*")  # bind 到所有接口
        self.sock.bind(bind_addr)
        
        print(f"[ZMQ PUB] bind to {bind_addr}, topic='{topic}'")
        
        # 等待连接建立
        time.sleep(0.5)
    
    def send(self, command: str, base_x: float = 0.0, base_y: float = 0.0, pitch: float = 0.0):
        """
        发送控制命令
        
        :param command: 命令类型 (start/stop/reset/base)
        :param base_x: X 坐标 (米)
        :param base_y: Y 坐标 (米)
        :param pitch: 偏航角 (弧度)
        """
        msg = {
            "command": command,
            "base_X": base_x,
            "base_Y": base_y,
            "pitch": pitch
        }
        
        payload = json.dumps(msg).encode("utf-8")
        self.sock.send_multipart([self.topic.encode("utf-8"), payload])
        
        print(f"[TX] cmd={command}, x={base_x:+.4f}, y={base_y:+.4f}, yaw={pitch:+.4f}")
    
    def close(self):
        self.sock.close()


def test_start_stop():
    """测试启动和停止"""
    sender = ZMQControlSender()
    
    try:
        print("\n===== Test: Start/Stop =====")
        
        # 发送 start 命令
        sender.send("start")
        time.sleep(0.5)
        
        # 发送一些位置
        sender.send("base", base_x=0.1, base_y=0.0, pitch=0.0)
        time.sleep(0.2)
        
        # 发送 stop 命令
        sender.send("stop")
        time.sleep(0.5)
        
        print("Test start/stop completed")
        
    finally:
        sender.close()


def test_linear_motion():
    """测试直线运动"""
    sender = ZMQControlSender()
    
    try:
        print("\n===== Test: Linear Motion (X direction) =====")
        
        # 启动
        sender.send("start")
        time.sleep(0.3)
        
        # 沿 X 轴直线运动
        for i in range(20):
            x = i * 0.05  # 每次移动 5cm
            sender.send("base", base_x=x, base_y=0.0, pitch=0.0)
            time.sleep(0.1)
        
        # 停止
        sender.send("stop")
        print("Linear motion test completed")
        
    finally:
        sender.close()


def test_circular_motion():
    """测试圆周运动"""
    sender = ZMQControlSender()
    
    try:
        print("\n===== Test: Circular Motion =====")
        
        # 启动
        sender.send("start")
        time.sleep(0.3)
        
        # 圆周运动参数
        radius = 0.3  # 半径 30cm
        center_x, center_y = 0.0, 0.0
        steps = 36  # 36 步完成一圈
        
        for i in range(steps + 1):
            angle = 2 * math.pi * i / steps
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            yaw = angle + math.pi / 2  # 切线方向
            
            sender.send("base", base_x=x, base_y=y, pitch=yaw)
            time.sleep(0.1)
        
        # 停止
        sender.send("stop")
        print("Circular motion test completed")
        
    finally:
        sender.close()


def test_square_motion():
    """测试正方形轨迹"""
    sender = ZMQControlSender()
    
    try:
        print("\n===== Test: Square Motion =====")
        
        # 启动
        sender.send("start")
        time.sleep(0.3)
        
        # 正方形顶点 (边长 0.5m)
        waypoints = [
            (0.0, 0.0, 0.0),           # 起点
            (0.5, 0.0, 0.0),           # 右
            (0.5, 0.5, math.pi/2),     # 右上
            (0.0, 0.5, math.pi),       # 左上
            (0.0, 0.0, -math.pi/2),    # 回到起点
        ]
        
        for x, y, yaw in waypoints:
            # 每个点发送多次，让底盘有时间到达
            for _ in range(10):
                sender.send("base", base_x=x, base_y=y, pitch=yaw)
                time.sleep(0.1)
        
        # 停止
        sender.send("stop")
        print("Square motion test completed")
        
    finally:
        sender.close()


def test_reset():
    """测试重置里程计"""
    sender = ZMQControlSender()
    
    try:
        print("\n===== Test: Reset Odometry =====")
        
        sender.send("reset")
        time.sleep(1.0)
        
        print("Reset test completed")
        
    finally:
        sender.close()


def interactive_mode():
    """交互模式 - 手动输入坐标"""
    sender = ZMQControlSender()
    
    try:
        print("\n===== Interactive Mode =====")
        print("Commands:")
        print("  start          - 启动控制")
        print("  stop           - 停止控制")
        print("  reset          - 重置里程计")
        print("  x y yaw        - 发送位置 (例如: 0.5 0.2 0.1)")
        print("  q              - 退出")
        print()
        
        active = False
        
        while True:
            try:
                cmd = input("> ").strip().lower()
                
                if cmd == "q":
                    break
                elif cmd == "start":
                    sender.send("start")
                    active = True
                elif cmd == "stop":
                    sender.send("stop")
                    active = False
                elif cmd == "reset":
                    sender.send("reset")
                    active = False
                else:
                    # 尝试解析为坐标
                    parts = cmd.split()
                    if len(parts) >= 2:
                        x = float(parts[0])
                        y = float(parts[1])
                        yaw = float(parts[2]) if len(parts) >= 3 else 0.0
                        
                        if not active:
                            print("Warning: not active, sending 'start' first")
                            sender.send("start")
                            active = True
                            time.sleep(0.2)
                        
                        sender.send("base", base_x=x, base_y=y, pitch=yaw)
                    else:
                        print("Unknown command. Use 'x y [yaw]' format for positions.")
                        
            except ValueError as e:
                print(f"Invalid input: {e}")
            except KeyboardInterrupt:
                break
                
    finally:
        sender.send("stop")
        sender.close()
        print("\nExited.")


def main():
    parser = argparse.ArgumentParser(description="Test zoh_fb_cmd.py via ZMQ")
    parser.add_argument("--test", "-t", choices=["start_stop", "linear", "circular", "square", "reset", "all"],
                        default="all", help="Test case to run")
    parser.add_argument("--interactive", "-i", action="store_true", help="Interactive mode")
    parser.add_argument("--addr", default="tcp://10.8.0.90:4399", help="ZMQ address")
    
    args = parser.parse_args()
    
    if args.interactive:
        interactive_mode()
        return
    
    if args.test == "all":
        test_start_stop()
        time.sleep(1)
        test_linear_motion()
        time.sleep(1)
        test_circular_motion()
        time.sleep(1)
        test_square_motion()
        time.sleep(1)
        test_reset()
    elif args.test == "start_stop":
        test_start_stop()
    elif args.test == "linear":
        test_linear_motion()
    elif args.test == "circular":
        test_circular_motion()
    elif args.test == "square":
        test_square_motion()
    elif args.test == "reset":
        test_reset()


if __name__ == "__main__":
    main()
