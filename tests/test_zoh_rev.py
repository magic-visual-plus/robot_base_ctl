#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_zoh_rev.py

测试 zoh_rev.py 的 ROS2 ref_pose 控制功能
通过 ROS2 发布 ref_pose 和 ref_twist，验证控制器响应
"""

import time
import math
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry


REF_POSE_TOPIC = "/ref_pose"
REF_TWIST_TOPIC = "/ref_twist"
CMD_VEL_TOPIC = "/cmd_vel"


def yaw_to_quat(yaw: float):
    """将 yaw 角转换为四元数 (仅 z 轴旋转)"""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class RefPoseSender(Node):
    """ROS2 ref_pose 发布器"""

    def __init__(self, rate_hz: float = 50.0):
        super().__init__("test_ref_pose_sender")

        self.pub_pose = self.create_publisher(PoseStamped, REF_POSE_TOPIC, 10)
        self.pub_twist = self.create_publisher(TwistStamped, REF_TWIST_TOPIC, 10)

        # 订阅 cmd_vel 用于验证
        self.last_cmd = None
        self.cmd_count = 0
        self.create_subscription(Twist, CMD_VEL_TOPIC, self.cb_cmd_vel, 10)

        self.rate_hz = rate_hz
        self.get_logger().info(f"[TEST] RefPoseSender started, rate={rate_hz}Hz")

    def cb_cmd_vel(self, msg: Twist):
        """接收 cmd_vel 用于验证"""
        self.last_cmd = msg
        self.cmd_count += 1

    def send_pose(self, x: float, y: float, yaw: float):
        """发送参考位姿"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_to_quat(yaw)

        self.pub_pose.publish(msg)
        self.get_logger().debug(f"[TX pose] x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

    def send_twist(self, vx: float, vy: float, wz: float):
        """发送参考速度 (前馈)"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz

        self.pub_twist.publish(msg)
        self.get_logger().debug(f"[TX twist] vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}")

    def send_pose_and_twist(self, x: float, y: float, yaw: float,
                            vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
        """同时发送位姿和速度"""
        self.send_pose(x, y, yaw)
        self.send_twist(vx, vy, wz)

    def print_cmd_status(self):
        """打印 cmd_vel 状态"""
        if self.last_cmd:
            self.get_logger().info(
                f"[RX cmd_vel] count={self.cmd_count}, "
                f"vx={self.last_cmd.linear.x:+.3f}, "
                f"vy={self.last_cmd.linear.y:+.3f}, "
                f"wz={self.last_cmd.angular.z:+.3f}"
            )
        else:
            self.get_logger().info(f"[RX cmd_vel] count={self.cmd_count}, no cmd received yet")


def test_static_pose(node: RefPoseSender, duration: float = 3.0):
    """测试静止目标位姿"""
    print("\n===== Test: Static Pose =====")
    print("发送固定位置 (0.2, 0.1, 0.0)，等待控制器响应")

    start_t = time.monotonic()
    while (time.monotonic() - start_t) < duration:
        node.send_pose(0.2, 0.1, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    node.print_cmd_status()
    print("Static pose test completed")


def test_linear_motion_x(node: RefPoseSender, distance: float = 0.5, duration: float = 5.0):
    """测试 X 方向直线运动"""
    print("\n===== Test: Linear Motion (X direction) =====")
    print(f"沿 X 轴移动 {distance}m，持续 {duration}s")

    start_t = time.monotonic()
    while (time.monotonic() - start_t) < duration:
        t = time.monotonic() - start_t
        progress = min(t / duration, 1.0)
        x = distance * progress
        vx = distance / duration  # 恒定速度前馈

        node.send_pose_and_twist(x, 0.0, 0.0, vx, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    node.print_cmd_status()
    print("Linear motion X test completed")


def test_linear_motion_y(node: RefPoseSender, distance: float = 0.3, duration: float = 3.0):
    """测试 Y 方向直线运动 (侧移)"""
    print("\n===== Test: Linear Motion (Y direction - lateral) =====")
    print(f"沿 Y 轴移动 {distance}m，持续 {duration}s")

    start_t = time.monotonic()
    while (time.monotonic() - start_t) < duration:
        t = time.monotonic() - start_t
        progress = min(t / duration, 1.0)
        y = distance * progress
        vy = distance / duration

        node.send_pose_and_twist(0.0, y, 0.0, 0.0, vy, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    node.print_cmd_status()
    print("Linear motion Y test completed")


def test_rotation(node: RefPoseSender, angle: float = math.pi / 2, duration: float = 3.0):
    """测试原地旋转"""
    print("\n===== Test: Rotation =====")
    print(f"原地旋转 {math.degrees(angle):.1f} 度，持续 {duration}s")

    start_t = time.monotonic()
    while (time.monotonic() - start_t) < duration:
        t = time.monotonic() - start_t
        progress = min(t / duration, 1.0)
        yaw = angle * progress
        wz = angle / duration

        node.send_pose_and_twist(0.0, 0.0, yaw, 0.0, 0.0, wz)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    node.print_cmd_status()
    print("Rotation test completed")


def test_circular_motion(node: RefPoseSender, radius: float = 0.3, duration: float = 10.0):
    """测试圆周运动"""
    print("\n===== Test: Circular Motion =====")
    print(f"圆周运动，半径 {radius}m，持续 {duration}s")

    omega = 2 * math.pi / duration  # 角速度 (一圈/duration秒)
    linear_v = radius * omega  # 线速度

    start_t = time.monotonic()
    while (time.monotonic() - start_t) < duration:
        t = time.monotonic() - start_t
        angle = omega * t

        x = radius * math.cos(angle) - radius  # 起点在 (-radius, 0)
        y = radius * math.sin(angle)
        yaw = angle + math.pi / 2  # 切线方向

        # 前馈速度 (世界坐标系)
        vx = -linear_v * math.sin(angle)
        vy = linear_v * math.cos(angle)
        wz = omega

        node.send_pose_and_twist(x, y, yaw, vx, vy, wz)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    node.print_cmd_status()
    print("Circular motion test completed")


def test_square_trajectory(node: RefPoseSender, side: float = 0.4, duration_per_side: float = 3.0):
    """测试正方形轨迹"""
    print("\n===== Test: Square Trajectory =====")
    print(f"正方形轨迹，边长 {side}m，每边 {duration_per_side}s")

    waypoints = [
        (side, 0.0, 0.0),       # 右
        (side, side, 0.0),     # 右上
        (0.0, side, 0.0),      # 左上
        (0.0, 0.0, 0.0),       # 回到起点
    ]

    for i, (x_target, y_target, yaw_target) in enumerate(waypoints):
        print(f"  -> Waypoint {i + 1}: ({x_target:.2f}, {y_target:.2f})")

        start_t = time.monotonic()
        while (time.monotonic() - start_t) < duration_per_side:
            node.send_pose(x_target, y_target, yaw_target)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)

    node.print_cmd_status()
    print("Square trajectory test completed")


def test_pose_only(node: RefPoseSender, duration: float = 5.0):
    """测试仅发送 pose (无 twist 前馈)"""
    print("\n===== Test: Pose Only (no feedforward) =====")
    print("仅发送位姿，不发送速度前馈")

    start_t = time.monotonic()
    while (time.monotonic() - start_t) < duration:
        t = time.monotonic() - start_t
        # 简单的三角形轨迹
        x = 0.2 * math.sin(0.5 * t)
        y = 0.1 * math.sin(1.0 * t)

        node.send_pose(x, y, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    node.print_cmd_status()
    print("Pose only test completed")


def interactive_mode(node: RefPoseSender):
    """交互模式 - 手动输入坐标"""
    print("\n===== Interactive Mode =====")
    print("Commands:")
    print("  p x y yaw      - 发送位姿 (例如: p 0.5 0.2 0.1)")
    print("  t vx vy wz     - 发送速度 (例如: t 0.1 0.0 0.0)")
    print("  pt x y yaw vx vy wz  - 同时发送")
    print("  s              - 显示状态")
    print("  q              - 退出")
    print()

    while True:
        try:
            cmd = input("> ").strip()

            if cmd == "q":
                break
            elif cmd == "s":
                node.print_cmd_status()
            elif cmd.startswith("pt "):
                parts = cmd[3:].split()
                if len(parts) >= 6:
                    x, y, yaw = float(parts[0]), float(parts[1]), float(parts[2])
                    vx, vy, wz = float(parts[3]), float(parts[4]), float(parts[5])
                    node.send_pose_and_twist(x, y, yaw, vx, vy, wz)
                else:
                    print("Usage: pt x y yaw vx vy wz")
            elif cmd.startswith("p "):
                parts = cmd[2:].split()
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    yaw = float(parts[2]) if len(parts) >= 3 else 0.0
                    node.send_pose(x, y, yaw)
                else:
                    print("Usage: p x y [yaw]")
            elif cmd.startswith("t "):
                parts = cmd[2:].split()
                if len(parts) >= 2:
                    vx = float(parts[0])
                    vy = float(parts[1])
                    wz = float(parts[2]) if len(parts) >= 3 else 0.0
                    node.send_twist(vx, vy, wz)
                else:
                    print("Usage: t vx vy [wz]")
            else:
                print("Unknown command")

            # 处理 ROS2 回调
            rclpy.spin_once(node, timeout_sec=0.01)

        except ValueError as e:
            print(f"Invalid input: {e}")
        except KeyboardInterrupt:
            break

    print("\nExited.")


def main():
    parser = argparse.ArgumentParser(description="Test zoh_rev.py via ROS2 ref_pose")
    parser.add_argument(
        "--test", "-t",
        choices=["static", "linear_x", "linear_y", "rotation", "circular", "square", "pose_only", "all"],
        default="all",
        help="Test case to run"
    )
    parser.add_argument("--interactive", "-i", action="store_true", help="Interactive mode")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish rate (Hz)")

    args = parser.parse_args()

    rclpy.init()
    node = RefPoseSender(rate_hz=args.rate)

    try:
        # 等待连接建立
        time.sleep(0.5)

        if args.interactive:
            interactive_mode(node)
        elif args.test == "all":
            test_static_pose(node)
            time.sleep(1)
            test_linear_motion_x(node)
            time.sleep(1)
            test_linear_motion_y(node)
            time.sleep(1)
            test_rotation(node)
            time.sleep(1)
            test_circular_motion(node)
            time.sleep(1)
            test_square_trajectory(node)
            time.sleep(1)
            test_pose_only(node)
        elif args.test == "static":
            test_static_pose(node)
        elif args.test == "linear_x":
            test_linear_motion_x(node)
        elif args.test == "linear_y":
            test_linear_motion_y(node)
        elif args.test == "rotation":
            test_rotation(node)
        elif args.test == "circular":
            test_circular_motion(node)
        elif args.test == "square":
            test_square_trajectory(node)
        elif args.test == "pose_only":
            test_pose_only(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
