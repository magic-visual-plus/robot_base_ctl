#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist, TwistStamped

# 你现有的底盘控制库（按你项目实际路径修改）
from lekiwi_base import LeKiwiBaseConfig, LeKiwiBaseController


class LeKiwiBaseBridge(Node):
    def __init__(self):
        super().__init__('lekiwi_base_bridge')

        # params
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("vx_max", 0.5)  # m/s or your unit (see note)
        self.declare_parameter("vy_max", 0.5)
        self.declare_parameter("wz_max", 1.5)  # rad/s

        port = self.get_parameter("port").value
        cmd_topic = self.get_parameter("cmd_topic").value

        self.vx_max = float(self.get_parameter("vx_max").value)
        self.vy_max = float(self.get_parameter("vy_max").value)
        self.wz_max = float(self.get_parameter("wz_max").value)

        # connect base
        cfg = LeKiwiBaseConfig(port=port)
        self.base = LeKiwiBaseController(cfg)
        self.base.connect()  # 如果你的库是 open()/start()，改成对应的

        # subscribe cmd_vel
        self.sub = self.create_subscription(Twist, cmd_topic, self.cb_cmd, 10)
        # publish feedback velocity
        self.pub_fb = self.create_publisher(TwistStamped, "/base_vel_fb", 10)
        self.fb_timer = self.create_timer(0.02, self.publish_fb)  # 50Hz

        # safety: stop if cmd_vel timeout
        self.last_cmd_time = self.get_clock().now()
        self.timeout_sec = 0.5
        self.timer = self.create_timer(0.05, self.watchdog)

        self.get_logger().info(f"LeKiwi base bridge started. port={port}, cmd={cmd_topic}")

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def cb_cmd(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        vx = self.clamp(msg.linear.x, -self.vx_max, self.vx_max)
        vy = self.clamp(msg.linear.y, -self.vy_max, self.vy_max)
        wz = self.clamp(msg.angular.z, -self.wz_max, self.wz_max)
        
        print(f"recv cmd ctrl vx: {vx}, vy: {vy}, wz: {wz}")

        # ====== 核心：把 ROS2 的 vx/vy/wz 转给你的底盘 ======
        # 如果你的底盘接口是 set_body_velocity(vx, vy, wz)，就直接用
        # wz should be in deg/s (degrees per second), convert from rad/s
        wz_deg = wz * 180.0 / 3.141592653589793
        self.base.set_body_velocity(vx, vy, wz_deg)

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.timeout_sec:
            # 超时就刹车，防止失控
            self.base.set_body_velocity(0.0, 0.0, 0.0)
    def publish_fb(self):
        if not self.base.is_connected:
            return
        try:
            v = self.base.read_body_velocity()
            vx = float(v.get("x.vel", 0.0))
            vy = float(v.get("y.vel", 0.0))
            wz_deg = float(v.get("theta.vel", 0.0))
            wz = wz_deg * math.pi / 180.0  # deg/s -> rad/s

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            msg.twist.angular.z = wz
            self.pub_fb.publish(msg)

        except Exception as e:
            # 不要刷屏：debug 就行
            self.get_logger().debug(f"publish_fb failed: {e}")

    def destroy_node(self):
        try:
            self.base.set_body_velocity(0.0, 0.0, 0.0)
            self.base.stop()  # 如果你的库是 disconnect()/stop()，改对应的
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LeKiwiBaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
