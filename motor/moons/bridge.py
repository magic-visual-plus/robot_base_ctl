#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist, TwistStamped

# 你现有的底盘控制库（按你项目实际路径修改）
from oni_ctrl import LeKiwiBaseConfig, LeKiwiBaseController


class LeKiwiBaseBridge(Node):
    def __init__(self):
        super().__init__('lekiwi_base_bridge')
        # ---- ROS params ----
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("vx_max", 0.5)  # m/s
        self.declare_parameter("vy_max", 0.5)  # m/s
        self.declare_parameter("wz_max", 1.5)  # rad/s

        cmd_topic = self.get_parameter("cmd_topic").value
        self.vx_max = float(self.get_parameter("vx_max").value)
        self.vy_max = float(self.get_parameter("vy_max").value)
        self.wz_max = float(self.get_parameter("wz_max").value)

        # ---- CANopen params (match oni_ctrl.LeKiwiBaseConfig) ----
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("bitrate", 1000000)
        self.declare_parameter("eds_path", "/home/nvidia/github/robot_base_ctl/motor/moons/CANOPEN-EDS-MBDV-Servo-SingleAxis-V1.1.1.eds")
        self.declare_parameter("encoder_cpr", 2**16)
        self.declare_parameter("gear_ratio", 10.0)
        self.declare_parameter("wheel_radius", 0.1015)
        self.declare_parameter("base_radius", 0.203)
        self.declare_parameter("max_wheel_rad_s", 2.0 * math.pi)
        self.declare_parameter("dir_left", 1)
        self.declare_parameter("dir_back", 1)
        self.declare_parameter("dir_right", 1)

        # read params
        can_channel = self.get_parameter("can_channel").value
        bitrate = int(self.get_parameter("bitrate").value)
        eds_path = self.get_parameter("eds_path").value

        encoder_cpr = int(self.get_parameter("encoder_cpr").value)
        gear_ratio = float(self.get_parameter("gear_ratio").value)
        wheel_radius = float(self.get_parameter("wheel_radius").value)
        base_radius = float(self.get_parameter("base_radius").value)
        max_wheel_rad_s = float(self.get_parameter("max_wheel_rad_s").value)

        dir_left = int(self.get_parameter("dir_left").value)
        dir_back = int(self.get_parameter("dir_back").value)
        dir_right = int(self.get_parameter("dir_right").value)

        # connect base
        cfg = LeKiwiBaseConfig(
            can_channel=can_channel,
            bitrate=bitrate,
            eds_path=eds_path,
            encoder_cpr=encoder_cpr,
            gear_ratio=gear_ratio,
            wheel_radius=wheel_radius,
            base_radius=base_radius,
            max_wheel_rad_s=max_wheel_rad_s,
            dir_left=dir_left,
            dir_back=dir_back,
            dir_right=dir_right,
        )
        self.base = LeKiwiBaseController(cfg)
        self.base.connect()


        # subscribe cmd_vel
        self.sub = self.create_subscription(Twist, cmd_topic, self.cb_cmd, 10)
        # publish feedback velocity
        self.pub_fb = self.create_publisher(TwistStamped, "/base_vel_fb", 10)
        self.fb_timer = self.create_timer(0.02, self.publish_fb)  # 50Hz

        # safety: stop if cmd_vel timeout
        self.last_cmd_time = self.get_clock().now()
        self.timeout_sec = 0.5
        self.timer = self.create_timer(0.05, self.watchdog)

        self.get_logger().info(f"LeKiwi base bridge started. can={can_channel}@{bitrate}, cmd={cmd_topic}")


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
