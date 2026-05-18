#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 bridge: /cmd_vel -> LeKiwiBaseController (CANopen DS402 omni base)

实际控制逻辑全部来自 joystick/oni_ctrl.py（通过 moons/oni_ctrl.py 代理导入）。
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

from _paths import EDS_DUAL_AXES
from oni_ctrl import LeKiwiBaseConfig, LeKiwiBaseController


class LeKiwiBaseBridge(Node):
    def __init__(self):
        super().__init__("lekiwi_base_bridge")

        # ---- ROS params ----
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("vx_max", 0.2)
        self.declare_parameter("vy_max", 0.2)
        self.declare_parameter("wz_max", 1.0)

        cmd_topic = self.get_parameter("cmd_topic").value
        self.vx_max = float(self.get_parameter("vx_max").value)
        self.vy_max = float(self.get_parameter("vy_max").value)
        self.wz_max = float(self.get_parameter("wz_max").value)

        # ---- CANopen params ----
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("bitrate", 1_000_000)
        self.declare_parameter("eds_path", EDS_DUAL_AXES)
        self.declare_parameter("encoder_cpr", 2**16)
        self.declare_parameter("gear_ratio", 10.0)
        self.declare_parameter("wheel_radius", 0.1015)
        self.declare_parameter("base_radius", 0.203)
        self.declare_parameter("max_wheel_rad_s", 4.0 * math.pi)
        self.declare_parameter("dir_left", 1)
        self.declare_parameter("dir_back", 1)
        self.declare_parameter("dir_right", 1)
        self.declare_parameter("base_velocity_mode", "PV")
        self.declare_parameter("apply_uniform_profile_dynamics", True)
        self.declare_parameter("base_profile_accel_pulses_s2", 600_000)
        self.declare_parameter("base_profile_decel_pulses_s2", 600_000)
        self.declare_parameter("motor_diagnostics_profile", "tongchuan_mdx")

        cfg = LeKiwiBaseConfig(
            can_channel=str(self.get_parameter("can_channel").value),
            bitrate=int(self.get_parameter("bitrate").value),
            eds_path=str(self.get_parameter("eds_path").value),
            encoder_cpr=int(self.get_parameter("encoder_cpr").value),
            gear_ratio=float(self.get_parameter("gear_ratio").value),
            wheel_radius=float(self.get_parameter("wheel_radius").value),
            base_radius=float(self.get_parameter("base_radius").value),
            max_wheel_rad_s=float(self.get_parameter("max_wheel_rad_s").value),
            dir_left=int(self.get_parameter("dir_left").value),
            dir_back=int(self.get_parameter("dir_back").value),
            dir_right=int(self.get_parameter("dir_right").value),
            base_velocity_mode=str(self.get_parameter("base_velocity_mode").value),
            apply_uniform_profile_dynamics=bool(self.get_parameter("apply_uniform_profile_dynamics").value),
            base_profile_accel_pulses_s2=int(self.get_parameter("base_profile_accel_pulses_s2").value),
            base_profile_decel_pulses_s2=int(self.get_parameter("base_profile_decel_pulses_s2").value),
            motor_diagnostics_profile=str(self.get_parameter("motor_diagnostics_profile").value),
        )
        self.base = LeKiwiBaseController(cfg)
        self.base.connect()

        self.sub = self.create_subscription(Twist, cmd_topic, self.cb_cmd, 10)
        self.pub_fb = self.create_publisher(TwistStamped, "/base_vel_fb", 10)
        # 与 zoh_rev / TPDO 对齐：20 Hz；无新 TPDO 时在 publish_fb 内跳过重复发布
        self.fb_timer = self.create_timer(0.05, self.publish_fb)

        self.last_cmd_time = self.get_clock().now()
        self.timeout_sec = 0.5
        self._watchdog_zero_sent = False
        self._last_fb_cache_ts = 0.0
        self.timer = self.create_timer(0.05, self.watchdog)

        self.get_logger().info(
            f"LeKiwi base bridge started. can={cfg.can_channel}@{cfg.bitrate}, "
            f"mode={cfg.base_velocity_mode}, cmd={cmd_topic}"
        )

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def cb_cmd(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self._watchdog_zero_sent = False
        vx = self._clamp(msg.linear.x, -self.vx_max, self.vx_max)
        vy = self._clamp(msg.linear.y, -self.vy_max, self.vy_max)
        wz = self._clamp(msg.angular.z, -self.wz_max, self.wz_max)
        wz_deg = math.degrees(wz)
        self.base.set_body_velocity(vx, vy, wz_deg)

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.timeout_sec and not self._watchdog_zero_sent:
            self.base.set_body_velocity(0.0, 0.0, 0.0)
            self._watchdog_zero_sent = True

    def publish_fb(self):
        if not self.base.is_connected:
            return
        try:
            newest_ts = self.base.newest_wheel_feedback_cache_ts()
            if newest_ts > 0.0 and newest_ts <= self._last_fb_cache_ts:
                return
            if newest_ts > 0.0:
                self._last_fb_cache_ts = newest_ts

            v = self.base.read_body_velocity()
            vx = float(v.get("x.vel", 0.0))
            vy = float(v.get("y.vel", 0.0))
            wz_deg = float(v.get("theta.vel", 0.0))
            wz = math.radians(wz_deg)

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            msg.twist.angular.z = wz
            self.pub_fb.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"publish_fb failed: {e}")

    def destroy_node(self):
        try:
            self.base.set_body_velocity(0.0, 0.0, 0.0)
            self.base.stop()
            self.base.disconnect()
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
