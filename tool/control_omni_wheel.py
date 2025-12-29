#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z) from quaternion (x,y,z,w)
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class OmniDeltaServo(Node):
    """
    Input:  Twist on delta_topic (interpreted as delta pose per tick, in base_link frame)
        linear.x = dx [m]
        linear.y = dy [m]
        angular.z = dtheta [rad]

    Feedback: /odom (nav_msgs/Odometry)

    Output: /cmd_vel (geometry_msgs/Twist)
        linear.x = vx [m/s] in base_link
        linear.y = vy [m/s] in base_link
        angular.z = wz [rad/s]
    """

    def __init__(self):
        super().__init__("omni_delta_servo")

        # ---- Parameters ----
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("delta_topic", "/delta_cmd")   # your 30Hz incremental input
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("control_rate_hz", 30.0)

        # Feedforward filtering + deadband on DELTA
        self.declare_parameter("deadband_dx", 0.001)          # m
        self.declare_parameter("deadband_dy", 0.001)          # m
        self.declare_parameter("deadband_dtheta", 0.005)      # rad (~0.29 deg)
        self.declare_parameter("ff_lowpass_alpha", 0.8)       # 0..1, higher = smoother

        # Feedback gains (SE(2) error -> velocity)
        self.declare_parameter("kx", 2.5)                     # 1/s
        self.declare_parameter("ky", 2.5)                     # 1/s
        self.declare_parameter("kth", 4.0)                    # 1/s

        # Optional: reduce translation when yaw error is large
        self.declare_parameter("yaw_slowdown", True)

        # Speed limits
        self.declare_parameter("vmax", 0.30)                  # m/s
        self.declare_parameter("wmax", 1.50)                  # rad/s

        # Acceleration limits (slew rate limit)
        self.declare_parameter("amax_v", 1.0)                 # m/s^2
        self.declare_parameter("amax_w", 4.0)                 # rad/s^2

        # ---- Load params ----
        self.odom_topic = self.get_parameter("odom_topic").value
        self.delta_topic = self.get_parameter("delta_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.dt = 1.0 / max(1e-6, self.rate_hz)

        self.db_dx = float(self.get_parameter("deadband_dx").value)
        self.db_dy = float(self.get_parameter("deadband_dy").value)
        self.db_dth = float(self.get_parameter("deadband_dtheta").value)
        self.alpha = float(self.get_parameter("ff_lowpass_alpha").value)

        self.kx = float(self.get_parameter("kx").value)
        self.ky = float(self.get_parameter("ky").value)
        self.kth = float(self.get_parameter("kth").value)

        self.yaw_slowdown = bool(self.get_parameter("yaw_slowdown").value)

        self.vmax = float(self.get_parameter("vmax").value)
        self.wmax = float(self.get_parameter("wmax").value)
        self.amax_v = float(self.get_parameter("amax_v").value)
        self.amax_w = float(self.get_parameter("amax_w").value)

        # ---- State ----
        self.have_odom = False
        self.cur = Pose2D()
        self.ref = Pose2D()
        self.ref_inited = False

        # Buffer for latest delta (per 30Hz tick)
        self.dx_cmd = 0.0
        self.dy_cmd = 0.0
        self.dth_cmd = 0.0
        self.have_delta = False

        # Feedforward filtered velocities
        self.vx_ff_f = 0.0
        self.vy_ff_f = 0.0
        self.wz_ff_f = 0.0

        # Previous output (for accel limiting)
        self.vx_out_prev = 0.0
        self.vy_out_prev = 0.0
        self.wz_out_prev = 0.0

        # ---- ROS I/O ----
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 50)
        self.sub_delta = self.create_subscription(Twist, self.delta_topic, self.cb_delta, 50)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 50)

        self.timer = self.create_timer(self.dt, self.control_step)

        self.get_logger().info(
            f"Started omni_delta_servo | odom: {self.odom_topic} | delta: {self.delta_topic} | cmd: {self.cmd_vel_topic} | {self.rate_hz:.1f}Hz"
        )
        self.get_logger().info(
            "Delta msg format: Twist (linear.x=dx[m], linear.y=dy[m], angular.z=dtheta[rad]) in base_link frame."
        )

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        self.cur = Pose2D(p.x, p.y, yaw)
        self.have_odom = True

        # Initialize ref at first odom
        if not self.ref_inited:
            self.ref = Pose2D(p.x, p.y, yaw)
            self.ref_inited = True

    def cb_delta(self, msg: Twist):
        dx = float(msg.linear.x)
        dy = float(msg.linear.y)
        dth = float(msg.angular.z)

        # deadband on delta input
        if abs(dx) < self.db_dx:
            dx = 0.0
        if abs(dy) < self.db_dy:
            dy = 0.0
        if abs(dth) < self.db_dth:
            dth = 0.0

        self.dx_cmd = dx
        self.dy_cmd = dy
        self.dth_cmd = dth
        self.have_delta = True

    def _clamp(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def _slew(self, v: float, v_prev: float, amax: float) -> float:
        dv = self._clamp(v - v_prev, -amax * self.dt, amax * self.dt)
        return v_prev + dv

    def control_step(self):
        if not self.have_odom or not self.ref_inited:
            return

        # 1) Update moving reference pose using the latest delta (in base frame)
        #    Ref integration in odom frame:
        #    [x_ref, y_ref] += R(theta_ref) * [dx, dy]
        #    theta_ref += dtheta
        dx = self.dx_cmd if self.have_delta else 0.0
        dy = self.dy_cmd if self.have_delta else 0.0
        dth = self.dth_cmd if self.have_delta else 0.0

        c = math.cos(self.ref.yaw)
        s = math.sin(self.ref.yaw)
        self.ref.x += c * dx - s * dy
        self.ref.y += s * dx + c * dy
        self.ref.yaw = wrap_to_pi(self.ref.yaw + dth)

        # 2) Feedforward velocity from delta/dt, then low-pass
        vx_ff = dx / self.dt
        vy_ff = dy / self.dt
        wz_ff = dth / self.dt

        self.vx_ff_f = self.alpha * self.vx_ff_f + (1.0 - self.alpha) * vx_ff
        self.vy_ff_f = self.alpha * self.vy_ff_f + (1.0 - self.alpha) * vy_ff
        self.wz_ff_f = self.alpha * self.wz_ff_f + (1.0 - self.alpha) * wz_ff

        # 3) SE(2) error: e = T^{-1} T_ref (error expressed in base frame)
        dxw = self.ref.x - self.cur.x
        dyw = self.ref.y - self.cur.y
        c0 = math.cos(self.cur.yaw)
        s0 = math.sin(self.cur.yaw)

        ex =  c0 * dxw + s0 * dyw
        ey = -s0 * dxw + c0 * dyw
        eth = wrap_to_pi(self.ref.yaw - self.cur.yaw)

        # 4) Feedback (position servo)
        vx_fb = self.kx * ex
        vy_fb = self.ky * ey
        wz_fb = self.kth * eth

        # Optional: slow down translation when yaw error is large (prevents "sideways fight")
        if self.yaw_slowdown:
            lam = max(0.0, math.cos(eth))
            vx_fb *= lam
            vy_fb *= lam

        # 5) Combine
        vx = self.vx_ff_f + vx_fb
        vy = self.vy_ff_f + vy_fb
        wz = self.wz_ff_f + wz_fb

        # 6) Speed clamp
        vx = self._clamp(vx, -self.vmax, self.vmax)
        vy = self._clamp(vy, -self.vmax, self.vmax)
        wz = self._clamp(wz, -self.wmax, self.wmax)

        # 7) Acceleration (slew-rate) limiting
        vx = self._slew(vx, self.vx_out_prev, self.amax_v)
        vy = self._slew(vy, self.vy_out_prev, self.amax_v)
        wz = self._slew(wz, self.wz_out_prev, self.amax_w)

        self.vx_out_prev, self.vy_out_prev, self.wz_out_prev = vx, vy, wz

        # 8) Publish cmd_vel
        out = Twist()
        out.linear.x = float(vx)
        out.linear.y = float(vy)
        out.angular.z = float(wz)
        self.pub_cmd.publish(out)

        # (optional debug at low rate)
        # self.get_logger().info(f"ex={ex:+.3f} ey={ey:+.3f} eth={eth:+.3f} | vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f}")


def main():
    rclpy.init()
    node = OmniDeltaServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
