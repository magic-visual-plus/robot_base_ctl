#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped

ODOM_TOPIC = "/rko_lio/odometry"
REF_POSE_TOPIC  = "/ref_pose"
REF_TWIST_TOPIC = "/ref_twist"
CMD_TOPIC  = "/cmd_vel"

CONTROL_HZ = 200.0
DT_CTRL = 1.0 / CONTROL_HZ

KP_POS = 0.5
KI_POS = 0.0
I_LIM  = 0.30

KP_YAW = 1.3
W_MAX  = 1.2

V_FRONT_MAX = 0.5
V_LEFT_MAX  = 0.5

A_FRONT_MAX = 1.1
A_LEFT_MAX  = 1.1
A_W_MAX     = 2.5

def clamp(x, lo, hi): return max(lo, min(hi, x))
def wrap_to_pi(a): return (a + math.pi) % (2*math.pi) - math.pi
def quat_to_yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
def world_to_body(yaw, dx_w, dy_w):
    front =  math.cos(yaw)*dx_w + math.sin(yaw)*dy_w
    left  = -math.sin(yaw)*dx_w + math.cos(yaw)*dy_w
    return front, left

class Assist(Node):
    def __init__(self):
        super().__init__("assist_controller_pose_twist")

        self.odom = None
        self.ref_pose = None
        self.ref_twist = None

        self.create_subscription(Odometry, ODOM_TOPIC, self.cb_odom, 50)
        self.create_subscription(PoseStamped,  REF_POSE_TOPIC,  self.cb_pose, 50)
        self.create_subscription(TwistStamped, REF_TWIST_TOPIC, self.cb_tw,   50)

        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        self.intF = 0.0
        self.intL = 0.0
        self.last_vf = self.last_vl = self.last_wz = 0.0

        self.last_t = time.monotonic()
        self.timer = self.create_timer(DT_CTRL, self.on_timer)

        self.get_logger().info("assist 200Hz started")

    def cb_odom(self, m): self.odom = m
    def cb_pose(self, m): self.ref_pose = m
    def cb_tw(self, m): self.ref_twist = m

    def get_pose(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        return float(p.x), float(p.y), float(quat_to_yaw(q))

    def _slew(self, target, last, amax, dt):
        dv = amax * dt
        return clamp(target, last-dv, last+dv)

    def on_timer(self):
        if self.odom is None or self.ref_pose is None or self.ref_twist is None:
            return

        now = time.monotonic()
        dt = clamp(now - self.last_t, 0.5*DT_CTRL, 3.0*DT_CTRL)
        self.last_t = now

        # ref (world)
        x_ref = self.ref_pose.pose.position.x
        y_ref = self.ref_pose.pose.position.y
        yaw_ref = quat_to_yaw(self.ref_pose.pose.orientation)

        vx_w = self.ref_twist.twist.linear.x
        vy_w = self.ref_twist.twist.linear.y
        wz_ff = self.ref_twist.twist.angular.z

        # odom
        x, y, yaw = self.get_pose()

        # FF -> body
        vff_f, vff_l = world_to_body(yaw, vx_w, vy_w)

        # FB error -> body
        err_f, err_l = world_to_body(yaw, x_ref - x, y_ref - y)

        self.intF = clamp(self.intF + err_f*dt, -I_LIM, I_LIM)
        self.intL = clamp(self.intL + err_l*dt, -I_LIM, I_LIM)

        vfb_f = KP_POS*err_f + KI_POS*self.intF
        vfb_l = KP_POS*err_l + KI_POS*self.intL

        yaw_err = wrap_to_pi(yaw_ref - yaw)
        wz_fb = KP_YAW * yaw_err

        vf = vff_f + vfb_f
        vl = vff_l + vfb_l
        wz = wz_ff + wz_fb

        vf = clamp(vf, -V_FRONT_MAX, V_FRONT_MAX)
        vl = clamp(vl, -V_LEFT_MAX,  V_LEFT_MAX)
        wz = clamp(wz, -W_MAX,       W_MAX)

        vf = self._slew(vf, self.last_vf, A_FRONT_MAX, dt)
        vl = self._slew(vl, self.last_vl, A_LEFT_MAX,  dt)
        wz = self._slew(wz, self.last_wz, A_W_MAX,      dt)

        self.last_vf, self.last_vl, self.last_wz = vf, vl, wz

        cmd = Twist()
        cmd.linear.x = float(vf)
        cmd.linear.y = float(vl)
        cmd.angular.z = float(wz)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    n = Assist()
    try:
        rclpy.spin(n)
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
