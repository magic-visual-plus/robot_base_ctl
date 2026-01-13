#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
waypoint_pose_servo_assist_style.py (ROS2)

A waypoint streaming pose servo using the SAME control logic style as
lekiwi_teleop_assist_2dof_pid:
  - Reference pose ref(t) is generated internally from waypoints
  - Feedforward: reference velocity along the segment (world -> body)
  - Feedback: PID on (ref - odom) error (world -> body)
  - Default yaw policy: HOLD initial heading (wz_cmd = 0) to avoid spin issues

Waypoints (local test frame):
  (0,0) -> (0,1) -> (1,1) -> (0,0)

Topics:
  Sub: odom_topic (nav_msgs/Odometry)
  Pub: cmd_topic  (geometry_msgs/Twist)

Run:
  python3 waypoint_pose_servo_assist_style.py --ros-args \
    -p odom_topic:=/rko_lio/odometry \
    -p cmd_topic:=/cmd_vel \
    -p v_ref:=0.25
"""

import math
import csv
from dataclasses import dataclass
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# ---------------------------
# Math helpers
# ---------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def rot_world_to_body(dx: float, dy: float, yaw: float) -> Tuple[float, float]:
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    ex = cy * dx + sy * dy
    ey = -sy * dx + cy * dy
    return ex, ey

def rot_body_to_world(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    # [vx_w; vy_w] = R(yaw) * [vx_b; vy_b]
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    vx_w = cy * vx - sy * vy
    vy_w = sy * vx + cy * vy
    return vx_w, vy_w


# ---------------------------
# PID
# ---------------------------
@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    i_max: float
    d_lpf_hz: float = 0.0

class PID:
    def __init__(self, g: PIDGains):
        self.g = g
        self.i = 0.0
        self.e_prev: Optional[float] = None
        self.d_filt = 0.0

    def reset(self):
        self.i = 0.0
        self.e_prev = None
        self.d_filt = 0.0

    def step(self, e: float, dt: float, freeze_integrator: bool = False) -> float:
        if dt <= 0.0:
            return 0.0

        if self.e_prev is None:
            de = 0.0
        else:
            de = (e - self.e_prev) / dt
        self.e_prev = e

        # optional D LPF
        if self.g.d_lpf_hz and self.g.d_lpf_hz > 0.0:
            tau = 1.0 / (2.0 * math.pi * self.g.d_lpf_hz)
            alpha = dt / (tau + dt)
            self.d_filt += alpha * (de - self.d_filt)
            d_term = self.d_filt
        else:
            d_term = de

        if (not freeze_integrator) and (self.g.ki != 0.0):
            self.i += e * dt
            self.i = clamp(self.i, -self.g.i_max, self.g.i_max)

        return self.g.kp * e + self.g.ki * self.i + self.g.kd * d_term


# ---------------------------
# Waypoint reference generator
# ---------------------------
Segment = Tuple[float, float, float, float, float, float, float, float]
# (t0,t1, x0,y0, x1,y1, vx_w,vy_w) in LOCAL test world

def build_segments(wpts: List[Tuple[float, float]], v_ref: float, pause: float) -> List[Segment]:
    segs: List[Segment] = []
    t = 0.0
    for i in range(len(wpts) - 1):
        x0, y0 = wpts[i]
        x1, y1 = wpts[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        L = math.hypot(dx, dy)
        if L < 1e-9:
            continue
        T = L / max(1e-6, v_ref)
        vx = dx / T
        vy = dy / T
        segs.append((t, t + T, x0, y0, x1, y1, vx, vy))
        t += T
        if pause > 1e-6:
            segs.append((t, t + pause, x1, y1, x1, y1, 0.0, 0.0))
            t += pause
    return segs

def sample_ref(segs: List[Segment], total_T: float, t: float):
    # returns (x_ref, y_ref, vx_w, vy_w) in LOCAL test world
    if t <= 0.0:
        seg = segs[0]
    elif t >= total_T:
        seg = segs[-1]
    else:
        seg = segs[-1]
        for s in segs:
            if s[0] <= t < s[1]:
                seg = s
                break

    t0, t1, x0, y0, x1, y1, vx_w, vy_w = seg
    if (t1 - t0) < 1e-9:
        a = 1.0
    else:
        a = clamp((t - t0) / (t1 - t0), 0.0, 1.0)

    x_ref = (1 - a) * x0 + a * x1
    y_ref = (1 - a) * y0 + a * y1
    return x_ref, y_ref, vx_w, vy_w


# ---------------------------
# Main node
# ---------------------------
class WaypointPoseServo(Node):
    def __init__(self):
        super().__init__("waypoint_pose_servo_assist_style")

        # topics & rate
        self.declare_parameter("odom_topic", "/rko_lio/odometry")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("control_hz", 60.0)

        # waypoints / reference speed
        self.declare_parameter("v_ref", 0.25)
        self.declare_parameter("corner_pause", 0.0)

        # assist weights (same spirit as assist_k_xy)
        self.declare_parameter("k_xy", 0.35)

        # feedback correction max (avoid overpowering)
        self.declare_parameter("max_correction_v", 0.20)

        # final limits
        self.declare_parameter("vx_max", 0.35)
        self.declare_parameter("vy_max", 0.35)
        self.declare_parameter("wz_max", 1.2)

        # PID gains
        self.declare_parameter("kp_xy", 1.2)
        self.declare_parameter("ki_xy", 0.0)
        self.declare_parameter("kd_xy", 0.20)
        self.declare_parameter("imax_xy", 0.30)
        self.declare_parameter("d_lpf_xy_hz", 10.0)

        # yaw policy
        self.declare_parameter("hold_heading", False)  # default True (NO turning)
        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.15)
        self.declare_parameter("imax_yaw", 0.60)
        self.declare_parameter("d_lpf_yaw_hz", 10.0)

        # set current odom pose as local origin
        self.declare_parameter("set_odom_as_origin", True)

        # logging
        self.declare_parameter("out_csv", "/tmp/waypoint_pose_servo_assist_style.csv")
        self.declare_parameter("log_every_n", 1)

        # ---- read params
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.dt = 1.0 / max(1e-6, self.control_hz)

        self.v_ref = float(self.get_parameter("v_ref").value)
        self.corner_pause = float(self.get_parameter("corner_pause").value)

        self.k_xy = float(self.get_parameter("k_xy").value)
        self.max_corr_v = float(self.get_parameter("max_correction_v").value)

        self.vx_max = float(self.get_parameter("vx_max").value)
        self.vy_max = float(self.get_parameter("vy_max").value)
        self.wz_max = float(self.get_parameter("wz_max").value)

        self.hold_heading = bool(self.get_parameter("hold_heading").value)

        self.set_odom_as_origin = bool(self.get_parameter("set_odom_as_origin").value)

        gxy = PIDGains(
            kp=float(self.get_parameter("kp_xy").value),
            ki=float(self.get_parameter("ki_xy").value),
            kd=float(self.get_parameter("kd_xy").value),
            i_max=float(self.get_parameter("imax_xy").value),
            d_lpf_hz=float(self.get_parameter("d_lpf_xy_hz").value),
        )
        gyaw = PIDGains(
            kp=float(self.get_parameter("kp_yaw").value),
            ki=float(self.get_parameter("ki_yaw").value),
            kd=float(self.get_parameter("kd_yaw").value),
            i_max=float(self.get_parameter("imax_yaw").value),
            d_lpf_hz=float(self.get_parameter("d_lpf_yaw_hz").value),
        )
        self.pid_x = PID(gxy)
        self.pid_y = PID(gxy)
        self.pid_yaw = PID(gyaw)

        # ---- odom state
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0

        # ---- origin (odom -> local test world)
        self.origin_ready = False
        self.ox = self.oy = self.oyaw = 0.0

        # ---- ref state in ODOM world
        self.ref_x: Optional[float] = None
        self.ref_y: Optional[float] = None
        self.ref_yaw: Optional[float] = None
        self.ref_vx_w = 0.0
        self.ref_vy_w = 0.0

        # ---- trajectory segments in LOCAL test world
        self.wpts = [(0.0, 0.0), (1.0, 0.0), (1.0, -1.0), (0.0, 0.0)]
        self.segs = build_segments(self.wpts, self.v_ref, self.corner_pause)
        self.total_T = self.segs[-1][1]  # t_end
        self.t0 = None  # start time after origin captured

        # ---- ROS IO
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 50)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)

        # ---- CSV
        self.out_csv = self.get_parameter("out_csv").value
        self.log_every_n = int(self.get_parameter("log_every_n").value)
        self._tick = 0
        self._csv_file = open(self.out_csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow([
            "t",
            "x","y","yaw",
            "ref_x","ref_y","ref_yaw",
            "vx_ff","vy_ff","wz_ff",
            "ex","ey","eyaw",
            "vx_fb","vy_fb","wz_fb",
            "vx_cmd","vy_cmd","wz_cmd",
        ])

        # ---- timer
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"WaypointPoseServo(assist-style) started. odom={self.odom_topic} -> cmd={self.cmd_topic}\n"
            f"Segments={len(self.segs)} total_T={self.total_T:.2f}s v_ref={self.v_ref} hold_heading={self.hold_heading}"
        )

    def cb_odom(self, msg: Odometry):
        self.have_odom = True
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = yaw_from_quat(q)

        if (not self.origin_ready) and self.set_odom_as_origin:
            self.ox, self.oy, self.oyaw = self.x, self.y, self.yaw
            self.origin_ready = True
            self.t0 = self.get_clock().now()
            self.pid_x.reset(); self.pid_y.reset(); self.pid_yaw.reset()
            self.get_logger().info(
                f"Origin captured: ({self.ox:.3f},{self.oy:.3f},yaw={self.oyaw:.3f}), total_T={self.total_T:.2f}s"
            )

        if (not self.origin_ready) and (not self.set_odom_as_origin):
            self.origin_ready = True
            self.t0 = self.get_clock().now()

    def publish_stop(self):
        self.pub_cmd.publish(Twist())

    def local_to_odom(self, xl: float, yl: float) -> Tuple[float, float]:
        #odom = origin + R(oyaw) * local
        c = math.cos(self.oyaw)
        s = math.sin(self.oyaw)
        xo = self.ox + c * xl - s * yl
        yo = self.oy + s * xl + c * yl
        return xo, yo
    
    def control_loop(self):
        if not self.have_odom or not self.origin_ready or self.t0 is None:
            return

        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # ---- sample LOCAL reference
        x_l, y_l, vx_ref_w_l, vy_ref_w_l = sample_ref(self.segs, self.total_T, t)

        # ---- map ref pose to ODOM world
        ref_x, ref_y = self.local_to_odom(x_l, y_l)

        # ---- yaw reference policy
        if self.hold_heading:
            ref_yaw = self.oyaw  # hold initial heading
            wz_ff = 0.0
        else:
            # follow path direction (optional; can cause spin if yaw sign is wrong)
            if abs(vx_ref_w_l) + abs(vy_ref_w_l) > 1e-6:
                yaw_local = math.atan2(vy_ref_w_l, vx_ref_w_l)
            else:
                yaw_local = 0.0
            ref_yaw = wrap_to_pi(self.oyaw + yaw_local)
            wz_ff = 0.0  # simple: no yaw feedforward

        self.ref_x, self.ref_y, self.ref_yaw = ref_x, ref_y, ref_yaw

        # ---- reference WORLD velocity (odom world)
        # v_ref_w_odom = R(oyaw) * v_ref_w_local
        c0 = math.cos(self.oyaw)
        s0 = math.sin(self.oyaw)
        vx_ref_w = c0 * vx_ref_w_l - s0 * vy_ref_w_l
        vy_ref_w = s0 * vx_ref_w_l + c0 * vy_ref_w_l

        # ---- Feedforward in BODY frame (use CURRENT yaw)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        vx_ff =  cy * vx_ref_w + sy * vy_ref_w
        vy_ff = -sy * vx_ref_w + cy * vy_ref_w

        # ---- Feedback error (WORLD -> BODY)
        dx = ref_x - self.x
        dy = ref_y - self.y
        ex, ey = rot_world_to_body(dx, dy, self.yaw)

        eyaw = wrap_to_pi(ref_yaw - self.yaw)

        # ---- Feedback corrections (assist style)
        vx_fb_raw = self.k_xy * self.pid_x.step(ex, self.dt)
        vy_fb_raw = self.k_xy * self.pid_y.step(ey, self.dt)

        vx_fb = clamp(vx_fb_raw, -self.max_corr_v, self.max_corr_v)
        vy_fb = clamp(vy_fb_raw, -self.max_corr_v, self.max_corr_v)

        if self.hold_heading:
            wz_fb = 0.0
            eyaw = 0.0
        else:
            wz_fb = self.pid_yaw.step(eyaw, self.dt)
            wz_fb = clamp(wz_fb, -0.8, 0.8)

        # ---- Combine
        vx_cmd = vx_ff + vx_fb
        vy_cmd = vy_ff + vy_fb
        wz_cmd = wz_ff + wz_fb

        # ---- Final clamp
        vx_cmd = clamp(vx_cmd, -self.vx_max, self.vx_max)
        vy_cmd = clamp(vy_cmd, -self.vy_max, self.vy_max)
        wz_cmd = clamp(wz_cmd, -self.wz_max, self.wz_max)

        # ---- publish
        tw = Twist()
        tw.linear.x = float(vx_cmd)
        tw.linear.y = float(vy_cmd)
        tw.angular.z = float(wz_cmd)
        self.pub_cmd.publish(tw)

        # ---- log
        self._tick += 1
        if self.log_every_n > 0 and (self._tick % self.log_every_n == 0):
            self._csv.writerow([
                f"{t:.6f}",
                f"{self.x:.6f}", f"{self.y:.6f}", f"{self.yaw:.6f}",
                f"{ref_x:.6f}", f"{ref_y:.6f}", f"{ref_yaw:.6f}",
                f"{vx_ff:.6f}", f"{vy_ff:.6f}", f"{wz_ff:.6f}",
                f"{ex:.6f}", f"{ey:.6f}", f"{eyaw:.6f}",
                f"{vx_fb:.6f}", f"{vy_fb:.6f}", f"{wz_fb:.6f}",
                f"{vx_cmd:.6f}", f"{vy_cmd:.6f}", f"{wz_cmd:.6f}",
            ])
            if self._tick % int(max(1.0, self.control_hz)) == 0:
                self._csv_file.flush()

        # ---- finish
        if t > self.total_T + 0.6:
            self.get_logger().info("Trajectory finished. Stop.")
            self.publish_stop()
            self.timer.cancel()

    def destroy_node(self):
        try:
            self.publish_stop()
        except Exception:
            pass
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = WaypointPoseServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
