#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import csv
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from lerobot.datasets.lerobot_dataset import LeRobotDataset


# -----------------------------
# Math helpers
# -----------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q) -> float:
    # q: geometry_msgs/Quaternion
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rot_world_to_body(yaw: float, dx_w: float, dy_w: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    # body: front=x, left=y
    ex = c * dx_w + s * dy_w
    ey = -s * dx_w + c * dy_w
    return ex, ey


def rot_body_to_world(yaw: float, vx_b: float, vy_b: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    vx_w = c * vx_b - s * vy_b
    vy_w = s * vx_b + c * vy_b
    return vx_w, vy_w


# -----------------------------
# PID
# -----------------------------
@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    i_max: float
    d_lpf_hz: float = 0.0  # 0 disables


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

    def step(self, e: float, dt: float, freeze_i: bool = False) -> float:
        if dt <= 0.0:
            return 0.0

        # derivative on error
        if self.e_prev is None:
            de = 0.0
        else:
            de = (e - self.e_prev) / dt
        self.e_prev = e

        # optional d low-pass
        if self.g.d_lpf_hz and self.g.d_lpf_hz > 0.0:
            tau = 1.0 / (2.0 * math.pi * self.g.d_lpf_hz)
            alpha = dt / (tau + dt)
            self.d_filt += alpha * (de - self.d_filt)
            d_term = self.d_filt
        else:
            d_term = de

        # integral
        if not freeze_i and self.g.ki != 0.0:
            self.i += e * dt
            self.i = clamp(self.i, -self.g.i_max, self.g.i_max)

        return self.g.kp * e + self.g.ki * self.i + self.g.kd * d_term


# -----------------------------
# Streaming Pose Servo Node
# -----------------------------
class StreamingPoseServo(Node):
    """
    Follow an absolute pose trajectory from LeRobotDataset control[:4] at DATASET_HZ.

    Output Twist in body frame:
      - FF velocity from dataset adjacent-frame difference
      - FB correction from pose error (ref - odom)
    """

    def __init__(self):
        super().__init__("streaming_pose_servo_dataset")

        # ---- Parameters
        self.declare_parameter("dataset_path", "")
        self.declare_parameter("dataset_hz", 12.0)

        self.declare_parameter("odom_topic", "/rko_lio/odometry")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("control_hz", 50.0)

        # weights: how much correction to add on top of FF
        self.declare_parameter("k_xy", 0.35)
        self.declare_parameter("k_yaw", 0.35)

        # limits
        self.declare_parameter("vx_max", 0.35)   # front m/s
        self.declare_parameter("vy_max", 0.35)   # left  m/s
        self.declare_parameter("wz_max", 1.2)    # rad/s

        # PID gains for correction (FB)
        self.declare_parameter("kp_xy", 1.2)
        self.declare_parameter("ki_xy", 0.0)
        self.declare_parameter("kd_xy", 0.20)
        self.declare_parameter("imax_xy", 0.30)
        self.declare_parameter("d_lpf_xy_hz", 10.0)

        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.15)
        self.declare_parameter("imax_yaw", 0.60)
        self.declare_parameter("d_lpf_yaw_hz", 10.0)

        # end behavior
        self.declare_parameter("end_soft_stop_sec", 0.6)  # ramp down FF at end
        self.declare_parameter("align_on_start", True)    # align dataset pose to odom at start

        # logging
        self.declare_parameter("out_csv", "/tmp/streaming_pose_servo.csv")
        self.declare_parameter("log_every_n", 1)

        # ---- Read params
        self.dataset_path = self.get_parameter("dataset_path").value
        self.dataset_hz = float(self.get_parameter("dataset_hz").value)
        self.dt_ref = 1.0 / max(1e-6, self.dataset_hz)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.dt = 1.0 / max(1e-6, self.control_hz)

        self.k_xy = float(self.get_parameter("k_xy").value)
        self.k_yaw = float(self.get_parameter("k_yaw").value)

        self.vx_max = float(self.get_parameter("vx_max").value)
        self.vy_max = float(self.get_parameter("vy_max").value)
        self.wz_max = float(self.get_parameter("wz_max").value)

        self.end_soft_stop_sec = float(self.get_parameter("end_soft_stop_sec").value)
        self.align_on_start = bool(self.get_parameter("align_on_start").value)

        # ---- Load dataset
        if not self.dataset_path:
            raise RuntimeError("dataset_path parameter is empty")
        self.dataset = LeRobotDataset(repo_id="replay", root=self.dataset_path, video_backend="pyav")
        self.N = len(self.dataset)
        if self.N < 2:
            raise RuntimeError(f"dataset too short: {self.N}")
        self.get_logger().info(f"Loaded dataset: {self.dataset_path}, frames={self.N}, hz={self.dataset_hz}")

        # ---- PID for FB
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

        # ---- Odom state
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ---- Alignment (dataset world -> odom world) as a single SE(2) offset
        self.align_ready = False
        self.off_x = 0.0
        self.off_y = 0.0
        self.off_yaw = 0.0

        # ---- Timing
        self.t0 = self.get_clock().now()
        self.last_ctrl_stamp = self.get_clock().now()

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
            "idx_f", "alpha",
            "x", "y", "yaw",
            "x_ref", "y_ref", "yaw_ref",
            "vx_ff", "vy_ff", "wz_ff",
            "ex", "ey", "eyaw",
            "vx_fb", "vy_fb", "wz_fb",
            "vx_cmd", "vy_cmd", "wz_cmd",
        ])

        # ---- Timer
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"StreamingPoseServo started. odom={self.odom_topic} -> cmd={self.cmd_topic} | "
            f"control_hz={self.control_hz} dataset_hz={self.dataset_hz} csv={self.out_csv}"
        )

    # -------------------------
    # callbacks
    # -------------------------
    def cb_odom(self, msg: Odometry):
        self.have_odom = True
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = yaw_from_quat(q)

    # -------------------------
    # dataset helpers
    # -------------------------
    def get_dataset_pose(self, idx: int) -> Tuple[float, float, float]:
        ctrl = self.dataset[idx]["control"][:4]
        x, y, _, yaw = ctrl
        return float(x), float(y), float(yaw)

    def apply_alignment(self, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
        # rotate + translate + yaw offset
        c = math.cos(self.off_yaw)
        s = math.sin(self.off_yaw)
        xr = c * x - s * y
        yr = s * x + c * y
        return xr + self.off_x, yr + self.off_y, wrap_to_pi(yaw + self.off_yaw)

    def ensure_alignment(self):
        if self.align_ready:
            return
        if not self.have_odom:
            return

        # align first dataset pose to current odom pose
        x0_d, y0_d, yaw0_d = self.get_dataset_pose(0)

        if self.align_on_start:
            # want: apply_alignment(x0_d, y0_d, yaw0_d) == (x_odom, y_odom, yaw_odom)
            self.off_yaw = wrap_to_pi(self.yaw - yaw0_d)

            c = math.cos(self.off_yaw)
            s = math.sin(self.off_yaw)
            x0r = c * x0_d - s * y0_d
            y0r = s * x0_d + c * y0_d

            self.off_x = self.x - x0r
            self.off_y = self.y - y0r
            self.get_logger().info(
                f"Alignment set: off_x={self.off_x:.3f}, off_y={self.off_y:.3f}, off_yaw={self.off_yaw:.3f}rad"
            )
        else:
            self.off_x = 0.0
            self.off_y = 0.0
            self.off_yaw = 0.0
            self.get_logger().info("Alignment disabled: using raw dataset pose as ref.")

        self.align_ready = True
        self.t0 = self.get_clock().now()  # start time after alignment
        self.pid_x.reset(); self.pid_y.reset(); self.pid_yaw.reset()

    # -------------------------
    # publishing
    # -------------------------
    def publish_stop(self):
        self.pub_cmd.publish(Twist())

    # -------------------------
    # main control
    # -------------------------
    def control_loop(self):
        if not self.have_odom:
            return

        self.ensure_alignment()
        if not self.align_ready:
            return

        # elapsed time -> continuous frame index
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        f = t * self.dataset_hz   # floating index
        idx = int(math.floor(f))
        alpha = f - idx

        # if beyond end: soft stop
        if idx >= self.N - 1:
            # ramp down FF to zero over end_soft_stop_sec
            # keep FB trying to settle at last pose
            idx = self.N - 1
            alpha = 0.0

        # choose i0,i1 for interpolation
        i0 = clamp(idx, 0, self.N - 2)
        i1 = i0 + 1

        # read poses
        x0, y0, yaw0 = self.get_dataset_pose(i0)
        x1, y1, yaw1 = self.get_dataset_pose(i1)

        # interpolate (yaw needs wrap)
        dyaw01 = wrap_to_pi(yaw1 - yaw0)
        x_ref = (1.0 - alpha) * x0 + alpha * x1
        y_ref = (1.0 - alpha) * y0 + alpha * y1
        yaw_ref = wrap_to_pi(yaw0 + alpha * dyaw01)

        # apply alignment so ref is in odom-world
        x_ref, y_ref, yaw_ref = self.apply_alignment(x_ref, y_ref, yaw_ref)

        # reference velocity from adjacent frame diff (world)
        vx_ref_w = (x1 - x0) / self.dt_ref
        vy_ref_w = (y1 - y0) / self.dt_ref
        wz_ff = wrap_to_pi(yaw1 - yaw0) / self.dt_ref

        # apply yaw alignment to world velocity direction (rotate by off_yaw)
        # because x/y were rotated into odom world
        c0 = math.cos(self.off_yaw)
        s0 = math.sin(self.off_yaw)
        vx_ref_w_al = c0 * vx_ref_w - s0 * vy_ref_w
        vy_ref_w_al = s0 * vx_ref_w + c0 * vy_ref_w

        # world->body for FF using current yaw
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        vx_ff =  c * vx_ref_w_al + s * vy_ref_w_al   # front
        vy_ff = -s * vx_ref_w_al + c * vy_ref_w_al   # left

        # pose error (world) then to body
        dx_w = x_ref - self.x
        dy_w = y_ref - self.y
        ex, ey = rot_world_to_body(self.yaw, dx_w, dy_w)
        eyaw = wrap_to_pi(yaw_ref - self.yaw)

        # FB correction
        vx_fb_raw = self.pid_x.step(ex, self.dt, freeze_i=False)
        vy_fb_raw = self.pid_y.step(ey, self.dt, freeze_i=False)
        wz_fb_raw = self.pid_yaw.step(eyaw, self.dt, freeze_i=False)

        vx_fb = self.k_xy * vx_fb_raw
        vy_fb = self.k_xy * vy_fb_raw
        wz_fb = self.k_yaw * wz_fb_raw

        # combine
        vx_cmd = vx_ff + vx_fb
        vy_cmd = vy_ff + vy_fb
        wz_cmd = wz_ff + wz_fb

        # end soft stop: ramp down FF when near end
        # (when t exceeds total duration - end_soft_stop_sec)
        total_T = (self.N - 1) / self.dataset_hz
        if self.end_soft_stop_sec > 1e-3 and t > total_T - self.end_soft_stop_sec:
            r = clamp((total_T - t) / self.end_soft_stop_sec, 0.0, 1.0)
            vx_cmd = (vx_ff * r) + vx_fb
            vy_cmd = (vy_ff * r) + vy_fb
            wz_cmd = (wz_ff * r) + wz_fb

        # clamp
        vx_cmd = clamp(vx_cmd, -self.vx_max, self.vx_max)
        vy_cmd = clamp(vy_cmd, -self.vy_max, self.vy_max)
        wz_cmd = clamp(wz_cmd, -self.wz_max, self.wz_max)

        # publish Twist (body frame)
        tw = Twist()
        tw.linear.x = float(vx_cmd)
        tw.linear.y = float(vy_cmd)
        tw.angular.z = float(wz_cmd)
        self.pub_cmd.publish(tw)

        # log
        self._tick += 1
        if self.log_every_n > 0 and (self._tick % self.log_every_n == 0):
            self._csv.writerow([
                f"{t:.6f}",
                f"{float(i0):.3f}", f"{alpha:.3f}",
                f"{self.x:.6f}", f"{self.y:.6f}", f"{self.yaw:.6f}",
                f"{x_ref:.6f}", f"{y_ref:.6f}", f"{yaw_ref:.6f}",
                f"{vx_ff:.6f}", f"{vy_ff:.6f}", f"{wz_ff:.6f}",
                f"{ex:.6f}", f"{ey:.6f}", f"{eyaw:.6f}",
                f"{vx_fb:.6f}", f"{vy_fb:.6f}", f"{wz_fb:.6f}",
                f"{vx_cmd:.6f}", f"{vy_cmd:.6f}", f"{wz_cmd:.6f}",
            ])
            if self._tick % int(max(1.0, self.control_hz)) == 0:
                self._csv_file.flush()

        # stop when finished and stabilized a bit
        if t > total_T + 0.2:
            self.get_logger().info("Trajectory finished. Stopping.")
            self.publish_stop()
            # we can shutdown by destroying timer
            self.timer.cancel()


def main():
    rclpy.init()
    node = StreamingPoseServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
        try:
            node._csv_file.flush()
            node._csv_file.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
