#!/usr/bin/env python3
# -*- coding: utf-8 -*-
测试github1
"""
dataset_assist_2dof_pid_interp.py

- Dataset keyframes: 12Hz (idx increments at 12Hz)
- Control loop: CONTROL_HZ (e.g. 60Hz)
- Between keyframes: interpolate ref pose (x,y,yaw) => "补帧"
- Output: /cmd_vel

FF:
  use interpolated ref velocity (finite difference on interpolated ref)
FB:
  PID on (ref - odom) error (world -> body)

Also includes:
  - command accel limit (slew rate) to reduce wheel slip
  - robust timing (monotonic clock)
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from lerobot.datasets.lerobot_dataset import LeRobotDataset


# =========================
# CONFIG
# =========================
DATASET_PATH = "/opt/project/livox_to_point_cloud_ws/src/data_20251219_201132_100/data_20251219_201132_100"
DATASET_HZ   = 12.0
MAX_FRAMES   = 270

ODOM_TOPIC = "/rko_lio/odometry"
CMD_TOPIC  = "/cmd_vel"

CONTROL_HZ = 30.0           # 补帧控制频率（30/60都行，建议60）
DT_CTRL    = 1.0 / CONTROL_HZ
DT_KEY     = 1.0 / DATASET_HZ

# ---- gains
KP_POS = 0.9
KI_POS = 0.0
I_LIM  = 0.30

KP_YAW = 0.0               # 先关掉 yaw，确认平移能追上再开
W_MAX  = 1.2

# ---- velocity limits
V_FRONT_MAX = 0.35
V_LEFT_MAX  = 0.35

# ---- accel limits (m/s^2) 关键：减少打滑
A_FRONT_MAX = 0.8
A_LEFT_MAX  = 0.8
A_W_MAX     = 2.5


# =========================
# helpers
# =========================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def world_to_body(yaw, dx_w, dy_w):
    front =  math.cos(yaw) * dx_w + math.sin(yaw) * dy_w
    left  = -math.sin(yaw) * dx_w + math.cos(yaw) * dy_w
    return front, left


def lerp(a, b, t):
    return a + (b - a) * t


def lerp_yaw(y0, y1, t):
    # shortest-path interpolation on circle
    dy = wrap_to_pi(y1 - y0)
    return wrap_to_pi(y0 + dy * t)


# =========================
# main node
# =========================
class DatasetAssist2DOFInterp(Node):
    def __init__(self):
        super().__init__("dataset_assist_2dof_pid_interp")

        # dataset
        self.dataset = LeRobotDataset(
            repo_id="replay",
            root=DATASET_PATH,
            video_backend="pyav",
        )
        self.get_logger().info(f"Loaded dataset: frames={len(self.dataset)}, key_hz={DATASET_HZ}")

        # odom
        self.odom = None
        self.odom_new = False
        self.create_subscription(Odometry, ODOM_TOPIC, self.cb_odom, 50)

        # cmd
        self.pub_cmd = self.create_publisher(Twist, CMD_TOPIC, 10)

        # control timers
        self.t0 = time.monotonic()
        self.last_ctrl_t = time.monotonic()
        self.last_key_t  = time.monotonic()

        # keyframe indices
        self.idx = 1  # current keyframe index (uses idx-1 and idx)
        self.key_prev = None
        self.key_now  = None
        self._load_keyframes(self.idx)

        # PID integrator
        self.kp = KP_POS
        self.ki = KI_POS
        self.i_lim = I_LIM
        self.kp_yaw = KP_YAW

        self.int_errF = 0.0
        self.int_errL = 0.0

        # last command for accel limit
        self.last_vf = 0.0
        self.last_vl = 0.0
        self.last_wz = 0.0

    def cb_odom(self, msg: Odometry):
        self.odom = msg
        self.odom_new = True

    def wait_odom(self):
        self.get_logger().info("Waiting for odometry...")
        while rclpy.ok() and self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Got odometry.")

    def get_pose(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = quat_to_yaw(q)
        return p.x, p.y, yaw

    def _load_keyframes(self, idx):
        ctrl_now  = self.dataset[idx]["control"][:4]
        ctrl_prev = self.dataset[idx - 1]["control"][:4]
        x_ref, y_ref, _, yaw_ref = map(float, ctrl_now)
        x_prev, y_prev, _, yaw_prev = map(float, ctrl_prev)
        self.key_prev = (x_prev, y_prev, yaw_prev)
        self.key_now  = (x_ref,  y_ref,  yaw_ref)

    def _interp_ref(self, alpha):
        # alpha in [0,1]
        x0, y0, yaw0 = self.key_prev
        x1, y1, yaw1 = self.key_now
        x = lerp(x0, x1, alpha)
        y = lerp(y0, y1, alpha)
        yaw = lerp_yaw(yaw0, yaw1, alpha)
        return x, y, yaw

    def _slew(self, target, last, amax, dt):
        # accel limit
        dv_max = amax * dt
        return clamp(target, last - dv_max, last + dv_max)

    def run(self):
        self.wait_odom()
        self.get_logger().info(f"Interp assist started: CONTROL_HZ={CONTROL_HZ}")

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)

                now = time.monotonic()
                dt_ctrl = now - self.last_ctrl_t
                if dt_ctrl < DT_CTRL:
                    continue
                # keep dt bounded
                dt_ctrl = clamp(dt_ctrl, 0.5 * DT_CTRL, 3.0 * DT_CTRL)
                self.last_ctrl_t = now

                if self.idx >= len(self.dataset) or self.idx >= MAX_FRAMES:
                    self.get_logger().info("Finished.")
                    break

                if not self.odom_new:
                    continue
                self.odom_new = False

                # --- advance keyframe every DT_KEY (12Hz)
                if now - self.last_key_t >= DT_KEY:
                    self.last_key_t += DT_KEY
                    self.idx += 1
                    if self.idx >= len(self.dataset) or self.idx >= MAX_FRAMES:
                        self.get_logger().info("Reached end.")
                        break
                    self._load_keyframes(self.idx)

                # --- alpha within current keyframe interval
                alpha = (now - self.last_key_t) / DT_KEY
                alpha = clamp(alpha, 0.0, 1.0)

                # --- interpolated reference
                x_ref, y_ref, yaw_ref = self._interp_ref(alpha)

                # --- interpolated velocity FF (finite difference on interpolated ref)
                # use a small epsilon = dt_ctrl for derivative
                alpha2 = clamp(alpha + dt_ctrl / DT_KEY, 0.0, 1.0)
                x_ref2, y_ref2, yaw_ref2 = self._interp_ref(alpha2)

                vx_ff_w = (x_ref2 - x_ref) / dt_ctrl
                vy_ff_w = (y_ref2 - y_ref) / dt_ctrl
                wz_ff   = wrap_to_pi(yaw_ref2 - yaw_ref) / dt_ctrl

                # current odom
                x_now, y_now, yaw_now = self.get_pose()

                # FF world->body (use yaw_now)
                v_front_ff, v_left_ff = world_to_body(yaw_now, vx_ff_w, vy_ff_w)

                # FB error
                dx_w = x_ref - x_now
                dy_w = y_ref - y_now
                errF, errL = world_to_body(yaw_now, dx_w, dy_w)

                self.int_errF += errF * dt_ctrl
                self.int_errL += errL * dt_ctrl
                self.int_errF = clamp(self.int_errF, -self.i_lim, self.i_lim)
                self.int_errL = clamp(self.int_errL, -self.i_lim, self.i_lim)

                v_front_fb = self.kp * errF + self.ki * self.int_errF
                v_left_fb  = self.kp * errL + self.ki * self.int_errL

                yaw_err = wrap_to_pi(yaw_ref - yaw_now)
                wz_fb = self.kp_yaw * yaw_err

                # combine
                v_front = v_front_ff + v_front_fb
                v_left  = v_left_ff  + v_left_fb
                wz      = wz_ff + wz_fb

                # speed limits
                v_front = clamp(v_front, -V_FRONT_MAX, V_FRONT_MAX)
                v_left  = clamp(v_left,  -V_LEFT_MAX,  V_LEFT_MAX)
                wz      = clamp(wz,      -W_MAX,        W_MAX)

                # accel limits (slew)
                v_front = self._slew(v_front, self.last_vf, A_FRONT_MAX, dt_ctrl)
                v_left  = self._slew(v_left,  self.last_vl, A_LEFT_MAX,  dt_ctrl)
                wz      = self._slew(wz,      self.last_wz, A_W_MAX,      dt_ctrl)

                self.last_vf, self.last_vl, self.last_wz = v_front, v_left, wz

                # publish
                cmd = Twist()
                cmd.linear.x = float(v_front)
                cmd.linear.y = float(v_left)
                cmd.angular.z = float(wz)
                self.pub_cmd.publish(cmd)

                # log (少量打印避免刷屏)
                self.get_logger().info(
                    f"[key={self.idx:4d} a={alpha:.2f}] "
                    f"err(F/L)({errF:+.3f},{errL:+.3f}) "
                    f"FF({v_front_ff:+.3f},{v_left_ff:+.3f}) "
                    f"FB({v_front_fb:+.3f},{v_left_fb:+.3f}) "
                    f"CMD({v_front:+.3f},{v_left:+.3f},{wz:+.3f})"
                )

        finally:
            self.pub_cmd.publish(Twist())
            self.get_logger().info("Interp assist stopped.")


def main():
    rclpy.init()
    node = DatasetAssist2DOFInterp()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
