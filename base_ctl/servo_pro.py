#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dataset_assist_2dof_pid_interp_csv.py

- Keyframes from CSV (e.g. 12Hz)
- Control loop: CONTROL_HZ (e.g. 30/60Hz)
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
import re
from typing import Tuple, Optional

import pandas as pd

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# =========================
# CONFIG
# =========================
CSV_PATH   = "/opt/project/robot_base_ctl/data_20251219_201132_100/data_20251219_201132_100/control_data.csv"  # <<< 改成你的csv路径
DATASET_HZ = 12.0
MAX_FRAMES = 270

# 选择CSV里哪几列作为 (x, y, yaw)
# 你截图里有 base_x/base_y/base_pitch，我默认用 base_pitch 当 yaw（如果不对你改这里）
X_COL   = "base_x"
Y_COL   = "base_y"
YAW_COL = "base_pitch"     # <<< 如果真实yaw不在这，改成你的列名

ODOM_TOPIC = "/rko_lio/odometry"
CMD_TOPIC  = "/cmd_vel"

PLAYBACK_SCALE = 1   # 0.5 = 慢放2倍；0.25 = 慢放4倍

CONTROL_HZ = 200
DT_CTRL    = 1.0 / CONTROL_HZ
DT_KEY     = (1.0 / DATASET_HZ) / PLAYBACK_SCALE

# ---- gains
KP_POS = 1.5
KI_POS = 0.0
I_LIM  = 0.30

KP_YAW = 1.3            # 先关掉 yaw，确认平移能追上再开
W_MAX  = 1.2

# ---- velocity limits
V_FRONT_MAX = 0.5
V_LEFT_MAX  = 0.5

# ---- accel limits (m/s^2) 关键：减少打滑
A_FRONT_MAX = 1.1
A_LEFT_MAX  = 1.1
A_W_MAX     = 2.5
# ---- final hold (simple)
FINAL_HOLD_ENABLE = True
FINAL_POS_TOL = 0.03        # m，只看位置到点
FINAL_HOLD_TIMEOUT = 20.0   # s，防止卡死


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
    dy = wrap_to_pi(y1 - y0)
    return wrap_to_pi(y0 + dy * t)


_TENSOR_RE = re.compile(r"tensor\(\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*\)")


def parse_tensor_like(x) -> float:
    """
    支持：
      - tensor(0.0681)
      - "0.0681"
      - 0.0681
    """
    if x is None:
        raise ValueError("Empty value")
    if isinstance(x, (int, float)):
        return float(x)

    s = str(x).strip()
    m = _TENSOR_RE.fullmatch(s)
    if m:
        return float(m.group(1))

    # 兜底：直接float转换（比如已经是 "0.0681"）
    return float(s)


class CSVKeyframes:
    """
    提供与 LeRobotDataset 类似的最小接口：
      - len()
      - get_pose(idx) -> (x, y, yaw)
    """
    def __init__(self, csv_path: str, x_col: str, y_col: str, yaw_col: str):
        df = pd.read_csv(csv_path)
        for c in (x_col, y_col, yaw_col):
            if c not in df.columns:
                raise KeyError(f"CSV missing column '{c}'. Available: {list(df.columns)}")

        # 转成 float numpy arrays
        self.x = df[x_col].map(parse_tensor_like).astype(float).to_numpy()
        self.y = df[y_col].map(parse_tensor_like).astype(float).to_numpy()
        self.yaw = df[yaw_col].map(parse_tensor_like).astype(float).to_numpy()

    def __len__(self):
        return len(self.x)

    def get_pose(self, idx: int) -> Tuple[float, float, float]:
        return float(self.x[idx]), float(self.y[idx]), float(self.yaw[idx])


# =========================
# main node
# =========================
class DatasetAssist2DOFInterp(Node):
    def __init__(self):
        super().__init__("dataset_assist_2dof_pid_interp_csv")
        self.start_t = time.monotonic()
        self.last_loaded_idx: Optional[int] = None

        # keyframes from CSV
        self.kf = CSVKeyframes(CSV_PATH, X_COL, Y_COL, YAW_COL)
        self.get_logger().info(f"Loaded CSV keyframes: frames={len(self.kf)}, key_hz={DATASET_HZ}")
        self.get_logger().info(f"Using columns: x='{X_COL}', y='{Y_COL}', yaw='{YAW_COL}'")

        # odom
        self.odom = None
        self.create_subscription(Odometry, ODOM_TOPIC, self.cb_odom, 50)

        # cmd
        self.pub_cmd = self.create_publisher(Twist, CMD_TOPIC, 10)

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

        self.last_ctrl_t = time.monotonic()
                # final hold (simple)
        self.in_final_hold = False
        self.final_hold_t0 = None
        self.last_idx = min(len(self.kf) - 1, MAX_FRAMES - 1)


    def cb_odom(self, msg: Odometry):
        self.odom = msg

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

    def _load_keyframes(self, idx: int):
        x_ref, y_ref, yaw_ref = self.kf.get_pose(idx)
        x_prev, y_prev, yaw_prev = self.kf.get_pose(idx - 1)
        self.key_prev = (x_prev, y_prev, yaw_prev)
        self.key_now  = (x_ref,  y_ref,  yaw_ref)

    def _interp_ref(self, alpha: float):
        x0, y0, yaw0 = self.key_prev
        x1, y1, yaw1 = self.key_now
        x = lerp(x0, x1, alpha)
        y = lerp(y0, y1, alpha)
        yaw = lerp_yaw(yaw0, yaw1, alpha)
        return x, y, yaw

    def _slew(self, target, last, amax, dt):
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
                dt_ctrl = clamp(dt_ctrl, 0.5 * DT_CTRL, 3.0 * DT_CTRL)
                self.last_ctrl_t = now

                # --- continuous reference time
                t_ref = now - self.start_t
                k = int(t_ref / DT_KEY) + 1  # idx uses (idx-1, idx)


                # end condition -> simple final hold
                if k >= self.last_idx:
                    if FINAL_HOLD_ENABLE:
                        if not self.in_final_hold:
                            self.in_final_hold = True
                            self.final_hold_t0 = time.monotonic()
                            self.get_logger().info(
                                f"Enter FINAL_HOLD (simple) at last_idx={self.last_idx}, "
                                f"wait until dist<{FINAL_POS_TOL} or timeout {FINAL_HOLD_TIMEOUT}s."
                            )
                        # 把k钉死在最后一帧，不再推进
                        k = self.last_idx
                    else:
                        self.get_logger().info("Reached end.")
                        break


                # load keyframes only when idx changes
                if self.last_loaded_idx != k:
                    self.idx = k
                    self._load_keyframes(self.idx)
                    self.last_loaded_idx = k

                alpha = (t_ref - (k - 1) * DT_KEY) / DT_KEY
                alpha = clamp(alpha, 0.0, 1.0)

                # --- interpolated reference
                if self.in_final_hold:
                    # final hold：固定最后一帧为目标
                    x_ref, y_ref, yaw_ref = self.kf.get_pose(self.last_idx)
                    # 末端你可以保留FF/插值逻辑也行，但最后一帧本质速度应为0
                    vx_ff_w, vy_ff_w, wz_ff = 0.0, 0.0, 0.0
                else:
                    # --- interpolated reference
                    x_ref, y_ref, yaw_ref = self._interp_ref(alpha)

                    # --- interpolated velocity FF
                    alpha2 = clamp(alpha + dt_ctrl / DT_KEY, 0.0, 1.0)
                    x_ref2, y_ref2, yaw_ref2 = self._interp_ref(alpha2)

                    vx_ff_w = (x_ref2 - x_ref) / dt_ctrl
                    vy_ff_w = (y_ref2 - y_ref) / dt_ctrl
                    wz_ff   = wrap_to_pi(yaw_ref2 - yaw_ref) / dt_ctrl


                # current odom
                x_now, y_now, yaw_now = self.get_pose()
                # simple final hold completion check (only position)
                if self.in_final_hold:
                    dist = math.hypot(x_ref - x_now, y_ref - y_now)
                    if (time.monotonic() - self.final_hold_t0) > FINAL_HOLD_TIMEOUT:
                        self.get_logger().warn(f"FINAL_HOLD timeout, dist={dist:.3f}. Stop anyway.")
                        break
                    if dist < FINAL_POS_TOL:
                        self.get_logger().info(f"FINAL_HOLD reached, dist={dist:.3f}. Stop.")
                        break

                # FF world->body
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
