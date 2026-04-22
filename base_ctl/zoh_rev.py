#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
import csv
import os
import numpy as np
from collections import deque
from scipy.signal import savgol_filter

#死区逻辑 
POS_DEADBAND = 0.02          # 2cm
YAW_DEADBAND = math.radians(2.0)  # 2度（可调）

# Savitzky-Golay 滤波器参数
SG_ENABLE = False    # 是否启用 odom 平滑滤波
SG_WINDOW = 5       # 窗口大小（必须为奇数）
SG_POLYORDER = 2    # 多项式阶数

ODOM_TOPIC = "/rko_lio/odometry"
REF_POSE_TOPIC  = "/ref_pose"
REF_TWIST_TOPIC = "/ref_twist"
CMD_TOPIC  = "/cmd_vel"

LOG_CSV = "/opt/project/robot_base_ctl/data_20251219_201132_100/data_20251219_201132_100/violin.csv"
LOG_DECIM = 4     # 每10个控制周期记录1次；200Hz/10=20Hz
LOG_FLUSH_EVERY = 200  # 每写200行flush一次

CONTROL_HZ = 200.0
DT_CTRL = 1.0 / CONTROL_HZ

KP_POS = 1.2
KI_POS = 0.0
I_LIM  = 0.30

KP_YAW = 1.3
W_MAX  = 1.0

V_FRONT_MAX = 0.1
V_LEFT_MAX  = 0.1

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

        self.odom: Odometry | None = None
        self.ref_pose: PoseStamped | None = None
        self.ref_twist: TwistStamped | None = None

        # Savitzky-Golay 滤波缓冲区
        self.odom_buf_x = deque(maxlen=SG_WINDOW)
        self.odom_buf_y = deque(maxlen=SG_WINDOW)
        self.odom_buf_cos_yaw = deque(maxlen=SG_WINDOW)  # 用单位向量表示角度
        self.odom_buf_sin_yaw = deque(maxlen=SG_WINDOW)

        self.create_subscription(Odometry,     ODOM_TOPIC,      self.cb_odom, 10)
        self.create_subscription(PoseStamped,  REF_POSE_TOPIC,  self.cb_pose, 2)
        self.create_subscription(TwistStamped, REF_TWIST_TOPIC, self.cb_tw,   2)

        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        self.intF = 0.0
        self.intL = 0.0
        self.last_vf = self.last_vl = self.last_wz = 0.0

        self.last_t = time.monotonic()
        self.timer = self.create_timer(DT_CTRL, self.on_timer)

        self.last_pose_t = None
        self.last_tw_t = None
        self.ref_timeout = 0.3
        
        # debug counters
        self.pose_rx = 0
        self.tw_rx = 0
        self.cmd_tx = 0
        self.last_print_t = time.monotonic()
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_err = (0.0, 0.0, 0.0)

        self.get_logger().info(f"[CTL] started. output {CONTROL_HZ}Hz cmd_vel")
        self.get_logger().info(f"[CTL] odom smoothing: {'ON' if SG_ENABLE else 'OFF'} (Savitzky-Golay window={SG_WINDOW}, polyorder={SG_POLYORDER})")
        
                # ===== logging =====
        os.makedirs(os.path.dirname(LOG_CSV), exist_ok=True)
        self._log_f = open(LOG_CSV, "w", newline="")
        self._log_w = csv.writer(self._log_f)

        self._log_w.writerow([
            "t",
            "ref_x", "ref_y", "ref_yaw",
            "odom_x", "odom_y", "odom_yaw",
            "cmd_vf", "cmd_vl", "cmd_wz",
        ])
        self._log_n = 0
        self._log_tick = 0
        self.get_logger().info(f"[CTL] logging to {LOG_CSV} (decim={LOG_DECIM})")

    def cb_odom(self, m):
        self.odom = m
        # 将数据存入滤波缓冲区
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = quat_to_yaw(q)
        self.odom_buf_x.append(float(p.x))
        self.odom_buf_y.append(float(p.y))
        # 用单位向量存储角度，避免周期性问题
        self.odom_buf_cos_yaw.append(math.cos(yaw))
        self.odom_buf_sin_yaw.append(math.sin(yaw))

    def cb_pose(self, m):
        self.ref_pose = m
        self.last_pose_t = time.monotonic()
        self.pose_rx += 1

    def cb_tw(self, m):
        self.ref_twist = m
        self.last_tw_t = time.monotonic()
        self.tw_rx += 1

    def get_pose(self):
        # 获取原始数据
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        raw_x, raw_y, raw_yaw = float(p.x), float(p.y), float(quat_to_yaw(q))
        
        # 未启用滤波或缓冲区未满，返回原始数据
        if not SG_ENABLE or len(self.odom_buf_x) < SG_WINDOW:
            return raw_x, raw_y, raw_yaw
        
        # 应用 Savitzky-Golay 滤波
        x_arr = np.array(self.odom_buf_x)
        y_arr = np.array(self.odom_buf_y)
        cos_arr = np.array(self.odom_buf_cos_yaw)
        sin_arr = np.array(self.odom_buf_sin_yaw)
        
        x_smooth = savgol_filter(x_arr, SG_WINDOW, SG_POLYORDER)[-1]
        y_smooth = savgol_filter(y_arr, SG_WINDOW, SG_POLYORDER)[-1]
        
        # 对 cos/sin 分别滤波，再用 atan2 恢复角度
        cos_smooth = savgol_filter(cos_arr, SG_WINDOW, SG_POLYORDER)[-1]
        sin_smooth = savgol_filter(sin_arr, SG_WINDOW, SG_POLYORDER)[-1]
        yaw_smooth = math.atan2(sin_smooth, cos_smooth)
        
        return float(x_smooth), float(y_smooth), float(yaw_smooth)

    def _slew(self, target, last, amax, dt):
        dv = amax * dt
        return clamp(target, last-dv, last+dv)

    def on_timer(self):
        now = time.monotonic()

        if self.odom is None or self.ref_pose is None:
            return

        # ref_pose 超时 -> 停车
        if self.last_pose_t is None or (now - self.last_pose_t) > self.ref_timeout:
            self.intF = 0.0
            self.intL = 0.0
            self.pub.publish(Twist())
            return

        dt = clamp(now - self.last_t, 0.5*DT_CTRL, 3.0*DT_CTRL)
        self.last_t = now


        # ref (world)
        x_ref = float(self.ref_pose.pose.position.x)
        y_ref = float(self.ref_pose.pose.position.y)
        yaw_ref = quat_to_yaw(self.ref_pose.pose.orientation)

        # ref twist (world) - allow missing => 0
        if self.ref_twist is None:
            vx_w = 0.0
            vy_w = 0.0
            wz_ff = 0.0
        else:
            vx_w = float(self.ref_twist.twist.linear.x)
            vy_w = float(self.ref_twist.twist.linear.y)
            wz_ff = float(self.ref_twist.twist.angular.z)

        # odom
        x, y, yaw = self.get_pose()

        # FF -> body
        vff_f, vff_l = world_to_body(yaw, vx_w, vy_w)

        # FB error -> body
        err_f, err_l = world_to_body(yaw, x_ref - x, y_ref - y)
        pos_err = math.hypot(err_f, err_l)

        # ===== deadband =====
        if pos_err < POS_DEADBAND:
            # 死区逻辑
            err_f = 0.0
            err_l = 0.0
            self.intF = 0.0
            self.intL = 0.0
        else:
            self.intF = clamp(self.intF + err_f*dt, -I_LIM, I_LIM)
            self.intL = clamp(self.intL + err_l*dt, -I_LIM, I_LIM)

        vfb_f = KP_POS*err_f + KI_POS*self.intF
        vfb_l = KP_POS*err_l + KI_POS*self.intL

        yaw_err = wrap_to_pi(yaw_ref - yaw)
        if abs(yaw_err) < YAW_DEADBAND:
            yaw_err = 0.0

        wz_fb = KP_YAW * yaw_err


        vf = vff_f + vfb_f
        vl = vff_l + vfb_l
        wz = wz_ff + wz_fb

        vf = clamp(vf, -V_FRONT_MAX, V_FRONT_MAX)
        vl = clamp(vl, -V_LEFT_MAX,  V_LEFT_MAX)
        wz = clamp(wz, -W_MAX,       W_MAX)

        #vf = self._slew(vf, self.last_vf, A_FRONT_MAX, dt)
        #vl = self._slew(vl, self.last_vl, A_LEFT_MAX,  dt)
        #wz = self._slew(wz, self.last_wz, A_W_MAX,      dt)

        self.last_vf, self.last_vl, self.last_wz = vf, vl, wz

        cmd = Twist()
        cmd.linear.x = vf
        cmd.linear.y = vl
        cmd.angular.z = wz
        self.pub.publish(cmd)

        # ===== log (decimated) =====
        self._log_tick += 1
        if (self._log_tick % LOG_DECIM) == 0:
            t = now
            self._log_w.writerow([
                f"{now:.6f}",
                f"{x_ref:.6f}", f"{y_ref:.6f}", f"{yaw_ref:.6f}",
                f"{x:.6f}",     f"{y:.6f}",     f"{yaw:.6f}",
                f"{vf:.6f}",    f"{vl:.6f}",    f"{wz:.6f}",
            ])
            self._log_n += 1
            if (self._log_n % LOG_FLUSH_EVERY) == 0:
                self._log_f.flush()

        
        # debug
        self.cmd_tx += 1
        self.last_cmd = (vf, vl, wz)
        self.last_err = (err_f, err_l, yaw_err)

        if (now - self.last_print_t) > 1.0:
            self.last_print_t = now
            self.get_logger().info(
                f"[CTL] rx_pose={self.pose_rx}/s rx_twist={self.tw_rx}/s tx_cmd={self.cmd_tx}/s | "
                f"errF={self.last_err[0]:+.3f} errL={self.last_err[1]:+.3f} yawErr={self.last_err[2]:+.2f} | "
                f"cmd(vf,vl,wz)=({self.last_cmd[0]:+.3f},{self.last_cmd[1]:+.3f},{self.last_cmd[2]:+.3f})"
            )
            self.pose_rx = 0
            self.tw_rx = 0
            self.cmd_tx = 0

def main():
    rclpy.init()
    n = Assist()
    try:
        rclpy.spin(n)
    finally:
        # best-effort stop
        try:
            n.pub.publish(Twist())
        except Exception:
            pass
        try:
            n._log_f.flush()
            n._log_f.close()
        except Exception:
            pass

        n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
