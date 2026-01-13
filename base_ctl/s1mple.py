#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dataset_replay_ff_only.py  (ROS2)

最简单、最快出效果的方案：
- dataset 提供的是 "下一帧目标位姿" (x,y,_,yaw)
- 我们只用相邻两帧差分得到速度命令（FF）
- 不追绝对位姿、不做 position PI/PID（避免“ref 在跑、车追不上”而发散）
- yaw 可选：加一个很小的 P，用来轻微纠偏（默认开，强度很小）

输入方式完全照你原来的 listen 12Hz 的写法：
- LeRobotDataset 逐帧读 control[:4]
- 用 time.time() 控 12Hz
- odom 只用来拿 yaw（用于 world->body 旋转） + 可选 yaw P

输出：
- self.base.set_body_velocity(cmd_vx, cmd_vy, wz)
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lekiwi_base import LeKiwiBaseConfig, LeKiwiBaseController


# =========================
# 配置
# =========================
MAX_FRAMES = 300
PORT = "/dev/ttyACM0"
ODOM_TOPIC = "/rko_lio/odometry"

DATASET_PATH = "/opt/project/livox_to_point_cloud_ws/src/data_20251219_201132_100/data_20251219_201132_100"
DATASET_HZ = 12.0

# 真实速度上限（m/s）
V_FRONT_MAX = 0.5
V_LEFT_MAX  = 0.5

# yaw 角速度上限（rad/s）
W_MAX = 1.2

# 底盘命令上限（通常 -1..1）
CMD_MAX = 1.0

# 速度->cmd 标定（你原来就是 1）
G_LEFT_FROM_CMDVX  = 1.0
G_FRONT_FROM_CMDVY = 1.0

# ===== 可选：yaw 轻微纠偏（建议先开，系数很小）=====
YAW_P_ENABLED = True
KP_YAW = 0.4   # 很小就行（0.2~0.8）；太大容易“扭头追”


# =========================
# 工具函数
# =========================
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


# =========================
# 主节点
# =========================
class DatasetReplayFFOnly(Node):
    def __init__(self):
        super().__init__("dataset_replay_ff_only")

        # --- dataset ---
        self.dataset = LeRobotDataset(
            repo_id="replay",
            root=DATASET_PATH,
            video_backend="pyav"
        )
        self.dataset_idx = 1
        self.dt_dataset = 1.0 / DATASET_HZ
        self.last_dataset_time = time.time()
        self.get_logger().info(f"Loaded dataset, frames={len(self.dataset)}, hz={DATASET_HZ}")

        # --- 底盘 ---
        cfg = LeKiwiBaseConfig(port=PORT)
        self.base = LeKiwiBaseController(cfg)
        self.base.connect()

        # --- odom（只用来拿 yaw；位置不用于闭环） ---
        self.odom = None
        self.odom_new = False
        self.sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 50)

        # --- limits ---
        self.v_front_max = V_FRONT_MAX
        self.v_left_max  = V_LEFT_MAX
        self.wmax = W_MAX
        self.cmd_max = CMD_MAX
        self.g_left = G_LEFT_FROM_CMDVX
        self.g_front = G_FRONT_FROM_CMDVY

    def odom_cb(self, msg: Odometry):
        self.odom = msg
        self.odom_new = True

    def wait_odom(self):
        self.get_logger().info("Waiting for Odometry ...")
        while rclpy.ok() and self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Got Odometry.")

    def get_yaw_now(self) -> float:
        q = self.odom.pose.pose.orientation
        return quat_to_yaw(q)

    def true_vel_to_cmd(self, v_front, v_left):
        # front/left 是“真实车体速度” -> 底盘库需要的 cmd_vx/cmd_vy
        cmd_vx = v_left / self.g_left
        cmd_vy = v_front / self.g_front
        cmd_vx = clamp(cmd_vx, -self.cmd_max, self.cmd_max)
        cmd_vy = clamp(cmd_vy, -self.cmd_max, self.cmd_max)
        return cmd_vx, cmd_vy

    def run(self):
        self.wait_odom()
        self.get_logger().info("Dataset replay FF-only started.")

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)

                now = time.time()
                if now - self.last_dataset_time < self.dt_dataset:
                    continue

                if self.dataset_idx >= len(self.dataset):
                    self.get_logger().info("Dataset finished.")
                    break
                if self.dataset_idx >= MAX_FRAMES:
                    self.get_logger().info(f"Reached MAX_FRAMES={MAX_FRAMES}. Stop.")
                    break

                if not self.odom_new:
                    continue
                self.odom_new = False

                # ===== 当前 yaw（用于 world->body）=====
                yaw_now = self.get_yaw_now()

                # ===== dataset（只取 control 前4列：x,y,z,yaw；z忽略）=====
                ctrl_now  = self.dataset[self.dataset_idx]["control"][:4]
                ctrl_prev = self.dataset[self.dataset_idx - 1]["control"][:4]

                x_ref, y_ref, _, yaw_ref = ctrl_now
                x_prev, y_prev, _, yaw_prev = ctrl_prev

                # ===== FF：世界系速度（由相邻帧差分得到）=====
                vx_ff_w = (float(x_ref) - float(x_prev)) / self.dt_dataset
                vy_ff_w = (float(y_ref) - float(y_prev)) / self.dt_dataset

                dyaw = wrap_to_pi(float(yaw_ref) - float(yaw_prev))
                wz_ff = dyaw / self.dt_dataset

                # ===== world -> body（用当前 yaw_now）=====
                c = math.cos(yaw_now)
                s = math.sin(yaw_now)
                v_front_ff =  c * vx_ff_w + s * vy_ff_w
                v_left_ff  = -s * vx_ff_w + c * vy_ff_w

                # ===== 限幅（真实速度）=====
                v_front = clamp(v_front_ff, -self.v_front_max, self.v_front_max)
                v_left  = clamp(v_left_ff,  -self.v_left_max,  self.v_left_max)
                wz      = clamp(wz_ff,      -self.wmax,        self.wmax)

                # ===== 可选：yaw 轻微纠偏（只纠 yaw，不追位置）=====
                wz_fb = 0.0
                if YAW_P_ENABLED:
                    yaw_err = wrap_to_pi(float(yaw_ref) - float(yaw_now))
                    wz_fb = clamp(KP_YAW * yaw_err, -0.6, 0.6)  # 纠偏也限一下
                    wz = clamp(wz + wz_fb, -self.wmax, self.wmax)
                else:
                    yaw_err = 0.0

                # ===== 输出到底盘 =====
                cmd_vx, cmd_vy = self.true_vel_to_cmd(v_front, v_left)
                self.base.set_body_velocity(cmd_vx, cmd_vy, wz)

                self.get_logger().info(
                    f"[{self.dataset_idx}] "
                    f"FF(F/L)=({v_front_ff:+.3f},{v_left_ff:+.3f}) "
                    f"CLAMP(F/L)=({v_front:+.3f},{v_left:+.3f}) "
                    f"wz(ff+fb)=({wz_ff:+.2f}+{wz_fb:+.2f}) "
                    f"yaw_err={yaw_err:+.3f} "
                    f"cmd(vx,vy,wz)=({cmd_vx:+.3f},{cmd_vy:+.3f},{wz:+.3f})"
                )

                self.dataset_idx += 1
                self.last_dataset_time = now

        finally:
            self.base.stop()
            self.base.disconnect()
            self.get_logger().info("Dataset replay stopped.")


def main():
    rclpy.init()
    node = DatasetReplayFFOnly()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
