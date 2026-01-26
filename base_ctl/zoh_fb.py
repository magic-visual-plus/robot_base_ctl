#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
csv_ref_pub_seq.py

顺序播放 CSV：
- 每次 timer 触发：读当前 idx 的一行 -> 发布 /ref_pose -> idx += 1
- 不使用 k，不使用 DT_KEY，不做插值，不做 ZOH
- 播放到 MAX_FRAMES 或 CSV 末尾就停止（或者你也可以改成循环）
"""

import math
import re
import time
import pandas as pd

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped



CSV_PATH   = "/opt/project/robot_base_ctl/data_20251219_201132_100/data_20251219_201132_100/control_data.csv"
MAX_FRAMES = 270

X_COL   = "base_x"
Y_COL   = "base_y"
YAW_COL = "base_pitch"  

REF_POSE_TOPIC = "/ref_pose"

PUBLISH_HZ = 10.0        
DT_PUB = 1.0 / PUBLISH_HZ
# ====================================


_TENSOR_RE = re.compile(r"tensor\(\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*\)")

def parse_tensor_like(x):
    """把 tensor(0.123) 或 '0.123' 统一转 float"""
    if isinstance(x, (int, float)):
        return float(x)
    s = str(x).strip()
    m = _TENSOR_RE.fullmatch(s)
    return float(m.group(1)) if m else float(s)

def yaw_to_quat(yaw):
    """平面 yaw -> quaternion (只绕 Z)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CSVRefPubSeq(Node):
    def __init__(self):
        super().__init__("csv_ref_pub_seq")

        df = pd.read_csv(CSV_PATH)

        self.x   = df[X_COL].map(parse_tensor_like).astype(float).to_numpy()
        self.y   = df[Y_COL].map(parse_tensor_like).astype(float).to_numpy()
        self.yaw = df[YAW_COL].map(parse_tensor_like).astype(float).to_numpy()

        self.n_total = len(self.x)
        self.n_play  = min(self.n_total, MAX_FRAMES)

        self.idx = 0

        self.pub_pose = self.create_publisher(PoseStamped, REF_POSE_TOPIC, 10)
        self.timer = self.create_timer(DT_PUB, self.on_timer)

        self.last_print_t = time.monotonic()

        self.get_logger().info(f"[SEQ] CSV loaded: total={self.n_total}, play={self.n_play}")
        self.get_logger().info(f"[SEQ] publish_hz={PUBLISH_HZ}, topic={REF_POSE_TOPIC}")
        self.get_logger().info("[SEQ] mode=read one row -> publish one msg -> idx++")

    def on_timer(self):
        # 1) 播完了就停止并退出节点
        if self.idx >= self.n_play:
            self.get_logger().info("[SEQ] done. stop timer and shutdown node.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        # 2) 取当前行
        row = self.idx   # <<< 当前发布的 CSV 行号（0-based）
        x = float(self.x[row])
        y = float(self.y[row])
        yaw = float(self.yaw[row])

        # 3) 组 PoseStamped
        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y

        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # 4) 发布
        self.pub_pose.publish(pose)
        self.get_logger().info(f"[SEQ] publish row={row+2} x={x:+.4f} y={y:+.4f} yaw={yaw:+.4f}")

        # 5) idx++
        self.idx += 1

        # # 6) 每秒打印一次状态
        # now = time.monotonic()
        # if now - self.last_print_t > 1.0:
        #     self.last_print_t = now
        #     self.get_logger().info(f"[SEQ] idx={self.idx}/{self.n_play} x={x:+.3f} y={y:+.3f} yaw={yaw:+.2f}")


def main():
    rclpy.init()
    node = CSVRefPubSeq()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
