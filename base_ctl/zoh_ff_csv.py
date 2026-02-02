#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import re
import time
import pandas as pd

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped


CSV_PATH   = "/opt/project/robot_base_ctl/data_20251219_201132_100/data_20251219_201132_100/control_data.csv"
MAX_FRAMES = 270

X_COL   = "base_x"
Y_COL   = "base_y"
YAW_COL = "base_pitch"   # 不是yaw就改

REF_POSE_TOPIC  = "/ref_pose"
REF_TWIST_TOPIC = "/ref_twist"

PUBLISH_HZ = 10.0
DT_PUB = 1.0 / PUBLISH_HZ

_TENSOR_RE = re.compile(r"tensor\(\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*\)")

def parse_tensor_like(x):
    if isinstance(x, (int, float)):
        return float(x)
    s = str(x).strip()
    m = _TENSOR_RE.fullmatch(s)
    return float(m.group(1)) if m else float(s)

def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def wrap_to_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi


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

        self.pub_pose  = self.create_publisher(PoseStamped,  REF_POSE_TOPIC,  10)
        self.pub_twist = self.create_publisher(TwistStamped, REF_TWIST_TOPIC, 10)

        self.timer = self.create_timer(DT_PUB, self.on_timer)

        self.get_logger().info(f"[SEQ] CSV total={self.n_total}, play={self.n_play}")
        self.get_logger().info(f"[SEQ] publish_hz={PUBLISH_HZ} pose={REF_POSE_TOPIC} twist={REF_TWIST_TOPIC}")
        self.get_logger().info("[SEQ] FF twist = finite-diff between consecutive rows (world frame)")

    def on_timer(self):
        if self.idx >= self.n_play:
            self.get_logger().info("[SEQ] done. stop timer.")
            self.timer.cancel()
            return

        row = self.idx
        x = float(self.x[row])
        y = float(self.y[row])
        yaw = float(self.yaw[row])

        stamp = self.get_clock().now().to_msg()

        # ---- pose ----
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

        # ---- twist FF (world) ----
        tw = TwistStamped()
        tw.header.stamp = stamp
        tw.header.frame_id = "map"

        if row == 0:
            vx = vy = wz = 0.0
        else:
            dt = DT_PUB  # 因为你就是按这个节拍“读一条发一条”
            vx = (x - float(self.x[row-1])) / dt
            vy = (y - float(self.y[row-1])) / dt
            dyaw = wrap_to_pi(yaw - float(self.yaw[row-1]))
            wz = dyaw / dt

        tw.twist.linear.x  = vx
        tw.twist.linear.y  = vy
        tw.twist.angular.z = wz

        self.pub_pose.publish(pose)
        self.pub_twist.publish(tw)

        # 打印对齐用（可删）
        self.get_logger().info(
            f"[SEQ] row={row+2} x={x:+.4f} y={y:+.4f} yaw={yaw:+.4f} | "
            f"ff(vx,vy,wz)=({vx:+.3f},{vy:+.3f},{wz:+.3f})"
        )

        self.idx += 1


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
