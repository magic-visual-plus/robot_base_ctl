#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
csv_ref_pub_zoh.py

ZOH (Zero-Order Hold) version:
- No interpolation between keyframes
- Publish /ref_pose at PUBLISH_HZ
- Hold the latest keyframe pose until next keyframe time arrives
- /ref_twist publishes 0 by default (recommended for stability)

Key timing:
DT_KEY = (1/DATASET_HZ) / PLAYBACK_SCALE
k = floor(t_ref / DT_KEY)

So pose updates only when k changes; between updates, pose is held.
"""

import math, time, re
from typing import Optional
import pandas as pd

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped


CSV_PATH   = "/opt/project/robot_base_ctl/data_20251219_201132_100/data_20251219_201132_100/control_data.csv"
DATASET_HZ = 30.0
MAX_FRAMES = 270

X_COL   = "base_x"
Y_COL   = "base_y"
YAW_COL = "base_pitch"   # 不是yaw就改

REF_POSE_TOPIC  = "/ref_pose"
REF_TWIST_TOPIC = "/ref_twist"

PLAYBACK_SCALE = 1
PUBLISH_HZ     = 10.0
DT_PUB = 1.0 / PUBLISH_HZ
DT_KEY = (1.0 / DATASET_HZ) / PLAYBACK_SCALE  # 关键帧之间隔多少秒（按慢放缩放）


def clamp(x, lo, hi): return max(lo, min(hi, x))
def wrap_to_pi(a): return (a + math.pi) % (2*math.pi) - math.pi

_TENSOR_RE = re.compile(r"tensor\(\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*\)")
def parse_tensor_like(x):
    if isinstance(x, (int, float)):
        return float(x)
    s = str(x).strip()
    m = _TENSOR_RE.fullmatch(s)
    return float(m.group(1)) if m else float(s)

def yaw_to_quat(yaw):
    # only Z rotation
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CSVKeyframes:
    def __init__(self, path):
        df = pd.read_csv(path)
        self.x   = df[X_COL].map(parse_tensor_like).astype(float).to_numpy()
        self.y   = df[Y_COL].map(parse_tensor_like).astype(float).to_numpy()
        self.yaw = df[YAW_COL].map(parse_tensor_like).astype(float).to_numpy()

    def __len__(self): return len(self.x)

    def pose(self, i):
        i = int(clamp(i, 0, len(self.x) - 1))
        return float(self.x[i]), float(self.y[i]), float(self.yaw[i])


class CSVRefPubZOH(Node):
    def __init__(self):
        super().__init__("csv_ref_pub_zoh")

        self.kf = CSVKeyframes(CSV_PATH)
        self.last_idx = min(len(self.kf) - 1, MAX_FRAMES - 1)  # 最大允许的索引

        self.pub_pose  = self.create_publisher(PoseStamped,  REF_POSE_TOPIC,  10)
        self.pub_twist = self.create_publisher(TwistStamped, REF_TWIST_TOPIC, 10)

        self.start_t = time.monotonic()
        self.last_pub_t = time.monotonic()

        # ZOH state
        self.hold_k: Optional[int] = None
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_yaw = 0.0

        # debug
        self.sent_count = 0
        self.last_print_t = time.monotonic()

        self.timer = self.create_timer(DT_PUB, self.on_timer)

        self.get_logger().info(f"[REF-ZOH] CSV frames={len(self.kf)} publish_hz={PUBLISH_HZ}")
        self.get_logger().info(f"[REF-ZOH] DT_KEY={DT_KEY:.6f}s (dataset_hz={DATASET_HZ}, scale={PLAYBACK_SCALE})")
        self.get_logger().info(f"[REF-ZOH] topics: {REF_POSE_TOPIC}, {REF_TWIST_TOPIC}")
        self.get_logger().info("[REF-ZOH] twist=0 (recommended). Control side should do smoothing/accel-limit.")

    def _update_hold_if_needed(self, k: int):
        """Update held pose only when keyframe index changes."""
        if self.hold_k != k:
            self.hold_k = k
            self.hold_x, self.hold_y, self.hold_yaw = self.kf.pose(k)

    def on_timer(self):
        now = time.monotonic()
        dt = max(now - self.last_pub_t, DT_PUB)
        self.last_pub_t = now

        t_ref = now - self.start_t

        # keyframe index for ZOH: floor(t / DT_KEY)
        k = int(t_ref / DT_KEY)
        k = int(clamp(k, 0, self.last_idx))

        self._update_hold_if_needed(k)

        # publish pose (held)
        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = self.hold_x
        pose.pose.position.y = self.hold_y
        qx, qy, qz, qw = yaw_to_quat(self.hold_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # publish twist = 0 (ZOH)
        tw = TwistStamped()
        tw.header.stamp = stamp
        tw.header.frame_id = "map"
        tw.twist.linear.x = 0.0
        tw.twist.linear.y = 0.0
        tw.twist.angular.z = 0.0

        self.pub_pose.publish(pose)
        #self.pub_twist.publish(tw)

        # debug
        self.sent_count += 1
        if (now - self.last_print_t) > 1.0:
            self.last_print_t = now
            self.get_logger().info(
                f"[REF-ZOH] sent={self.sent_count} "
                f"k={self.hold_k} t_ref={t_ref:.2f}s | "
                f"x={self.hold_x:+.3f} y={self.hold_y:+.3f} yaw={self.hold_yaw:+.2f} | "
                f"twist=0"
            )


def main():
    rclpy.init()
    node = CSVRefPubZOH()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
