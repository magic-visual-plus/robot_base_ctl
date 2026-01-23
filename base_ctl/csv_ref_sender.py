#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time, re
from typing import Tuple, Optional
import pandas as pd

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from builtin_interfaces.msg import Time as RosTime


CSV_PATH   = "/opt/project/robot_base_ctl/data_20251219_201132_100/data_20251219_201132_100/control_data.csv"
DATASET_HZ = 12.0
MAX_FRAMES = 270

X_COL   = "base_x"
Y_COL   = "base_y"
YAW_COL = "base_pitch"   # 不是yaw就改

REF_POSE_TOPIC  = "/ref_pose"
REF_TWIST_TOPIC = "/ref_twist"

PLAYBACK_SCALE = 1.0
PUBLISH_HZ = 30.0        # <<< 你VR输出30Hz就写30；你想补帧到200就写200
DT_PUB = 1.0 / PUBLISH_HZ
DT_KEY = (1.0 / DATASET_HZ) / PLAYBACK_SCALE


def clamp(x, lo, hi): return max(lo, min(hi, x))
def wrap_to_pi(a): return (a + math.pi) % (2*math.pi) - math.pi
def lerp(a,b,t): return a + (b-a)*t
def lerp_yaw(y0,y1,t):
    dy = wrap_to_pi(y1-y0)
    return wrap_to_pi(y0 + dy*t)

_TENSOR_RE = re.compile(r"tensor\(\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*\)")
def parse_tensor_like(x):
    if isinstance(x,(int,float)): return float(x)
    s = str(x).strip()
    m = _TENSOR_RE.fullmatch(s)
    return float(m.group(1)) if m else float(s)

def yaw_to_quat(yaw):
    # 只绕Z轴：q = [0,0,sin(y/2),cos(y/2)]
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))


class CSVKeyframes:
    def __init__(self, path):
        df = pd.read_csv(path)
        self.x   = df[X_COL].map(parse_tensor_like).astype(float).to_numpy()
        self.y   = df[Y_COL].map(parse_tensor_like).astype(float).to_numpy()
        self.yaw = df[YAW_COL].map(parse_tensor_like).astype(float).to_numpy()
    def __len__(self): return len(self.x)
    def pose(self,i): return float(self.x[i]), float(self.y[i]), float(self.yaw[i])


class CSVRefPub(Node):
    def __init__(self):
        super().__init__("csv_ref_pub_pose_twist")
        self.kf = CSVKeyframes(CSV_PATH)
        self.last_idx = min(len(self.kf)-1, MAX_FRAMES-1)

        self.pub_pose  = self.create_publisher(PoseStamped,  REF_POSE_TOPIC,  10)
        self.pub_twist = self.create_publisher(TwistStamped, REF_TWIST_TOPIC, 10)

        self.start_t = time.monotonic()
        self.last_pub_t = time.monotonic()

        self.last_loaded_k: Optional[int] = None
        self.key_prev = None
        self.key_now  = None

        self.timer = self.create_timer(DT_PUB, self.on_timer)

        self.get_logger().info(f"CSV loaded frames={len(self.kf)} publish_hz={PUBLISH_HZ}")

    def _load(self,k):
        self.key_prev = self.kf.pose(k-1)
        self.key_now  = self.kf.pose(k)

    def _interp(self, a):
        x0,y0,yaw0 = self.key_prev
        x1,y1,yaw1 = self.key_now
        return (lerp(x0,x1,a), lerp(y0,y1,a), lerp_yaw(yaw0,yaw1,a))

    def on_timer(self):
        now = time.monotonic()
        dt = max(now - self.last_pub_t, DT_PUB)
        self.last_pub_t = now

        t_ref = now - self.start_t
        k = int(t_ref / DT_KEY) + 1
        if k >= self.last_idx:
            k = self.last_idx

        if self.last_loaded_k != k:
            self._load(k)
            self.last_loaded_k = k

        a = clamp((t_ref - (k-1)*DT_KEY)/DT_KEY, 0.0, 1.0)
        x,y,yaw = self._interp(a)

        # 用插值轨迹算速度（世界系）
        a2 = clamp(a + dt/DT_KEY, 0.0, 1.0)
        x2,y2,yaw2 = self._interp(a2)
        vx_w = (x2-x)/dt
        vy_w = (y2-y)/dt
        wz   = wrap_to_pi(yaw2-yaw)/dt

        # stamp
        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx,qy,qz,qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        tw = TwistStamped()
        tw.header.stamp = stamp
        tw.header.frame_id = "map"   # 表示这是世界系速度
        tw.twist.linear.x = vx_w
        tw.twist.linear.y = vy_w
        tw.twist.angular.z = wz

        self.pub_pose.publish(pose)
        self.pub_twist.publish(tw)


def main():
    rclpy.init()
    node = CSVRefPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
