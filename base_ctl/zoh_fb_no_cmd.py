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

import time
import zmq
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

X_COL   = "base_x"
Y_COL   = "base_y"
YAW_COL = "base_pitch"  

REF_POSE_TOPIC = "/ref_pose"

PUBLISH_HZ = 10.0        
DT_PUB = 1.0 / PUBLISH_HZ
# ====================================



def yaw_to_quat(yaw):
    """平面 yaw -> quaternion (只绕 Z)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CSVRefPubSeq(Node):
    def __init__(self):
        super().__init__("csv_ref_pub_seq")

        
        
        # ===== ZMQ =====
        self.zmq_addr = "tcp://10.8.0.90:4399"
        self.zmq_topic = "control"

        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect(self.zmq_addr)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, self.zmq_topic)

        self.poller = zmq.Poller()
        self.poller.register(self.sock, zmq.POLLIN)

        self.get_logger().info(f"[ZMQ] connect {self.zmq_addr}, sub '{self.zmq_topic}'")

        self.idx = 0

        self.pub_pose = self.create_publisher(PoseStamped, REF_POSE_TOPIC, 10)
        self.timer = self.create_timer(DT_PUB, self.on_timer)

        self.last_print_t = time.monotonic()

        self.get_logger().info(f"[REF] publish_hz={PUBLISH_HZ}, topic={REF_POSE_TOPIC}")
        self.get_logger().info("[REF] mode=ZMQ control -> /ref_pose")

    def on_timer(self):
       # 1) poll：0ms 非阻塞
        socks = dict(self.poller.poll(timeout=0))
        if self.sock not in socks:
            return

        try:
            topic, payload = self.sock.recv_multipart(flags=zmq.NOBLOCK)
            print("topic=", topic)
            msg = json.loads(payload.decode("utf-8"))

            x = float(msg["base_X"])
            y = float(msg["base_Y"])
            yaw = float(msg["pitch"])   # 你现在就是这么叫的（注意：这里被当成yaw）

            # 2) 组 PoseStamped
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

            # 3) 发布
            self.pub_pose.publish(pose)
            self.get_logger().info(f"[ZMQ->REF] x={x:+.4f} y={y:+.4f} yaw={yaw:+.4f}")

        except Exception as e:
            self.get_logger().warning(f"[ZMQ] error: {e}")

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
