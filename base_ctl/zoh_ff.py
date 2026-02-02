#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import zmq
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped

REF_POSE_TOPIC  = "/ref_pose"
REF_TWIST_TOPIC = "/ref_twist"

PUBLISH_HZ = 10.0
DT_PUB = 1.0 / PUBLISH_HZ


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def wrap_to_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ZMQRefPub(Node):
    def __init__(self):
        super().__init__("zmq_ref_pub_pose_twist")

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

        # pubs
        self.pub_pose  = self.create_publisher(PoseStamped,  REF_POSE_TOPIC,  10)
        self.pub_twist = self.create_publisher(TwistStamped, REF_TWIST_TOPIC, 10)

        self.timer = self.create_timer(DT_PUB, self.on_timer)

        # last state for finite diff
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.last_t = None  # monotonic time

        self.get_logger().info(f"[REF] publish_hz={PUBLISH_HZ} pose={REF_POSE_TOPIC} twist={REF_TWIST_TOPIC}")
        self.get_logger().info("[REF] mode=ZMQ control -> /ref_pose + /ref_twist (finite-diff)")

    def _recv_latest(self):
        """
        把当前 ZMQ 缓冲里能取到的消息尽量取完，返回“最新一条”
        这样避免 backlog 导致你发的是旧消息。
        """
        latest = None
        while True:
            socks = dict(self.poller.poll(timeout=0))
            if self.sock not in socks:
                break
            try:
                topic, payload = self.sock.recv_multipart(flags=zmq.NOBLOCK)
                latest = (topic, payload)
            except zmq.Again:
                break
        return latest

    def on_timer(self):
        got = self._recv_latest()
        if got is None:
            return

        try:
            topic, payload = got
            msg = json.loads(payload.decode("utf-8"))

            # 你当前字段名（注意大小写）
            x   = float(msg["base_X"])
            y   = float(msg["base_Y"])
            yaw = float(msg["pitch"])   # 你现在把 pitch 当 yaw 用

            now_t = time.monotonic()

            # dt：用真实到达时间差更合理
            if self.last_t is None:
                dt = DT_PUB
            else:
                dt = now_t - self.last_t
                # 防止 dt 太小/太大（比如 burst 或断流后恢复）
                dt = clamp(dt, 0.001, 0.5)

            # ---- pose ----
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

            # ---- twist (world frame) ----
            tw = TwistStamped()
            tw.header.stamp = stamp
            tw.header.frame_id = "map"

            if self.last_x is None:
                vx = vy = wz = 0.0
            else:
                vx = (x - self.last_x) / dt
                vy = (y - self.last_y) / dt
                dyaw = wrap_to_pi(yaw - self.last_yaw)
                wz = dyaw / dt

            tw.twist.linear.x  = vx
            tw.twist.linear.y  = vy
            tw.twist.angular.z = wz

            # publish
            self.pub_pose.publish(pose)
            self.pub_twist.publish(tw)

            self.get_logger().info(
                f"[ZMQ->REF] x={x:+.4f} y={y:+.4f} yaw={yaw:+.4f} | "
                f"ff(vx,vy,wz)=({vx:+.3f},{vy:+.3f},{wz:+.3f}) dt={dt:.3f}"
            )

            # update last
            self.last_x, self.last_y, self.last_yaw, self.last_t = x, y, yaw, now_t

        except Exception as e:
            self.get_logger().warning(f"[ZMQ] error: {e}")


def main():
    rclpy.init()
    node = ZMQRefPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
