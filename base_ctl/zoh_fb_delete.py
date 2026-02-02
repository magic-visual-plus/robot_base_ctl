#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import zmq
import json
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor

REF_POSE_TOPIC = "/ref_pose"


def yaw_to_quat(yaw):
    """平面 yaw -> quaternion (只绕 Z)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CSVRefPubSeq(Node):
    def __init__(self):
        super().__init__("csv_ref_pub_seq")

        # ===== ZMQ =====
        self.zmq_addr = "tcp://10.8.0.90:4399"
        self.zmq_topic = b"control"

        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect(self.zmq_addr)

        # 订阅 control
        self.sock.setsockopt(zmq.SUBSCRIBE, self.zmq_topic)

        # 想“每条都要”，就把队列上限调大点，减少丢包风险
        self.sock.setsockopt(zmq.RCVHWM, 10000)

        self.get_logger().info(f"[ZMQ] connect {self.zmq_addr}, sub '{self.zmq_topic.decode()}'")

        # ===== ROS pubs =====
        self.pub_pose = self.create_publisher(PoseStamped, REF_POSE_TOPIC, 10)

        # ===== 控制状态：start/end 门控 =====
        self.active = False  # 默认不跑

        # ===== reset odom service client =====
        self.reset_cli = self.create_client(Trigger, "/rko_lio/reset_odometry")
        if not self.reset_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning("[SRV] /rko_lio/reset_odometry not ready yet (will try when called)")
        else:
            self.get_logger().info("[SRV] /rko_lio/reset_odometry ready")

        # ===== DEBUG / 状态统计 =====
        self.rx_total = 0
        self.rx_base = 0
        self.tx_pose = 0
        self.reset_calls = 0
        self.seq = 0

        self.last_cmd = "none"

        # ===== 线程控制 =====
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info("[REF] mode=ZMQ recv (blocking) -> publish /ref_pose per message")

    def destroy_node(self):
        # 让线程退出
        self._running = False
        try:
            self.sock.close(0)
        except Exception:
            pass
        return super().destroy_node()

    def call_reset(self):
        """异步调用 reset_odometry"""
        self.reset_calls += 1
        req = Trigger.Request()
        fut = self.reset_cli.call_async(req)

        def _done_cb(f):
            try:
                resp = f.result()
                self.get_logger().info(f"[RESET] done success={resp.success} msg='{resp.message}'")
            except Exception as e:
                self.get_logger().warning(f"[RESET] call failed: {e}")

        fut.add_done_callback(_done_cb)

    def _publish_pose(self, x: float, y: float, yaw: float):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y

        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pub_pose.publish(pose)
        self.tx_pose += 1
        self.seq += 1

    def _rx_loop(self):
        """像你纯 ZMQ while 一样：每条消息到就立刻处理+发布"""
        while self._running and rclpy.ok():
            try:
                topic, payload = self.sock.recv_multipart()  # 阻塞等消息，来一条就处理一条
                if topic != self.zmq_topic:
                    continue

                msg = json.loads(payload)
                self.rx_total += 1

                cmd = str(msg.get("command", "base")).lower()
                self.last_cmd = cmd

                # 先把收到的东西都打出来（你要每条都输出）
                x_any = msg.get("base_X", None)
                y_any = msg.get("base_Y", None)
                yaw_any = msg.get("pitch", None)
                self.get_logger().info(f"[RX] active={self.active} cmd={cmd} | base_X={x_any} base_Y={y_any} pitch={yaw_any}")

                if cmd == "start":
                    self.active = True
                    self.get_logger().info("[STATE] active=True")
                    continue

                if cmd == "end":
                    self.active = False
                    self.get_logger().info("[STATE] active=False")
                    continue

                if cmd == "reset":
                    self.active = False
                    self.get_logger().info("[CMD] reset -> call /rko_lio/reset_odometry")
                    if self.reset_cli.service_is_ready():
                        self.call_reset()
                    else:
                        self.get_logger().warning("[SRV] /rko_lio/reset_odometry not ready")
                    continue

                if cmd == "base":
                    self.rx_base += 1
                    if not self.active:
                        continue  # 没 start 就不发

                    x = float(msg["base_X"])
                    y = float(msg["base_Y"])
                    yaw = float(msg["pitch"])

                    self._publish_pose(x, y, yaw)
                    self.get_logger().info(f"[TX] seq={self.seq} /ref_pose x={x:+.4f} y={y:+.4f} yaw={yaw:+.4f}")
                    continue

                # 其他 cmd：你要的话也可以继续加
                continue

            except Exception as e:
                self.get_logger().warning(f"[ZMQ] error: {e}")


def main():
    rclpy.init()
    node = CSVRefPubSeq()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
