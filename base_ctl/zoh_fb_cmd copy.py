#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
zoh_fb_cmd.py

订阅 `/phi/motion/teleop/whole_body` 的 `WholeBodyData`，根据 `command_type`
维护 active 状态，并将 `base_x/base_y/base_pitch` 转为 `/ref_pose` 的 `PoseStamped` 发布。
"""

import math

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from pymbc_msgs.msg import WholeBodyData  # type: ignore

X_COL   = "base_x"
Y_COL   = "base_y"
YAW_COL = "base_pitch"  

REF_POSE_TOPIC = "/ref_pose"
WHOLE_BODY_TOPIC = "/phi/motion/teleop/whole_body"

PUBLISH_HZ = 10.0        
DT_PUB = 1.0 / PUBLISH_HZ
# ====================================



def yaw_to_quat(yaw):
    """平面 yaw -> quaternion (只绕 Z)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CSVRefPubSeq(Node):
    def __init__(self):
        super().__init__("csv_ref_pub_seq")

        self.active = False   # 默认不发

        # If no new teleop msg arrives within this window, stop publishing cached data.
        self.declare_parameter("teleop_timeout_sec", 2.0)
    
        
        # ===== ROS2 Topic =====
        self.last_msg: WholeBodyData | None = None
        self.has_msg = False
        self.last_rx_monotonic: float | None = None
        self.create_subscription(WholeBodyData, WHOLE_BODY_TOPIC, self.cb_whole_body, 10)
        self.get_logger().info(f"[ROS2] subscribe {WHOLE_BODY_TOPIC} -> /ref_pose")

        self.idx = 0

        self.pub_pose = self.create_publisher(PoseStamped, REF_POSE_TOPIC, 10)
        self.timer = self.create_timer(DT_PUB, self.on_timer)
        self.reset_cli = self.create_client(Trigger, "/rko_lio/reset_odometry")

        
        self.last_print_t = time.monotonic()

        self.get_logger().info(f"[REF] publish_hz={PUBLISH_HZ}, topic={REF_POSE_TOPIC}")
        self.get_logger().info("[REF] mode=ROS2 whole_body -> /ref_pose")

    def cb_whole_body(self, msg: WholeBodyData):
        """
        WholeBodyData.command_type 映射见 `WholeBodyData.msg` 注释：
          0: start
          1: back_zero
          2: together
          3: body_ctl
          4: reset
          5: stop
        """
        cmd_type = int(msg.command_type)
        self.last_rx_monotonic = time.monotonic()

        if cmd_type in (0, 1, 2, 3):
            self.active = True
            self.last_msg = msg
            self.has_msg = True
            return

        if cmd_type == 4:  # reset
            self.active = False
            self.get_logger().info("[CMD] reset -> call reset_odometry, active=False")
            if self.reset_cli.service_is_ready():
                self.reset_cli.call_async(Trigger.Request())
            return

        if cmd_type == 5:  # stop
            self.active = False
            self.get_logger().info("[CMD] stop -> active=False")
            return

        # 未知 command_type：保留 active/只更新数据便于后续恢复
        self.last_msg = msg
        self.has_msg = True

    def on_timer(self):
        # ROS2 订阅版本：按 10Hz 将缓存的 base_x/base_y/base_pitch 发布到 /ref_pose
        try:
            if self.active is False:
                return
            if (not self.has_msg) or (self.last_msg is None):
                return
            # Stop publishing if upstream teleop topic is stale (no new msg).
            timeout_sec = float(self.get_parameter("teleop_timeout_sec").value)
            if timeout_sec > 0.0:
                if self.last_rx_monotonic is None:
                    return
                age = time.monotonic() - self.last_rx_monotonic
                if age > timeout_sec:
                    # Consider teleop "disconnected": stop output until a new message arrives.
                    self.active = False
                    # Optional: keep last_msg for debugging; do not publish it anymore.
                    # Throttle warning to avoid log spam.
                    now_t = time.monotonic()
                    if now_t - self.last_print_t > 2.0:
                        self.last_print_t = now_t
                        self.get_logger().warning(
                            f"[ROS2->REF] teleop stale for {age:.2f}s (> {timeout_sec:.2f}s), set active=False"
                        )
                    return

            x = float(self.last_msg.base_x)
            y = float(self.last_msg.base_y)
            yaw = float(self.last_msg.base_pitch)

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
            print(f"-------> pose: {pose}_{time.time()}")
            self.pub_pose.publish(pose)
            self.get_logger().info(f"[ROS2->REF] x={x:+.4f} y={y:+.4f} yaw={yaw:+.4f}")
        except Exception as e:
            self.get_logger().warning(f"[ROS2] error: {e}")
        return

       
       

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
