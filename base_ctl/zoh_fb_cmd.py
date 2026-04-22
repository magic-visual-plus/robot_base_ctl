#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
zoh_fb_cmd.py

提供原生 Action `nav2_msgs/action/NavigateToPose`：goal 为 `geometry_msgs/PoseStamped`，
持续发布 `/ref_pose` 直至里程计反馈到达容差内（与 Nav2 接口一致，便于用标准客户端调用）。
"""

import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty

REF_POSE_TOPIC = "/ref_pose"
ODOM_TOPIC = "/rko_lio/odometry"


def yaw_to_quat(yaw):
    """平面 yaw -> quaternion (只绕 Z)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class CSVRefPubSeq(Node):
    def __init__(self):
        super().__init__("csv_ref_pub_seq")

        # 默认与项目约定路径一致；若需 Nav2 默认名可改为 navigate_to_pose
        self.declare_parameter(
            "action_server_name", "/phi/motion/control/navigate_to_pose"
        )
        self.declare_parameter("action_timeout_sec", 120.0)
        self.declare_parameter("action_stable_time_sec", 0.3)
        self.declare_parameter("action_pos_tolerance_m", 0.08)
        self.declare_parameter("action_yaw_tolerance_rad", math.radians(2.0))

        self.odom: Odometry | None = None
        self.create_subscription(Odometry, ODOM_TOPIC, self.cb_odom, 10)

        self.pub_pose = self.create_publisher(PoseStamped, REF_POSE_TOPIC, 10)

        self._action_running = False
        self._cb_group = ReentrantCallbackGroup()
        action_name = str(self.get_parameter("action_server_name").value)
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            action_name,
            execute_callback=self.execute_navigate_to_pose,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f"[ACTION] NavigateToPose server: {action_name}")

        self.get_logger().info(f"[REF] topic={REF_POSE_TOPIC} (NavigateToPose only)")

    def cb_odom(self, msg: Odometry):
        self.odom = msg

    def get_pose_from_odom(self):
        """与 zoh_rev.py Assist.get_pose、base_ctl.get_pose 一致：平面 x,y,yaw = odom.pose.pose"""
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        return float(p.x), float(p.y), float(quat_to_yaw(q))

    def current_pose_stamped(self) -> PoseStamped:
        """Feedback：与 `ros2 topic echo /rko_lio/odometry` 中 pose.pose 一致（同坐标、同四元数）。"""
        ps = PoseStamped()
        ps.header = self.odom.header
        ps.pose = self.odom.pose.pose
        return ps

    def publish_ref_pose(self, x: float, y: float, yaw: float):
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
        self.pub_pose.publish(pose)

    def goal_callback(self, _goal_request):
        if self._action_running:
            self.get_logger().warn("NavigateToPose rejected: another goal is running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_navigate_to_pose(self, goal_handle):
        req = goal_handle.request
        ps = req.pose
        x = float(ps.pose.position.x)
        y = float(ps.pose.position.y)
        yaw = float(quat_to_yaw(ps.pose.orientation))

        timeout_sec = float(self.get_parameter("action_timeout_sec").value)
        stable_need = float(self.get_parameter("action_stable_time_sec").value)
        pos_tol = float(self.get_parameter("action_pos_tolerance_m").value)
        yaw_tol = float(self.get_parameter("action_yaw_tolerance_rad").value)

        self._action_running = True
        self.get_logger().info(
            f"[ACTION] NavigateToPose goal: x={x:.4f} y={y:.4f} yaw={yaw:.4f} "
            f"frame={ps.header.frame_id!r}"
        )

        result = NavigateToPose.Result(result=Empty())
        t0 = time.monotonic()
        stable_t0 = None

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result

                if time.monotonic() - t0 > timeout_sec:
                    goal_handle.abort()
                    return result

                self.publish_ref_pose(x, y, yaw)

                if self.odom is None:
                    time.sleep(0.1)
                    continue

                cx, cy, cyaw = self.get_pose_from_odom()
                pos_err = math.hypot(x - cx, y - cy)
                yaw_err = wrap_to_pi(yaw - cyaw)

                elapsed = time.monotonic() - t0
                sec_i = int(elapsed)
                nsec = int((elapsed - sec_i) * 1e9)

                fb = NavigateToPose.Feedback()
                fb.current_pose = self.current_pose_stamped()
                fb.navigation_time = Duration(sec=sec_i, nanosec=nsec)
                fb.estimated_time_remaining = Duration(sec=0, nanosec=0)
                fb.number_of_recoveries = 0
                fb.distance_remaining = float(pos_err)
                goal_handle.publish_feedback(fb)

                if pos_err < pos_tol and abs(yaw_err) < yaw_tol:
                    if stable_t0 is None:
                        stable_t0 = time.monotonic()
                    elif time.monotonic() - stable_t0 >= stable_need:
                        goal_handle.succeed()
                        self.get_logger().info("[ACTION] NavigateToPose succeeded")
                        return result
                else:
                    stable_t0 = None

                time.sleep(0.1)
        finally:
            self._action_running = False

        goal_handle.abort()
        return result


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
