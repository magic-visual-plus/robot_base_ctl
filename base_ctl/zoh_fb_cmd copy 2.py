#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
zoh_fb_cmd.py

订阅 `/phi/motion/teleop/whole_body` 的 `WholeBodyData`，根据 `command_type`
维护 active 状态，并将 `base_x/base_y/base_pitch` 转为 `/ref_pose` 的 `PoseStamped` 发布。

另提供原生 Action `nav2_msgs/action/NavigateToPose`：goal 为 `geometry_msgs/PoseStamped`，
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
from std_srvs.srv import Trigger
from pymbc_msgs.msg import WholeBodyData  # type: ignore

REF_POSE_TOPIC = "/ref_pose"
WHOLE_BODY_TOPIC = "/phi/motion/teleop/whole_body"
ODOM_TOPIC = "/rko_lio/odometry"

PUBLISH_HZ = 10.0
DT_PUB = 1.0 / PUBLISH_HZ


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

        self.active = False

        self.declare_parameter("teleop_timeout_sec", 2.0)
        # 与 Nav2 默认 action 名一致，也可用 /my_ns/navigate_to_pose
        self.declare_parameter("action_server_name", "navigate_to_pose")
        self.declare_parameter("action_timeout_sec", 120.0)
        self.declare_parameter("action_stable_time_sec", 0.3)
        self.declare_parameter("action_pos_tolerance_m", 0.06)
        self.declare_parameter("action_yaw_tolerance_rad", math.radians(2.0))

        self.last_msg: WholeBodyData | None = None
        self.has_msg = False
        self.last_rx_monotonic: float | None = None
        self.create_subscription(WholeBodyData, WHOLE_BODY_TOPIC, self.cb_whole_body, 10)
        self.get_logger().info(f"[ROS2] subscribe {WHOLE_BODY_TOPIC} -> /ref_pose")

        self.odom: Odometry | None = None
        self.create_subscription(Odometry, ODOM_TOPIC, self.cb_odom, 10)

        self.pub_pose = self.create_publisher(PoseStamped, REF_POSE_TOPIC, 10)
        self.timer = self.create_timer(DT_PUB, self.on_timer)
        self.reset_cli = self.create_client(Trigger, "/rko_lio/reset_odometry")

        self.last_print_t = time.monotonic()

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

        self.get_logger().info(f"[REF] publish_hz={PUBLISH_HZ}, topic={REF_POSE_TOPIC}")
        self.get_logger().info("[REF] mode=ROS2 whole_body -> /ref_pose")

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

        self.last_msg = msg
        self.has_msg = True

    def on_timer(self):
        if self._action_running:
            return
        try:
            if self.active is False:
                return
            if (not self.has_msg) or (self.last_msg is None):
                return
            timeout_sec = float(self.get_parameter("teleop_timeout_sec").value)
            if timeout_sec > 0.0:
                if self.last_rx_monotonic is None:
                    return
                age = time.monotonic() - self.last_rx_monotonic
                if age > timeout_sec:
                    self.active = False
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

            self.publish_ref_pose(x, y, yaw)
            self.get_logger().info(f"[ROS2->REF] x={x:+.4f} y={y:+.4f} yaw={yaw:+.4f}")
        except Exception as e:
            self.get_logger().warning(f"[ROS2] error: {e}")


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
