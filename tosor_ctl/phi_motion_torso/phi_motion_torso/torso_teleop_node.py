#!/usr/bin/env python3
"""
Phi Robot Torso Teleop Control Node

订阅上层发送的目标高度指令，并通过 MoveToHeight action 调用底层控制节点。

接口约定：
- 输入 topic: /phi/motion/teleop/whole_body (pymbc_msgs/WholeBodyData)
  - waist_height: float64, unit=m
- 状态输入: /phi/motion/torso/status (phi_motion_torso/TorsoStatus)
- Action: /phi/motion/torso/move_to_height (phi_motion_torso/MoveToHeight)
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from pymbc_msgs.msg import WholeBodyData

from phi_motion_torso.action import MoveToHeight
from phi_motion_torso.msg import TorsoStatus


def waist_height_m_to_target_mm(waist_height_m: float, offset_mm: float) -> float:
    return float(waist_height_m) * 1000.0 + float(offset_mm)


class PhiTorsoTeleopNode(Node):
    """High-level teleop node: follow target_height_cmd with PP motion."""

    def __init__(self) -> None:
        super().__init__("phi_torso_teleop_node")

        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter("height_tolerance_mm", 1.0)
        self.declare_parameter("command_check_rate", 10.0)  # Hz, how often to evaluate/sync action
        self.declare_parameter("waist_height_offset_mm", 200.0)

        # Internal state
        self._desired_height: Optional[float] = None
        self._current_height: Optional[float] = None
        self._has_active_goal: bool = False
        self._current_goal_handle = None

        # Subscribers
        self.create_subscription(
            WholeBodyData,
            "/phi/motion/teleop/whole_body",
            self._on_whole_body,
            10,
        )

        self.create_subscription(
            TorsoStatus,
            "/phi/motion/torso/status",
            self._on_status,
            10,
        )

        # Action client
        self._action_client = ActionClient(
            self,
            MoveToHeight,
            "/phi/motion/torso/move_to_height",
            callback_group=self.callback_group,
        )

        # Timer to evaluate whether to send new goals
        rate = self.get_parameter("command_check_rate").value
        period = 1.0 / float(rate)
        self._timer = self.create_timer(period, self._control_loop, callback_group=self.callback_group)

        self.get_logger().info(
            f"Phi Torso Teleop Node started. "
            f"Listening to /phi/motion/teleop/whole_body (waist_height[m]) at {rate} Hz control loop."
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_whole_body(self, msg: WholeBodyData) -> None:
        # waist_height is meters; this node operates in millimeters.
        offset_mm = float(self.get_parameter("waist_height_offset_mm").value)
        new_height = waist_height_m_to_target_mm(msg.waist_height, offset_mm)
        old_height = self._desired_height
        self._desired_height = new_height

        # 如果命令在有目标执行时发生明显变化，则取消当前目标，准备跟随新高度
        if old_height is not None and self._has_active_goal and self._current_goal_handle is not None:
            tol = float(self.get_parameter("height_tolerance_mm").value)
            if abs(new_height - old_height) > tol:
                self.get_logger().info(
                    f"Target height changed during motion "
                    f"({old_height:.2f} -> {new_height:.2f} mm), cancel current goal and update."
                )
                try:
                    self._current_goal_handle.cancel_goal_async()
                except Exception:
                    # 取消失败时让控制循环在下一轮重新评估
                    pass

    def _on_status(self, msg: TorsoStatus) -> None:
        self._current_height = float(msg.current_height_mm)
        self._has_active_goal = bool(msg.has_active_goal)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        """Decide when to send a new MoveToHeight goal based on latest command."""
        if self._desired_height is None:
            return

        # Wait for action server if not ready
        if not self._action_client.server_is_ready():
            self._action_client.wait_for_server(timeout_sec=0.1)
            if not self._action_client.server_is_ready():
                return

        tol = float(self.get_parameter("height_tolerance_mm").value)

        # If we know current height and it's already close enough, no need to send a goal
        if self._current_height is not None:
            if abs(self._desired_height - self._current_height) <= tol and not self._has_active_goal:
                return

        # If a goal is active, we simply wait for it to finish; teleop频率很高，上层可以反复更新期望高度
        if self._has_active_goal:
            return

        # Send a new goal
        goal_msg = MoveToHeight.Goal()
        goal_msg.target_height_mm = self._desired_height

        self.get_logger().info(f"Sending teleop goal: {goal_msg.target_height_mm:.2f} mm")
        send_future = self._action_client.send_goal_async(goal_msg, feedback_callback=None)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Teleop goal rejected by torso action server")
            self._has_active_goal = False
            return

        self._has_active_goal = True
        self._current_goal_handle = goal_handle
        self.get_logger().info("Teleop goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        self._has_active_goal = False
        self._current_goal_handle = None
        if result.success:
            self.get_logger().info(
                f"Teleop goal finished: {result.final_height_mm:.2f} mm ({result.message})"
            )
        else:
            self.get_logger().warn(f"Teleop goal failed: {result.message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhiTorsoTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

