#!/usr/bin/env python3
"""
Phi torso teleop follow node (topic-only, no action).

Functions:
1) Subscribe /phi/motion/teleop/whole_body, convert waist_height (m) to mm
   with offset, and publish latest target to /phi/motion/torso/target_height_cmd.
2) Publish /phi/motion/torso as PoseStamped. The teleop-equivalent
   waist_height is written to pose.position.z.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

from pymbc_msgs.msg import WholeBodyData
from phi_motion_torso.msg import TorsoStatus


def waist_height_m_to_target_mm(waist_height_m: float, offset_mm: float) -> float:
    return float(waist_height_m) * 1000.0 + float(offset_mm)


def current_height_mm_to_waist_height_m(current_height_mm: float, offset_mm: float) -> float:
    return (float(current_height_mm) - float(offset_mm)) / 1000.0


class PhiTorsoTeleopFollowTopicNode(Node):
    def __init__(self) -> None:
        super().__init__("phi_torso_teleop_follow_topic_node")

        self.declare_parameter("waist_height_offset_mm", 150.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("zero_encoder_height_mm", 630.0)
        self.declare_parameter("pulse_per_mm", 1000000.0 / 34.25)

        self._desired_height_mm = None
        self._current_height_mm = 0.0
        self._current_encoder_pulse = 0

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

        self._target_pub = self.create_publisher(Float64, "/phi/motion/torso/target_height_cmd", 10)
        self._extend_pub = self.create_publisher(PoseStamped, "/phi/motion/torso", 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            "Started follow-topic teleop node: whole_body -> /phi/motion/torso/target_height_cmd "
            "and publish /phi/motion/torso."
        )

    def _height_to_encoder_pulse(self, height_mm: float) -> int:
        zero_h = float(self.get_parameter("zero_encoder_height_mm").value)
        pulse_per_mm = float(self.get_parameter("pulse_per_mm").value)
        return int(round((float(height_mm) - zero_h) * pulse_per_mm))

    def _on_whole_body(self, msg: WholeBodyData) -> None:
        offset_mm = float(self.get_parameter("waist_height_offset_mm").value)
        self._desired_height_mm = waist_height_m_to_target_mm(msg.waist_height, offset_mm)

    def _on_status(self, msg: TorsoStatus) -> None:
        self._current_height_mm = float(msg.current_height_mm)
        self._current_encoder_pulse = self._height_to_encoder_pulse(self._current_height_mm)

    def _on_timer(self) -> None:
        # Follow latest teleop data: when a new waist target has arrived,
        # keep publishing the latest target command to the low-level topic.
        if self._desired_height_mm is not None:
            cmd = Float64()
            cmd.data = float(self._desired_height_mm)
            self._target_pub.publish(cmd)

        offset_mm = float(self.get_parameter("waist_height_offset_mm").value)
        teleop_waist_height_m = current_height_mm_to_waist_height_m(self._current_height_mm, offset_mm)

        ext = PoseStamped()
        ext.header.stamp = self.get_clock().now().to_msg()
        ext.pose.position.z = float(teleop_waist_height_m)
        self._extend_pub.publish(ext)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhiTorsoTeleopFollowTopicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
