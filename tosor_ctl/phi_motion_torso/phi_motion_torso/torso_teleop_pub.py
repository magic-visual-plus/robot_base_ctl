#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from pymbc_msgs.msg import WholeBodyData


def waist_height_m_to_target_mm(waist_height_m: float, offset_mm: float) -> float:
    return float(waist_height_m) * 1000.0 + float(offset_mm)


class TorsoTeleopPublisher(Node):
    """
    Bridge teleop whole-body topic -> torso target height cmd.

    - Subscribe: /phi/motion/teleop/whole_body (pymbc_msgs/WholeBodyData)
      * waist_height is in meters
    - Publish:   /phi/motion/torso/target_height_cmd (std_msgs/Float64)
      * height in millimeters (mm)
    """

    def __init__(self):
        super().__init__("torso_teleop_publisher")

        self.declare_parameter("waist_height_offset_mm", 200.0)

        self._pub = self.create_publisher(Float64, "/phi/motion/torso/target_height_cmd", 10)
        self._sub = self.create_subscription(
            WholeBodyData,
            "/phi/motion/teleop/whole_body",
            self._on_whole_body,
            10,
        )

        self.get_logger().info(
            "Torso teleop bridge started. "
            "Sub: /phi/motion/teleop/whole_body (waist_height[m]) -> "
            "Pub: /phi/motion/torso/target_height_cmd (mm)"
        )

    def _on_whole_body(self, msg: WholeBodyData) -> None:
        # waist_height is meters; torso teleop expects millimeters.
        offset_mm = float(self.get_parameter("waist_height_offset_mm").value)
        height_mm = waist_height_m_to_target_mm(msg.waist_height, offset_mm)

        out = Float64()
        out.data = height_mm
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TorsoTeleopPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()