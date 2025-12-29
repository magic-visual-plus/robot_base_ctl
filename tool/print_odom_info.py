import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# read odometry topic and print x,y,yaw (rad) info

odom_topic_name = "/rko_lio/odometry"

def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def yaw_from_quat(qx, qy, qz, qw):
    siny = 2.0 * (qw*qz + qx*qy)
    cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny, cosy)

class OdomYawDemo(Node):
    def __init__(self):
        super().__init__('odom_yaw_demo')
        self.sub = self.create_subscription(Odometry, odom_topic_name, self.cb, 10)
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0

    def cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        self.get_logger().info(
            f"x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f} rad ({self.yaw*180/math.pi:.1f} deg)"
        )

def main():
    rclpy.init()
    node = OdomYawDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
