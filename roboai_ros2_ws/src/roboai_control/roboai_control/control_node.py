import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        self.latest_odom = None
        self.latest_path = None
        self.target_index = 0

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Path, "/path", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("control_node started")

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def path_callback(self, msg: Path) -> None:
        self.latest_path = msg
        self.target_index = 0
        self.get_logger().info(f"Received path with {len(msg.poses)} poses")

    def control_loop(self) -> None:
        if self.latest_odom is None or self.latest_path is None or not self.latest_path.poses:
            return

        if self.target_index >= len(self.latest_path.poses):
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        target = self.latest_path.poses[self.target_index].pose.position
        current = self.latest_odom.pose.pose.position

        dx = target.x - current.x
        dy = target.y - current.y
        dist = math.hypot(dx, dy)

        if dist < 0.1:
            self.target_index += 1
            return

        twist = Twist()
        twist.linear.x = min(0.2, 0.5 * dist)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
