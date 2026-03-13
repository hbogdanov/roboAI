from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from roboai.core.control.waypoint_follower import WaypointFollower
from roboai.core.types import Pose2D


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        self.declare_parameter("linear_speed", 0.22)
        self.declare_parameter("angular_gain", 2.8)
        self.declare_parameter("waypoint_tolerance", 0.12)

        self.follower = WaypointFollower(
            linear_speed=float(self.get_parameter("linear_speed").value),
            angular_gain=float(self.get_parameter("angular_gain").value),
            waypoint_tolerance=float(self.get_parameter("waypoint_tolerance").value),
        )
        self.latest_pose: Pose2D | None = None

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Path, "/path", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("control_node started, following /path with waypoint_follower")

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            theta=_yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def path_callback(self, msg: Path) -> None:
        waypoints = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        self.follower.set_path(waypoints[1:] if len(waypoints) > 1 else waypoints)

    def control_loop(self) -> None:
        if self.latest_pose is None:
            return
        linear, angular = self.follower.command(self.latest_pose)
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
