from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node

from roboai.core.frontier import rank_frontier_targets
from roboai.core.occupancy_grid import OccupancyGrid as CoreOccupancyGrid


class FrontierNode(Node):
    def __init__(self) -> None:
        super().__init__("frontier_node")

        self.declare_parameter("frontier_policy", "information_gain")

        self.latest_map: OccupancyGrid | None = None
        self.latest_pose: tuple[float, float, float] | None = None

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

        self.get_logger().info("frontier_node started, selecting frontiers from /map")

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            _yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def publish_goal(self) -> None:
        if self.latest_map is None or self.latest_pose is None:
            return

        grid = _from_ros_map(self.latest_map)
        targets = rank_frontier_targets(
            grid=grid,
            robot_xy=(self.latest_pose[0], self.latest_pose[1]),
            robot_theta=self.latest_pose[2],
            policy=str(self.get_parameter("frontier_policy").value),
        )
        if not targets:
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(targets[0][0])
        goal.pose.position.y = float(targets[0][1])
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)


def _from_ros_map(msg: OccupancyGrid) -> CoreOccupancyGrid:
    grid = CoreOccupancyGrid(
        width=int(msg.info.width),
        height=int(msg.info.height),
        resolution=float(msg.info.resolution),
        origin=(float(msg.info.origin.position.x), float(msg.info.origin.position.y)),
    )
    data = np.asarray(msg.data, dtype=int).reshape((grid.height, grid.width))
    grid.grid = np.where(data < 0, -1, np.where(data > 50, 1, 0)).astype(np.int8)
    return grid


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
