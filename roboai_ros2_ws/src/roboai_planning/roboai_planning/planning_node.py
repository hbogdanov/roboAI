from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node

from roboai.core.occupancy_grid import OccupancyGrid as CoreOccupancyGrid
from roboai.core.planners.astar import plan_astar
from roboai.core.planners.smoothing import shortcut_smooth_path


class PlanningNode(Node):
    def __init__(self) -> None:
        super().__init__("planning_node")

        self.declare_parameter("robot_radius", 0.16)

        self.latest_map: OccupancyGrid | None = None
        self.latest_goal: PoseStamped | None = None
        self.latest_pose: tuple[float, float] | None = None

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(PoseStamped, "/goal", self.goal_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.timer = self.create_timer(0.75, self.publish_path)

        self.get_logger().info("planning_node started, planning A* paths from /map + /goal")

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def goal_callback(self, msg: PoseStamped) -> None:
        self.latest_goal = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def publish_path(self) -> None:
        if self.latest_map is None or self.latest_goal is None or self.latest_pose is None:
            return

        grid = _from_ros_map(self.latest_map)
        result = plan_astar(
            grid=grid,
            start_xy=self.latest_pose,
            goal_xy=(float(self.latest_goal.pose.position.x), float(self.latest_goal.pose.position.y)),
            robot_radius=float(self.get_parameter("robot_radius").value),
            allow_unknown=False,
        )
        if not result.success or not result.path:
            return

        path = shortcut_smooth_path(
            grid=grid,
            path=result.path,
            robot_radius=float(self.get_parameter("robot_radius").value),
            allow_unknown=False,
        )
        self.path_pub.publish(_to_ros_path(path, self.get_clock().now().to_msg()))


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


def _to_ros_path(waypoints: list[tuple[float, float]], stamp) -> Path:
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = "map"
    for x, y in waypoints:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
