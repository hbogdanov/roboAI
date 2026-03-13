from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from roboai.core.occupancy_grid import OccupancyGrid as CoreOccupancyGrid
from roboai.core.types import LaserScan2D, Pose2D


class MappingNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_node")

        self.declare_parameter("map_width", 400)
        self.declare_parameter("map_height", 400)
        self.declare_parameter("map_resolution", 0.05)

        width = int(self.get_parameter("map_width").value)
        height = int(self.get_parameter("map_height").value)
        resolution = float(self.get_parameter("map_resolution").value)
        origin = (-(width * resolution) / 2.0, -(height * resolution) / 2.0)

        self.grid = CoreOccupancyGrid(width=width, height=height, resolution=resolution, origin=origin)
        self.latest_pose: Pose2D | None = None

        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)

        self.get_logger().info("mapping_node started, publishing /map from /scan + /odom")

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            theta=_yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def scan_callback(self, msg: LaserScan) -> None:
        if self.latest_pose is None:
            return

        ranges = np.asarray(msg.ranges, dtype=float)
        ranges[~np.isfinite(ranges)] = float(msg.range_max)
        angles = np.asarray(
            [msg.angle_min + idx * msg.angle_increment for idx in range(len(msg.ranges))],
            dtype=float,
        )
        scan = LaserScan2D(angles=angles, ranges=ranges, max_range=float(msg.range_max))
        self.grid.update_from_scan(self.latest_pose, scan)
        self.map_pub.publish(_to_ros_map(self.grid, self.get_clock().now().to_msg()))


def _to_ros_map(grid: CoreOccupancyGrid, stamp) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = "map"
    msg.info.resolution = float(grid.resolution)
    msg.info.width = int(grid.width)
    msg.info.height = int(grid.height)
    msg.info.origin.position.x = float(grid.origin[0])
    msg.info.origin.position.y = float(grid.origin[1])
    msg.info.origin.orientation.w = 1.0
    data = []
    for value in grid.grid.flatten():
        if value < 0:
            data.append(-1)
        elif value == 0:
            data.append(0)
        else:
            data.append(100)
    msg.data = data
    return msg


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
