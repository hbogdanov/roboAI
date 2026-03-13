from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan

from roboai.core.occupancy_grid import OccupancyGrid as CoreOccupancyGrid
from roboai.core.types import LaserScan2D, Pose2D


def yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def pose2d_from_odom(msg: Odometry) -> Pose2D:
    return Pose2D(
        x=float(msg.pose.pose.position.x),
        y=float(msg.pose.pose.position.y),
        theta=yaw_from_quaternion(msg.pose.pose.orientation),
    )


def scan2d_from_ros(msg: LaserScan) -> LaserScan2D:
    angles = np.asarray(
        [msg.angle_min + idx * msg.angle_increment for idx in range(len(msg.ranges))],
        dtype=float,
    )
    ranges = np.asarray(msg.ranges, dtype=float)
    ranges[~np.isfinite(ranges)] = float(msg.range_max)
    return LaserScan2D(angles=angles, ranges=ranges, max_range=float(msg.range_max))


def ros_map_from_core(grid: CoreOccupancyGrid, stamp, frame_id: str = "map") -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.info.resolution = float(grid.resolution)
    msg.info.width = int(grid.width)
    msg.info.height = int(grid.height)
    msg.info.origin.position.x = float(grid.origin[0])
    msg.info.origin.position.y = float(grid.origin[1])
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(value * 100) if value > 0 else int(value) for value in grid.grid.flatten()]
    return msg


def core_map_from_ros(msg: OccupancyGrid) -> CoreOccupancyGrid:
    grid = CoreOccupancyGrid(
        width=int(msg.info.width),
        height=int(msg.info.height),
        resolution=float(msg.info.resolution),
        origin=(float(msg.info.origin.position.x), float(msg.info.origin.position.y)),
    )
    data = np.asarray(msg.data, dtype=int).reshape((grid.height, grid.width))
    mapped = np.full_like(data, -1, dtype=np.int8)
    mapped[data == 0] = 0
    mapped[data > 0] = 1
    grid.grid = mapped
    return grid


def pose_stamped_from_xy(x: float, y: float, stamp, frame_id: str = "map") -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.w = 1.0
    return msg


def path_from_waypoints(waypoints: list[tuple[float, float]], stamp, frame_id: str = "map") -> Path:
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = frame_id
    for x, y in waypoints:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path
