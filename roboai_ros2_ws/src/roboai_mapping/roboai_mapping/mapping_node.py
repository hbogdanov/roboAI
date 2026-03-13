import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")

        self.latest_odom = None
        self.latest_scan = None

        self.width = 200
        self.height = 200
        self.resolution = 0.05
        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0

        self.grid = [-1] * (self.width * self.height)

        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)

        self.timer = self.create_timer(0.5, self.publish_map)

        self.get_logger().info("mapping_node started")

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def world_to_grid(self, x: float, y: float):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return gx, gy
        return None

    def mark_cell(self, x: float, y: float, value: int) -> None:
        cell = self.world_to_grid(x, y)
        if cell is None:
            return
        gx, gy = cell
        idx = gy * self.width + gx
        self.grid[idx] = value

    def publish_map(self) -> None:
        if self.latest_odom is None or self.latest_scan is None:
            return

        rx = self.latest_odom.pose.pose.position.x
        ry = self.latest_odom.pose.pose.position.y

        robot_cell = self.world_to_grid(rx, ry)
        if robot_cell is not None:
            gx, gy = robot_cell
            self.grid[gy * self.width + gx] = 0

        angle = self.latest_scan.angle_min
        for r in self.latest_scan.ranges:
            if math.isfinite(r):
                ex = rx + r * math.cos(angle)
                ey = ry + r * math.sin(angle)
                self.mark_cell(ex, ey, 100)
            angle += self.latest_scan.angle_increment

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
