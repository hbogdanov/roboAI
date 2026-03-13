import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PlanningNode(Node):
    def __init__(self):
        super().__init__("planning_node")

        self.latest_odom = None
        self.latest_goal = None

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(PoseStamped, "/goal", self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)

        self.timer = self.create_timer(0.5, self.publish_path)
        self.get_logger().info("planning_node started")

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def goal_callback(self, msg: PoseStamped) -> None:
        self.latest_goal = msg
        self.get_logger().info("Received goal on /goal")

    def publish_path(self) -> None:
        if self.latest_odom is None or self.latest_goal is None:
            return

        sx = self.latest_odom.pose.pose.position.x
        sy = self.latest_odom.pose.pose.position.y
        gx = self.latest_goal.pose.position.x
        gy = self.latest_goal.pose.position.y

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        steps = 20
        for i in range(steps + 1):
            t = i / steps
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = sx + t * (gx - sx)
            pose.pose.position.y = sy + t * (gy - sy)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
