import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class FrontierNode(Node):
    def __init__(self):
        super().__init__("frontier_node")

        self.have_map = False
        self.goal_sent = False

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)

        self.timer = self.create_timer(1.0, self.publish_goal)
        self.get_logger().info("frontier_node started")

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.have_map = True
        self.latest_map = msg

    def publish_goal(self) -> None:
        if not self.have_map or self.goal_sent:
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = 1.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.goal_sent = True
        self.get_logger().info("Published exploration goal to /goal")


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
