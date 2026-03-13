from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="roboai_mapping",
            executable="mapping_node",
            name="mapping_node",
            output="screen",
        ),
        Node(
            package="roboai_frontier",
            executable="frontier_node",
            name="frontier_node",
            output="screen",
        ),
        Node(
            package="roboai_planning",
            executable="planning_node",
            name="planning_node",
            output="screen",
        ),
        Node(
            package="roboai_control",
            executable="control_node",
            name="control_node",
            output="screen",
        ),
    ])
