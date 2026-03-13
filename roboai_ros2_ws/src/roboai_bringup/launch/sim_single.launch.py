from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value="roboai_ros2_ws/src/roboai_bringup/config/default.yaml",
        ),
        Node(package="roboai_mapping", executable="mapping_node", name="mapping", parameters=[params]),
        Node(package="roboai_frontier", executable="frontier_node", name="frontier", parameters=[params]),
        Node(package="roboai_planning", executable="planning_node", name="planning", parameters=[params]),
        Node(package="roboai_control", executable="control_node", name="control", parameters=[params]),
        Node(package="roboai_manager", executable="manager_node", name="manager", parameters=[params]),
        Node(package="roboai_viz", executable="viz_node", name="viz", parameters=[params]),
    ])
