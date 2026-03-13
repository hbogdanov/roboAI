from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("roboai_bringup"), "config", "default.yaml"]
            ),
        ),
        Node(
            package="roboai_mapping",
            executable="mapping_node",
            name="mapping_node",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="roboai_frontier",
            executable="frontier_node",
            name="frontier_node",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="roboai_planning",
            executable="planning_node",
            name="planning_node",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="roboai_control",
            executable="control_node",
            name="control_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
