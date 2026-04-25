import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("figure8_trajectory_mode")
    config_file = os.path.join(
        package_share, "config", "figure8_velocity_mode.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file", default_value=config_file, description="Figure8 velocity-mode config file")

    node = Node(
        package="figure8_trajectory_mode",
        executable="figure8_velocity_mode_node",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    return LaunchDescription([
        config_file_arg,
        node,
    ])
