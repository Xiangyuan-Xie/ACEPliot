import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("rc_manual_trajectory_mode")
    config_file = os.path.join(
        package_share, "config", "rc_manual_trajectory_mode.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file", default_value=config_file, description="RC manual config file")

    node = Node(
        package="rc_manual_trajectory_mode",
        executable="rc_manual_trajectory_mode_node",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    return LaunchDescription([
        config_file_arg,
        node,
    ])
