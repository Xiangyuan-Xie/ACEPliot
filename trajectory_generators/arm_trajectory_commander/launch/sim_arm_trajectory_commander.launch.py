import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("arm_trajectory_commander")
    config_file = os.path.join(
        package_share, "config", "sim_arm_trajectory_commander.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=config_file,
            description="Simulation YAML config file for leader-style arm trajectory"),
        DeclareLaunchArgument(
            "sim_clock_topic",
            default_value="/acesim/clock",
            description="Simulation clock topic used when use_sim_time is true"),
        Node(
            package="arm_trajectory_commander",
            executable="arm_trajectory_commander_node",
            name="arm_trajectory_commander",
            parameters=[
                LaunchConfiguration("config_file"),
                {"sim_clock_topic": LaunchConfiguration("sim_clock_topic")},
            ],
            output="screen",
        ),
    ])
