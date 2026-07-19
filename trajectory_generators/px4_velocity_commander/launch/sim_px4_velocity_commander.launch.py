import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("px4_velocity_commander")
    config_file = os.path.join(
        package_share, "config", "sim_px4_velocity_commander.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=config_file,
            description="Simulation YAML config file for PX4 velocity commander"),
        DeclareLaunchArgument(
            "measured_odometry_topic",
            default_value="/acesim/vehicle/odometry",
            description="ACESim odometry topic used for measured velocity"),
        DeclareLaunchArgument(
            "sim_clock_topic",
            default_value="/acesim/clock",
            description="Simulation clock topic used when use_sim_time is true"),
        Node(
            package="px4_velocity_commander",
            executable="px4_velocity_commander_node",
            name="px4_velocity_commander",
            parameters=[
                LaunchConfiguration("config_file"),
                {"measured_odometry_topic": LaunchConfiguration("measured_odometry_topic")},
                {"sim_clock_topic": LaunchConfiguration("sim_clock_topic")},
            ],
            output="screen",
        ),
    ])
