"""Run closed-loop control from an explicitly supplied calibrated YAML file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                description=(
                    "Absolute path to a calibrated fully actuated configuration "
                    "with closed_loop and calibration_confirmed enabled"
                ),
            ),
            Node(
                package="flying_hand_mode",
                executable="flying_hand_fully_actuated",
                name="flying_hand_fully_actuated",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
