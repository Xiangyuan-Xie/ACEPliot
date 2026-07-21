from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("am_ee_pose_commander"))
    default_config = package_share / "config" / "sim_am_ee_pose_commander.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="AM EE Pose ACESim parameter file",
            ),
            DeclareLaunchArgument(
                "upper_model_path",
                description="Absolute path to the 63D AM EE Pose ONNX policy",
            ),
            Node(
                package="am_ee_pose_commander",
                executable="am_ee_pose_commander_node",
                name="am_ee_pose_commander",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"upper_model_path": LaunchConfiguration("upper_model_path")},
                ],
            ),
        ]
    )
