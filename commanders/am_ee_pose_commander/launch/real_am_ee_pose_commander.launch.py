from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("am_ee_pose_commander"))
    default_config = package_share / "config" / "real_am_ee_pose_commander.yaml"
    start_agent = LaunchConfiguration("start_micro_xrce_agent")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="AM EE Pose real-flight parameter file",
            ),
            DeclareLaunchArgument(
                "upper_model_path",
                description="Absolute path to the 63D AM EE Pose ONNX policy",
            ),
            DeclareLaunchArgument(
                "start_micro_xrce_agent",
                default_value="false",
                description="Start MicroXRCEAgent udp4 -p 8888",
            ),
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
                output="screen",
                condition=IfCondition(start_agent),
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
