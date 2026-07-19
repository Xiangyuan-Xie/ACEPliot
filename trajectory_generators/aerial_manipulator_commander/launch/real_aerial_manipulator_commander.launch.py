import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    velocity_share = get_package_share_directory("px4_velocity_commander")
    arm_share = get_package_share_directory("arm_trajectory_commander")

    velocity_launch = os.path.join(
        velocity_share, "launch", "real_px4_velocity_commander.launch.py")
    arm_launch = os.path.join(
        arm_share, "launch", "real_arm_trajectory_commander.launch.py")
    velocity_config = os.path.join(
        velocity_share, "config", "real_px4_velocity_commander.yaml")
    arm_config = os.path.join(
        arm_share, "config", "real_arm_trajectory_commander.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "velocity_config_file",
            default_value=velocity_config,
            description="Real-flight YAML config file for PX4 velocity commander"),
        DeclareLaunchArgument(
            "arm_config_file",
            default_value=arm_config,
            description="Real deploy YAML config file for arm trajectory commander"),
        DeclareLaunchArgument(
            "mocap_pose_topic",
            default_value="xxy/pose",
            description="Nokov PoseStamped topic used for measured velocity"),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(velocity_launch),
                    launch_arguments={
                        "config_file": LaunchConfiguration("velocity_config_file"),
                        "mocap_pose_topic": LaunchConfiguration("mocap_pose_topic"),
                    }.items(),
                ),
            ],
        ),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(arm_launch),
                    launch_arguments={
                        "config_file": LaunchConfiguration("arm_config_file"),
                    }.items(),
                ),
            ],
        ),
    ])
