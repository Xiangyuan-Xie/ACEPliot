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
        velocity_share, "launch", "sim_px4_velocity_commander.launch.py")
    arm_launch = os.path.join(
        arm_share, "launch", "sim_arm_trajectory_commander.launch.py")
    velocity_config = os.path.join(
        velocity_share, "config", "sim_px4_velocity_commander.yaml")
    arm_config = os.path.join(
        arm_share, "config", "sim_arm_trajectory_commander.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "velocity_config_file",
            default_value=velocity_config,
            description="Simulation YAML config file for PX4 velocity commander"),
        DeclareLaunchArgument(
            "arm_config_file",
            default_value=arm_config,
            description="Simulation YAML config file for arm trajectory commander"),
        DeclareLaunchArgument(
            "measured_odometry_topic",
            default_value="/acesim/vehicle/odometry",
            description="ACESim vehicle odometry topic used by PX4 velocity commander"),
        DeclareLaunchArgument(
            "sim_clock_topic",
            default_value="/acesim/clock",
            description="ACESim simulation clock topic shared by both commanders"),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(velocity_launch),
                    launch_arguments={
                        "config_file": LaunchConfiguration("velocity_config_file"),
                        "measured_odometry_topic": LaunchConfiguration("measured_odometry_topic"),
                        "sim_clock_topic": LaunchConfiguration("sim_clock_topic"),
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
                        "sim_clock_topic": LaunchConfiguration("sim_clock_topic"),
                    }.items(),
                ),
            ],
        ),
    ])
