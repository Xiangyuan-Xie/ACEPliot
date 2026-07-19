import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("px4_velocity_commander")
    config_file = os.path.join(
        package_share, "config", "real_px4_velocity_commander.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=config_file,
            description="Real-flight YAML config file for PX4 velocity commander"),
        DeclareLaunchArgument(
            "mocap_pose_topic",
            default_value="xxy/pose",
            description="Nokov PoseStamped topic used for measured velocity"),
        DeclareLaunchArgument(
            "measurement_source",
            default_value="pose_stamped",
            description="Measured velocity source: pose_stamped or odometry_pose"),
        Node(
            package="px4_velocity_commander",
            executable="px4_velocity_commander_node",
            name="px4_velocity_commander",
            parameters=[
                LaunchConfiguration("config_file"),
                {"mocap_pose_topic": LaunchConfiguration("mocap_pose_topic")},
                {"measurement_source": LaunchConfiguration("measurement_source")},
            ],
            output="screen",
        ),
    ])
