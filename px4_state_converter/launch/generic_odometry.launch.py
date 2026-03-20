#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Get the shared directory of the package
    pkg_share = get_package_share_directory('px4_state_converter')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='nokov_mocap_config.yaml',
        description='Determine which external odometry source to publish to PX4'
    )

    # Launch the generic odometry node
    generic_odometry_node = Node(
        package='px4_state_converter',
        executable='odom_to_px4_node',
        parameters=[
            # Load the configuration file
            PathJoinSubstitution([
                pkg_share, 'config', LaunchConfiguration('config_file')
            ])
        ],
        output='screen',
        emulate_tty=True,
    )

    # PX4 and ROS2 communication middleware
    micro_xrce_agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        # output='screen',
    )
    return LaunchDescription([
        config_file_arg,

        micro_xrce_agent_process,
        generic_odometry_node,
    ])
