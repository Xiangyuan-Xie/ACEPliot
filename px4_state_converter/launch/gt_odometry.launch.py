#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('px4_state_converter')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='lidar_config.yaml',
        description='Determine which external odometry source to publish to PX4'
    )
    start_micro_xrce_agent_arg = DeclareLaunchArgument(
        'start_micro_xrce_agent',
        default_value='false',
        description='Start MicroXRCEAgent udp4 -p 8888 from this launch file'
    )

    gt_odometry_converter_node = Node(
        package='px4_state_converter',
        executable='gt_odom_from_px4_node',
        parameters=[
            # Load the configuration file
            PathJoinSubstitution([
                pkg_share, 'config', LaunchConfiguration('config_file')
            ])
        ],
        output='screen',
    )

    # PX4 and ROS2 communication middleware
    micro_xrce_agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        condition=IfCondition(LaunchConfiguration('start_micro_xrce_agent')),
        # output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        start_micro_xrce_agent_arg,
        gt_odometry_converter_node,
        micro_xrce_agent_process
    ])
