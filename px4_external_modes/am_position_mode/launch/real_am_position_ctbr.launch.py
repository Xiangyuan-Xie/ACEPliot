import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    pkg_am_position_mode = get_package_share_directory('am_position_mode')
    config_file = os.path.join(
        pkg_am_position_mode, 'config', 'real_am_position_ctbr.yaml')
    with open(config_file, 'r', encoding='utf-8') as file:
        config = yaml.safe_load(file)

    mode_config = config['mode']
    model_path = mode_config['model_path']
    if not os.path.isabs(model_path):
        model_path = os.path.join(pkg_am_position_mode, model_path)

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=config_file, description='YAML config file for real CTBR launch')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=model_path, description='ONNX Model Path')
    metadata_path_arg = DeclareLaunchArgument(
        'metadata_path', default_value=mode_config.get('metadata_path', os.path.splitext(model_path)[0] + '.json'),
        description='Policy metadata JSON path')
    use_ros2_odom_arg = DeclareLaunchArgument(
        'use_ros2_odom', default_value=str(mode_config['use_ros2_odom']).lower(),
        description='Use external ROS2 odometry')
    offboard_control_mode_topic_arg = DeclareLaunchArgument(
        'offboard_control_mode_topic', default_value=mode_config['offboard_control_mode_topic'],
        description='PX4 OffboardControlMode topic')
    trajectory_setpoint_topic_arg = DeclareLaunchArgument(
        'trajectory_setpoint_topic', default_value=mode_config['trajectory_setpoint_topic'],
        description='PX4 TrajectorySetpoint topic')
    offboard_setpoint_timeout_s_arg = DeclareLaunchArgument(
        'offboard_setpoint_timeout_s', default_value=str(mode_config.get('offboard_setpoint_timeout_s', 0.5)),
        description='Timeout for external Offboard references in seconds')

    position_mode_node = Node(
        package='am_position_mode',
        executable='am_position_ctbr_mode',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'metadata_path': LaunchConfiguration('metadata_path'),
            'use_sim_time': False,
            'use_ros2_odom': LaunchConfiguration('use_ros2_odom'),
            'offboard_control_mode_topic': LaunchConfiguration('offboard_control_mode_topic'),
            'trajectory_setpoint_topic': LaunchConfiguration('trajectory_setpoint_topic'),
            'offboard_setpoint_timeout_s': LaunchConfiguration('offboard_setpoint_timeout_s'),
            'ctbr_collective_scale': mode_config.get('ctbr_collective_scale', 1.0),
        }],
        output='screen',
    )

    micro_xrce_agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        model_path_arg,
        metadata_path_arg,
        use_ros2_odom_arg,
        offboard_control_mode_topic_arg,
        trajectory_setpoint_topic_arg,
        offboard_setpoint_timeout_s_arg,
        micro_xrce_agent_process,
        position_mode_node,
    ])
