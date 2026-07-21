import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_am_position_mode = get_package_share_directory('am_position_mode')
    config_file = os.path.join(
        pkg_am_position_mode, 'config', 'real_am_position.yaml')
    with open(config_file, 'r', encoding='utf-8') as file:
        config = yaml.safe_load(file)

    mode_config = config['mode']
    model_path = mode_config['model_path']
    if not os.path.isabs(model_path):
        model_path = os.path.join(pkg_am_position_mode, model_path)
    metadata_path = mode_config.get(
        'metadata_path', os.path.splitext(model_path)[0] + '.json')
    if not os.path.isabs(metadata_path):
        metadata_path = os.path.join(pkg_am_position_mode, metadata_path)

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=model_path, description='ONNX Model Path')
    metadata_path_arg = DeclareLaunchArgument(
        'metadata_path',
        default_value=metadata_path,
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
        'offboard_setpoint_timeout_s',
        default_value=str(mode_config.get('offboard_setpoint_timeout_s', 0.5)),
        description='Timeout for external Offboard references in seconds')
    collective_scale_arg = DeclareLaunchArgument(
        'collective_scale', default_value=str(mode_config.get('collective_scale', 1.0)),
        description='Scale applied to the normalized collective thrust command')
    start_micro_xrce_agent_arg = DeclareLaunchArgument(
        'start_micro_xrce_agent',
        default_value='false',
        description='Start MicroXRCEAgent udp4 -p 8888 from this launch file')

    position_mode_node = Node(
        package='am_position_mode',
        executable='am_position_mode',
        name='am_position',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'metadata_path': LaunchConfiguration('metadata_path'),
            'use_sim_time': False,
            'use_ros2_odom': ParameterValue(
                LaunchConfiguration('use_ros2_odom'), value_type=bool),
            'offboard_control_mode_topic': LaunchConfiguration('offboard_control_mode_topic'),
            'trajectory_setpoint_topic': LaunchConfiguration('trajectory_setpoint_topic'),
            'offboard_setpoint_timeout_s': ParameterValue(
                LaunchConfiguration('offboard_setpoint_timeout_s'),
                value_type=float),
            'collective_scale': ParameterValue(
                LaunchConfiguration('collective_scale'), value_type=float),
        }],
        output='screen',
    )

    micro_xrce_agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_micro_xrce_agent')),
    )

    return LaunchDescription([
        model_path_arg,
        metadata_path_arg,
        use_ros2_odom_arg,
        offboard_control_mode_topic_arg,
        trajectory_setpoint_topic_arg,
        offboard_setpoint_timeout_s_arg,
        collective_scale_arg,
        start_micro_xrce_agent_arg,
        micro_xrce_agent_process,
        position_mode_node,
    ])
