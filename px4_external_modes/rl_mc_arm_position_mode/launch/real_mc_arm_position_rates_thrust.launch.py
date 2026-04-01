import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    pkg_rl_mc_arm_position_mode = get_package_share_directory('rl_mc_arm_position_mode')
    pkg_px4_state_converter = get_package_share_directory('px4_state_converter')
    config_file = os.path.join(
        pkg_rl_mc_arm_position_mode, 'config', 'real_mc_arm_position_rates_thrust.yaml')
    with open(config_file, 'r', encoding='utf-8') as file:
        config = yaml.safe_load(file)

    mode_config = config['mode']
    airlink_config = config['airlink']
    model_path = mode_config['model_path']
    if not os.path.isabs(model_path):
        model_path = os.path.join(pkg_rl_mc_arm_position_mode, model_path)

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=config_file, description='YAML config file for real rates thrust launch')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=model_path, description='ONNX Model Path')
    use_ros2_odom_arg = DeclareLaunchArgument(
        'use_ros2_odom', default_value=str(mode_config['use_ros2_odom']).lower(),
        description='Use external ROS2 odometry')
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value=mode_config['cmd_vel_topic'], description='External cmd_vel topic')

    airlink_mode_arg = DeclareLaunchArgument(
        'airlink_mode', default_value=airlink_config['link_mode'],
        description='AirLink PX4 uplink type: serial or udp')
    airlink_serial_device_arg = DeclareLaunchArgument(
        'airlink_serial_device', default_value=airlink_config['serial_device'],
        description='PX4 MAVLink serial device for AirLink')
    airlink_wifi_broadcast_ip_arg = DeclareLaunchArgument(
        'airlink_wifi_broadcast_ip', default_value=airlink_config['wifi_broadcast_ip'],
        description='WiFi broadcast IP for AirLink')

    position_mode_node = Node(
        package='rl_mc_arm_position_mode',
        executable='rl_mc_arm_position_rates_thrust_mode',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'use_sim_time': False,
            'use_ros2_odom': LaunchConfiguration('use_ros2_odom'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'cmd_vel_timeout_s': 0.5,
        }],
        output='screen',
    )

    airlink_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_px4_state_converter, 'launch', 'airlink.launch.py')
        ),
        launch_arguments={
            'link_mode': LaunchConfiguration('airlink_mode'),
            'serial_device': LaunchConfiguration('airlink_serial_device'),
            'wifi_broadcast_ip': LaunchConfiguration('airlink_wifi_broadcast_ip'),
        }.items(),
    )

    micro_xrce_agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        model_path_arg,
        use_ros2_odom_arg,
        cmd_vel_topic_arg,
        airlink_mode_arg,
        airlink_serial_device_arg,
        airlink_wifi_broadcast_ip_arg,
        airlink_launch,
        micro_xrce_agent_process,
        position_mode_node,
    ])
