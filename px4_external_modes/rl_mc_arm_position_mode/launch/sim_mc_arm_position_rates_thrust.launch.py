import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    pkg_rl_mc_arm_position_mode = get_package_share_directory('rl_mc_arm_position_mode')
    model_path = os.path.join(pkg_rl_mc_arm_position_mode, "weights", 'policy.onnx')

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=model_path, description='ONNX Model Path')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time')
    use_ros2_odom_arg = DeclareLaunchArgument(
        'use_ros2_odom', default_value='false', description='Use external ROS2 odometry')

    position_mode_node = Node(
        package='rl_mc_arm_position_mode',
        executable='rl_mc_arm_position_rates_thrust_mode',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_ros2_odom': LaunchConfiguration('use_ros2_odom'),
        }]
    )

    micro_xrce_agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
    )

    return LaunchDescription([
        model_path_arg,
        use_sim_time_arg,
        use_ros2_odom_arg,
        micro_xrce_agent_process,
        position_mode_node,
    ])
