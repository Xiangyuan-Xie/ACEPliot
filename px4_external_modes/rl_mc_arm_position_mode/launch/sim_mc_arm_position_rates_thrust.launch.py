import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_rl_mc_arm_position_mode = get_package_share_directory('rl_mc_arm_position_mode')
    model_path = os.path.join(pkg_rl_mc_arm_position_mode, "weights", 'policy.onnx')

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=model_path, description='ONNX Model Path')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time')
    sim_clock_topic_arg = DeclareLaunchArgument(
        'sim_clock_topic', default_value='/acesim/clock', description='Simulation clock topic')
    use_ros2_odom_arg = DeclareLaunchArgument(
        'use_ros2_odom', default_value='false', description='Use external ROS2 odometry')
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/rl_arm_position/cmd_vel', description='External cmd_vel topic')
    cmd_vel_timeout_s_arg = DeclareLaunchArgument(
        'cmd_vel_timeout_s', default_value='0.5', description='External command timeout (s)')

    position_mode_node = Node(
        package='rl_mc_arm_position_mode',
        executable='rl_mc_arm_position_rates_thrust_mode',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sim_clock_topic': LaunchConfiguration('sim_clock_topic'),
            'use_ros2_odom': LaunchConfiguration('use_ros2_odom'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'cmd_vel_timeout_s': LaunchConfiguration('cmd_vel_timeout_s'),
        }],
    )

    return LaunchDescription([
        model_path_arg,
        use_sim_time_arg,
        sim_clock_topic_arg,
        use_ros2_odom_arg,
        cmd_vel_topic_arg,
        cmd_vel_timeout_s_arg,
        position_mode_node,
    ])
