from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    velocity_share = Path(get_package_share_directory("velocity_tracking_benchmark"))
    arm_share = Path(get_package_share_directory("arm_motion_benchmark"))
    velocity_config = velocity_share / "config" / "real_velocity_tracking_benchmark.yaml"
    arm_config = arm_share / "config" / "real_arm_motion_benchmark.yaml"
    velocity = Node(
        package="velocity_tracking_benchmark",
        executable="velocity_tracking_benchmark",
        name="velocity_tracking_benchmark",
        output="screen",
        parameters=[LaunchConfiguration("velocity_config_file")],
    )
    arm = Node(
        package="arm_motion_benchmark",
        executable="arm_motion_benchmark",
        name="arm_motion_benchmark",
        output="screen",
        parameters=[LaunchConfiguration("arm_config_file")],
    )
    reporter = Node(
        package="benchmark_reporting",
        executable="benchmark_reporter",
        name="aerial_manipulation_reporter",
        output="screen",
        parameters=[
            LaunchConfiguration("velocity_config_file"),
            LaunchConfiguration("arm_config_file"),
            {
                "report_type": "aerial",
                "benchmark_name": "aerial_manipulation_benchmark",
                "benchmark_package": "aerial_manipulation_benchmark",
                "output_dir": LaunchConfiguration("output_dir"),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "velocity_config_file", default_value=str(velocity_config)
            ),
            DeclareLaunchArgument("arm_config_file", default_value=str(arm_config)),
            DeclareLaunchArgument("output_dir", default_value=""),
            velocity,
            arm,
            reporter,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=reporter,
                    on_exit=[EmitEvent(event=Shutdown(reason="benchmark report complete"))],
                )
            ),
        ]
    )
