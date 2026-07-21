from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("velocity_tracking_benchmark"))
    config = share / "config" / "real_velocity_tracking_benchmark.yaml"
    workload = Node(
        package="velocity_tracking_benchmark",
        executable="velocity_tracking_benchmark",
        name="velocity_tracking_benchmark",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )
    reporter = Node(
        package="benchmark_reporting",
        executable="benchmark_reporter",
        name="velocity_tracking_reporter",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "report_type": "velocity",
                "benchmark_name": "velocity_tracking_benchmark",
                "benchmark_package": "velocity_tracking_benchmark",
                "output_dir": LaunchConfiguration("output_dir"),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=str(config)),
            DeclareLaunchArgument("output_dir", default_value=""),
            workload,
            reporter,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=reporter,
                    on_exit=[EmitEvent(event=Shutdown(reason="benchmark report complete"))],
                )
            ),
        ]
    )
