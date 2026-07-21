from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    trajectory_share = Path(get_package_share_directory("figure8_trajectory"))
    commander_share = Path(get_package_share_directory("am_ee_pose_commander"))
    trajectory_config = trajectory_share / "config" / "ee_figure8_trajectory.yaml"
    commander_config = commander_share / "config" / "sim_am_ee_pose_commander.yaml"
    commander = Node(
        package="am_ee_pose_commander",
        executable="am_ee_pose_commander_node",
        name="am_ee_pose_commander",
        output="screen",
        parameters=[
            LaunchConfiguration("commander_config_file"),
            {"upper_model_path": LaunchConfiguration("upper_model_path")},
        ],
    )
    trajectory = Node(
        package="figure8_trajectory",
        executable="ee_figure8_trajectory",
        name="ee_figure8_trajectory",
        output="screen",
        parameters=[
            LaunchConfiguration("trajectory_config_file"),
            {
                "use_sim_clock": True,
                "sim_clock_topic": "/acesim/clock",
                "loops_to_run": ParameterValue(
                    LaunchConfiguration("loops_to_run"), value_type=int
                ),
            },
        ],
    )
    reporter = Node(
        package="benchmark_reporting",
        executable="benchmark_reporter",
        name="ee_figure8_reporter",
        output="screen",
        parameters=[
            LaunchConfiguration("trajectory_config_file"),
            {
                "use_sim_clock": True,
                "sim_clock_topic": "/acesim/clock",
                "report_type": "ee_figure8",
                "benchmark_name": "ee_figure8_tracking_benchmark",
                "benchmark_package": "figure8_tracking_benchmark",
                "output_dir": LaunchConfiguration("output_dir"),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("trajectory_config_file", default_value=str(trajectory_config)),
            DeclareLaunchArgument("commander_config_file", default_value=str(commander_config)),
            DeclareLaunchArgument("upper_model_path"),
            DeclareLaunchArgument("loops_to_run", default_value="1"),
            DeclareLaunchArgument("output_dir", default_value=""),
            commander,
            trajectory,
            reporter,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=reporter,
                    on_exit=[EmitEvent(event=Shutdown(reason="benchmark report complete"))],
                )
            ),
        ]
    )
