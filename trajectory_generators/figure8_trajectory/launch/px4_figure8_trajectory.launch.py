from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("figure8_trajectory"))
    default_config = package_share / "config" / "px4_figure8_trajectory.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=str(default_config)),
            DeclareLaunchArgument("use_sim_clock", default_value="false"),
            Node(
                package="figure8_trajectory",
                executable="px4_figure8_trajectory",
                name="px4_figure8_trajectory",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "use_sim_clock": ParameterValue(
                            LaunchConfiguration("use_sim_clock"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
