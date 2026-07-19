from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    config = (
        Path(get_package_share_directory("flying_hand_quadrotor_mode"))
        / "config"
        / "real_flying_hand_quadrotor_shadow.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="flying_hand_quadrotor_mode",
                executable="flying_hand_quadrotor_mode_node",
                name="flying_hand_quadrotor_mode",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
