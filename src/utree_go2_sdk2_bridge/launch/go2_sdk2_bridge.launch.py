from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("utree_go2_sdk2_bridge"))
        / "config"
        / "go2_sdk2_bridge.yaml"
    )
    config = LaunchConfiguration("config")
    network_interface = LaunchConfiguration("network_interface")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("network_interface", default_value="enP8p1s0"),
            Node(
                package="utree_go2_sdk2_bridge",
                executable="go2_sdk2_bridge_node",
                name="go2_sdk2_bridge",
                output="screen",
                parameters=[config, {"network_interface": network_interface}],
            ),
        ]
    )
