from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("utree_go2_sdk2_bridge"))
        / "config"
        / "go2_sdk2_bridge.yaml"
    )
    config = LaunchConfiguration("config")
    network_interface = LaunchConfiguration("network_interface")
    max_vx = LaunchConfiguration("max_vx")
    max_vy = LaunchConfiguration("max_vy")
    max_yaw_rate = LaunchConfiguration("max_yaw_rate")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("network_interface", default_value="enP8p1s0"),
            DeclareLaunchArgument("max_vx", default_value="0.1"),
            DeclareLaunchArgument("max_vy", default_value="0.05"),
            DeclareLaunchArgument("max_yaw_rate", default_value="0.2"),
            Node(
                package="utree_go2_sdk2_bridge",
                executable="go2_sdk2_bridge_node",
                name="go2_sdk2_bridge",
                output="screen",
                parameters=[
                    config,
                    {
                        "network_interface": network_interface,
                        "max_vx": ParameterValue(max_vx, value_type=float),
                        "max_vy": ParameterValue(max_vy, value_type=float),
                        "max_yaw_rate": ParameterValue(max_yaw_rate, value_type=float),
                    },
                ],
            ),
        ]
    )
