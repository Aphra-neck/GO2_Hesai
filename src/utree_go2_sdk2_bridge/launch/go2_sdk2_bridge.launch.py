from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bridge_node(context):
    overrides = {
        "network_interface": LaunchConfiguration("network_interface")
    }
    for name in ("max_vx", "max_vy", "max_yaw_rate"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = ParameterValue(value, value_type=float)

    return [
        Node(
            package="utree_go2_sdk2_bridge",
            executable="go2_sdk2_bridge_node",
            name="go2_sdk2_bridge",
            output="screen",
            parameters=[LaunchConfiguration("config"), overrides],
        )
    ]


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("utree_go2_sdk2_bridge"))
        / "config"
        / "go2_sdk2_bridge.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("network_interface", default_value="enP8p1s0"),
            DeclareLaunchArgument("max_vx", default_value=""),
            DeclareLaunchArgument("max_vy", default_value=""),
            DeclareLaunchArgument("max_yaw_rate", default_value=""),
            OpaqueFunction(function=_bridge_node),
        ]
    )
