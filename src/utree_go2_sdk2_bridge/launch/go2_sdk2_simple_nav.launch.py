from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    overrides = {"network_interface": LaunchConfiguration("network_interface")}
    for name in ("max_vx", "max_vy", "max_yaw_rate"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = float(value)
    adapter_config = str(
        Path(get_package_share_directory("utree_dog_navigation"))
        / "config"
        / "terrain_navigation.yaml"
    )
    body_yaw_offset = LaunchConfiguration("body_yaw_offset").perform(context)
    adapter = Node(
        package="utree_dog_navigation",
        executable="body_odom_adapter_node",
        name="simple_body_odom_adapter",
        output="screen",
        parameters=[
            adapter_config,
            {
                "input_odom_topic": "/lio/odom",
                "output_odom_topic": "/lio/body_odom",
                "world_frame": "world",
                "body_frame": "base_link",
                "yaw_offset": float(body_yaw_offset),
            },
        ],
    )
    return [
        adapter,
        Node(
            package="utree_go2_sdk2_bridge",
            executable="go2_sdk2_simple_nav_node",
            name="go2_sdk2_simple_nav",
            output="screen",
            parameters=[LaunchConfiguration("config"), overrides],
        )
    ]


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("utree_go2_sdk2_bridge"))
        / "config"
        / "go2_sdk2_simple_nav.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("network_interface", default_value="enP8p1s0"),
            DeclareLaunchArgument(
                "body_yaw_offset",
                default_value="-1.5707963267948966",
            ),
            DeclareLaunchArgument("max_vx", default_value=""),
            DeclareLaunchArgument("max_vy", default_value=""),
            DeclareLaunchArgument("max_yaw_rate", default_value=""),
            OpaqueFunction(function=_node),
        ]
    )
