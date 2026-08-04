from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("utree_dog_navigation"))
        / "config"
        / "terrain_navigation.yaml"
    )
    default_rviz_config = str(
        Path(get_package_share_directory("utree_dog_navigation"))
        / "rviz"
        / "hesai_navigation.rviz"
    )
    config = LaunchConfiguration("config")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_to_base_link_tf",
                output="screen",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "imu", "--child-frame-id", "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_to_hesai_lidar_tf",
                output="screen",
                arguments=[
                    "--x", "0.171", "--y", "0", "--z", "0.0908",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "imu", "--child-frame-id", "hesai_lidar",
                ],
            ),
            Node(
                package="utree_dog_navigation",
                executable="terrain_mapper_node",
                name="terrain_mapper",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="utree_dog_navigation",
                executable="body_lattice_planner_node",
                name="body_lattice_planner",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
            ),
        ]
    )
