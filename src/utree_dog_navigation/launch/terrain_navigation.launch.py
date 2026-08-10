from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    body_yaw_offset = LaunchConfiguration("body_yaw_offset")
    body_frame = LaunchConfiguration("body_frame")
    planning_mode = LaunchConfiguration("planning_mode")
    flat_ground_confirmed = LaunchConfiguration("flat_ground_confirmed")
    verified_flat_start = LaunchConfiguration("verified_flat_start")

    body_odom_adapter = Node(
        package="utree_dog_navigation",
        executable="body_odom_adapter_node",
        name="body_odom_adapter",
        output="screen",
        parameters=[
            config,
            {
                "yaw_offset": ParameterValue(
                    body_yaw_offset,
                    value_type=float,
                ),
                "body_frame": body_frame,
            },
        ],
    )

    imu_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_to_base_link_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", body_yaw_offset,
            "--frame-id", "imu", "--child-frame-id", body_frame,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument(
                "body_yaw_offset",
                default_value="-1.5707963267948966",
            ),
            DeclareLaunchArgument("body_frame", default_value="base_link"),
            DeclareLaunchArgument("planning_mode", default_value="terrain"),
            DeclareLaunchArgument("flat_ground_confirmed", default_value="false"),
            DeclareLaunchArgument(
                "verified_flat_start",
                default_value="false",
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=body_odom_adapter,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="body odometry adapter exited"
                            )
                        )
                    ],
                )
            ),
            body_odom_adapter,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=imu_to_base_link_tf,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="IMU-to-body static transform exited"
                            )
                        )
                    ],
                )
            ),
            imu_to_base_link_tf,
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
                parameters=[
                    config,
                    {
                        "planning_mode": planning_mode,
                        "body_frame": body_frame,
                        "flat_ground_confirmed": ParameterValue(
                            flat_ground_confirmed,
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="utree_dog_navigation",
                executable="body_lattice_planner_node",
                name="body_lattice_planner",
                output="screen",
                parameters=[
                    config,
                    {
                        "planning_mode": planning_mode,
                        "body_frame": body_frame,
                        "flat_ground_confirmed": ParameterValue(
                            flat_ground_confirmed,
                            value_type=bool,
                        ),
                        "verified_flat_start.enabled": ParameterValue(
                            verified_flat_start,
                            value_type=bool,
                        ),
                    },
                ],
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
