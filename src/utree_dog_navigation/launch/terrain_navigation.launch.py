from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_rviz_config(path):
    config_path = Path(path)
    try:
        if not config_path.is_file():
            raise OSError("not a regular file")
        return config_path.read_bytes()
    except (AttributeError, OSError):
        raise RuntimeError(
            f"RViz config must be a readable regular file: {path}"
        ) from None


def _validate_navigation_startup(
    context,
    *,
    rviz,
    rviz_config,
    planning_mode,
    enable_legacy_terrain,
    allow_custom_rviz_config,
    flat_ground_confirmed,
    flat_obstacle_rviz_config,
    legacy_terrain_rviz_config,
):
    rviz_value = rviz.perform(context)
    rviz_config_value = rviz_config.perform(context)
    planning_mode_value = planning_mode.perform(context)
    legacy_value = enable_legacy_terrain.perform(context)
    custom_rviz_value = allow_custom_rviz_config.perform(context)
    confirmation_value = flat_ground_confirmed.perform(context)

    if rviz_value not in ("true", "false"):
        raise RuntimeError(f"rviz must be true or false, got: {rviz_value}")
    if planning_mode_value not in ("terrain", "flat_obstacle"):
        raise RuntimeError(
            "planning_mode must be terrain or flat_obstacle, got: "
            f"{planning_mode_value}"
        )
    if legacy_value not in ("true", "false"):
        raise RuntimeError(
            "enable_legacy_terrain must be true or false, got: "
            f"{legacy_value}"
        )
    if custom_rviz_value not in ("true", "false"):
        raise RuntimeError(
            "allow_custom_rviz_config must be true or false, got: "
            f"{custom_rviz_value}"
        )
    if confirmation_value not in ("true", "false"):
        raise RuntimeError(
            "flat_ground_confirmed must be true or false, got: "
            f"{confirmation_value}"
        )
    if planning_mode_value == "terrain" and legacy_value != "true":
        raise RuntimeError("terrain mode requires enable_legacy_terrain=true")
    if planning_mode_value != "terrain" and legacy_value == "true":
        raise RuntimeError(
            "enable_legacy_terrain=true requires planning_mode=terrain"
        )
    if planning_mode_value == "flat_obstacle" and confirmation_value != "true":
        raise RuntimeError(
            "flat_obstacle mode requires flat_ground_confirmed=true"
        )

    candidate_rviz_content = _read_rviz_config(rviz_config_value)
    flat_obstacle_rviz_content = _read_rviz_config(
        flat_obstacle_rviz_config
    )
    legacy_terrain_rviz_content = _read_rviz_config(
        legacy_terrain_rviz_config
    )
    if flat_obstacle_rviz_content == legacy_terrain_rviz_content:
        raise RuntimeError(
            "official flat-obstacle and legacy terrain RViz configs "
            "must not be identical"
        )

    if planning_mode_value == "flat_obstacle":
        expected_rviz_content = flat_obstacle_rviz_content
        incompatible_rviz_content = legacy_terrain_rviz_content
        incompatible_message = (
            "flat_obstacle mode cannot use legacy terrain RViz config"
        )
    else:
        expected_rviz_content = legacy_terrain_rviz_content
        incompatible_rviz_content = flat_obstacle_rviz_content
        incompatible_message = (
            "terrain mode cannot use flat-obstacle RViz config"
        )

    if candidate_rviz_content == expected_rviz_content:
        return []
    if candidate_rviz_content == incompatible_rviz_content:
        raise RuntimeError(
            f"{incompatible_message}: {rviz_config_value}"
        )
    if custom_rviz_value != "true":
        raise RuntimeError(
            "custom navigation RViz config requires "
            "allow_custom_rviz_config=true: "
            f"{rviz_config_value}"
        )
    return []


def generate_launch_description():
    package_share = Path(get_package_share_directory("utree_dog_navigation"))
    default_config = str(
        package_share / "config" / "terrain_navigation.yaml"
    )
    flat_obstacle_rviz_config = str(
        package_share / "rviz" / "flat_obstacle_navigation.rviz"
    )
    legacy_terrain_rviz_config = str(
        package_share / "rviz" / "hesai_navigation.rviz"
    )
    default_rviz_config = flat_obstacle_rviz_config
    config = LaunchConfiguration("config")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    body_yaw_offset = LaunchConfiguration("body_yaw_offset")
    body_frame = LaunchConfiguration("body_frame")
    lidar_offset_x = LaunchConfiguration("lidar_offset_x")
    lidar_offset_y = LaunchConfiguration("lidar_offset_y")
    lidar_offset_z = LaunchConfiguration("lidar_offset_z")
    planning_mode = LaunchConfiguration("planning_mode")
    enable_legacy_terrain = LaunchConfiguration("enable_legacy_terrain")
    allow_custom_rviz_config = LaunchConfiguration("allow_custom_rviz_config")
    flat_ground_confirmed = LaunchConfiguration("flat_ground_confirmed")
    verified_flat_start = LaunchConfiguration("verified_flat_start")
    record_3d_maps = LaunchConfiguration("record_3d_maps")
    record_3d_maps_output = LaunchConfiguration("record_3d_maps_output")
    record_3d_maps_max_snapshots = LaunchConfiguration(
        "record_3d_maps_max_snapshots"
    )
    record_3d_maps_max_megabytes = LaunchConfiguration(
        "record_3d_maps_max_megabytes"
    )
    record_3d_maps_source_git_sha = LaunchConfiguration(
        "record_3d_maps_source_git_sha"
    )

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
            DeclareLaunchArgument("lidar_offset_x", default_value="0.171"),
            DeclareLaunchArgument("lidar_offset_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_offset_z", default_value="0.0908"),
            DeclareLaunchArgument("planning_mode", default_value="flat_obstacle"),
            DeclareLaunchArgument("enable_legacy_terrain", default_value="false"),
            DeclareLaunchArgument("allow_custom_rviz_config", default_value="false"),
            DeclareLaunchArgument("flat_ground_confirmed", default_value="false"),
            DeclareLaunchArgument(
                "verified_flat_start",
                default_value="false",
            ),
            DeclareLaunchArgument("record_3d_maps", default_value="false"),
            DeclareLaunchArgument(
                "record_3d_maps_output",
                default_value="~/go2_map_exports",
            ),
            DeclareLaunchArgument(
                "record_3d_maps_max_snapshots",
                default_value="120",
            ),
            DeclareLaunchArgument(
                "record_3d_maps_max_megabytes",
                default_value="100",
            ),
            DeclareLaunchArgument(
                "record_3d_maps_source_git_sha",
                default_value="unknown",
            ),
            OpaqueFunction(
                function=_validate_navigation_startup,
                kwargs={
                    "rviz": rviz,
                    "rviz_config": rviz_config,
                    "planning_mode": planning_mode,
                    "enable_legacy_terrain": enable_legacy_terrain,
                    "allow_custom_rviz_config": allow_custom_rviz_config,
                    "flat_ground_confirmed": flat_ground_confirmed,
                    "flat_obstacle_rviz_config": flat_obstacle_rviz_config,
                    "legacy_terrain_rviz_config": legacy_terrain_rviz_config,
                },
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
                    "--x", lidar_offset_x,
                    "--y", lidar_offset_y,
                    "--z", lidar_offset_z,
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "imu", "--child-frame-id", "hesai_lidar",
                ],
            ),
            Node(
                package="utree_dog_navigation",
                executable="terrain_mapper_node",
                name="terrain_mapper",
                output="screen",
                on_exit=[
                    EmitEvent(event=Shutdown(reason="terrain mapper exited"))
                ],
                parameters=[
                    config,
                    {
                        "planning_mode": planning_mode,
                        "enable_legacy_terrain": ParameterValue(
                            enable_legacy_terrain,
                            value_type=bool,
                        ),
                        "body_frame": body_frame,
                        "body_yaw_offset": ParameterValue(
                            body_yaw_offset,
                            value_type=float,
                        ),
                        "flat_obstacle.lidar_offset.x": ParameterValue(
                            lidar_offset_x,
                            value_type=float,
                        ),
                        "flat_obstacle.lidar_offset.y": ParameterValue(
                            lidar_offset_y,
                            value_type=float,
                        ),
                        "flat_obstacle.lidar_offset.z": ParameterValue(
                            lidar_offset_z,
                            value_type=float,
                        ),
                        "flat_ground_confirmed": ParameterValue(
                            flat_ground_confirmed,
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="utree_dog_navigation",
                executable="flat_obstacle_map_recorder.py",
                name="flat_obstacle_map_recorder",
                output="screen",
                parameters=[
                    {
                        "topic": "/flat_obstacle_filtered_map_3d",
                        "output_directory": record_3d_maps_output,
                        "max_snapshots": ParameterValue(
                            record_3d_maps_max_snapshots,
                            value_type=int,
                        ),
                        "max_total_megabytes": ParameterValue(
                            record_3d_maps_max_megabytes,
                            value_type=int,
                        ),
                        "source_git_sha": record_3d_maps_source_git_sha,
                        "navigation_config": config,
                        "planning_mode": planning_mode,
                        "flat_ground_confirmed": ParameterValue(
                            flat_ground_confirmed,
                            value_type=bool,
                        ),
                        "body_yaw_offset": ParameterValue(
                            body_yaw_offset,
                            value_type=float,
                        ),
                        "lidar_offset_x": ParameterValue(
                            lidar_offset_x,
                            value_type=float,
                        ),
                        "lidar_offset_y": ParameterValue(
                            lidar_offset_y,
                            value_type=float,
                        ),
                        "lidar_offset_z": ParameterValue(
                            lidar_offset_z,
                            value_type=float,
                        ),
                    },
                ],
                condition=IfCondition(record_3d_maps),
            ),
            Node(
                package="utree_dog_navigation",
                executable="body_lattice_planner_node",
                name="body_lattice_planner",
                output="screen",
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="body lattice planner exited")
                    )
                ],
                parameters=[
                    config,
                    {
                        "planning_mode": planning_mode,
                        "enable_legacy_terrain": ParameterValue(
                            enable_legacy_terrain,
                            value_type=bool,
                        ),
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
