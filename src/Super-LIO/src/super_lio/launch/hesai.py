import os
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_super_lio = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg_super_lio, 'config', 'hesai.yaml')
    rviz_config_file = os.path.join(pkg_super_lio, 'rviz', 'lio.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RVIZ2 (the integrated workflow uses planning RViz)'
    )
    declare_dense_output_arg = DeclareLaunchArgument(
        'dense_output',
        default_value='false',
        description='Publish the full undistorted scan on /lio/cloud_world'
    )
    rviz_flag = LaunchConfiguration('rviz')

    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[
            config_yaml,
            {
                'lio.output.dense': ParameterValue(
                    LaunchConfiguration('dense_output'), value_type=bool),
            },
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='super_lio',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz_flag)
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_dense_output_arg)
    ld.add_action(super_lio_node)
    ld.add_action(rviz2_node)

    return ld
