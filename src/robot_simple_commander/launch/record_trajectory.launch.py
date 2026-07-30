#!/usr/bin/env python3
"""Record trajectory (hardware + EKF + map TF + record_path node).

  ros2 launch robot_simple_commander record_trajectory.launch.py
  # other terminal: ros2 run teleop_twist_keyboard teleop_twist_keyboard
  # Ctrl+C here → saves to recorded/lab_route.yaml (override with output_file:=…)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share(*parts: str) -> str:
    root = Path(get_package_share_directory('robot_simple_commander'))
    return str(root.joinpath(*parts))


def generate_launch_description():
    parts_dir = _share('launch', 'parts')
    default_out = _share('recorded', 'lab_route.yaml')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'output_file',
        default_value=default_out,
        description='Where to save the path yaml (default: package recorded/)'))
    ld.add_action(DeclareLaunchArgument('min_separation', default_value='0.05'))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=_share('maps', 'lab2_new.yaml')))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False'))

    ld.add_action(LogInfo(msg=[
        'Record trajectory → ', LaunchConfiguration('output_file'), '\n',
        'Drive in another terminal:\n',
        '  ros2 run teleop_twist_keyboard teleop_twist_keyboard\n',
        'Then Ctrl+C here to save.',
    ]))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(parts_dir) / 'hardware.launch.py')),
    ))

    ld.add_action(Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[_share('config', 'robot_localization_ekf.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    ))

    ld.add_action(Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map'),
        }],
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server'],
        }],
    ))
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom', output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
    ))

    ld.add_action(Node(
        package='robot_simple_commander',
        executable='record_path',
        name='record_path',
        output='screen',
        parameters=[{
            'output_file': LaunchConfiguration('output_file'),
            'min_separation': LaunchConfiguration('min_separation'),
            'parent_frame': 'map',
            'child_frame': 'base_link',
        }],
    ))
    return ld
