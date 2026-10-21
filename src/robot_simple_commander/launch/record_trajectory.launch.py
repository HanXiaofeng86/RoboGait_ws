#!/usr/bin/env python3
"""Record trajectory (hardware + EKF + AMCL + record_path node).

  ros2 launch robot_simple_commander record_trajectory.launch.py
  # Set the name in config/route_name.yaml (or name:=my_loop)
  # RViz 2D Pose Estimate first, then drive; Ctrl+C here → recorded/<name>.yaml
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

from robot_simple_commander.lib.route_name import load_route_name
from robot_simple_commander.lib.route_name import recorded_yaml
from robot_simple_commander.lib.route_name import route_name_config


def _share(*parts: str) -> str:
    root = Path(get_package_share_directory('robot_simple_commander'))
    return str(root.joinpath(*parts))


def _lab_map() -> str:
    return str(
        Path(get_package_share_directory('robot_slam_mapping'))
        / 'maps' / 'mini_lab3.yaml')


def generate_launch_description():
    parts_dir = _share('launch', 'parts')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'name', default_value='',
        description=(
            'Trajectory name (no .yaml). Empty → config/route_name.yaml. '
            'Saves to recorded/<name>.yaml')))
    ld.add_action(DeclareLaunchArgument(
        'output_file', default_value='',
        description='Absolute yaml; if set, ignores name / route_name.yaml'))
    ld.add_action(DeclareLaunchArgument('min_separation', default_value='0.05'))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=_lab_map(),
        description='Map yaml (default: current lab mini_lab3)'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False'))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Open RViz (map + laser). False if no display.'))
    ld.add_action(DeclareLaunchArgument(
        'use_gamepad', default_value='True',
        description=(
            'PS4 pad → /cmd_vel. Do not also launch mini_teleop '
            '(that starts a second chassis).')))

    ld.add_action(LogInfo(msg=[
        'Edit the name in ', str(route_name_config()), '\n',
        '1. Wait for the red laser in RViz.\n',
        '2. Toolbar → 2D Pose Estimate (green arrow): put the robot where it is.\n',
        '3. Drive with the PS4 pad (left stick forward, right stick turn).\n',
        '4. Ctrl+C here to save.',
    ]))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(parts_dir) / 'rviz.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(parts_dir) / 'hardware.launch.py')),
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('roverrobotics_driver'))
            / 'launch' / 'ps4_controller.launch.py')),
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('use_gamepad'), "' == 'True'"])),
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
        package='nav2_amcl', executable='amcl', name='amcl',
        output='screen',
        parameters=[
            _share('config', 'nav2_mppi_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    ))

    ld.add_action(OpaqueFunction(function=_record_path_node))
    return ld


def _record_path_node(context, *args, **kwargs):
    output_file = context.launch_configurations.get('output_file', '').strip()
    if output_file:
        out = output_file
        label = out
    else:
        name = load_route_name(context.launch_configurations.get('name', ''))
        out = str(recorded_yaml(name))
        label = f'{name} → {out}'
    return [
        LogInfo(msg=['Record as ', label]),
        Node(
            package='robot_simple_commander',
            executable='record_path',
            name='record_path',
            output='screen',
            parameters=[{
                'output_file': out,
                'min_separation': float(
                    context.launch_configurations['min_separation']),
                'parent_frame': 'map',
                'child_frame': 'base_link',
                'wait_for_initial_pose': True,
            }],
        ),
    ]
