#!/usr/bin/env python3
"""Follow trajectory (hardware + nav_stack/MPPI + follow_path node).

  ros2 launch robot_simple_commander follow_trajectory.launch.py \\
    path_file:=…/recorded/lab_route.yaml
  ros2 launch robot_simple_commander follow_trajectory.launch.py \\
    path_name:=left_90
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def _share(*parts: str) -> str:
    root = Path(get_package_share_directory('robot_simple_commander'))
    return str(root.joinpath(*parts))


def generate_launch_description():
    parts_dir = _share('launch', 'parts')
    paths_dir = _share('paths')
    default_recorded = _share('recorded', 'lab_route.yaml')

    path_name = LaunchConfiguration('path_name')
    path_file = LaunchConfiguration('path_file')
    # empty path_file → built-in paths/<path_name>.yaml
    resolved = PythonExpression(
        ["'", path_file, "' if '", path_file, "' else '",
         paths_dir, "/' + '", path_name, "' + '.yaml'"])

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('path_name', default_value='straight'))
    ld.add_action(DeclareLaunchArgument(
        'path_file',
        default_value='',
        description=(
            'Absolute path yaml; empty → paths/<path_name>.yaml. '
            f'Recorded default example: {default_recorded}')))
    ld.add_action(DeclareLaunchArgument('amcl', default_value='False'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False'))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=_share('maps', 'lab2_new.yaml')))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(parts_dir) / 'hardware.launch.py')),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(parts_dir) / 'nav_stack.launch.py')),
        launch_arguments={
            'amcl': LaunchConfiguration('amcl'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
        }.items(),
    ))

    ld.add_action(TimerAction(
        period=8.0,
        actions=[
            Node(
                package='robot_simple_commander',
                executable='follow_path',
                name='robogait_follow_path',
                output='screen',
                parameters=[{
                    'path_file': resolved,
                    'wait_for_nav2': False,
                    'controller_id': 'FollowPath',
                    'goal_checker_id': 'general_goal_checker',
                }],
            ),
        ],
    ))
    return ld
