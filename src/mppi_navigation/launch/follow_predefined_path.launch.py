#!/usr/bin/env python3
"""Mode 2: send a predefined geometric path to the already-running stack.

Two-step test flow:
    1. ros2 launch mppi_navigation rviz_goal_navigation.launch.py
    2. ros2 launch mppi_navigation follow_predefined_path.launch.py path_name:=left_90

The path yaml stores geometry only; MPPI computes all velocities online.
Paths in 'local' frame start at the robot's current pose.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    paths_dir = str(Path(get_package_share_directory('mppi_navigation')) / 'paths')

    path_name = LaunchConfiguration('path_name')
    path_file = LaunchConfiguration('path_file')

    # path_file overrides path_name when given.
    resolved_path = PythonExpression(
        ["'", path_file, "' if '", path_file, "' else '",
         paths_dir, "/' + '", path_name, "' + '.yaml'"])

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'path_name', default_value='straight',
        description='Name of a bundled test path: straight, left_90, '
                    'right_90, s_curve, obstacle_avoid'))
    ld.add_action(DeclareLaunchArgument(
        'path_file', default_value='',
        description='Absolute path to a custom path yaml (overrides path_name)'))

    ld.add_action(
        Node(
            package='mppi_navigation',
            executable='follow_path',
            name='follow_path_client',
            output='screen',
            parameters=[{'path_file': resolved_path}],
        )
    )

    return ld
