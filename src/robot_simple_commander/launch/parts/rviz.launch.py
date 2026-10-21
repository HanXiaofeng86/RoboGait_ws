#!/usr/bin/env python3
"""Optional RViz: this package's record/follow layout (map, laser, trail).

Included by record/follow entries. Not a standalone entry.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = str(
        Path(get_package_share_directory('robot_simple_commander'))
        / 'rviz' / 'record_follow.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Open RViz (map + laser + recorded trail). False if no display.'))
    ld.add_action(DeclareLaunchArgument(
        'rviz_config',
        default_value=default_cfg,
        description='RViz config yaml (record vs follow layout).'))
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_rviz, "' == 'True'"])),
    ))
    return ld
