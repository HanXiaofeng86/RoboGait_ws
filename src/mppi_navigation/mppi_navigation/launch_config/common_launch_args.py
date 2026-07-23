"""Shared launch argument declarations.

Note: boolean values must be Python literals ('True'/'False') because Nav2
launch files evaluate them inside PythonExpression conditions.
"""

from launch.actions import DeclareLaunchArgument


def use_sim_time_arg(default: str = 'False') -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        'use_sim_time', default_value=default,
        description='Use simulation (Gazebo) clock if true')


def autostart_arg(default: str = 'True') -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        'autostart', default_value=default,
        description='Automatically startup the nav2 stack')


def log_level_arg(default: str = 'info') -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        'log_level', default_value=default, description='ROS log level')
