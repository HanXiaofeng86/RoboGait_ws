#!/usr/bin/env python3
"""Hardware only: chassis + URDF + lidar + IMU (owned by this package)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rover_driver_share = get_package_share_directory('roverrobotics_driver')
    default_model = get_package_share_path('roverrobotics_description') / 'urdf/mini.urdf'
    mini_launch = str(Path(rover_driver_share) / 'launch' / 'mini.launch.py')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'model', default_value=str(default_model),
        description='Absolute path to robot urdf'))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mini_launch),
        launch_arguments={'model': LaunchConfiguration('model')}.items(),
    ))
    return ld
