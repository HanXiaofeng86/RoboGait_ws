#!/usr/bin/env python3
"""只启动硬件：底盘驱动 + URDF + 雷达 + IMU。

用法:
  ros2 launch mppi_navigation rover_mini_hardware.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rover_driver_share = get_package_share_directory('roverrobotics_driver')

    rover_description_path = get_package_share_path('roverrobotics_description')
    default_model_path = rover_description_path / 'urdf/mini.urdf'

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )

    mini_launch_path = str(Path(rover_driver_share) / 'launch' / 'mini.launch.py')

    ld = LaunchDescription()
    ld.add_action(model_arg)

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mini_launch_path),
            launch_arguments={
                'model': LaunchConfiguration('model'),
            }.items(),
        )
    )

    return ld

