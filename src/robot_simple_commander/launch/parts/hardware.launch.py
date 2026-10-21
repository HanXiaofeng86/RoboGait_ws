#!/usr/bin/env python3
"""Hardware: same Mini bringup as daily nav (lidar RESET + wait for can0).

Do not Include roverrobotics_driver mini.launch.py directly — that skips
RESET and RPLidar S2 dies with health=2 (exit 255).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hw = str(
        Path(get_package_share_directory('robot_mppi_navigation'))
        / 'launch' / 'rover_mini_hardware.launch.py')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('reset_lidar', default_value='True'))
    ld.add_action(DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hw),
        launch_arguments={
            'reset_lidar': LaunchConfiguration('reset_lidar'),
            'lidar_port': LaunchConfiguration('lidar_port'),
        }.items(),
    ))
    return ld
