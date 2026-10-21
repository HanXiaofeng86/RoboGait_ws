#!/usr/bin/env python3
"""Gamepad teleop with lidar RESET first.

Vendor ``roverrobotics_driver mini_teleop.launch.py`` starts the S2 with no
RESET, so health=2 after a previous Ctrl+C and rplidar exits 255.

This launch: rover_mini_hardware (soft-reset + wait for can0) + PS4 pad.

  ros2 launch robot_mppi_navigation mini_teleop.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mppi_share = Path(get_package_share_directory('robot_mppi_navigation'))
    rover_share = Path(get_package_share_directory('roverrobotics_driver'))

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('reset_lidar', default_value='True'))
    ld.add_action(DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(mppi_share / 'launch' / 'rover_mini_hardware.launch.py')),
        launch_arguments={
            'reset_lidar': LaunchConfiguration('reset_lidar'),
            'lidar_port': LaunchConfiguration('lidar_port'),
        }.items(),
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rover_share / 'launch' / 'ps4_controller.launch.py')),
    ))
    return ld
