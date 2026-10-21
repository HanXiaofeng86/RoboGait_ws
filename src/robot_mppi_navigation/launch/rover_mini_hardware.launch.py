#!/usr/bin/env python3
"""Hardware only: chassis driver + URDF + lidar + IMU.

By default: soft-reset RPLidar, wait until can0 has frames, then start drivers.
That avoids health=2 lidar deaths and the first-connect CAN stale FATAL.

Usage:
  ros2 launch robot_mppi_navigation rover_mini_hardware.launch.py
  ros2 launch robot_mppi_navigation rover_mini_hardware.launch.py reset_lidar:=False
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _reset_rplidar_cmd():
    exe = os.path.join(
        get_package_prefix('robot_mppi_navigation'),
        'lib', 'robot_mppi_navigation', 'reset_rplidar')
    return [exe]


def generate_launch_description():
    rover_driver_share = get_package_share_directory('roverrobotics_driver')

    rover_description_path = get_package_share_path('roverrobotics_description')
    default_model_path = rover_description_path / 'urdf/mini.urdf'

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    reset_lidar_arg = DeclareLaunchArgument(
        'reset_lidar',
        default_value='True',
        description='Soft-reset RPLidar (0xA5 0x40) before starting hardware'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar',
        description='Serial device for reset_rplidar'
    )

    mini_launch_path = str(Path(rover_driver_share) / 'launch' / 'mini.launch.py')
    mini_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mini_launch_path),
        launch_arguments={
            'model': LaunchConfiguration('model'),
        }.items(),
    )

    wait_can_exe = os.path.join(
        get_package_prefix('robot_mppi_navigation'),
        'lib', 'robot_mppi_navigation', 'wait_for_can')
    wait_can_cmd = ExecuteProcess(
        cmd=[wait_can_exe],
        output='screen',
        condition=IfCondition(LaunchConfiguration('reset_lidar')),
    )

    # After CAN has frames, a short settle before the driver binds the socket.
    mini_after_reset = TimerAction(period=1.5, actions=[mini_include])

    reset_cmd = ExecuteProcess(
        cmd=_reset_rplidar_cmd() + [LaunchConfiguration('lidar_port')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('reset_lidar')),
    )

    start_wait_can = RegisterEventHandler(
        OnProcessExit(
            target_action=reset_cmd,
            on_exit=[wait_can_cmd],
        ),
        condition=IfCondition(LaunchConfiguration('reset_lidar')),
    )

    start_after_reset = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_can_cmd,
            on_exit=[mini_after_reset],
        ),
        condition=IfCondition(LaunchConfiguration('reset_lidar')),
    )

    # Skip reset path: start hardware immediately.
    mini_no_reset = GroupAction(
        actions=[mini_include],
        condition=UnlessCondition(LaunchConfiguration('reset_lidar')),
    )

    ld = LaunchDescription()
    ld.add_action(model_arg)
    ld.add_action(reset_lidar_arg)
    ld.add_action(lidar_port_arg)
    ld.add_action(reset_cmd)
    ld.add_action(start_wait_can)
    ld.add_action(start_after_reset)
    ld.add_action(mini_no_reset)

    return ld
