#!/usr/bin/env python3
"""调试用：只起 EKF（日常请用 rviz_goal_navigation.launch.py）。

用法:
  ros2 launch robot_mppi_navigation ekf_amcl_localization.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression


def generate_launch_description():
    # For Mode 1 we primarily need EKF output at `/odometry/filtered`.
    # Map->odom is provided by SLAM when slam:=True.
    rover_driver_share = get_package_share_directory('roverrobotics_driver')
    robot_localizer_launch = str(Path(rover_driver_share) / 'launch' / 'robot_localizer.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_ekf = LaunchConfiguration('enable_ekf')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation/Gazebo clock if true'
    )
    declare_enable_ekf = DeclareLaunchArgument(
        'enable_ekf',
        default_value='True',
        description='Whether to start robot_localizer (EKF) node'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_ekf)

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_localizer_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(PythonExpression(["'", enable_ekf, "' == 'True'"])),
        )
    )

    return ld
