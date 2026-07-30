#!/usr/bin/env python3
"""One-shot bringup: hardware + Nav2/MPPI + AMCL + RViz (daily nav entry).

Usage:
  ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')
    amcl = LaunchConfiguration('amcl')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')

    ld = LaunchDescription()

    # Boolean args used in IfCondition/PythonExpression must be True/False.
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    ))
    ld.add_action(DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack'
    ))
    ld.add_action(DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='True=online SLAM; False=map_server localization'
    ))
    ld.add_action(DeclareLaunchArgument(
        'amcl',
        default_value='True',
        description='When slam:=False: True=AMCL; False=static map->odom'
    ))
    ld.add_action(DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS log level'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz2'
    ))

    pkg_share = get_package_share_directory('robot_mppi_navigation')
    cfg_rviz = Path(pkg_share) / 'rviz' / 'rviz_goal_navigation.rviz'
    rviz_args = []
    if cfg_rviz.is_file() and cfg_rviz.stat().st_size > 0:
        rviz_args = ['-d', str(cfg_rviz)]

    # Hardware only: do not pass empty model; rover_mini_hardware defaults to mini.urdf.
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'rover_mini_hardware.launch.py')
            ),
        )
    )

    # Nav2 + MPPI + EKF + localization
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'nav2_mppi_controller.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'slam': slam,
                'amcl': amcl,
                'log_level': log_level,
            }.items(),
        )
    )

    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_args,
            output='screen',
            condition=IfCondition(PythonExpression(["'", use_rviz, "' == 'True'"])),
        )
    )

    return ld
