#!/usr/bin/env python3
"""B1 one-shot: pure-MPPI Nav2 + person layer (person → goal → MPPI).

  ros2 launch robogait_person_follow person_follow.launch.py
  ros2 launch robogait_person_follow person_follow.launch.py use_rviz:=False
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_share = get_package_share_directory('robot_mppi_navigation')
    follow_share = get_package_share_directory('robogait_person_follow')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('use_gimbal', default_value='true'),
        DeclareLaunchArgument('device_name', default_value='/dev/ttyUSB0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'rviz_goal_navigation.launch.py')),
            launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(follow_share, 'launch', 'person_layer.launch.py')),
            launch_arguments={
                'use_gimbal': LaunchConfiguration('use_gimbal'),
                'device_name': LaunchConfiguration('device_name'),
            }.items(),
        ),
    ])
