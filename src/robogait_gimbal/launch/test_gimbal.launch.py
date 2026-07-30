#!/usr/bin/env python3
"""B0: test gimbal only — perception + filter + pan servo (no chassis / Nav2).

  ros2 launch robogait_gimbal test_gimbal.launch.py
  ros2 launch robogait_gimbal test_gimbal.launch.py device_name:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_cfg = os.path.join(
        get_package_share_directory('robogait_perception'),
        'config', 'perception.yaml')
    gimbal_cfg = os.path.join(
        get_package_share_directory('robogait_gimbal'),
        'config', 'gimbal.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_nuitrack', default_value='true'),
        DeclareLaunchArgument('use_gimbal', default_value='true'),
        DeclareLaunchArgument('device_name', default_value='/dev/ttyUSB0'),

        Node(
            package='orbbec_person_publisher',
            executable='nuitrack_person_node',
            name='nuitrack_person_node',
            output='screen',
            parameters=[{'image_width': 640, 'save_csv': False}],
            condition=IfCondition(LaunchConfiguration('use_nuitrack')),
        ),
        Node(
            package='robogait_perception',
            executable='distance_orbbec',
            name='distance_orbbec',
            output='screen',
            parameters=[perception_cfg],
        ),
        Node(
            package='robogait_person_follow',
            executable='distance_filter',
            name='distance_filter',
            output='screen',
        ),
        Node(
            package='robogait_gimbal',
            executable='camera_tracker',
            name='camera_tracker',
            output='screen',
            parameters=[gimbal_cfg],
            condition=IfCondition(LaunchConfiguration('use_gimbal')),
        ),
        Node(
            package='robogait_gimbal',
            executable='dynamixel_driver',
            name='dynamixel_driver',
            output='screen',
            parameters=[
                gimbal_cfg,
                {'device_name': LaunchConfiguration('device_name')},
            ],
            condition=IfCondition(LaunchConfiguration('use_gimbal')),
        ),
    ])
