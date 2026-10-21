#!/usr/bin/env python3
"""Person-follow layer on an already-running pure-MPPI Nav2 stack (terminal 2).

Default person source is Nuitrack (skeleton). Overlay rover_workspace_final first.
SDK (`person_source:=sdk`) is the point-cloud demo only — do not run both.

  Terminal 1: ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
  Terminal 2: ros2 launch robogait_person_follow person_layer.launch.py
  Terminal 2 camera RViz:  ... person_layer.launch.py use_rviz:=True
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_share = get_package_share_directory('robogait_perception')
    follow_share = get_package_share_directory('robogait_person_follow')
    gimbal_cfg = os.path.join(
        get_package_share_directory('robogait_gimbal'),
        'config', 'gimbal.yaml')
    person_cfg = os.path.join(follow_share, 'config', 'person_follow.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'person_source',
            default_value='nuitrack',
            description="'nuitrack' = skeleton (default); 'sdk' = point cloud only",
        ),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('use_gimbal', default_value='true'),
        DeclareLaunchArgument(
            'device_name',
            default_value=(
                '/dev/serial/by-id/'
                'usb-FTDI_USB__-__Serial_Converter_FT8ISNFV-if00-port0'),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_share, 'launch', 'person_source.launch.py')),
            launch_arguments={
                'person_source': LaunchConfiguration('person_source'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_robot_model': 'False',
                'publish_static_tf': 'True',
            }.items(),
        ),
        Node(
            package='robogait_person_follow',
            executable='distance_filter',
            name='distance_filter',
            output='screen',
            parameters=[person_cfg],
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
        Node(
            package='robogait_person_follow',
            executable='person_nav_goal',
            name='person_nav_goal',
            output='screen',
            parameters=[person_cfg],
        ),
    ])
