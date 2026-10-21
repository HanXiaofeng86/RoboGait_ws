#!/usr/bin/env python3
"""B0: test gimbal only — Nuitrack skeleton + pan/tilt (no chassis / Nav2).

Default person source is Nuitrack (the method that locked a torso on this
robot). Overlay the old workspace first so `orbbec_person_publisher` is found.
SDK is only for the teacher point-cloud demo (`person_source:=sdk`).
Do not start SDK and Nuitrack together (same USB camera).

  source ~/rover_workspace_final/install/setup.bash
  source ~/RoboGait_ws/install/setup.bash
  ros2 launch robogait_gimbal test_gimbal.launch.py

  # Point cloud demo (no skeleton on this NUC):
  ros2 launch robogait_gimbal test_gimbal.launch.py person_source:=sdk

Use this (not person_source alone) so the Dynamixel pan/tilt actually follows you.
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
            description="'nuitrack' = skeleton (default); 'sdk' = point cloud demo",
        ),
        DeclareLaunchArgument('use_rviz', default_value='True'),
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
    ])
