#!/usr/bin/env python3
"""B3 straight follow: distance only. No Nav2, no turning, gimbal held still.

Does not start person_nav_goal. Do not run this together with
rviz_goal_navigation, person_follow, or follow_trajectory — those also
write the speed chain.

  source ~/rover_workspace_final/install/setup.bash
  source ~/RoboGait_ws/install/setup.bash
  ros2 launch robogait_person_follow straight_follow.launch.py

Person stands 3 m in front of the camera. The robot drives forward to
catch up, backs up slowly if they come closer, and does not steer.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('robot_mppi_navigation')
    perception_share = get_package_share_directory('robogait_perception')
    follow_share = get_package_share_directory('robogait_person_follow')
    gimbal_cfg = os.path.join(
        get_package_share_directory('robogait_gimbal'), 'config', 'gimbal.yaml')
    person_cfg = os.path.join(follow_share, 'config', 'person_follow.yaml')
    straight_cfg = os.path.join(follow_share, 'config', 'straight_follow.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'person_source',
            default_value='nuitrack',
            description="'nuitrack' skeleton (default). Do not combine with SDK.",
        ),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument(
            'device_name',
            default_value=(
                '/dev/serial/by-id/'
                'usb-FTDI_USB__-__Serial_Converter_FT8ISNFV-if00-port0'),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'rover_mini_hardware.launch.py')),
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
            parameters=[gimbal_cfg, {'freeze_gimbal': True}],
        ),
        Node(
            package='robogait_gimbal',
            executable='dynamixel_driver',
            name='dynamixel_driver',
            output='screen',
            parameters=[
                gimbal_cfg,
                {
                    'device_name': LaunchConfiguration('device_name'),
                    'freeze_gimbal': True,
                },
            ],
        ),
        Node(
            package='robogait_person_follow',
            executable='straight_follow',
            name='straight_follow',
            output='screen',
            parameters=[straight_cfg],
        ),
        Node(
            package='robot_mppi_navigation',
            executable='safety_gate',
            name='safety_gate',
            output='screen',
            parameters=[{
                'input_topic': 'cmd_vel_smoothed',
                'output_topic': 'cmd_vel',
                'odom_topic': 'odometry/wheels',
                'cmd_timeout': 0.5,
                'odom_timeout': 0.5,
                'check_frequency': 20.0,
            }],
        ),
    ])
