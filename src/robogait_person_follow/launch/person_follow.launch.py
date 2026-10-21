#!/usr/bin/env python3
"""B1 one-shot: pure-MPPI Nav2 + person layer (person → goal → MPPI).

Default person source is Nuitrack. Overlay rover_workspace_final first.
Do not run SDK and Nuitrack together (same USB camera).

  source ~/rover_workspace_final/install/setup.bash
  source ~/RoboGait_ws/install/setup.bash
  ros2 launch robogait_person_follow person_follow.launch.py
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
        DeclareLaunchArgument(
            'person_source',
            default_value='nuitrack',
            description="'nuitrack' = skeleton (default); 'sdk' = point cloud only",
        ),
        DeclareLaunchArgument(
            'device_name',
            default_value=(
                '/dev/serial/by-id/'
                'usb-FTDI_USB__-__Serial_Converter_FT8ISNFV-if00-port0'),
        ),

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
                'person_source': LaunchConfiguration('person_source'),
                'use_rviz': 'False',
            }.items(),
        ),
    ])
