#!/usr/bin/env python3
"""PS4 gamepad only (name matches rover_workspace mini_teleop).

Pad only — does not start a second chassis. Publishes /cmd_vel_smoothed
so safety_gate (from the nav/mapping stack) owns /cmd_vel.

Usage:
  ros2 launch robot_slam_mapping mini_teleop.launch.py

Usually included by slam_mapping.launch.py (use_gamepad:=True).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    driver_share = Path(get_package_share_directory('roverrobotics_driver'))
    controller_config = driver_share / 'config' / 'ps4_controller_config.yaml'
    topics_config = driver_share / 'config' / 'topics.yaml'

    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
        ),
        Node(
            package='roverrobotics_input_manager',
            executable='joys_manager.py',
            name='joy_manager',
            output='screen',
            parameters=[{
                'controller': str(controller_config),
                'topics': str(topics_config),
            }],
            remappings=[('/cmd_vel', '/cmd_vel_smoothed')],
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
