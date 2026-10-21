#!/usr/bin/env python3
"""Lightweight online SLAM mapping (no full Nav2 / MPPI).

Starts only what mapping needs:
  hardware (chassis, lidar, IMU) + lidar soft-reset
  EKF (odom -> base_link)
  slam_toolbox (this package's mapper YAML)
  safety_gate + PS4 pad (/cmd_vel_smoothed -> /cmd_vel)
  RViz: map, laser, robot, and a save-map bar

Put the robot at the experiment start before launching. That pose is the
map origin. Save from the RViz bar (or map_saver_cli) before Ctrl+C.

Usage:
  ros2 launch robot_slam_mapping slam_mapping.launch.py
  # use_rviz:=False skips the window

Save map (while this launch is still running):
  ros2 run nav2_map_server map_saver_cli -f \\
    ~/RoboGait_ws/src/robot_slam_mapping/maps/my_lab
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
    mapping_share = get_package_share_directory('robot_slam_mapping')
    nav_share = get_package_share_directory('robot_mppi_navigation')
    default_slam_params = str(
        Path(mapping_share) / 'config' / 'mapper_params_online_async.yaml')
    ekf_params = str(
        Path(nav_share) / 'config' / 'robot_localization_ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gamepad = LaunchConfiguration('use_gamepad')
    slam_params_file = LaunchConfiguration('slam_params_file')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument(
        'use_gamepad', default_value='True',
        description='Include mini_teleop.launch.py'))
    ld.add_action(DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params,
        description='slam_toolbox YAML owned by this package'))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Open mapping RViz (map, laser, save bar).'))

    # Hardware: chassis + lidar + IMU (auto soft-resets lidar first).
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'rover_mini_hardware.launch.py')),
    ))

    # Wheel + IMU -> /odometry/filtered and odom->base_link TF.
    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
    ))

    # Display-only copy of /scan, dropped onto the map plane (/scan_flat).
    ld.add_action(Node(
        package='robot_slam_mapping',
        executable='flatten_scan',
        name='flatten_scan',
        output='screen',
    ))

    # Online SLAM (publishes /map and map->odom). Still uses the real /scan.
    ld.add_action(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    ))

    # Pad publishes /cmd_vel_smoothed; gate owns /cmd_vel for the driver.
    ld.add_action(Node(
        package='robot_mppi_navigation',
        executable='safety_gate',
        name='safety_gate',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_timeout': 0.5,
            'odom_timeout': 0.5,
        }],
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_share, 'launch', 'mini_teleop.launch.py')),
        condition=IfCondition(PythonExpression(["'", use_gamepad, "' == 'True'"])),
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(Path(mapping_share) / 'rviz' / 'mapping.rviz')],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('use_rviz'), "' == 'True'"])),
    ))

    return ld
