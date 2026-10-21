#!/usr/bin/env python3
"""One-shot bringup: hardware, then Nav2/MPPI + AMCL + RViz (daily nav).

Nav2/RViz start only after /scan and /odometry/wheels exist, so the
lidar-reset window does not spam missing odom/map TF.

Usage:
  ros2 launch robot_mppi_navigation rviz_goal_navigation.launch.py
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    use_external_slam_params = LaunchConfiguration('use_external_slam_params')
    slam_params_file = LaunchConfiguration('slam_params_file')

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
    ld.add_action(DeclareLaunchArgument(
        'use_external_slam_params',
        default_value='False',
        description='True=slam_toolbox YAML from slam_params_file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'slam_params_file',
        default_value='',
        description='Path to slam_toolbox YAML (with use_external_slam_params)'
    ))
    pkg_share = get_package_share_directory('robot_mppi_navigation')
    default_map = str(Path(pkg_share) / 'maps' / 'mini_slab1.yaml')
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml (forwarded to Nav2)'
    ))
    map_yaml = LaunchConfiguration('map')
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

    wait_sensors = Node(
        package='robot_mppi_navigation',
        executable='wait_for_nav_sensors',
        name='wait_for_nav_sensors',
        output='screen',
    )
    ld.add_action(wait_sensors)

    nav2_args = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'slam': slam,
        'amcl': amcl,
        'log_level': log_level,
        'use_external_slam_params': use_external_slam_params,
        'slam_params_file': slam_params_file,
        'map': map_yaml,
    }
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_mppi_controller.launch.py')
        ),
        launch_arguments=nav2_args.items(),
    )

    scan_viz = Node(
        package='robot_mppi_navigation',
        executable='scan_viz',
        name='scan_viz',
        output='screen',
    )
    cancel_nav_on_pose = Node(
        package='robot_mppi_navigation',
        executable='cancel_nav_on_pose',
        name='cancel_nav_on_pose',
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_rviz, "' == 'True'"])),
    )

    # Delay Nav2/RViz until /scan and wheel odom exist (after lidar reset).
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=wait_sensors,
            on_exit=[GroupAction(actions=[nav2_include, scan_viz, cancel_nav_on_pose, rviz])],
        ),
    ))

    return ld
