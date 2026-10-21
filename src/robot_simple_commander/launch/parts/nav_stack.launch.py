#!/usr/bin/env python3
"""EKF + localization + Nav2/MPPI + smoother + safety (this package only).

Does not use robot_mppi_navigation. Own config under share/robot_simple_commander/config.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _share(name: str) -> str:
    root = Path(get_package_share_directory('robot_simple_commander'))
    return str(root / name)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    slam = LaunchConfiguration('slam')
    amcl = LaunchConfiguration('amcl')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')

    use_slam = PythonExpression(["'", slam, "' == 'True'"])
    use_map_localization = PythonExpression(["'", slam, "' != 'True'"])
    use_amcl = PythonExpression(
        ["'", slam, "' != 'True' and '", amcl, "' == 'True'"])
    use_static_tf = PythonExpression(
        ["'", slam, "' != 'True' and '", amcl, "' != 'True'"])

    lifecycle_nodes = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower',
        'velocity_smoother',
    ]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        },
        convert_types=True,
    )

    slam_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'slam_launch.py')

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='True'))
    ld.add_action(DeclareLaunchArgument('use_respawn', default_value='False'))
    ld.add_action(DeclareLaunchArgument('log_level', default_value='info'))
    ld.add_action(DeclareLaunchArgument('slam', default_value='False'))
    ld.add_action(DeclareLaunchArgument(
        'amcl', default_value='False',
        description='False=static map→odom; True=AMCL (follow launch sets True)'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=_share('config/nav2_mppi_params.yaml')))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=_share('maps/lab2_new.yaml')))

    ld.add_action(Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[_share('config/robot_localization_ekf.yaml'),
                    {'use_sim_time': use_sim_time}],
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        condition=IfCondition(use_slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_respawn': use_respawn,
            'log_level': log_level,
            'params_file': params_file,
        }.items(),
    ))

    ld.add_action(Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', condition=IfCondition(use_map_localization),
        parameters=[{'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml_file}],
    ))
    ld.add_action(Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        output='screen', condition=IfCondition(use_amcl),
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        condition=IfCondition(use_amcl),
        parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                     'node_names': ['map_server', 'amcl']}],
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        condition=IfCondition(use_static_tf),
        parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                     'node_names': ['map_server']}],
    ))
    ld.add_action(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom', output='screen',
        condition=IfCondition(use_static_tf),
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
    ))

    for pkg, exe, name, remaps in [
        ('nav2_controller', 'controller_server', 'controller_server',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', 'smoother_server', []),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server',
         [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower',
         []),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', 'cmd_vel_nav')]),
    ]:
        ld.add_action(Node(
            package=pkg, executable=exe, name=name, output='screen',
            respawn=use_respawn, respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remaps,
        ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                     'node_names': lifecycle_nodes}],
    ))

    ld.add_action(Node(
        package='robot_simple_commander', executable='safety_gate',
        name='safety_gate', output='screen',
        parameters=[configured_params],
    ))
    return ld
