#!/usr/bin/env python3
"""Navigation layer only (no hardware, no RViz): EKF + localization + Nav2/MPPI + velocity chain + safety gate.

Usually Included by rviz_goal_navigation.launch.py — do not start a second copy beside hardware.

    controller_server (MPPI) -> cmd_vel_nav
    velocity_smoother         -> cmd_vel_smoothed
    safety_gate (pass/zero)   -> cmd_vel -> rover driver

Localization (choose one):
    slam:=True, use_external_slam_params:=False
        → nav2 slam_launch (slam_toolbox block inside params_file)
    slam:=True, use_external_slam_params:=True, slam_params_file:=...
        → async_slam_toolbox_node with that YAML (e.g. robot_slam_mapping)
    slam:=False amcl:=True     map_server + AMCL on a known map (default)
    slam:=False amcl:=False    map_server + static identity map->odom
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _pkg_config(name: str) -> str:
    return str(Path(get_package_share_directory('robot_mppi_navigation')) / 'config' / name)


def _nav_to_pose_bt() -> str:
    return str(
        Path(get_package_share_directory('robot_mppi_navigation'))
        / 'behavior_trees' / 'navigate_to_pose_real_robot.xml'
    )


def _default_map_file() -> str:
    return str(
        Path(get_package_share_directory('robot_mppi_navigation')) / 'maps' / 'mini_slab1.yaml'
    )


def generate_launch_description():
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    slam_launch_path = os.path.join(nav2_bringup_share, 'launch', 'slam_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    slam = LaunchConfiguration('slam')
    amcl = LaunchConfiguration('amcl')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_external_slam_params = LaunchConfiguration('use_external_slam_params')
    slam_params_file = LaunchConfiguration('slam_params_file')

    use_builtin_slam = PythonExpression(
        ["'", slam, "' == 'True' and '", use_external_slam_params, "' != 'True'"])
    # External: slam_toolbox params owned by another package (e.g. robot_slam_mapping)
    use_external_slam = PythonExpression(
        ["'", slam, "' == 'True' and '", use_external_slam_params, "' == 'True'"])
    use_map_localization = PythonExpression(["'", slam, "' != 'True'"])
    use_amcl = PythonExpression(
        ["'", slam, "' != 'True' and '", amcl, "' == 'True'"])
    use_static_tf = PythonExpression(
        ["'", slam, "' != 'True' and '", amcl, "' != 'True'"])

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # Jazzy launch_ros only loads extensionless RewrittenYaml temps via ParameterFile.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack'))
    ld.add_action(DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Respawn nodes if they crash'))
    ld.add_action(DeclareLaunchArgument(
        'log_level', default_value='info',
        description='ROS log level'))
    ld.add_action(DeclareLaunchArgument(
        'slam', default_value='False',
        description='True=online SLAM; False=map_server localization'))
    ld.add_action(DeclareLaunchArgument(
        'amcl', default_value='True',
        description='When slam:=False: True=AMCL map->odom; '
                    'False=static identity map->odom'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=_pkg_config('nav2_pure_mppi_params.yaml'),
        description='Nav2 parameters file (MPPI, costmaps, smoother, safety gate)'))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=_default_map_file(),
        description='Full path to map yaml (used when slam:=False)'))
    ld.add_action(DeclareLaunchArgument(
        'use_external_slam_params', default_value='False',
        description='True=start slam_toolbox with slam_params_file '
                    '(mapping package); False=nav2 slam_launch + params_file'))
    ld.add_action(DeclareLaunchArgument(
        'slam_params_file', default_value='',
        description='slam_toolbox YAML when use_external_slam_params:=True'))

    # ---------- EKF (odom->base_link, /odometry/filtered) ----------
    ld.add_action(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[_pkg_config('robot_localization_ekf.yaml'),
                        {'use_sim_time': use_sim_time}],
        )
    )

    # ---------- Localization path A1: Nav2 slam_launch (params_file) ----------
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            condition=IfCondition(use_builtin_slam),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_respawn': use_respawn,
                'log_level': log_level,
                'params_file': params_file,
            }.items(),
        )
    )

    # ---------- Localization path A2: external slam_toolbox YAML ----------
    external_slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        condition=IfCondition(use_external_slam),
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'use_lifecycle_manager': False,
            },
        ],
    )
    ld.add_action(external_slam)
    ld.add_action(EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(external_slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(use_external_slam),
    ))
    ld.add_action(RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=external_slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='slam_toolbox configured, activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(external_slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(use_external_slam),
    ))
    ld.add_action(
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver',
            output='screen',
            condition=IfCondition(use_external_slam),
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        )
    )
    ld.add_action(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            condition=IfCondition(use_external_slam),
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_saver'],
            }],
        )
    )

    # ---------- Localization path B: map_server + identity map->odom ----------
    ld.add_action(
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            condition=IfCondition(use_map_localization),
            parameters=[{'use_sim_time': use_sim_time,
                         'yaml_filename': map_yaml_file}],
        )
    )
    # AMCL provides the real map->odom transform when amcl:=True.
    ld.add_action(
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            condition=IfCondition(use_amcl),
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        )
    )
    ld.add_action(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            condition=IfCondition(use_amcl),
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'node_names': ['map_server', 'amcl']}],
        )
    )
    ld.add_action(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            condition=IfCondition(use_static_tf),
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'node_names': ['map_server']}],
        )
    )
    # Temporary identity TF when neither SLAM nor AMCL is running.
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            output='screen',
            condition=IfCondition(use_static_tf),
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'map', '--child-frame-id', 'odom'],
        )
    )

    # ---------- Nav2 backend with explicit cmd_vel chain ----------
    # MPPI output renamed to cmd_vel_nav so nothing else writes /cmd_vel.
    ld.add_action(
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        )
    )
    ld.add_action(
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        )
    )
    ld.add_action(
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        )
    )
    ld.add_action(
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            # Recovery behaviors also go through smoother + safety gate.
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        )
    )
    ld.add_action(
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, {
                'default_nav_to_pose_bt_xml': _nav_to_pose_bt(),
            }],
            arguments=['--ros-args', '--log-level', log_level],
        )
    )
    ld.add_action(
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
        )
    )
    # Velocity smoother: cmd_vel_nav -> cmd_vel_smoothed
    ld.add_action(
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        )
    )
    ld.add_action(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'node_names': lifecycle_nodes}],
        )
    )

    # Safety gate: cmd_vel_smoothed -> cmd_vel (pass unchanged or zero).
    ld.add_action(
        Node(
            package='robot_mppi_navigation',
            executable='safety_gate',
            name='safety_gate',
            output='screen',
            parameters=[configured_params],
        )
    )

    return ld
