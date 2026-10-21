#!/usr/bin/env python3
"""Follow trajectory (hardware + wait sensors + nav_stack/MPPI + follow_path).

Nav2 starts only after /scan and wheel odom exist (lidar RESET window).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_simple_commander.lib.route_name import follow_yaml
from robot_simple_commander.lib.route_name import load_route_name
from robot_simple_commander.lib.route_name import route_name_config


def _share(*parts: str) -> str:
    root = Path(get_package_share_directory('robot_simple_commander'))
    return str(root.joinpath(*parts))


def _lab_map() -> str:
    return str(
        Path(get_package_share_directory('robot_slam_mapping'))
        / 'maps' / 'mini_lab3.yaml')


def _resolve_path_yaml(context, *args, **kwargs):
    path_file = context.launch_configurations.get('path_file', '').strip()
    override = (
        context.launch_configurations.get('name', '').strip()
        or context.launch_configurations.get('path_name', '').strip())
    name = load_route_name(override)
    resolved = str(follow_yaml(name, path_file=path_file))
    return [
        LogInfo(msg=[
            'Follow → ', resolved, '  (edit ', str(route_name_config()), ')\n',
            'Robot does NOT start by itself.\n',
            'Wait ~15–20 s for lidar RESET + CAN. RViz opens after that.\n',
            'Do not Ctrl+C yet. When the panel says the stack is ready:\n',
            '2D Pose Estimate at the route start, then 开始.\n',
            'After it finishes, 回到起点 drives back along the same route.\n',
            '停止 cancels motion and leaves this launch running. Ctrl+C exits.',
        ]),
        Node(
            package='robot_simple_commander',
            executable='follow_path',
            name='robogait_follow_path',
            output='screen',
            parameters=[{
                'path_file': resolved,
                'wait_for_nav2': True,
                'wait_for_initial_pose': True,
                'wait_for_start': True,
                'controller_id': 'FollowPath',
                'goal_checker_id': 'general_goal_checker',
            }],
        ),
    ]


def generate_launch_description():
    parts_dir = _share('launch', 'parts')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'name', default_value='',
        description=(
            'Trajectory name (no .yaml). Empty → config/route_name.yaml. '
            'Looks in recorded/ first, then paths/')))
    ld.add_action(DeclareLaunchArgument(
        'path_name', default_value='',
        description='Same as name= (kept for old commands)'))
    ld.add_action(DeclareLaunchArgument(
        'path_file',
        default_value='',
        description='Absolute yaml; if set, ignores name'))
    ld.add_action(DeclareLaunchArgument(
        'amcl', default_value='True',
        description='True=AMCL so RViz 2D Pose Estimate can place the robot'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False'))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Open RViz (map + laser). False if no display.'))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=_lab_map(),
        description='Map yaml (default: current lab mini_lab3)'))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(parts_dir) / 'hardware.launch.py')),
    ))

    wait_sensors = Node(
        package='robot_mppi_navigation',
        executable='wait_for_nav_sensors',
        name='wait_for_nav_sensors',
        output='screen',
    )
    ld.add_action(wait_sensors)

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(parts_dir) / 'nav_stack.launch.py')),
        launch_arguments={
            'amcl': LaunchConfiguration('amcl'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(parts_dir) / 'rviz.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': _share('rviz', 'follow.rviz'),
        }.items(),
    )
    # Same as daily nav: Nav2/RViz after /scan + wheel odom, so RViz is not
    # stuck on missing map/odom during lidar RESET.
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=wait_sensors,
            on_exit=[GroupAction(actions=[nav_stack, rviz])],
        ),
    ))

    ld.add_action(OpaqueFunction(function=_resolve_path_yaml))
    return ld
