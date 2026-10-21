#!/usr/bin/env python3
"""B0/B2 person front-end: nuitrack (skeleton) or sdk (point cloud).

Gait / gimbal tracking uses Nuitrack (the method that actually locked a
torso on this NUC). The K4A SDK path is for the teacher point-cloud demo.
Do not start both — they open the same USB camera.

  # Tracking (needs overlay; see check below)
  source ~/rover_workspace_final/install/setup.bash
  source ~/RoboGait_ws/install/setup.bash
  ros2 launch robogait_perception person_source.launch.py person_source:=nuitrack

  # Point cloud only (no Nuitrack, body tracking off)
  ros2 launch robogait_perception person_source.launch.py person_source:=sdk
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NUITRACK_OVERLAY_HINT = (
    'orbbec_person_publisher (Nuitrack) not found. Overlay the old workspace, '
    'then this one, and do not start person_sdk at the same time:\n'
    '  source ~/rover_workspace_final/install/setup.bash\n'
    '  source ~/RoboGait_ws/install/setup.bash\n'
    '  ros2 launch robogait_gimbal test_gimbal.launch.py'
)


def nuitrack_share_or_none():
    try:
        return get_package_share_directory('orbbec_person_publisher')
    except PackageNotFoundError:
        return None


def _check_person_source(context, *args, **kwargs):
    source = LaunchConfiguration('person_source').perform(context)
    if source == 'nuitrack':
        if nuitrack_share_or_none() is None:
            raise RuntimeError(NUITRACK_OVERLAY_HINT)
        return [LogInfo(msg=(
            'Person source: Nuitrack (torso). Static TF orbbec_link → '
            'rgbd_camera_link_optical. Do not also launch person_sdk — '
            'same USB camera.'))]
    return [LogInfo(msg=(
        'Person source: SDK point cloud (k4abt body off by default). '
        'For gait/gimbal skeleton use person_source:=nuitrack.'))]


def generate_launch_description():
    share = get_package_share_directory('robogait_perception')
    perception_cfg = os.path.join(share, 'config', 'perception.yaml')
    rviz_sdk = os.path.join(share, 'rviz', 'orbbec_sdk.rviz')
    rviz_nuitrack = os.path.join(share, 'rviz', 'orbbec_nuitrack.rviz')
    wrapper_lib = os.path.expanduser('~/orbbec_wrapper_v2/lib')
    nuitrack_lib = '/usr/local/lib/nuitrack'
    mini_xacro = os.path.join(share, 'urdf', 'mini_with_orbbec.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', mini_xacro]), value_type=str)

    is_sdk = PythonExpression(["'", LaunchConfiguration('person_source'), "' == 'sdk'"])
    is_nuitrack = PythonExpression(["'", LaunchConfiguration('person_source'), "' == 'nuitrack'"])
    use_rviz_sdk = PythonExpression([
        "'", LaunchConfiguration('use_rviz'), "'.lower() in ('true', '1') and '",
        LaunchConfiguration('person_source'), "' == 'sdk'",
    ])
    use_rviz_nuitrack = PythonExpression([
        "'", LaunchConfiguration('use_rviz'), "'.lower() in ('true', '1') and '",
        LaunchConfiguration('person_source'), "' == 'nuitrack'",
    ])
    ld_sdk = wrapper_lib + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    ld_nuitrack = nuitrack_lib + ':' + os.environ.get('LD_LIBRARY_PATH', '')

    return LaunchDescription([
        DeclareLaunchArgument(
            'person_source',
            default_value='sdk',
            description=(
                "'nuitrack' = skeleton (gimbal/gait); "
                "'sdk' = Femto Mega K4A point cloud (no body by default)"),
        ),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument(
            'use_robot_model',
            default_value='True',
            description='Publish Mini URDF (chassis + camera). Set False if Nav2 already did.',
        ),
        DeclareLaunchArgument(
            'publish_static_tf',
            default_value='False',
            description='If True, person_sdk publishes payload→orbbec_link (use when Nav2 already has the Mini URDF).',
        ),
        OpaqueFunction(function=_check_person_source),

        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=ld_sdk,
            condition=IfCondition(is_sdk),
        ),
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=ld_nuitrack,
            condition=IfCondition(is_nuitrack),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                # Gimbal pan/tilt must come from dynamixel_driver, not JSP zeros.
                'source_list': ['gimbal_joint_states'],
                'rate': 20,
            }],
            condition=IfCondition(LaunchConfiguration('use_robot_model')),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(LaunchConfiguration('use_robot_model')),
        ),
        Node(
            package='robogait_perception',
            executable='person_sdk',
            name='person_sdk',
            output='screen',
            parameters=[
                perception_cfg,
                {'publish_static_tf': LaunchConfiguration('publish_static_tf')},
            ],
            condition=IfCondition(is_sdk),
        ),
        Node(
            package='orbbec_person_publisher',
            executable='nuitrack_person_node',
            name='nuitrack_person_node',
            output='screen',
            parameters=[{'image_width': 640, 'save_csv': False}],
            condition=IfCondition(is_nuitrack),
        ),
        # Nuitrack cloud lives in rgbd_camera_link_optical (X right, Y down,
        # Z out of the lens). Mini URDF orbbec_link is camera_link (X forward,
        # Z up). Nuitrack does not publish this TF — identity would map optical
        # Z onto orbbec_link Z and the cloud flies into the sky.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='orbbec_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                '--frame-id', 'orbbec_link',
                '--child-frame-id', 'rgbd_camera_link_optical',
            ],
            output='screen',
            condition=IfCondition(is_nuitrack),
        ),
        Node(
            package='robogait_perception',
            executable='distance_orbbec',
            name='distance_orbbec',
            output='screen',
            parameters=[perception_cfg],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='orbbec_rviz',
            arguments=['-d', rviz_sdk],
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'},
            condition=IfCondition(use_rviz_sdk),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='orbbec_rviz',
            arguments=['-d', rviz_nuitrack],
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'},
            condition=IfCondition(use_rviz_nuitrack),
        ),
    ])
