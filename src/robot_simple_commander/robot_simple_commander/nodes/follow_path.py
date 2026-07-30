"""Follow a geometric path yaml via nav2_simple_commander.BasicNavigator.

Requires an already-running Nav2 stack (e.g. robot_mppi_navigation bringup).
Path yaml stores geometry only; MPPI computes velocities online.

    ros2 run robot_simple_commander follow_path --ros-args \\
      -p path_file:=/path/to/route.yaml
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

from robot_simple_commander.lib.path_io import load_path_yaml


def _yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _to_map_frame(navigator, tf_buffer, poses):
    try:
        tf = tf_buffer.lookup_transform(
            'map', 'base_link', rclpy.time.Time(),
            timeout=Duration(seconds=5.0))
    except Exception as exc:
        navigator.get_logger().error(f'TF map->base_link unavailable: {exc}')
        return None

    rx = tf.transform.translation.x
    ry = tf.transform.translation.y
    ryaw = _quat_to_yaw(tf.transform.rotation)
    c, s = math.cos(ryaw), math.sin(ryaw)
    return [(rx + x * c - y * s, ry + x * s + y * c, ryaw + yaw)
            for x, y, yaw in poses]


def _build_path(navigator, poses):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = navigator.get_clock().now().to_msg()
    for x, y, yaw in poses:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        path.poses.append(ps)
    return path


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator(node_name='robogait_follow_path')

    navigator.declare_parameter('path_file', '')
    navigator.declare_parameter('controller_id', 'FollowPath')
    navigator.declare_parameter('goal_checker_id', 'general_goal_checker')
    navigator.declare_parameter('wait_for_nav2', True)

    path_file = navigator.get_parameter('path_file').value
    if not path_file:
        navigator.get_logger().error('path_file parameter is required')
        navigator.destroy_node()
        rclpy.try_shutdown()
        raise SystemExit(1)

    frame, poses = load_path_yaml(path_file)
    navigator.get_logger().info(
        f'Loaded {len(poses)} poses from {path_file} (frame={frame})')

    tf_buffer = Buffer()
    TransformListener(tf_buffer, navigator)

    if frame == 'local':
        # Give TF listener a moment to fill.
        for _ in range(50):
            rclpy.spin_once(navigator, timeout_sec=0.1)
        poses = _to_map_frame(navigator, tf_buffer, poses)
        if poses is None:
            navigator.destroy_node()
            rclpy.try_shutdown()
            raise SystemExit(1)

    path = _build_path(navigator, poses)

    path_pub = navigator.create_publisher(Path, 'predefined_path', 1)
    path_pub.publish(path)

    if navigator.get_parameter('wait_for_nav2').value:
        navigator.get_logger().info('Waiting for Nav2 to become active...')
        navigator.waitUntilNav2Active()

    controller_id = navigator.get_parameter('controller_id').value
    goal_checker_id = navigator.get_parameter('goal_checker_id').value
    navigator.get_logger().info(
        'Sending FollowPath via BasicNavigator (MPPI online)...')
    navigator.followPath(path, controller_id, goal_checker_id)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback is not None and i % 20 == 0:
            navigator.get_logger().info(
                f'Distance to goal: {feedback.distance_to_goal:.2f} m')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('FollowPath succeeded')
        code = 0
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('FollowPath canceled')
        code = 1
    else:
        navigator.get_logger().error(f'FollowPath failed ({result})')
        code = 1

    navigator.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
