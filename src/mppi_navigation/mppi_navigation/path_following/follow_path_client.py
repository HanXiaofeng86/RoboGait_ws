"""Mode 2: send a predefined geometric path to Nav2 FollowPath (MPPI).

    ros2 run mppi_navigation follow_path --ros-args -p path_file:=<yaml>

The path yaml stores only geometry. If frame == 'local' the path is
transformed to the map frame at the robot's current pose before sending.
Velocities are always computed online by MPPI.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path

from mppi_navigation.path_following.path_io import load_path_yaml


def _yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FollowPathClient(Node):

    def __init__(self):
        super().__init__('follow_path_client')
        self.declare_parameter('path_file', '')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'general_goal_checker')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._path_pub = self.create_publisher(Path, 'predefined_path', 1)
        self._client = ActionClient(self, FollowPath, 'follow_path')

    def run(self) -> bool:
        path_file = self.get_parameter('path_file').value
        if not path_file:
            self.get_logger().error('path_file parameter is required')
            return False

        frame, poses = load_path_yaml(path_file)
        self.get_logger().info(f'Loaded {len(poses)} poses from {path_file} '
                               f'(frame={frame})')

        if frame == 'local':
            poses = self._to_map_frame(poses)
            if poses is None:
                return False

        path = self._build_path(poses)
        self._path_pub.publish(path)

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('follow_path action server not available')
            return False

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.get_parameter('controller_id').value
        goal.goal_checker_id = self.get_parameter('goal_checker_id').value

        self.get_logger().info('Sending FollowPath goal (MPPI computes '
                               'velocities online)...')
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('FollowPath goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        self.get_logger().info(f'FollowPath finished with status {status}')
        return status == 4  # STATUS_SUCCEEDED

    def _to_map_frame(self, poses):
        """Compose local path with the robot's current pose in map."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=5.0))
        except Exception as exc:  # tf2 raises several lookup error types
            self.get_logger().error(f'TF map->base_link unavailable: {exc}')
            return None

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        ryaw = _quat_to_yaw(tf.transform.rotation)
        c, s = math.cos(ryaw), math.sin(ryaw)
        return [(rx + x * c - y * s, ry + x * s + y * c, ryaw + yaw)
                for x, y, yaw in poses]

    def _build_path(self, poses):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
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
    node = FollowPathClient()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
