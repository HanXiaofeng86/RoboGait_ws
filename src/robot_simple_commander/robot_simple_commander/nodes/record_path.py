"""Record robot poses while driving; save geometry yaml for follow_path.

Publishes a live trail so RViz can show sampled points as you drive:
  /recorded_path  nav_msgs/Path          (green line)
  /recorded_poses geometry_msgs/PoseArray
  /recorded_dots  visualization_msgs/Marker (green spheres)

    ros2 launch robot_simple_commander record_trajectory.launch.py
    # 2D Pose Estimate first, then PS4 pad; Ctrl+C here → save
"""

import math
from pathlib import Path as FsPath

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from robot_simple_commander.lib.path_io import save_path_yaml


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _yaw_to_quat(yaw):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


_LATCH = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class RecordPath(Node):

    def __init__(self):
        super().__init__('record_path')
        self.declare_parameter('output_file', '')
        self.declare_parameter('min_separation', 0.05)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('wait_for_initial_pose', True)

        self._poses = []
        self._last = None  # (x, y)
        self._wait_pose = bool(
            self.get_parameter('wait_for_initial_pose').value)
        self._tf_ok = False
        self._armed = not self._wait_pose
        self._pose_ready_time = None
        self._last_wait_log = self.get_clock().now()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._path_pub = self.create_publisher(NavPath, 'recorded_path', _LATCH)
        self._pose_pub = self.create_publisher(
            PoseArray, 'recorded_poses', _LATCH)
        self._dot_pub = self.create_publisher(Marker, 'recorded_dots', _LATCH)
        self._initial_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose',
            self._on_initial_pose, 10)

        period = 1.0 / max(1.0, float(self.get_parameter('rate_hz').value))
        self._timer = self.create_timer(period, self._tick)
        out = str(self.get_parameter('output_file').value)
        if self._wait_pose:
            self.get_logger().info(
                'Do not drive yet.\n'
                '  1. Wait for the red laser in RViz.\n'
                '  2. Toolbar → 2D Pose Estimate (green arrow).\n'
                '     Click-drag where the robot really is; laser should sit on walls.\n'
                '  3. Then drive with the PS4 pad. Green trail grows.\n'
                '  4. Ctrl+C here to save.\n'
                f'  file: {out}')
        else:
            self.get_logger().info(
                'Recording. Drive, then Ctrl+C to save.\n'
                f'  file: {out}\n'
                f'  min_separation={self.get_parameter("min_separation").value} m. '
                'Watch the green trail in RViz.')

    def _on_initial_pose(self, _msg):
        if not self._tf_ok:
            self.get_logger().warn(
                'Too early — wait for the red laser, then 2D Pose Estimate again.')
            return
        if self._armed:
            self.get_logger().info(
                '2D Pose Estimate again — AMCL jumped. Keep driving.')
            return
        self._armed = True
        self._pose_ready_time = self.get_clock().now() + Duration(seconds=1.0)
        self.get_logger().info(
            'Initial pose set. Drive with the PS4 pad. Watch the green trail.')

    def _tick(self):
        parent = self.get_parameter('parent_frame').value
        child = self.get_parameter('child_frame').value
        try:
            tf = self._tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            now = self.get_clock().now()
            if (now - self._last_wait_log).nanoseconds > 8e9:
                self.get_logger().info(
                    'Waiting for map→base_link (lidar + AMCL). '
                    'Do not click 2D Pose yet.')
                self._last_wait_log = now
            return

        if not self._tf_ok:
            self._tf_ok = True
            if self._wait_pose and not self._armed:
                self.get_logger().info(
                    'Localization is up. Now 2D Pose Estimate in RViz.')

        if not self._armed:
            now = self.get_clock().now()
            if (now - self._last_wait_log).nanoseconds > 8e9:
                self.get_logger().info(
                    'Still waiting for 2D Pose Estimate in RViz...')
                self._last_wait_log = now
            return
        if self._pose_ready_time is not None:
            if self.get_clock().now() < self._pose_ready_time:
                return
            self._pose_ready_time = None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = _quat_to_yaw(tf.transform.rotation)
        min_sep = float(self.get_parameter('min_separation').value)

        if self._last is not None:
            dx = x - self._last[0]
            dy = y - self._last[1]
            if (dx * dx + dy * dy) < (min_sep * min_sep):
                return

        self._last = (x, y)
        self._poses.append((x, y, yaw))
        self._publish_trail(parent)
        n = len(self._poses)
        if n == 1 or n <= 5 or n % 10 == 0:
            self.get_logger().info(
                f'Sampled {n} pose(s) — green trail in RViz')

    def _publish_trail(self, frame: str):
        now = self.get_clock().now().to_msg()
        path = NavPath()
        path.header.frame_id = frame
        path.header.stamp = now
        poses = PoseArray()
        poses.header = path.header
        for x, y, yaw in self._poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = _yaw_to_quat(yaw)
            path.poses.append(ps)
            pose = Pose()
            pose.position = ps.pose.position
            pose.orientation = ps.pose.orientation
            poses.poses.append(pose)

        dots = Marker()
        dots.header = path.header
        dots.ns = 'recorded'
        dots.id = 0
        dots.type = Marker.SPHERE_LIST
        dots.action = Marker.ADD
        dots.pose.orientation.w = 1.0
        dots.scale.x = 0.10
        dots.scale.y = 0.10
        dots.scale.z = 0.10
        dots.color.r = 0.15
        dots.color.g = 0.95
        dots.color.b = 0.25
        dots.color.a = 1.0
        dots.points = [
            Point(x=float(x), y=float(y), z=0.06) for x, y, _ in self._poses]

        self._path_pub.publish(path)
        self._pose_pub.publish(poses)
        self._dot_pub.publish(dots)

    def save(self) -> bool:
        out = self.get_parameter('output_file').value
        if not out:
            self.get_logger().error('output_file parameter is required')
            return False
        if not self._armed:
            self.get_logger().error(
                'Never got 2D Pose Estimate. Set the green arrow in RViz '
                'before driving, then Ctrl+C to save.')
            return False
        if len(self._poses) < 2:
            self.get_logger().error(
                f'Need at least 2 poses, got {len(self._poses)}. '
                'Drive until the green trail is a line, then Ctrl+C.')
            return False

        out_path = FsPath(str(out)).expanduser()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        frame = self.get_parameter('parent_frame').value
        save_path_yaml(str(out_path), self._poses, frame=frame)
        self.get_logger().info(
            f'Saved {len(self._poses)} poses → {out_path} (frame={frame})')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = RecordPath()
    ok = False
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        ok = node.save()
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
