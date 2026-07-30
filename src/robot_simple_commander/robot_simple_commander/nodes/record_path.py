"""Record robot poses while driving; save geometry yaml for follow_path.

Use with this package's record_trajectory.launch.py (does not need robot_mppi_navigation).

    ros2 launch robot_simple_commander record_trajectory.launch.py
    # other terminal: ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # Ctrl+C on record terminal → saves to recorded/lab_route.yaml
"""

import math
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from robot_simple_commander.lib.path_io import save_path_yaml


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RecordPath(Node):

    def __init__(self):
        super().__init__('record_path')
        self.declare_parameter('output_file', '')
        self.declare_parameter('min_separation', 0.05)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('rate_hz', 20.0)

        self._poses = []
        self._last = None  # (x, y)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        period = 1.0 / max(1.0, float(self.get_parameter('rate_hz').value))
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            'Recording: drive the robot, Ctrl+C to save. '
            f"min_separation={self.get_parameter('min_separation').value} m")

    def _tick(self):
        parent = self.get_parameter('parent_frame').value
        child = self.get_parameter('child_frame').value
        try:
            tf = self._tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            return

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
        if len(self._poses) == 1 or len(self._poses) % 20 == 0:
            self.get_logger().info(f'Sampled {len(self._poses)} pose(s)')

    def save(self) -> bool:
        out = self.get_parameter('output_file').value
        if not out:
            self.get_logger().error('output_file parameter is required')
            return False
        if len(self._poses) < 2:
            self.get_logger().error(
                f'Need at least 2 poses, got {len(self._poses)}')
            return False

        path = Path(out).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        frame = self.get_parameter('parent_frame').value
        save_path_yaml(str(path), self._poses, frame=frame)
        self.get_logger().info(
            f'Saved {len(self._poses)} poses → {path} (frame={frame})')
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
