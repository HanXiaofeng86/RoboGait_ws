"""PID tracker: person pixel offset -> gimbal target angle (degrees)."""

from __future__ import annotations

from collections import deque

import rclpy
from rclpy.node import Node
from robogait_person_interfaces.msg import User
from std_msgs.msg import Float32, Int32


class CameraTracker(Node):
    """Map horizontal pixel error to a pan angle command for the Dynamixel driver."""

    def __init__(self) -> None:
        super().__init__('camera_tracker')
        self.declare_parameter('kp', 0.52)
        self.declare_parameter('kd', 0.08)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('max_integral', 1.0)
        self.declare_parameter('filter_window', 7)
        self.declare_parameter('image_center', 320.0)
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('hfov_deg', 69.0)
        self.declare_parameter('dead_zone_deg', 2.5)
        self.declare_parameter('dead_zone_outer_deg', 5.0)

        self._kp = float(self.get_parameter('kp').value)
        self._kd = float(self.get_parameter('kd').value)
        self._ki = float(self.get_parameter('ki').value)
        self._max_i = float(self.get_parameter('max_integral').value)
        self._window = max(1, int(self.get_parameter('filter_window').value))
        self._center = float(self.get_parameter('image_center').value)
        self._width = float(self.get_parameter('image_width').value)
        self._hfov = float(self.get_parameter('hfov_deg').value)
        self._dead_inner = float(self.get_parameter('dead_zone_deg').value)
        self._dead_outer = float(self.get_parameter('dead_zone_outer_deg').value)

        self._pixels: deque[float] = deque(maxlen=self._window)
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = self.get_clock().now()
        self._person = User()

        self._angle_pub = self.create_publisher(Float32, 'camera_position', 10)
        self.create_subscription(User, 'user_coordinates', self._on_user, 10)
        self.create_subscription(Int32, 'person_center_pixel', self._on_pixel, 10)

    def _on_user(self, msg: User) -> None:
        self._person = msg

    def _on_pixel(self, msg: Int32) -> None:
        if msg.data <= 0 or self._person.detection == 0:
            self._integral = 0.0
            self._angle_pub.publish(Float32(data=0.0))
            self.get_logger().warn('Person not detected — gimbal hold', throttle_duration_sec=5.0)
            return

        self._pixels.append(float(msg.data))
        center = sum(self._pixels) / len(self._pixels)
        error_deg = (self._center - center) * (self._hfov / self._width if self._width > 0 else 0.0)

        now = self.get_clock().now()
        dt = max((now - self._prev_time).nanoseconds / 1e9, 1e-3)
        self._prev_time = now

        err_p = error_deg
        err_d = (err_p - self._prev_error) / dt
        self._integral = max(0.0, min(self._max_i, self._integral + err_p * dt))
        pid = self._kp * err_p + self._ki * self._integral + self._kd * err_d
        pid = max(-90.0, min(90.0, pid))
        self._prev_error = err_p

        out = Float32()
        if abs(pid) <= self._dead_inner:
            self._integral = 0.0
            out.data = 0.0
        elif abs(pid) >= self._dead_outer:
            out.data = float(pid)
        else:
            self._integral = 0.0
            out.data = 0.0
        self._angle_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
