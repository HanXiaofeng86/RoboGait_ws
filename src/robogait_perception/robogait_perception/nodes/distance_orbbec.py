#!/usr/bin/env python3
"""Bridge Orbbec person topics into filtered person messages for Mode B."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from robogait_person_interfaces.msg import User
from std_msgs.msg import Int32


class DistanceOrbbec(Node):
    """Subscribe to camera person topics; publish user_raw + person_center_pixel."""

    def __init__(self) -> None:
        super().__init__('distance_orbbec')
        self.declare_parameter('person_position_topic', '/orbbec/person_position')
        self.declare_parameter('person_center_pixel_topic', '/orbbec/person_center_pixel')
        self.declare_parameter('num_users_topic', '/orbbec/num_users')
        self.declare_parameter('default_center_pixel', 320)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('min_z_m', 0.15)
        self.declare_parameter('max_z_m', 15.0)
        self.declare_parameter('hold_last_valid_sec', 0.4)
        self.declare_parameter('publish_period_sec', 0.1)

        pos_topic = self.get_parameter('person_position_topic').value
        pixel_topic = self.get_parameter('person_center_pixel_topic').value
        num_topic = self.get_parameter('num_users_topic').value
        self._default_pixel = int(self.get_parameter('default_center_pixel').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)
        self._min_z = float(self.get_parameter('min_z_m').value)
        self._max_z = float(self.get_parameter('max_z_m').value)
        self._hold_sec = float(self.get_parameter('hold_last_valid_sec').value)

        self._user_pub = self.create_publisher(User, 'user_raw', 10)
        self._pixel_pub = self.create_publisher(Int32, 'person_center_pixel', 10)

        self.create_subscription(Point, pos_topic, self._on_position, 10)
        self.create_subscription(Int32, pixel_topic, self._on_pixel, 10)
        self.create_subscription(Int32, num_topic, self._on_num_users, 10)

        self._last_pos_time = self.get_clock().now()
        self._position: tuple[float, float, float] | None = None
        self._center_pixel: int | None = None
        self._num_users = 0
        self._last_valid: User | None = None
        self._last_valid_pixel = self._default_pixel
        self._last_valid_time = self.get_clock().now()

        period = float(self.get_parameter('publish_period_sec').value)
        self.create_timer(period, self._on_timer)

    def _on_position(self, msg: Point) -> None:
        self._last_pos_time = self.get_clock().now()
        self._position = (msg.x, msg.y, msg.z)
        self._publish()

    def _on_pixel(self, msg: Int32) -> None:
        self._center_pixel = msg.data
        if self._position is not None:
            self._publish()

    def _on_num_users(self, msg: Int32) -> None:
        self._num_users = msg.data

    def _on_timer(self) -> None:
        # Only for lost-person timeout; new detections publish from _on_position.
        elapsed = (self.get_clock().now() - self._last_pos_time).nanoseconds / 1e9
        if elapsed >= self._timeout:
            self._publish()

    def _publish(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._last_pos_time).nanoseconds / 1e9

        user = User()
        pixel = Int32()

        if self._num_users >= 2:
            user.detection = 0
            pixel.data = 0
        elif self._position is not None and elapsed < self._timeout:
            x, y, z = self._position
            if self._min_z <= z <= self._max_z:
                user.detection = 1
                user.x, user.y, user.z = x, y, z
                # Depth lock must still yaw: never publish pixel=0 while z>0.
                if self._center_pixel is not None and self._center_pixel > 0:
                    pixel.data = self._center_pixel
                else:
                    pixel.data = self._default_pixel
                self._last_valid = user
                self._last_valid_pixel = pixel.data
                self._last_valid_time = now
                # Nuitrack torso has y (up); SDK depth-blob zeros y.
                joint = 'torso' if abs(y) > 1e-6 else 'depth'
                self.get_logger().info(
                    f'Person locked joint={joint} pixel={pixel.data} z={z:.2f} m',
                    throttle_duration_sec=2.0)
            else:
                user.detection = 0
                pixel.data = 0
        elif (
            self._last_valid is not None
            and (now - self._last_valid_time).nanoseconds / 1e9 < self._hold_sec
        ):
            user = self._last_valid
            user.detection = 1
            pixel.data = self._last_valid_pixel
        else:
            user.detection = 0
            pixel.data = 0

        self._user_pub.publish(user)
        self._pixel_pub.publish(pixel)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistanceOrbbec()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
