"""Straight person follow: distance only, no turn, no Nav2.

Person stands in front of the frozen camera. z is forward distance (m).
Farther than the band → drive forward. Closer → reverse slowly. Inside the
band, lost, or farther than max_distance_m → stop. angular.z is always 0.

Publishes cmd_vel_smoothed so safety_gate remains the only /cmd_vel writer.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from robogait_person_interfaces.msg import User


def straight_speed(
    distance_m: float,
    distance_min: float,
    distance_max: float,
    max_distance_m: float,
    kp: float,
    max_forward: float,
    max_reverse: float,
) -> float:
    """Forward speed (m/s). Positive drives toward a person who is too far."""
    if distance_m <= 0.05 or distance_m > max_distance_m:
        return 0.0
    if distance_m > distance_max:
        raw = kp * (distance_m - distance_max)
        return min(max_forward, raw)
    if distance_m < distance_min:
        raw = kp * (distance_m - distance_min)
        return max(-max_reverse, raw)
    return 0.0


class StraightFollow(Node):
    """Hold a distance band while driving only forward/back."""

    def __init__(self) -> None:
        super().__init__('straight_follow')
        self.declare_parameter('target_distance_min', 3.0)
        self.declare_parameter('target_distance_max', 3.1)
        self.declare_parameter('max_distance_m', 5.0)
        self.declare_parameter('lost_timeout_sec', 1.2)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('max_forward_m_s', 0.70)
        self.declare_parameter('max_reverse_m_s', 0.30)
        self.declare_parameter('max_accel_m_s2', 0.70)
        self.declare_parameter('brake_accel_m_s2', 1.2)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('cmd_topic', 'cmd_vel_smoothed')
        self.declare_parameter('user_topic', 'user_coordinates')

        self._dmin = float(self.get_parameter('target_distance_min').value)
        self._dmax = float(self.get_parameter('target_distance_max').value)
        self._dstop = float(self.get_parameter('max_distance_m').value)
        self._lost = float(self.get_parameter('lost_timeout_sec').value)
        self._kp = float(self.get_parameter('kp').value)
        self._vmax = float(self.get_parameter('max_forward_m_s').value)
        self._vmin = float(self.get_parameter('max_reverse_m_s').value)
        self._accel = float(self.get_parameter('max_accel_m_s2').value)
        self._brake = float(self.get_parameter('brake_accel_m_s2').value)
        hz = float(self.get_parameter('publish_hz').value)
        self._dt = 1.0 / hz if hz > 1.0 else 0.05

        self._distance: float | None = None
        self._last_seen = self.get_clock().now()
        self._seen = False
        self._cmd = 0.0

        self._pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10)
        self.create_subscription(
            User, str(self.get_parameter('user_topic').value), self._on_user, 10)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'Straight follow: band {self._dmin:.2f}–{self._dmax:.2f} m, '
            f'forward ≤ {self._vmax:.2f} m/s, reverse ≤ {self._vmin:.2f} m/s, '
            f'angular = 0 → {self.get_parameter("cmd_topic").value}')

    def _on_user(self, msg: User) -> None:
        if msg.detection > 0 and msg.z > 0.05:
            self._distance = float(msg.z)
            self._last_seen = self.get_clock().now()
            self._seen = True

    def _desired(self) -> float:
        if not self._seen or self._distance is None:
            return 0.0
        age = (self.get_clock().now() - self._last_seen).nanoseconds / 1e9
        if age > self._lost:
            return 0.0
        return straight_speed(
            self._distance, self._dmin, self._dmax, self._dstop,
            self._kp, self._vmax, self._vmin)

    def _tick(self) -> None:
        desired = self._desired()
        delta = desired - self._cmd
        slowing = abs(desired) < abs(self._cmd)
        limit = (self._brake if slowing else self._accel) * self._dt
        if delta > limit:
            delta = limit
        elif delta < -limit:
            delta = -limit
        self._cmd += delta
        if abs(self._cmd) < 0.02 and abs(desired) < 0.02:
            self._cmd = 0.0

        out = Twist()
        out.linear.x = self._cmd
        out.angular.z = 0.0
        self._pub.publish(out)

        dist = self._distance if self._distance is not None else -1.0
        self.get_logger().info(
            f'distance={dist:.2f} m cmd={self._cmd:.2f} m/s',
            throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
