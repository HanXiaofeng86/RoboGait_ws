"""PID tracker: keep the person in the Femto Mega FOV (pan + tilt).

Pan uses horizontal pixel error (pelvis X, or depth-blob X if no skeleton).
Tilt uses camera-frame elevation atan2(y, z) so we do not need a vertical pixel.

Gait: lock the pelvis on the optical axis (target_elevation_deg = 0). Tilt is
heavily filtered with a large dead zone so walking bob does not nod the camera.
No skeleton yet (joint=depth, pos.y=0): still pan so the person stays in Color
FOV; tilt stays at idle until pelvis locks. freeze_gimbal holds idle for
straight chassis-follow tests.
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from rclpy.node import Node
from robogait_person_interfaces.msg import User
from std_msgs.msg import Float32, Int32


class CameraTracker(Node):
    """Publish pan/tilt degree errors for dynamixel_driver."""

    def __init__(self) -> None:
        super().__init__('camera_tracker')
        self.declare_parameter('user_topic', 'user_raw')
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('max_integral', 1.0)
        self.declare_parameter('tilt_kp', 0.22)
        self.declare_parameter('tilt_kd', 0.0)
        self.declare_parameter('tilt_ki', 0.0)
        self.declare_parameter('filter_window', 3)
        self.declare_parameter('tilt_filter_window', 18)
        self.declare_parameter('image_center', 320.0)
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('hfov_deg', 69.0)
        self.declare_parameter('dead_zone_deg', 1.5)
        self.declare_parameter('dead_zone_outer_deg', 2.0)
        self.declare_parameter('tilt_dead_zone_deg', 5.0)
        self.declare_parameter('target_elevation_deg', 0.0)
        self.declare_parameter('max_tilt_cmd_deg', 20.0)
        self.declare_parameter('lock_joint', 'pelvis')
        self.declare_parameter('track_enable', True)
        self.declare_parameter('freeze_gimbal', False)
        self.declare_parameter('enable_gimbal_pan', True)
        self.declare_parameter('enable_gimbal_tilt', True)
        self.declare_parameter('min_track_z_m', 1.0)

        self._kp = float(self.get_parameter('kp').value)
        self._kd = float(self.get_parameter('kd').value)
        self._ki = float(self.get_parameter('ki').value)
        self._tilt_kp = float(self.get_parameter('tilt_kp').value)
        self._tilt_kd = float(self.get_parameter('tilt_kd').value)
        self._tilt_ki = float(self.get_parameter('tilt_ki').value)
        self._max_i = float(self.get_parameter('max_integral').value)
        self._window = max(1, int(self.get_parameter('filter_window').value))
        self._tilt_window = max(1, int(self.get_parameter('tilt_filter_window').value))
        self._center = float(self.get_parameter('image_center').value)
        self._width = float(self.get_parameter('image_width').value)
        self._hfov = float(self.get_parameter('hfov_deg').value)
        self._dead_inner = float(self.get_parameter('dead_zone_deg').value)
        self._dead_outer = float(self.get_parameter('dead_zone_outer_deg').value)
        self._tilt_dead = float(self.get_parameter('tilt_dead_zone_deg').value)
        self._target_el = float(self.get_parameter('target_elevation_deg').value)
        self._max_tilt_cmd = max(1.0, float(self.get_parameter('max_tilt_cmd_deg').value))
        self._lock_joint = str(self.get_parameter('lock_joint').value)
        self._track_enable = bool(self.get_parameter('track_enable').value)
        self._freeze = bool(self.get_parameter('freeze_gimbal').value)
        self._enable_pan = bool(self.get_parameter('enable_gimbal_pan').value)
        self._enable_tilt = bool(self.get_parameter('enable_gimbal_tilt').value)
        self._min_track_z = max(0.15, float(self.get_parameter('min_track_z_m').value))

        self._pixels: deque[float] = deque(maxlen=self._window)
        self._elev: deque[float] = deque(maxlen=self._tilt_window)
        self._integral = 0.0
        self._tilt_integral = 0.0
        self._prev_error = 0.0
        self._prev_tilt_error = 0.0
        self._prev_time = self.get_clock().now()
        self._person = User()
        self._last_pixel = 0

        self._pan_pub = self.create_publisher(Float32, 'camera_position', 10)
        self._tilt_pub = self.create_publisher(Float32, 'camera_tilt', 10)
        user_topic = str(self.get_parameter('user_topic').value)
        self.create_subscription(User, user_topic, self._on_user, 10)
        self.create_subscription(Int32, 'person_center_pixel', self._on_pixel, 10)

        if self._freeze or not self._track_enable:
            self.get_logger().info(
                'Gimbal frozen (freeze_gimbal / track_enable) — servos stay at idle')
        else:
            self.get_logger().info(
                f'Tracker lock={self._lock_joint} pan={self._enable_pan} '
                f'tilt={self._enable_tilt} target_el={self._target_el:.1f}° '
                f'tilt_kp={self._tilt_kp:.2f} tilt_dead={self._tilt_dead:.1f}° '
                f'min_z={self._min_track_z:.2f} m')

    def _pid(
        self, error: float, prev: float, integral: float,
        kp: float, kd: float, ki: float, dt: float,
    ) -> tuple[float, float, float]:
        err_d = (error - prev) / dt
        integral = max(-self._max_i, min(self._max_i, integral + error * dt))
        pid = kp * error + ki * integral + kd * err_d
        pid = max(-90.0, min(90.0, pid))
        return pid, error, integral

    def _apply_deadzone(
        self, pid: float, integral: float, inner: float,
    ) -> tuple[float, float]:
        if abs(pid) <= inner:
            return 0.0, 0.0
        return float(pid), integral

    def _on_user(self, msg: User) -> None:
        self._person = msg
        # Pixel often arrives before detection=1; run pan as soon as user_raw is valid.
        if msg.detection == 1 and msg.z >= self._min_track_z:
            self._track(self._last_pixel)

    def _pixel_from_user(self, raw_pixel: int) -> float:
        """Horizontal image x. Depth lock may omit pixel; then use camera-frame X."""
        if raw_pixel > 0:
            return float(raw_pixel)
        if self._person.z > 0.15 and self._hfov > 0.0:
            az = math.degrees(math.atan2(self._person.x, self._person.z))
            return self._center + az * (self._width / self._hfov)
        return 0.0

    def _on_pixel(self, msg: Int32) -> None:
        self._last_pixel = int(msg.data)
        self._track(self._last_pixel)

    def _track(self, raw_pixel: int) -> None:
        now = self.get_clock().now()
        dt = max((now - self._prev_time).nanoseconds / 1e9, 1e-3)
        self._prev_time = now

        too_close = (
            self._person.detection == 1
            and 0.15 < self._person.z < self._min_track_z)
        has_person = self._person.detection == 1 and self._person.z >= self._min_track_z
        pixel = self._pixel_from_user(raw_pixel) if has_person else 0.0
        if not has_person or pixel <= 0.0:
            self._integral = 0.0
            self._tilt_integral = 0.0
            self._pixels.clear()
            self._elev.clear()
            if too_close:
                self.get_logger().info(
                    f'Person too close z={self._person.z:.2f} m '
                    f'(need ≥{self._min_track_z:.1f} m) — gimbal holds',
                    throttle_duration_sec=4.0)
            else:
                self.get_logger().info(
                    'Waiting for person — gimbal stays at idle angle',
                    throttle_duration_sec=8.0)
            return

        if self._freeze or not self._track_enable:
            self.get_logger().info(
                'Person seen but gimbal frozen (straight-follow / idle lock)',
                throttle_duration_sec=8.0)
            return

        # Depth-blob fallback zeros Y: pan only (keep Color FOV). Tilt waits for pelvis.
        from_depth_blob = abs(self._person.y) <= 1e-6

        if self._enable_pan:
            self._pixels.append(pixel)
            center = sum(self._pixels) / len(self._pixels)
            pan_err = (self._center - center) * (
                self._hfov / self._width if self._width > 0 else 0.0)
            pan_pid, self._prev_error, self._integral = self._pid(
                pan_err, self._prev_error, self._integral,
                self._kp, self._kd, self._ki, dt)
            pan_out, self._integral = self._apply_deadzone(
                pan_pid, self._integral, self._dead_inner)
            self._pan_pub.publish(Float32(data=pan_out))
            if abs(pan_out) > 0.05:
                self.get_logger().info(
                    f'Pan {"depth" if from_depth_blob else "skeleton"} '
                    f'pixel={center:.0f} err={pan_err:.1f}° cmd={pan_out:.1f}°',
                    throttle_duration_sec=2.0)

        if self._enable_tilt and not from_depth_blob:
            elev = math.degrees(math.atan2(self._person.y, self._person.z))
            self._elev.append(elev)
            elev_f = sum(self._elev) / len(self._elev)
            # Positive: person is above the optical axis → look up.
            tilt_err = elev_f - self._target_el
            tilt_pid, self._prev_tilt_error, self._tilt_integral = self._pid(
                tilt_err, self._prev_tilt_error, self._tilt_integral,
                self._tilt_kp, self._tilt_kd, self._tilt_ki, dt)
            tilt_out, self._tilt_integral = self._apply_deadzone(
                tilt_pid, self._tilt_integral, self._tilt_dead)
            tilt_out = max(-self._max_tilt_cmd, min(self._max_tilt_cmd, tilt_out))
            self._tilt_pub.publish(Float32(data=tilt_out))


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
