"""Smooth user_raw and publish user_coordinates."""

from __future__ import annotations

from collections import deque

import rclpy
from rclpy.node import Node
from robogait_person_interfaces.msg import User


class DistanceFilter(Node):
    """Moving-average filter with short dropout hold."""

    def __init__(self) -> None:
        super().__init__('distance_filter')
        self.declare_parameter('filter_window', 5)
        self.declare_parameter('dropout_frames', 20)

        self._window = int(self.get_parameter('filter_window').value)
        self._dropout_max = int(self.get_parameter('dropout_frames').value)
        self._xs: deque[float] = deque(maxlen=self._window)
        self._ys: deque[float] = deque(maxlen=self._window)
        self._zs: deque[float] = deque(maxlen=self._window)
        self._empty_frames = 0
        self._last_valid = User()

        self._pub = self.create_publisher(User, 'user_coordinates', 10)
        self.create_subscription(User, 'user_raw', self._on_raw, 10)

    def _mean(self, values: deque[float]) -> float:
        nonzero = [v for v in values if v != 0.0]
        if not nonzero:
            return 0.0
        return sum(nonzero) / len(nonzero)

    def _on_raw(self, msg: User) -> None:
        out = User()
        if msg.detection > 0 and msg.z > 0.0:
            self._xs.append(float(msg.x))
            self._ys.append(float(msg.y))
            self._zs.append(float(msg.z))
            out.detection = 1
            out.x = self._mean(self._xs)
            out.y = self._mean(self._ys)
            out.z = self._mean(self._zs)
            self._last_valid = out
            self._empty_frames = 0
        else:
            self._empty_frames += 1
            if self._empty_frames <= self._dropout_max and self._last_valid.z > 0.0:
                out = self._last_valid
                out.detection = 1
            else:
                out.detection = 0
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistanceFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
