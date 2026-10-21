"""Block until lidar + wheel odom are publishing, then exit.

Used so Nav2/RViz do not start during the lidar-reset window (no odom TF yet).
"""

from __future__ import annotations

import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

NEEDED = (
    ('/scan', LaserScan),
    ('/odometry/wheels', Odometry),
)


class WaitForNavSensors(Node):

    def __init__(self) -> None:
        super().__init__('wait_for_nav_sensors')
        self.seen: set[str] = set()
        for name, msg_type in NEEDED:
            self.create_subscription(
                msg_type, name,
                lambda _msg, n=name: self.seen.add(n), 10)
        self.get_logger().info(
            'Waiting for /scan and /odometry/wheels before starting Nav2…')


def main(argv=None) -> int:
    del argv
    timeout_s = 60.0
    rclpy.init()
    node = WaitForNavSensors()
    deadline = time.monotonic() + timeout_s
    names = [n for n, _ in NEEDED]
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
            if node.seen.issuperset(names):
                node.get_logger().info('Hardware topics are live. Starting Nav2.')
                return 0
        missing = [n for n in names if n not in node.seen]
        node.get_logger().error(
            f'Timed out waiting for {missing}. Starting Nav2 anyway.')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
