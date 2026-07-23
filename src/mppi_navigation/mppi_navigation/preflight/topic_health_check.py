"""Preflight: verify required topics are publishing.

Usage:  python3 -m mppi_navigation.preflight.topic_health_check [timeout_s]
Exit code 0 = all topics alive, 1 = at least one silent topic.
"""

import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan

REQUIRED = [
    ('/scan', LaserScan),
    ('/imu/data', Imu),
    ('/odometry/wheels', Odometry),
    ('/odometry/filtered', Odometry),
    ('/map', OccupancyGrid),
]

OPTIONAL = [
    ('/cmd_vel_nav', Twist),
    ('/cmd_vel_smoothed', Twist),
    ('/cmd_vel', Twist),
]


class TopicHealthCheck(Node):

    def __init__(self, topics):
        super().__init__('preflight_topic_check')
        self.seen = set()
        for name, msg_type in topics:
            self.create_subscription(
                msg_type, name,
                lambda _msg, n=name: self.seen.add(n), 10)


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = TopicHealthCheck(REQUIRED + OPTIONAL)

    end = node.get_clock().now().nanoseconds * 1e-9 + timeout
    required_names = {n for n, _ in REQUIRED}
    while node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if required_names <= node.seen:
            break

    ok = True
    for name, _ in REQUIRED:
        status = 'OK  ' if name in node.seen else 'FAIL'
        if name not in node.seen:
            ok = False
        print(f'[{status}] {name}')
    for name, _ in OPTIONAL:
        status = 'OK  ' if name in node.seen else '----'
        print(f'[{status}] {name} (optional)')

    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
