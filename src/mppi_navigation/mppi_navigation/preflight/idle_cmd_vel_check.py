"""Preflight (Test 1, system idle): confirm /cmd_vel stays zero with no goal.

Usage:  python3 -m mppi_navigation.preflight.idle_cmd_vel_check [duration_s]
Exit code 0 = no non-zero command observed, 1 = robot commanded to move.
"""

import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from mppi_navigation.safety.twist_copy import is_zero


class IdleCheck(Node):

    def __init__(self):
        super().__init__('preflight_idle_check')
        self.violations = 0
        self.samples = 0
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)

    def _on_cmd(self, msg: Twist):
        self.samples += 1
        if not is_zero(msg, eps=1e-6):
            self.violations += 1
            self.get_logger().warn(
                f'Non-zero cmd while idle: v={msg.linear.x:.3f} '
                f'w={msg.angular.z:.3f}')


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = IdleCheck()

    end = node.get_clock().now().nanoseconds * 1e-9 + duration
    while node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f'samples={node.samples} violations={node.violations}')
    ok = node.violations == 0
    print('IDLE TEST ' + ('PASSED' if ok else 'FAILED'))

    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
