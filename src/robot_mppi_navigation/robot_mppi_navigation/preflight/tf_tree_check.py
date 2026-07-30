"""Preflight: verify the TF chain map -> odom -> base_link exists.

Usage:  python3 -m robot_mppi_navigation.preflight.tf_tree_check [timeout_s]
Exit code 0 = complete chain, 1 = missing transform(s).
"""

import sys

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from robot_mppi_navigation.localization.tf_frame_names import REQUIRED_CHAIN


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = Node('preflight_tf_check')
    buffer = Buffer()
    _listener = TransformListener(buffer, node)

    end = node.get_clock().now().nanoseconds * 1e-9 + timeout
    while node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    ok = True
    for parent, child in REQUIRED_CHAIN:
        can = buffer.can_transform(parent, child, rclpy.time.Time())
        status = 'OK  ' if can else 'FAIL'
        if not can:
            ok = False
        print(f'[{status}] {parent} -> {child}')

    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
