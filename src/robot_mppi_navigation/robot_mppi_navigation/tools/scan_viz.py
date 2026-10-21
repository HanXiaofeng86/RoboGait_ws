"""Republish /scan with now() stamps for RViz.

AMCL/Nav2 keep using /scan. RViz LaserScan looks at /scan_viz so a 10–40 ms
TF lag does not drop every frame after 2D Pose Estimate.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanViz(Node):

    def __init__(self) -> None:
        super().__init__('scan_viz')
        self._pub = self.create_publisher(LaserScan, '/scan_viz', 10)
        self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)
        self._pub.publish(out)


def main(argv=None) -> int:
    del argv
    rclpy.init()
    node = ScanViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
