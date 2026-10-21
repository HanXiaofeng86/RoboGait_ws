#!/usr/bin/env python3
"""Draw the lidar scan on the map plane.

slam_toolbox keeps the real /scan in lidar_link (about 0.33 m above
base_link). This node republishes those same ranges as /scan_flat in
scan_ground, a frame straight under the lidar at base_link height, so a
side view shows the scan on the map.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


class FlattenScan(Node):
    def __init__(self):
        super().__init__('flatten_scan')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._ground_ready = False
        self._pub = self.create_publisher(LaserScan, '/scan_flat', qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos)

    def _on_scan(self, msg: LaserScan):
        if not self._ground_ready and not self._publish_ground_frame(msg.header.frame_id):
            return
        msg.header.frame_id = 'scan_ground'
        self._pub.publish(msg)

    def _publish_ground_frame(self, lidar_frame: str) -> bool:
        try:
            tf = self._buffer.lookup_transform(
                'base_link', lidar_frame, rclpy.time.Time())
        except TransformException:
            return False
        height = tf.transform.translation.z
        static = TransformStamped()
        static.header.stamp = self.get_clock().now().to_msg()
        static.header.frame_id = lidar_frame
        static.child_frame_id = 'scan_ground'
        static.transform.translation.z = -height
        static.transform.rotation.w = 1.0
        self._static_tf.sendTransform(static)
        self._ground_ready = True
        self.get_logger().info(
            'scan_ground is %.3f m under %s; /scan_flat lies on the map plane'
            % (height, lidar_frame))
        return True


def main():
    rclpy.init()
    node = FlattenScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
