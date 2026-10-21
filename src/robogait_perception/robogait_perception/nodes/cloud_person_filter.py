#!/usr/bin/env python3
"""Nuitrack full cloud → /orbbec/points_person (cylinder around torso).

SDK already publishes both topics — do not run this with person_sdk
(same /orbbec/points_person name). Nuitrack and SDK must not share the camera.
"""

from __future__ import annotations

import math
import struct

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def _field_offset(msg: PointCloud2, name: str) -> int | None:
    for f in msg.fields:
        if f.name == name:
            return int(f.offset)
    return None


class CloudPersonFilter(Node):
    """Subscribe to full /orbbec/points + person XYZ; republish person-only cloud."""

    def __init__(self) -> None:
        super().__init__('cloud_person_filter')
        self.declare_parameter('cloud_topic', '/orbbec/points')
        self.declare_parameter('person_cloud_topic', '/orbbec/points_person')
        self.declare_parameter('person_position_topic', '/orbbec/person_position')
        self.declare_parameter('cloud_person_radius_m', 0.55)
        self.declare_parameter('cloud_person_floor_margin_m', 0.95)
        self.declare_parameter('cloud_person_head_margin_m', 0.70)
        self.declare_parameter('min_person_z_m', 0.15)

        radius = float(self.get_parameter('cloud_person_radius_m').value)
        self._radius2 = max(0.15, radius) ** 2
        self._floor_margin = max(0.3, float(self.get_parameter('cloud_person_floor_margin_m').value))
        self._head_margin = max(0.2, float(self.get_parameter('cloud_person_head_margin_m').value))
        self._min_z = float(self.get_parameter('min_person_z_m').value)

        self._person: tuple[float, float, float] | None = None

        viz_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        cloud_in = self.get_parameter('cloud_topic').value
        cloud_out = self.get_parameter('person_cloud_topic').value
        pos_topic = self.get_parameter('person_position_topic').value

        self._pub = self.create_publisher(PointCloud2, cloud_out, viz_qos)
        self.create_subscription(PointCloud2, cloud_in, self._on_cloud, viz_qos)
        self.create_subscription(Point, pos_topic, self._on_position, 10)

        self.get_logger().info(
            f'Nuitrack person cloud: {cloud_in} → {cloud_out} '
            f'radius={math.sqrt(self._radius2):.2f} m')

    def _on_position(self, msg: Point) -> None:
        if msg.z >= self._min_z:
            self._person = (msg.x, msg.y, msg.z)
        else:
            self._person = None

    def _on_cloud(self, msg: PointCloud2) -> None:
        ox = _field_offset(msg, 'x')
        oy = _field_offset(msg, 'y')
        oz = _field_offset(msg, 'z')
        if ox is None or oy is None or oz is None:
            self.get_logger().error('PointCloud2 missing x/y/z', throttle_duration_sec=5.0)
            return

        person = self._person
        if person is None:
            self._pub.publish(self._empty_like(msg))
            return

        px, py, pz = person
        step = int(msg.point_step)
        n_in = int(msg.width) * int(msg.height)
        src = msg.data
        out = bytearray()
        n_out = 0
        # Nuitrack real / User.msg: X right, Y up, Z forward. Same numbers as the cloud.
        for i in range(n_in):
            off = i * step
            x = struct.unpack_from('<f', src, off + ox)[0]
            y = struct.unpack_from('<f', src, off + oy)[0]
            z = struct.unpack_from('<f', src, off + oz)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            dx = x - px
            dz = z - pz
            if dx * dx + dz * dz > self._radius2:
                continue
            if y < py - self._floor_margin:
                continue
            if y > py + self._head_margin:
                continue
            out.extend(src[off:off + step])
            n_out += 1

        filtered = PointCloud2()
        filtered.header = msg.header
        filtered.height = 1
        filtered.width = n_out
        filtered.fields = msg.fields
        filtered.is_bigendian = msg.is_bigendian
        filtered.point_step = step
        filtered.row_step = step * n_out
        filtered.is_dense = False
        filtered.data = bytes(out)
        self._pub.publish(filtered)

    def _empty_like(self, msg: PointCloud2) -> PointCloud2:
        empty = PointCloud2()
        empty.header = msg.header
        empty.height = 1
        empty.width = 0
        empty.fields = msg.fields
        empty.is_bigendian = msg.is_bigendian
        empty.point_step = msg.point_step
        empty.row_step = 0
        empty.is_dense = False
        empty.data = b''
        return empty


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CloudPersonFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
