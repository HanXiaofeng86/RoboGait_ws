"""Programmatic readiness checks used by preflight modules."""

import rclpy
from tf2_ros import Buffer

from robot_mppi_navigation.localization.tf_frame_names import REQUIRED_CHAIN


def tf_chain_ready(buffer: Buffer) -> bool:
    """True when every transform in map -> odom -> base_link is available."""
    return all(
        buffer.can_transform(parent, child, rclpy.time.Time())
        for parent, child in REQUIRED_CHAIN
    )
