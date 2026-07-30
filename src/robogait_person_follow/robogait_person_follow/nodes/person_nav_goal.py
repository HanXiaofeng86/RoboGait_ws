"""Turn filtered person pose into Nav2 goals; MPPI owns all chassis velocity.

User.msg is treated as camera/base-relative: z = forward (m), x = lateral (m).
Standoff goal in map: stay ``target_distance_m`` from the person, facing them.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from robogait_person_interfaces.msg import User
from tf2_ros import Buffer, TransformListener


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PersonNavGoal(Node):
    """Publish / send NavigateToPose so MPPI drives the full [v, ω] command."""

    def __init__(self) -> None:
        super().__init__('person_nav_goal')
        self.declare_parameter('target_distance_m', 2.4)
        self.declare_parameter('dead_zone_m', 0.15)
        self.declare_parameter('goal_update_min_m', 0.30)
        self.declare_parameter('goal_update_period_sec', 0.5)
        self.declare_parameter('lost_timeout_sec', 1.2)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('goal_topic', 'person_nav_goal')

        self._target = float(self.get_parameter('target_distance_m').value)
        self._dead = float(self.get_parameter('dead_zone_m').value)
        self._min_move = float(self.get_parameter('goal_update_min_m').value)
        self._period = float(self.get_parameter('goal_update_period_sec').value)
        self._lost_timeout = float(self.get_parameter('lost_timeout_sec').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)

        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._client = ActionClient(
            self, NavigateToPose, str(self.get_parameter('action_name').value))
        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('goal_topic').value), 10)

        self._last_user: User | None = None
        self._last_seen = self.get_clock().now()
        self._last_sent_xy: tuple[float, float] | None = None
        self._last_send_time = self.get_clock().now()
        self._goal_active = False
        self._goal_handle = None
        self._warn_nav_missing = 0

        self.create_subscription(User, 'user_coordinates', self._on_user, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'Pure-MPPI person goal: standoff={self._target:.2f} m '
            f'(no cmd_vel PID blend)')

    def _on_user(self, msg: User) -> None:
        if msg.detection > 0 and msg.z > 0.05:
            self._last_user = msg
            self._last_seen = self.get_clock().now()

    def _robot_pose(self) -> tuple[float, float, float] | None:
        try:
            tf = self._tf.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            return None
        return (
            tf.transform.translation.x,
            tf.transform.translation.y,
            _quat_to_yaw(tf.transform.rotation),
        )

    def _standoff_goal(
        self, rx: float, ry: float, ryaw: float, user: User,
    ) -> PoseStamped | None:
        # User: z forward, x lateral in base/camera frame (same as legacy follow).
        c, s = math.cos(ryaw), math.sin(ryaw)
        px = rx + user.z * c - user.x * s
        py = ry + user.z * s + user.x * c

        dx = rx - px
        dy = ry - py
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return None

        ux, uy = dx / dist, dy / dist
        gx = px + self._target * ux
        gy = py + self._target * uy
        gyaw = math.atan2(py - gy, px - gx)

        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        qx, qy, qz, qw = _yaw_to_quat(gyaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _tick(self) -> None:
        now = self.get_clock().now()
        lost = (now - self._last_seen).nanoseconds / 1e9 > self._lost_timeout
        if lost or self._last_user is None:
            self._cancel_goal()
            return

        robot = self._robot_pose()
        if robot is None:
            return
        rx, ry, ryaw = robot
        goal = self._standoff_goal(rx, ry, ryaw, self._last_user)
        if goal is None:
            return

        self._goal_pub.publish(goal)

        err = math.hypot(goal.pose.position.x - rx, goal.pose.position.y - ry)
        if err <= self._dead:
            self._cancel_goal()
            return

        if self._last_sent_xy is not None:
            moved = math.hypot(
                goal.pose.position.x - self._last_sent_xy[0],
                goal.pose.position.y - self._last_sent_xy[1])
            since = (now - self._last_send_time).nanoseconds / 1e9
            if moved < self._min_move and since < self._period:
                return

        self._send_goal(goal)

    def _send_goal(self, pose: PoseStamped) -> None:
        if not self._client.wait_for_server(timeout_sec=0.0):
            self._warn_nav_missing += 1
            if self._warn_nav_missing % 50 == 1:
                self.get_logger().warning(
                    'navigate_to_pose action not ready (is Nav2 running?)')
            return
        self._warn_nav_missing = 0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)
        self._last_sent_xy = (pose.pose.position.x, pose.pose.position.y)
        self._last_send_time = self.get_clock().now()
        self._goal_active = True

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Goal send failed: {exc}')
            return
        if not handle.accepted:
            self.get_logger().warning('NavigateToPose rejected')
            self._goal_active = False
            return
        self._goal_handle = handle

    def _cancel_goal(self) -> None:
        if not self._goal_active:
            return
        handle = self._goal_handle
        self._goal_active = False
        self._goal_handle = None
        self._last_sent_xy = None
        if handle is not None:
            handle.cancel_goal_async()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonNavGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
