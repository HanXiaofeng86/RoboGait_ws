"""Safety gate: forward smoothed cmd unchanged, or zero."""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from robot_simple_commander.safety.pass_or_zero_policy import Decision, apply
from robot_simple_commander.safety.stop_conditions import SafetyState
from robot_simple_commander.safety.twist_copy import is_zero, zero_twist


class CmdVelSafetyGate(Node):

    def __init__(self):
        super().__init__('safety_gate')
        self.declare_parameter('input_topic', 'cmd_vel_smoothed')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odometry/filtered')
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('check_frequency', 20.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self._state = SafetyState(
            cmd_timeout=float(self.get_parameter('cmd_timeout').value),
            odom_timeout=float(self.get_parameter('odom_timeout').value),
        )
        self._moving = False
        self._last_reasons = []

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_cmd, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(Bool, 'safety_estop', self._on_estop, 10)

        period = 1.0 / float(self.get_parameter('check_frequency').value)
        self.create_timer(period, self._watchdog)
        self.get_logger().info(
            f'Safety gate: {input_topic} -> {output_topic}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, _msg: Odometry):
        self._state.last_odom_time = self._now()

    def _on_estop(self, msg: Bool):
        self._state.estop_active = msg.data

    def _on_cmd(self, msg: Twist):
        now = self._now()
        self._state.last_cmd_time = now
        decision = Decision.PASS if self._state.is_safe(now) else Decision.ZERO
        out = apply(decision, msg)
        self._pub.publish(out)
        self._moving = not is_zero(out)
        if decision is Decision.ZERO:
            self._log_reasons(now)

    def _watchdog(self):
        now = self._now()
        if self._state.is_safe(now):
            return
        if self._moving:
            self._pub.publish(zero_twist())
            self._moving = False
            self._log_reasons(now)

    def _log_reasons(self, now: float):
        reasons = self._state.stop_reasons(now)
        if reasons != self._last_reasons:
            self.get_logger().warn('Zero velocity: ' + '; '.join(reasons))
            self._last_reasons = reasons


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
