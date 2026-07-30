"""Twist helpers for the safety gate: copy unchanged or produce zero."""

from geometry_msgs.msg import Twist


def zero_twist() -> Twist:
    return Twist()


def copy_twist(msg: Twist) -> Twist:
    out = Twist()
    out.linear.x = msg.linear.x
    out.linear.y = msg.linear.y
    out.linear.z = msg.linear.z
    out.angular.x = msg.angular.x
    out.angular.y = msg.angular.y
    out.angular.z = msg.angular.z
    return out


def is_zero(msg: Twist, eps: float = 1e-9) -> bool:
    return (abs(msg.linear.x) < eps and abs(msg.linear.y) < eps
            and abs(msg.linear.z) < eps and abs(msg.angular.x) < eps
            and abs(msg.angular.y) < eps and abs(msg.angular.z) < eps)
