"""PASS or ZERO only — never invent velocities."""

from enum import Enum

from geometry_msgs.msg import Twist

from robot_simple_commander.safety.twist_copy import copy_twist, zero_twist


class Decision(Enum):
    PASS = 'pass'
    ZERO = 'zero'


def apply(decision: Decision, cmd: Twist) -> Twist:
    if decision is Decision.PASS:
        return copy_twist(cmd)
    return zero_twist()
