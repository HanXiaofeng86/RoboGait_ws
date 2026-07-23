"""The only two outputs the safety layer may produce: PASS or ZERO.

The safety layer must never generate an alternative trajectory-following
command (design doc section 13).
"""

from enum import Enum

from geometry_msgs.msg import Twist

from mppi_navigation.safety.twist_copy import copy_twist, zero_twist


class Decision(Enum):
    PASS = 'pass'
    ZERO = 'zero'


def apply(decision: Decision, cmd: Twist) -> Twist:
    if decision is Decision.PASS:
        return copy_twist(cmd)
    return zero_twist()
