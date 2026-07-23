"""Unit tests for safety-gate helpers (no robot required)."""

from geometry_msgs.msg import Twist

from mppi_navigation.safety.pass_or_zero_policy import Decision, apply
from mppi_navigation.safety.stop_conditions import SafetyState
from mppi_navigation.safety.twist_copy import copy_twist, is_zero, zero_twist


def _twist(vx=0.0, wz=0.0) -> Twist:
    t = Twist()
    t.linear.x = vx
    t.angular.z = wz
    return t


def test_zero_twist():
    assert is_zero(zero_twist())


def test_copy_twist_independent():
    src = _twist(0.5, 0.2)
    dst = copy_twist(src)
    src.linear.x = 9.0
    assert dst.linear.x == 0.5
    assert dst.angular.z == 0.2


def test_apply_pass_and_zero():
    cmd = _twist(0.4, -0.1)
    out = apply(Decision.PASS, cmd)
    assert out.linear.x == 0.4
    assert is_zero(apply(Decision.ZERO, cmd))


def test_safety_state_estop():
    s = SafetyState(last_cmd_time=10.0, last_odom_time=10.0, estop_active=True)
    assert not s.is_safe(10.1)
    assert 'emergency stop' in s.stop_reasons(10.1)[0]


def test_safety_state_stale_cmd():
    s = SafetyState(last_cmd_time=1.0, last_odom_time=10.0, cmd_timeout=0.5)
    reasons = s.stop_reasons(now=2.0)
    assert any('command stale' in r for r in reasons)
    assert not s.is_safe(2.0)


def test_safety_state_ok():
    s = SafetyState(last_cmd_time=10.0, last_odom_time=10.0)
    assert s.is_safe(10.2)
    assert s.stop_reasons(10.2) == []
