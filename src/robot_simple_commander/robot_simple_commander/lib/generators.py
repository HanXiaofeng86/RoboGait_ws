"""Geometric test path generators (TEST_PLAN T2–T6 style).

Paths are lists of (x, y, yaw) in a local frame starting at the origin
facing +x. Only geometry is stored; velocities are always computed online
by MPPI (never record/replay cmd_vel).
"""

import math


def _straight_segment(x0, y0, yaw, length, step):
    n = max(2, int(round(length / step)) + 1)
    pts = []
    for i in range(n):
        d = length * i / (n - 1)
        pts.append((x0 + d * math.cos(yaw), y0 + d * math.sin(yaw), yaw))
    return pts


def make_straight(length: float = 3.0, step: float = 0.05):
    return _straight_segment(0.0, 0.0, 0.0, length, step)


def make_left_90(leg: float = 2.0, step: float = 0.05):
    first = _straight_segment(0.0, 0.0, 0.0, leg, step)
    second = _straight_segment(leg, 0.0, math.pi / 2.0, leg, step)
    return first + second[1:]


def make_right_90(leg: float = 2.0, step: float = 0.05):
    first = _straight_segment(0.0, 0.0, 0.0, leg, step)
    second = _straight_segment(leg, 0.0, -math.pi / 2.0, leg, step)
    return first + second[1:]


def make_s_curve(length: float = 4.0, amplitude: float = 1.0,
                 step: float = 0.05):
    n = max(2, int(round(length / step)) + 1)
    pts = []
    for i in range(n):
        x = length * i / (n - 1)
        y = amplitude * math.sin(2.0 * math.pi * x / length)
        dy = amplitude * (2.0 * math.pi / length) * math.cos(
            2.0 * math.pi * x / length)
        pts.append((x, y, math.atan2(dy, 1.0)))
    return pts


def make_obstacle_avoid(length: float = 4.0, step: float = 0.05):
    return _straight_segment(0.0, 0.0, 0.0, length, step)


GENERATORS = {
    'straight': make_straight,
    'left_90': make_left_90,
    'right_90': make_right_90,
    's_curve': make_s_curve,
    'obstacle_avoid': make_obstacle_avoid,
}
