"""Heading alignment metrics relative to the path direction."""

import numpy as np


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def path_headings(path_x, path_y):
    """Tangent direction at each path point (forward differences)."""
    dx = np.gradient(path_x)
    dy = np.gradient(path_y)
    return np.arctan2(dy, dx)


def heading_errors(robot_x, robot_y, robot_yaw, path_x, path_y):
    """Heading error vs the nearest path point's tangent, per sample."""
    if len(path_x) < 2 or len(robot_x) == 0:
        return np.array([])
    tangents = path_headings(path_x, path_y)
    path = np.stack([path_x, path_y], axis=1)
    robot = np.stack([robot_x, robot_y], axis=1)
    nearest = np.linalg.norm(
        robot[:, None, :] - path[None, :, :], axis=2).argmin(axis=1)
    return _wrap(robot_yaw - tangents[nearest])


def summarize(errors):
    if len(errors) == 0:
        return {'rmse': float('nan'), 'max_abs': float('nan')}
    return {
        'rmse': float(np.sqrt(np.mean(errors ** 2))),
        'max_abs': float(np.max(np.abs(errors))),
    }
