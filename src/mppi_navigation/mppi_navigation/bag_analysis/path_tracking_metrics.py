"""Path tracking metrics: lateral error vs the global plan."""

import numpy as np


def cross_track_errors(robot_x, robot_y, path_x, path_y):
    """Distance from each robot position to the nearest path point."""
    if len(path_x) == 0 or len(robot_x) == 0:
        return np.array([])
    path = np.stack([path_x, path_y], axis=1)
    robot = np.stack([robot_x, robot_y], axis=1)
    # Pairwise nearest-point distance (paths are short enough for brute force)
    d = np.linalg.norm(robot[:, None, :] - path[None, :, :], axis=2)
    return d.min(axis=1)


def summarize(errors):
    if len(errors) == 0:
        return {'rmse': float('nan'), 'max': float('nan'), 'mean': float('nan')}
    return {
        'rmse': float(np.sqrt(np.mean(errors ** 2))),
        'max': float(np.max(errors)),
        'mean': float(np.mean(errors)),
    }
