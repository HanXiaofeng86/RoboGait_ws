"""Velocity smoothness metrics: acceleration and jerk of (v, w)."""

import numpy as np


def _derivative(t, x):
    if len(t) < 2:
        return np.array([])
    return np.gradient(x, t)


def smoothness(t, v, w):
    """Return acceleration/jerk statistics for linear and angular velocity."""
    acc_v = _derivative(t, v)
    acc_w = _derivative(t, w)
    jerk_v = _derivative(t, acc_v) if len(acc_v) else np.array([])
    jerk_w = _derivative(t, acc_w) if len(acc_w) else np.array([])

    def stats(x):
        if len(x) == 0:
            return {'rms': float('nan'), 'max_abs': float('nan')}
        return {'rms': float(np.sqrt(np.mean(x ** 2))),
                'max_abs': float(np.max(np.abs(x)))}

    return {
        'linear_acceleration': stats(acc_v),
        'angular_acceleration': stats(acc_w),
        'linear_jerk': stats(jerk_v),
        'angular_jerk': stats(jerk_w),
    }
