"""Plots for a navigation test bag (design doc section 12)."""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot_all(out_dir, odom, cmd, path_xy, path_err_t=None, path_err=None,
             heading_err=None):
    """Write standard plots to out_dir.

    odom: (t, x, y, yaw, v, w); cmd: (t, v, w); path_xy: (xs, ys)
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    t_o, x, y, _yaw, v_o, w_o = odom
    t_c, v_c, w_c = cmd
    px, py = path_xy

    fig, ax = plt.subplots(figsize=(8, 8))
    if len(px):
        ax.plot(px, py, 'g--', label='global plan')
    ax.plot(x, y, 'b-', label='robot trajectory')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.axis('equal')
    ax.legend()
    ax.set_title('Path vs robot trajectory')
    fig.savefig(out / 'trajectory.png', dpi=150)
    plt.close(fig)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax1.plot(t_c, v_c, label='cmd v')
    ax1.plot(t_o, v_o, alpha=0.6, label='odom v')
    ax1.set_ylabel('linear [m/s]')
    ax1.legend()
    ax2.plot(t_c, w_c, label='cmd w')
    ax2.plot(t_o, w_o, alpha=0.6, label='odom w')
    ax2.set_ylabel('angular [rad/s]')
    ax2.set_xlabel('time [s]')
    ax2.legend()
    fig.suptitle('Velocity vs time')
    fig.savefig(out / 'velocities.png', dpi=150)
    plt.close(fig)

    if path_err is not None and len(path_err):
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(path_err_t, path_err)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('path error [m]')
        ax.set_title('Cross-track error vs time')
        fig.savefig(out / 'path_error.png', dpi=150)
        plt.close(fig)

    if heading_err is not None and len(heading_err):
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(path_err_t, heading_err)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('heading error [rad]')
        ax.set_title('Heading error vs time')
        fig.savefig(out / 'heading_error.png', dpi=150)
        plt.close(fig)

    if len(t_c) > 1:
        import numpy as np
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(t_c[1:], np.diff(t_c) * 1000.0)
        ax.axhline(50.0, color='r', linestyle='--', label='20 Hz deadline')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('cmd period [ms]')
        ax.set_title('Controller period vs time')
        ax.legend()
        fig.savefig(out / 'controller_period.png', dpi=150)
        plt.close(fig)

    return out
