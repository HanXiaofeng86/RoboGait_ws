#!/usr/bin/env python3
"""Analyze a navigation test rosbag: metrics + plots.

Usage:
    python3 scripts/analyze_navigation_bag.py <bag_dir> [output_dir]

Requires the workspace to be sourced (uses robot_mppi_navigation modules).
"""

import json
import sys
from pathlib import Path

from robot_mppi_navigation.bag_analysis.controller_timing_metrics import timing
from robot_mppi_navigation.bag_analysis.generate_plots import plot_all
from robot_mppi_navigation.bag_analysis.heading_error_metrics import (
    heading_errors, summarize as heading_summary)
from robot_mppi_navigation.bag_analysis.load_rosbag import (
    last_path_xy, load_bag, odometry_series, twist_series)
from robot_mppi_navigation.bag_analysis.path_tracking_metrics import (
    cross_track_errors, summarize as path_summary)
from robot_mppi_navigation.bag_analysis.velocity_smoothness_metrics import smoothness

TOPICS = ['/cmd_vel', '/odometry/filtered', '/plan']


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    bag_dir = sys.argv[1]
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(bag_dir) / 'analysis'

    data = load_bag(bag_dir, TOPICS)

    if '/odometry/filtered' not in data or not data['/odometry/filtered'][1]:
        print('ERROR: bag has no /odometry/filtered data')
        sys.exit(1)

    odom = odometry_series(*data['/odometry/filtered'])
    t_o, x, y, yaw, _v, _w = odom

    if '/cmd_vel' in data and data['/cmd_vel'][1]:
        cmd = twist_series(*data['/cmd_vel'])
    else:
        cmd = (t_o[:0], t_o[:0], t_o[:0])

    px, py = last_path_xy(data.get('/plan', (None, []))[1])

    path_err = cross_track_errors(x, y, px, py)
    head_err = heading_errors(x, y, yaw, px, py)

    results = {
        'path_tracking': path_summary(path_err),
        'heading': heading_summary(head_err),
        'smoothness_cmd': smoothness(*cmd),
        'controller_timing': timing(cmd[0]),
    }

    out_dir.mkdir(parents=True, exist_ok=True)
    report = out_dir / 'metrics.json'
    report.write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))

    plot_all(out_dir, odom, cmd, (px, py),
             path_err_t=t_o[:len(path_err)], path_err=path_err,
             heading_err=head_err)
    print(f'\nReport: {report}\nPlots:  {out_dir}/')


if __name__ == '__main__':
    main()
