#!/usr/bin/env python3
"""Console entry: ros2 run robot_mppi_navigation teleop

Keyboard teleop remapped to /cmd_vel_smoothed so safety_gate owns /cmd_vel.
"""

import os
import sys


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    cmd = [
        'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
        '--ros-args', '-r', 'cmd_vel:=/cmd_vel_smoothed',
        *argv,
    ]
    os.execvp(cmd[0], cmd)


if __name__ == '__main__':
    raise SystemExit(main())
