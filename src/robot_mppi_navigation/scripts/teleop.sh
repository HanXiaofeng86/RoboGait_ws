#!/usr/bin/env bash
# Keyboard teleop that goes through safety_gate (cmd_vel_smoothed → cmd_vel).
# Use while slam_mapping / rviz_goal_navigation is running.
#
# Usage (from package source tree):
#   ./scripts/teleop.sh
# Or after source install:
#   ros2 run robot_mppi_navigation teleop
set -euo pipefail
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_smoothed "$@"
