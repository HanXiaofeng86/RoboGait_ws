#!/usr/bin/env bash
# Check TF chain map → odom → base_link (default 5 s).
# Usage: ./scripts/preflight_check_tf.sh [seconds]
set -e
python3 -m robot_mppi_navigation.preflight.tf_tree_check "${1:-5}"
