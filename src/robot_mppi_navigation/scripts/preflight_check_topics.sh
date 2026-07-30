#!/usr/bin/env bash
# Check that key topics are publishing (default observe 5 s).
# Usage: ./scripts/preflight_check_topics.sh [seconds]
set -e
python3 -m robot_mppi_navigation.preflight.topic_health_check "${1:-5}"
