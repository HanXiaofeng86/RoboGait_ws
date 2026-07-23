#!/usr/bin/env bash
# 检查关键话题是否在发布（默认观察 5 秒）
# 用法: ./scripts/preflight_check_topics.sh [秒数]
set -e
python3 -m mppi_navigation.preflight.topic_health_check "${1:-5}"
