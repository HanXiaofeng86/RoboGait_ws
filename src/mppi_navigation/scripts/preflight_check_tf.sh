#!/usr/bin/env bash
# 检查 TF 链 map → odom → base_link（默认 5 秒）
# 用法: ./scripts/preflight_check_tf.sh [秒数]
set -e
python3 -m mppi_navigation.preflight.tf_tree_check "${1:-5}"
