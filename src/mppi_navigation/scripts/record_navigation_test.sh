#!/usr/bin/env bash
# 录制导航测试 rosbag（话题列表见 config/diagnostic_recording_topics.yaml）
# 用法: ./scripts/record_navigation_test.sh [测试名]
set -e

TEST_NAME="${1:-navigation_test}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${HOME}/RoboGait_ws/bags/${TEST_NAME}_${STAMP}"

TOPICS_YAML="$(python3 - <<'EOF'
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
print(Path(get_package_share_directory('mppi_navigation')) / 'config' / 'diagnostic_recording_topics.yaml')
EOF
)"

TOPICS="$(python3 - "$TOPICS_YAML" <<'EOF'
import sys, yaml
with open(sys.argv[1]) as f:
    data = yaml.safe_load(f)
print(' '.join(data['topics']))
EOF
)"

mkdir -p "$(dirname "$OUT_DIR")"
echo "Recording to: $OUT_DIR"
echo "Topics: $TOPICS"
# shellcheck disable=SC2086
exec ros2 bag record -o "$OUT_DIR" $TOPICS
