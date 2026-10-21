#!/usr/bin/env bash
# Start Foxglove bridge on NUC; view topics in browser via SSH port forward.
# On laptop:  ssh -L 8765:localhost:8765 robogait@<nuc-ip>
# Then open:  https://app.foxglove.dev  -> Open connection -> Foxglove WebSocket -> ws://localhost:8765
set -euo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/RoboGait_ws/install/setup.bash"

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "[foxglove] Installing ros-humble-foxglove-bridge (needs sudo once)..."
  sudo apt-get update -qq
  sudo apt-get install -y ros-humble-foxglove-bridge
fi

echo "[foxglove] Bridge on ws://127.0.0.1:8765"
echo "[foxglove] On laptop: ssh -L 8765:localhost:8765 robogait@<nuc-ip>"
echo "[foxglove] Browser: https://app.foxglove.dev -> ws://localhost:8765"
echo "[foxglove] Add panels: Map (/map), LaserScan (/scan), TF"

exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
