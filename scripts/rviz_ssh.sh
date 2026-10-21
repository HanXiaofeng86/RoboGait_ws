#!/usr/bin/env bash
# Open RViz from an SSH session (no X11 forwarding needed).
# Window appears on the NUC monitor, not on your laptop.
set -euo pipefail

RVIZ_CONFIG="${1:-/home/robogait/.rviz2/rviz_indoor_slam.rviz}"

if [[ -n "${DISPLAY:-}" ]]; then
  echo "[rviz_ssh] Using existing DISPLAY=${DISPLAY}"
else
  for candidate in :1 :0; do
    export DISPLAY="${candidate}"
    if xdpyinfo >/dev/null 2>&1; then
      echo "[rviz_ssh] Using NUC local display DISPLAY=${DISPLAY}"
      break
    fi
    unset DISPLAY
  done
fi

if [[ -z "${DISPLAY:-}" ]]; then
  cat <<'EOF'
[rviz_ssh] No display found.

If you only use SSH:
  1) NUC monitor: log in on the NUC desktop once, then from SSH run:
       ~/RoboGait_ws/scripts/rviz_ssh.sh
  2) Laptop screen: use system terminal (not VS Code), connect with:
       ssh -Y robogait@<nuc-ip>
     then: rviz2 -d ~/.rviz2/rviz_indoor_slam.rviz
  3) Browser (recommended for remote): see scripts/foxglove_ssh.sh

EOF
  exit 1
fi

if [[ -z "${XAUTHORITY:-}" ]]; then
  uid="$(id -u)"
  for auth in "/run/user/${uid}/gdm/Xauthority" "${HOME}/.Xauthority"; do
    if [[ -f "${auth}" ]]; then
      export XAUTHORITY="${auth}"
      break
    fi
  done
fi

source /opt/ros/humble/setup.bash
source "${HOME}/RoboGait_ws/install/setup.bash"

exec rviz2 -d "${RVIZ_CONFIG}"
