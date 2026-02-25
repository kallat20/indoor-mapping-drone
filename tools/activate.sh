#!/usr/bin/env bash
# Usage: source tools/activate.sh
set -euo pipefail

# Script source mu edildi yoksa direkt mi çalıştı?
_sourced=0
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  _sourced=1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$REPO_ROOT/ros2_ws"

# ROS2 Humble underlay
set +u
source /opt/ros/humble/setup.bash
set -u

# Overlay
if [[ -f "$WS/install/setup.bash" ]]; then
  set +u
  source "$WS/install/setup.bash"
  set -u
else
  echo "[activate] ERROR: $WS/install/setup.bash yok."
  echo "[activate] Çözüm: (cd '$WS' && colcon build)"
  if [[ $_sourced -eq 1 ]]; then
    return 1
  else
    exit 1
  fi
fi

export INDOOR_MAPPING_WS_ACTIVE=1
export INDOOR_MAPPING_REPO_ROOT="$REPO_ROOT"
echo "[activate] OK: ROS_DISTRO=$ROS_DISTRO | WS=$WS"
