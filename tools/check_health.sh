#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/tools/activate.sh"

t() { timeout 3 bash -c "$*"; }

echo "== NODES =="
t "ros2 node list | sort"

echo "== TOPICS =="
t "ros2 topic list | sort | grep -E '^/clock$|^/scan$|^/pi_camera/|^/tf$|^/tf_static$' || true"

echo "== CLOCK (once) =="
t "ros2 topic echo /clock --once"

echo "== SCAN (once) =="
t "ros2 topic echo /scan --once >/dev/null; echo OK"

echo "== CAMERA INFO (once) =="
t "ros2 topic echo /pi_camera/camera_info --once >/dev/null; echo OK"

echo "== IMAGE HZ (short) =="
t "timeout 2 ros2 topic hz /pi_camera/image_raw || true"

echo "== TF base_link -> lidar_240_link (short) =="
t "timeout 2 ros2 run tf2_ros tf2_echo base_link lidar_240_link || true"

