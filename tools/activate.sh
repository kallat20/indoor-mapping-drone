#!/usr/bin/env bash


# ROS2 Humble underlay

set +u
source /opt/ros/humble/setup.bash
set -u

WS="$HOME/indoor-mapping-drone/ros2_ws"
if [ -f "$WS/install/setup.bash" ]; then
    set +u
    source "$WS/install/setup.bash"
    set -u
else
    echo "[activate] ERROR: $WS/install/setup.bash yok. Önce build al !!"
    return 1
fi

export INDOOR_MAPPING_WS_ACTIVE=1
echo "[activate] OK: ROS_DISTRO=$ROS_DISTRO | WS=$WS"