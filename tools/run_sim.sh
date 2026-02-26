#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="indoor_sim_viz"

#PATH 
PX4_DIR="$HOME/PX4-Autopilot"


SIM_CMD="cd '$REPO_ROOT/ros2_ws' && source '$REPO_ROOT/tools/activate.sh' && ros2 launch drone_bringup main.launch.xml"
DDS_CMD="MicroXRCEAgent udp4 -p 8888"
PX4_CMD="source '$REPO_ROOT/tools/activate.sh' && cd '$PX4_DIR' && export GZ_SIM_RESOURCE_PATH='$REPO_ROOT/ros2_ws/src/drone_gazebo/models':'$REPO_ROOT/ros2_ws/src/drone_gazebo/worlds':\${GZ_SIM_RESOURCE_PATH:-} && export PX4_GZ_WORLD='tugbot_depot' && export PX4_GZ_MODEL='x500_lidar_2d' && make px4_sitl gz_x500"
HEALTH_CMD="source '$REPO_ROOT/tools/activate.sh' && set +e && while true; do '$REPO_ROOT/tools/check_health.sh' || true; sleep 2; done"
SHELL_CMD="source '$REPO_ROOT/tools/activate.sh' && cd '$REPO_ROOT/ros2_ws' && exec bash -i"


if [ -n "${TMUX-}" ]; then
  echo "[run_sim_viz] Zaten tmux içindesin. Şunu kullan:"
  echo "  tmux attach -t $SESSION"
  exit 0
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "[run_sim_viz] Session zaten var: $SESSION (attach ediyorum)"
  exec tmux attach -t "$SESSION"
fi

# TMUX PENCERE VE PANEL (PANE) YAPILANDIRMASI
tmux new-session -d -s "$SESSION" -n "Core" -c "$REPO_ROOT" "bash -c \"$SIM_CMD\""
tmux set-option -t "$SESSION" remain-on-exit on

# Ekranı dörde bölüyoruz
tmux split-window -h -t "$SESSION:0" "bash -c \"$DDS_CMD\""
tmux split-window -v -t "$SESSION:0.1" "bash -c \"$PX4_CMD\""
tmux split-window -v -t "$SESSION:0.0" "bash -c \"$SHELL_CMD\""

# Düzeni ayarla
tmux select-layout -t "$SESSION:0" tiled

# Arka planda bir pencere (window) daha açıp health_check'i oraya koyalım ki ekran kalabalık olmasın
tmux new-window -t "$SESSION:1" -n "Health" "bash -c \"$HEALTH_CMD\""

# Ana ekrana (Core) geri dön ve bağlan
tmux select-window -t "$SESSION:0"
exec tmux attach -t "$SESSION"

