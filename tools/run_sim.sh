#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="indoor_sim_viz"
PX4_DIR="$HOME/PX4-Autopilot"

SIM_CMD="cd '$REPO_ROOT/ros2_ws' && source '$REPO_ROOT/tools/activate.sh' && ros2 launch drone_bringup main.launch.xml"
DDS_CMD="MicroXRCEAgent udp4 -p 8888"
PX4_CMD="source '$REPO_ROOT/tools/activate.sh' && cd '$PX4_DIR' && export PX4_GZ_STANDALONE=1 && export PX4_SYS_AUTOSTART=4001 && export PX4_GZ_WORLD=world_demo && export PX4_GZ_MODEL_NAME=x500_lidar_2d && ./build/px4_sitl_default/bin/px4"
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

# Core panel: sim
tmux new-session -d -s "$SESSION" -n "Core" -c "$REPO_ROOT" "bash -c \"$SIM_CMD\""
tmux set-option -t "$SESSION" remain-on-exit on

# Sağ üst: DDS agent
tmux split-window -h -t "$SESSION:0" "bash -c \"$DDS_CMD\""

# Sağ alt: PX4 (Gazebo world hazır olması için kısa bekleme)
tmux split-window -v -t "$SESSION:0.1" "bash -c \"sleep 3; $PX4_CMD\""

# Sol alt: interaktif shell
tmux split-window -v -t "$SESSION:0.0" "bash -c \"$SHELL_CMD\""

tmux select-layout -t "$SESSION:0" tiled

# Health ayrı pencere
tmux new-window -t "$SESSION:1" -n "Health" "bash -c \"$HEALTH_CMD\""

tmux select-window -t "$SESSION:0"
exec tmux attach -t "$SESSION"

<<'EXIT'
# Session davranışı
tmux set-option -t "$SESSION" remain-on-exit off
tmux set-option -t "$SESSION" destroy-unattached on

# Core main pane'i oluştur (ilk pane)
tmux new-session -d -s "$SESSION" -n "Core" -c "$REPO_ROOT" "bash -c \"$SIM_CMD\""

# Core pane id'sini al
CORE_PANE_ID="$(tmux display-message -p -t "$SESSION:0.0" "#{pane_id}")"

# Core pane kapanırsa tüm session'ı kapat
tmux set-hook -t "$SESSION" pane-exited \
  "if -F '#{==:#{hook_pane},$CORE_PANE_ID}' 'kill-session -t $SESSION'"
EXIT

