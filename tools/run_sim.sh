#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="indoor_sim_viz"

SIM_CMD="cd '$REPO_ROOT/ros2_ws' && source '$REPO_ROOT/tools/activate.sh' && ros2 launch drone_bringup main.launch.xml"
RVIZ_CMD="cd '$REPO_ROOT/ros2_ws' && source '$REPO_ROOT/tools/activate.sh' && ros2 launch drone_bringup viz.launch.xml"
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

tmux new-session -d -s "$SESSION" -c "$REPO_ROOT" "bash -c \"$SIM_CMD\""
tmux set-option -t "$SESSION" remain-on-exit on
tmux split-window -h -t "$SESSION:0" "bash -c \"$HEALTH_CMD\""
tmux split-window -v -t "$SESSION:0.1" "bash -c \"$SHELL_CMD\""
tmux split-window -v -t "$SESSION:0.0" "bash -c \"$RVIZ_CMD\""

tmux select-layout -t "$SESSION:0" tiled
tmux select-pane -t "$SESSION:0.0"
exec tmux attach -t "$SESSION"

