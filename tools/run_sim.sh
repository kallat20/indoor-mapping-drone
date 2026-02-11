#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="indoor_sim"

SIM_CMD="cd '$REPO_ROOT/ros2_ws' && source '$REPO_ROOT/tools/activate.sh' && ros2 launch drone_bringup main.launch.xml"
HEALTH_CMD="source '$REPO_ROOT/tools/activate.sh' && while true; do '$REPO_ROOT/tools/check_health.sh'; sleep 2; done"
SHELL_CMD="source '$REPO_ROOT/tools/activate.sh' && cd '$REPO_ROOT/ros2_ws' && bash"

# Eğer zaten tmux içindeysen, yeni session açmak yerine attach etmeyi söyle
if [ -n "${TMUX-}" ]; then
  echo "[run_sim] Zaten tmux içindesin. Yeni session yerine şunu kullan:"
  echo "  tmux attach -t $SESSION"
  exit 0
fi

# Session zaten varsa direkt attach
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "[run_sim] Session zaten var: $SESSION (attach ediyorum)"
  exec tmux attach -t "$SESSION"
fi

# 1) Yeni tmux session başlat: sol panel sim
tmux new-session -d -s "$SESSION" -c "$REPO_ROOT" "bash -c \"$SIM_CMD\""

# 2) Sağ tarafa split: health
tmux split-window -h -t "$SESSION:0" "bash -c \"$HEALTH_CMD\""

# 3) Sağ alt split: interactive shell
tmux split-window -v -t "$SESSION:0.1" "bash -c \"$SHELL_CMD\""

# 4) Pane boyutları ve odak
tmux select-layout -t "$SESSION:0" tiled
tmux select-pane -t "$SESSION:0.0"

# Attach
exec tmux attach -t "$SESSION"
