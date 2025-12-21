#!/usr/bin/env bash
set -euo pipefail

# Make sure we are not inside another tmux
unset TMUX || true

SESSION="teleop"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "[INFO] Using tmux session: $SESSION"
echo "[INFO] Script dir: $SCRIPT_DIR"

# Sanity checks
if [ ! -x "$SCRIPT_DIR/ssh_robot.expect" ]; then
    echo "[ERROR] $SCRIPT_DIR/ssh_robot.expect not found or not executable" >&2
    exit 1
fi

if [ ! -x "$SCRIPT_DIR/run_teleop_local.sh" ]; then
    echo "[ERROR] $SCRIPT_DIR/run_teleop_local.sh not found or not executable" >&2
    exit 1
fi

##############################
# 1) Kill existing session
##############################
tmux kill-session -t "$SESSION" >/dev/null 2>&1 || true

##############################
# 2) Create session: LEFT pane (robot)
##############################

# Window 0, pane 0 runs the expect script
tmux new-session -d -s "$SESSION" "expect \"$SCRIPT_DIR/ssh_robot.expect\""

##############################
# 3) Create RIGHT pane (local teleop)
##############################

# Split the current pane horizontally. The NEW right pane runs the local script.
tmux split-window -h -t "$SESSION:0.0" "bash \"$SCRIPT_DIR/run_teleop_local.sh\""

##############################
# 4) Attach to the session
##############################

tmux attach -t "$SESSION"
