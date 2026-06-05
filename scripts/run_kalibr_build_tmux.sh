#!/usr/bin/env bash
set -euo pipefail
SESSION=kalibr_build_fast_calib
LOG=/home/zcy/fast_calib_ws/src/FAST-Calib/logs/kalibr_build_$(date +%Y%m%d_%H%M%S).log
SCRIPT=/home/zcy/fast_calib_ws/src/FAST-Calib/scripts/build_kalibr_official.sh

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" "bash -lc 'set -o pipefail; $SCRIPT 2>&1 | tee $LOG; echo BUILD_EXIT:\${PIPESTATUS[0]}; sleep 10'"
echo "$SESSION"
echo "$LOG"
