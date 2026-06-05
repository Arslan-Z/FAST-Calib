#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/zcy/fast_calib_ws/src/FAST-Calib
exec "$ROOT/pipelines/kalibr_rgb_thermal/scripts/record.sh" d455_boson "$@"
