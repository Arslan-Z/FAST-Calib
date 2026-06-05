#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
VIS_DIR="$ROOT/tools/visualization"
UV_BIN=${UV_BIN:-$(command -v uv 2>/dev/null || command -v /home/zcy/.local/bin/uv 2>/dev/null || true)}
PYTHON_BIN=${PYTHON_BIN:-/usr/bin/python3.8}
VENV_DIR=${FAST_CALIB_VIS_VENV:-$ROOT/.venv-fast-calib-visualization}

if [[ -z "$UV_BIN" ]]; then
  echo "uv not found. Install with: python3 -m pip install --user uv" >&2
  exit 1
fi
if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Python 3.8 not found at $PYTHON_BIN. ROS Noetic visualization needs Python 3.8." >&2
  exit 1
fi

set +u
source /opt/ros/noetic/setup.bash
set -u

if [[ ! -d "$VENV_DIR" ]]; then
  "$UV_BIN" venv --python "$PYTHON_BIN" --system-site-packages "$VENV_DIR"
fi

# Keep pip dependencies isolated while preserving ROS/cv_bridge from system site packages.
"$UV_BIN" pip install --python "$VENV_DIR/bin/python" -r "$VIS_DIR/requirements.txt"

exec "$VENV_DIR/bin/python" "$VIS_DIR/visualize_lidar_camera_projection.py" "$@"
