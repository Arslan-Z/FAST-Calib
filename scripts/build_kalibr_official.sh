#!/usr/bin/env bash
set -euo pipefail
WS=${KALIBR_WS:-/home/zcy/kalibr_ws}
ROS_PREFIX=/home/zcy/ros_noetic_catkin_ws/install_isolated
ROS_SETUP="$ROS_PREFIX/setup.bash"
if [[ ! -d "$WS/src/kalibr" ]]; then
  echo "Missing official Kalibr source: $WS/src/kalibr" >&2
  exit 1
fi

set +u
source /opt/ros/noetic/setup.bash
source "$ROS_SETUP" 2>/dev/null || true
set -u
cd "$WS"
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCATKIN_ENABLE_TESTING=OFF -j12
source "$WS/devel/setup.bash"
rospack find kalibr
KALIBR_BIN="$WS/devel/lib/kalibr/kalibr_calibrate_cameras"
if [[ ! -x "$KALIBR_BIN" ]]; then
  echo "Missing Kalibr camera calibration binary: $KALIBR_BIN" >&2
  exit 1
fi
set +e
"$KALIBR_BIN" --help >/tmp/kalibr_calibrate_cameras_help.txt 2>&1
help_code=$?
set -e
if ! grep -q "Calibrate the intrinsics and extrinsics" /tmp/kalibr_calibrate_cameras_help.txt; then
  cat /tmp/kalibr_calibrate_cameras_help.txt >&2
  exit ${help_code:-1}
fi
