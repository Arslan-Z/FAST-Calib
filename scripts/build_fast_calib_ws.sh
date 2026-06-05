#!/usr/bin/env bash
set -euo pipefail

WS=/home/zcy/fast_calib_ws

set +u
source /opt/ros/noetic/setup.bash
source /home/zcy/ros_noetic_catkin_ws/install_isolated/setup.bash 2>/dev/null || true
set -u

cd "$WS"

if [[ -f build/.built_by ]] && grep -q 'catkin build' build/.built_by; then
  catkin build fast_calib --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
else
  catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
fi

source "$WS/devel/setup.bash"
rospack find fast_calib
