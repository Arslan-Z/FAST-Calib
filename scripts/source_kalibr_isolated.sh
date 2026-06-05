#!/usr/bin/env bash
set +u
source /opt/ros/noetic/setup.bash
source /home/zcy/ros_noetic_catkin_ws/install_isolated/setup.bash 2>/dev/null || true
KALIBR_WS=${KALIBR_WS:-/home/zcy/kalibr_ws}
export CMAKE_PREFIX_PATH="$KALIBR_WS/devel:${CMAKE_PREFIX_PATH:-}"
export ROS_PACKAGE_PATH="$KALIBR_WS/src/kalibr/aslam_offline_calibration:$KALIBR_WS/src/kalibr/aslam_cv:$KALIBR_WS/src/kalibr/aslam_optimizer:$KALIBR_WS/src/kalibr/aslam_nonparametric_estimation:$KALIBR_WS/src/kalibr/Schweizer-Messer:$KALIBR_WS/src/kalibr:${ROS_PACKAGE_PATH:-}"
export PYTHONPATH="$KALIBR_WS/devel/lib/python3/dist-packages:$KALIBR_WS/src/kalibr/aslam_offline_calibration/kalibr/python:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$KALIBR_WS/devel/lib:${LD_LIBRARY_PATH:-}"
export PATH="/opt/ros/noetic/bin:$PATH"
export KALIBR_CALIBRATE_CAMERAS_BIN="$KALIBR_WS/devel/lib/kalibr/kalibr_calibrate_cameras"
set -u
