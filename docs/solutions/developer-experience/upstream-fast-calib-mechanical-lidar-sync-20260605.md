---
module: FAST-Calib Sensor Suite
date: 2026-06-05
problem_type: developer_experience
component: tooling
symptoms:
  - "Need to update local FAST-Calib to upstream main without losing D455, Helios, M1 configs and desktop workflow"
  - "Mechanical RoboSense Helios should use upstream ring-aware LiDAR detection"
  - "Remote shell commands produced quoting errors during patching"
  - "ROS setup failed under the remote default zsh invocation"
root_cause: missing_workflow_step
resolution_type: workflow_improvement
severity: medium
tags: [fast-calib, upstream-sync, mechanical-lidar, robosense-helios, desktop-workflow, ros-noetic]
---

# Troubleshooting: Upstream FAST-Calib Sync for RoboSense Mechanical LiDAR

## Problem
The local NUC FAST-Calib checkout had machine-specific D455, RoboSense Helios, RoboSense M1, Kalibr, and desktop workflow changes on top of an older FAST-Calib base. The task was to move to upstream `hku-mars/main` while keeping those local workflows usable.

## Environment
- Module: FAST-Calib Sensor Suite
- Host: NUC11PHi7, user `zcy`
- Workspace: `/home/zcy/fast_calib_ws/src/FAST-Calib`
- ROS: Noetic
- Upstream base after sync: `hku-mars/main` at `1018ecf`
- New local branch: `teleop-upstream-mech`
- Date: 2026-06-05

## Symptoms
- Local `teleop` branch contained many uncommitted machine-specific files and deletions.
- Direct checkout/reset to upstream would have dropped local config, launch, Kalibr pipeline, tools, and docs.
- Upstream latest changed the core LiDAR path from XYZ-only `pcl::PointXYZ` to `Common::Point` with `ring` and separate solid/mechanical LiDAR detection.
- Shell quoting failed when patch commands embedded C++/Python strings inside `ssh '...'`.
- `source /opt/ros/noetic/setup.bash` failed under remote default `zsh` with `/home/zcy/fast_calib_ws/setup.sh` errors.

## What Didn't Work

**Directly treating the local branch as clean:**
- **Why it failed:** `git status` showed dozens of modified/deleted/untracked files. A direct reset would have lost local machine workflow.

**Using one large quoted SSH command for Python/perl patching:**
- **Why it failed:** single quotes inside Python/C++ strings broke the outer shell quote. This produced errors like `zsh: parse error near '}'` and Perl syntax errors around `${OpenCV_LIBRARIES}`.

**Running ROS build setup through the default remote shell:**
- **Why it failed:** remote `ssh 'cd ... && source ...'` used `zsh`, while ROS setup scripts are bash-oriented in this environment. The reliable form was `bash -lc 'source ... && catkin build ...'`.

## Solution

1. Preserve the old state before switching:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
snap=/tmp/fast_calib_teleop_snapshot_$(date +%Y%m%d_%H%M%S)
rsync -a --exclude .git ./ "$snap/"
git stash push -u -m "pre-upstream-mech-update-$(date +%Y%m%d_%H%M%S)"
```

2. Move the working branch to upstream latest without applying the old tree wholesale:

```bash
git fetch https://github.com/hku-mars/FAST-Calib.git main:refs/remotes/hku-mars/main
git checkout -B teleop-upstream-mech hku-mars/main
```

3. Restore only machine-specific files from the snapshot:

- `config/sensor_suite_*_d455.yaml`
- `config/kalibr/`
- `launch/sensor_suite_*_d455*.launch`
- local docs under `docs/`
- Kalibr pipeline wrappers under `pipelines/`, `scripts/`, `tools/`, `third_party/kalibr/`
- local README describing this NUC workspace

4. Keep upstream core mechanical LiDAR code:

- `include/common_lib.h` now uses `PCL_NO_PRECOMPILE` and `Common::Point` with `ring`.
- `src/data_preprocess.hpp` reads `ring` from `sensor_msgs/PointCloud2` when present.
- `src/lidar_detect.hpp` routes mechanical LiDAR through `detect_mech_lidar()`.
- `src/main.cpp` dispatches based on detected LiDAR type.

5. Reapply local compatibility changes on top of upstream core:

- Add back `Params.scene_id` and `sceneOutputName()` so per-scene outputs do not overwrite each other.
- Add `/usr/lib/x86_64-linux-gnu/libusb-1.0.so` to `fast_calib` and `multi_fast_calib` link libraries for this NUC.
- Update RoboSense config comments to clarify that raw `/rslidar_points_*` is still used, but upstream now consumes `ring` for mechanical LiDAR detection.
- Update `/home/zcy/calibration_env/desktop_shortcuts/check_sensor_sync.sh` to check required `PointCloud2` fields: `x y z ring timestamp`.

6. Build using bash:

```bash
cd /home/zcy/fast_calib_ws
bash -lc 'source /opt/ros/noetic/setup.bash && catkin build fast_calib'
```

Result: `fast_calib` built successfully.

## Compatibility Test on 2026-06-05

Live sensor checks with PTP running:

- `/rslidar_points_helios`: about 10 Hz, fields `x y z intensity ring timestamp`.
- `/rslidar_points_m1`: about 10 Hz, fields `x y z intensity ring timestamp`.
- `/camera/color/image_raw`: about 30 Hz.
- `check_sensor_sync.sh`: PASS after PTP was started first.

Negative test:

- Recording before PTP produced valid-looking topics and point fields, but the selected D455 image was about `1780651707s` away from the LiDAR target stamp.
- This is a time-domain problem, not a FAST-Calib format problem.
- The desktop workflow order must be: start PTP sync, start sensors, check status, record data, then run calibration.

PTP-backed Helios+D455 recording:

- Run ID: `compat_ptp_20260605_b`.
- Bag: `/home/zcy/calibrationdata/sensor_suite/fast_calib/robosense_helios_d455/runs/compat_ptp_20260605_b/scenes/scene_01/raw/input.bag`.
- Duration: `2.8s`.
- Topics: `/rslidar_points_helios` 28 messages, `/camera/color/image_raw/compressed` 82 messages, `/camera/color/camera_info` 83 messages.
- Image extraction delta: `0.010518789s`.

Actual FAST-Calib run on that bag:

- Loaded `1612800` Helios points.
- Extracted `1937` edge points using `mechanical LiDAR by neighbor distance`.
- Saved `circle_center_record.txt`.
- RMSE: `0.0072 m`.
- Saved `single_calib_result_scene_01.txt`, `qr_detect_scene_01.png`, and `colored_cloud_scene_01.pcd`.

Regression found and fixed:

- The launch path passed `output_path`, but the desktop script did not guarantee that the result directory existed.
- FAST-Calib could compute the extrinsic but failed when writing `circle_center_record.txt` or PCD output.
- Fix: `fast_calib_prepare_paths()` now creates the raw, image, metadata, and `FAST_CALIB_OUTPUT_PATH` directories before launching calibration.

## Data Loop Smoke Test on 2026-06-05 21:01

End-to-end order tested through persistent SSH/tmux:

```text
start PTP -> start sensor drivers -> check sync -> record -> extract image -> run FAST-Calib
```

Result:

- PTP and driver startup worked.
- `check_sensor_sync.sh` passed with phc2sys max recent offset `144ns`.
- M1 and Helios both published about `10Hz`.
- D455 color image and camera info published about `29Hz`.
- M1 and Helios `PointCloud2` fields included `x y z intensity ring timestamp`.

Helios+D455 smoke recording:

- Run ID: `loop_helios_20260605_2103`.
- Duration: `2.7s`.
- Topics: `/rslidar_points_helios` 27 messages, `/camera/color/image_raw/compressed` 82 messages, `/camera/color/camera_info` 82 messages.
- Image extraction delta: `-0.006351471s`.
- FAST-Calib loaded `1555200` points and extracted `817` mechanical-LiDAR edge points.
- It produced QR and colored-cloud outputs, proving the data path is connected.
- Calibration was not valid because no LiDAR target center set matched the board geometry; the resulting extrinsic was a zero matrix.

M1+D455 smoke recording:

- Run ID: `loop_m1_20260605_2105`.
- Duration: `2.7s`.
- Topics: `/rslidar_points_m1` 27 messages, `/camera/color/image_raw/compressed` 79 messages, `/camera/color/camera_info` 79 messages.
- Image extraction delta: `-0.002599955s`.
- FAST-Calib loaded `2126250` points and extracted `5837` mechanical-LiDAR edge points.
- It produced QR and colored-cloud outputs, proving the M1 data path is connected.
- Calibration was not valid because the detected LiDAR circles did not match the target geometry; the resulting extrinsic was a zero matrix.

Interpretation:

- The sensor-to-bag-to-image-to-FAST-Calib data loop is open for both Helios+D455 and M1+D455.
- A valid calibration still requires a capture where the target board is clearly visible to both D455 and the selected LiDAR, with enough circle returns to match the target geometry.
- A zero matrix result after successful data loading should be treated as a target visibility/detection failure, not as a format conversion problem.

## Why This Works

The important distinction is that upstream changed the algorithmic data path, not the local machine workflow. The correct merge strategy is therefore not a full overwrite. It is:

```text
upstream latest core algorithm
+ local D455/Helios/M1 configs
+ local launch wrappers
+ local desktop data collection workflow
+ local Kalibr RGB/thermal pipeline
```

RoboSense Helios should stay on the raw topic:

```text
/rslidar_points_helios
```

The latest FAST-Calib can use it directly if the `PointCloud2` includes `ring`. Converting to Velodyne format is unnecessary for FAST-Calib and would create another topic/config synchronization problem.

## Prevention

- Before upstream sync, always create both a git stash and an out-of-git snapshot of the working tree.
- Do not apply local snapshot files over upstream core files blindly.
- Treat `config/`, `launch/`, `docs/`, `scripts/`, `tools/`, and desktop scripts as separate workflow layers from `src/` and `include/` algorithm code.
- Use `ssh host 'bash -s' <<'REMOTE'` for complex remote scripts to avoid quote breakage.
- Use `bash -lc` for ROS build commands on this NUC.
- For mechanical LiDAR readiness, check actual `PointCloud2` fields before recording calibration data.

## Related Issues

No related issues documented yet.
