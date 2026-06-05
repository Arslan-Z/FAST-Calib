# LiDAR-Thermal Ground Truth Chain and Static Capture Plan

Date: 2026-06-05
Machine: NUC11PHi7
Workspace: `/home/zcy/fast_calib_ws/src/FAST-Calib`

## Goal

Use board-based calibration as ground truth for evaluating boardless LiDAR-thermal calibration:

1. Use Kalibr to calibrate D455 RGB and Boson thermal.
2. Use FAST-Calib to calibrate RoboSense Helios/M1 LiDAR and D455 RGB.
3. Compose the transforms to obtain LiDAR-to-thermal extrinsics.
4. Use that composed transform as the reference for boardless LiDAR-thermal calibration experiments.

## Verified Camera Intrinsics Contract

Kalibr RGB/thermal run used:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/d455_boson_20260604_220541/results/kalibr/calib_20260604_223257/input-camchain.yaml
```

Kalibr `cam0` is RGB:

```text
rostopic: /camera/color/image_raw
intrinsics: [648.7587039754668, 645.3432750164105, 618.3136242243808, 377.63382093027053]
distortion_coeffs: [-0.05763365848452647, 0.07152720319364425, 0.005078426073825444, -0.007102057999695309]
```

FAST-Calib RGB-LiDAR configs were checked against that Kalibr `cam0`:

```text
config/sensor_suite_d455_common.yaml: PASS
config/sensor_suite_robosense_helios_d455.yaml: PASS
config/sensor_suite_robosense_m1_d455.yaml: PASS
```

All differences were below `1e-7`, so the RGB camera model is shared across RGB-thermal and RGB-LiDAR calibration.

Do not use `config/qr_params.yaml` for this sensor-suite ground truth. It is an upstream/example config with old intrinsics and `/livox/lidar`. The workflow should use the sensor-suite launch files instead:

```text
sensor_suite_robosense_helios_d455.launch
sensor_suite_robosense_m1_d455.launch
```

## Transform Composition

Kalibr topic order is RGB then thermal:

```text
/camera/color/image_raw /flir_boson/image_raw
```

Kalibr output convention:

```text
cam1.T_cn_cnm1 = T_thermal_rgb
```

FAST-Calib output convention for this workflow:

```text
T_cam_lidar = T_rgb_lidar
```

Therefore:

```text
T_thermal_lidar = T_thermal_rgb * T_rgb_lidar
T_lidar_thermal = inverse(T_thermal_lidar)
```

Use `T_thermal_lidar` when projecting LiDAR points into the thermal camera frame. Use `T_lidar_thermal` only when the downstream algorithm explicitly expects thermal-to-LiDAR or LiDAR-frame pose notation.

## Current Data and Time-Sync Findings

Startup order must stay fixed:

```text
00 Start Sensor PTP Time Sync
01 Start Sensor Drivers
03 Check Sensor Sync
record data
run calibration
```

Latest full sync check passed with:

```text
phc2sys max recent sys offset: 144 ns
/rslidar_points_m1: about 10 Hz
/rslidar_points_helios: about 10 Hz
/camera/color/image_raw: about 29 Hz
/camera/color/camera_info: about 29 Hz
M1/Helios PointCloud2 fields: x y z intensity ring timestamp
```

RGB/thermal Kalibr bag time report for:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/d455_boson_20260604_220541/raw/input.bag
```

showed:

```text
RGB rate: 28.957 Hz
thermal rate: 8.565 Hz
strict_sync=0.020s keeps 97.4% of thermal frames by header stamp
RGB receive-header latency median: 0.043 s
thermal receive-header latency median: 0.000 s
```

Interpretation:

- `strict_sync=0.020s` is reasonable for Kalibr RGB/thermal calibration.
- RGB and thermal header semantics are not identical.
- During motion, the 30 Hz / 9 Hz / 10 Hz frame-rate mismatch can produce real spatial mismatch even if nearest timestamps are acceptable.
- For boardless evaluation captures, record only after objects and sensors are static.

## Data Storage Contract

FAST-Calib RGB-LiDAR data should stay under:

```text
/home/zcy/calibrationdata/sensor_suite/fast_calib/<sensor_slug>/runs/<run_id>/
  scenes/<scene_id>/raw/input.bag
  scenes/<scene_id>/image/image.png
  scenes/<scene_id>/metadata/image_sync_metadata.txt
  scenes/<scene_id>/metadata/scene_manifest.env
  results/fast_calib/
```

Kalibr RGB-thermal data should stay under:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/<run_id>/
  raw/input.bag
  run_manifest.env
  results/kalibr/<calib_id>/
```

Boardless LiDAR-thermal evaluation data should use a separate root to avoid mixing with board-based ground truth:

```text
/home/zcy/calibrationdata/sensor_suite/boardless_lidar_thermal/<suite_slug>/runs/<run_id>/
  captures/<capture_id>/raw/input.bag
  captures/<capture_id>/metadata/capture_manifest.env
  captures/<capture_id>/metadata/time_sync_report.txt
  results/<algorithm_name>/
```

Suggested `suite_slug` values:

```text
helios_boson_d455
m1_boson_d455
```

## Boardless Static Capture Workflow Proposal

The intended manual workflow is:

1. Start PTP sync.
2. Start all sensor drivers.
3. Run sensor sync check.
4. Move objects in front of the sensors.
5. Wait until objects are static.
6. Click a desktop shortcut to record exactly 10 seconds.
7. Move objects again.
8. Repeat step 6 for the next static arrangement.

The capture script should:

- Refuse to run unless PTP, LiDAR, RGB, and thermal topics are alive.
- Run `check_sensor_sync.sh` or a shorter equivalent preflight before recording.
- Auto-increment `capture_id` as `capture_001`, `capture_002`, etc.
- Record exactly 10 seconds by default.
- Record topics:

```text
/rslidar_points_helios or /rslidar_points_m1
/camera/color/image_raw/compressed
/camera/color/camera_info
/flir_boson/image_raw
/flir_boson/camera_info
```

- Write a manifest containing run ID, capture ID, topic list, duration, start time, and active PTP log path.
- After recording, generate a time-sync report for RGB/thermal and LiDAR/RGB nearest-frame deltas.
- Never overwrite an existing capture directory unless an explicit force flag is passed.

Recommended desktop layout:

```text
40-BOARDLESS-LIDAR-THERMAL/
  40 New Boardless Run.desktop
  41 Record 10s Helios Thermal Static.desktop
  42 Record 10s M1 Thermal Static.desktop
  43 Open Boardless Data Folder.desktop
```

The shortcuts should be after sync/status and after board-based ground-truth workflows, because this dataset depends on the same time-sync discipline.

## Current Issues Found and Fixed

Fixed on 2026-06-05:

- Sensor-suite launch defaults used an old direct path shape. They now default to the same `runs/<run_id>/scenes/<scene_id>/raw` and `results/fast_calib` layout as the desktop scripts.
- `calib_from_config.launch` and `multi_calib_from_config.launch` no longer default to upstream `qr_params.yaml`; they default to the Helios+D455 sensor-suite config.
- The desktop FAST-Calib path preparation creates raw, image, metadata, and result directories before launching calibration.

Remaining caution:

- `qr_params.yaml` is still present as an upstream example. Do not use it for D455/RoboSense ground-truth work.
- A successful data loop does not guarantee a valid calibration result. If FAST-Calib outputs a zero matrix after loading data, treat it as a target visibility/detection failure unless topic and timestamp checks also fail.
