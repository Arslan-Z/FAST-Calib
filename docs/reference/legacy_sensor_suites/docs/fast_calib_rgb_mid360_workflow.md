# FAST-Calib RGB + Mid360 Workflow

This document covers the current sensor-suite workflow for calibrating:

- left RGB camera to Livox Mid360 LiDAR
- right RGB camera to Livox Mid360 LiDAR

Project root:

```bash
/home/zcy/fast_calib_ws/src/FAST-Calib
```

FAST-Calib ROS package:

```bash
/home/zcy/fast_calib_ws/src/FAST-Calib/workspaces/fast_calib_ws/src/FAST-Calib
```

## What FAST-Calib Expects

FAST-Calib is not a synchronized online RGB/LiDAR calibrator. In the current
code it loads:

- one static RGB image from `image_path`
- one rosbag containing the LiDAR point cloud topic from `bag_path`

The board must not move during each scene capture. For a multi-scene result,
run three single-scene calibrations with different board placements, then run
`multi_fast_calib`.

The output transform printed by FAST-Calib is:

```text
T_cam_lidar
```

This maps LiDAR points into the RGB camera frame. Saved `Rcl` / `Pcl` have the
same meaning: camera-from-lidar.

## Camera Intrinsics Source

For the Mid360-to-thermal ground-truth chain, use the RGB intrinsics from the
selected RGB-thermal Kalibr camchains, not the standalone ROS checkerboard RGB
intrinsics.

Reason: the final chain is:

```text
T_thermal_mid360 = T_thermal_rgb * T_rgb_mid360
```

`T_thermal_rgb` comes from RGB-thermal Kalibr, so `T_rgb_mid360` should use the
same RGB camera model.

The standalone ROS checkerboard RGB intrinsics are still useful as an
independent reference:

```text
/home/zcy/fast_calib_ws/src/FAST-Calib/camera/rgb/rgb_left/rgb_left.yaml
/home/zcy/fast_calib_ws/src/FAST-Calib/camera/rgb/rgb_right/rgb_right.yaml
```

But the active FAST-Calib configs now use these selected RGB-thermal Kalibr
camchains:

```text
/home/zcy/fast_calib_ws/src/FAST-Calib/output/kalibr/rgb_left_thermal_left_20260528_205814/input-camchain.yaml
/home/zcy/fast_calib_ws/src/FAST-Calib/output/kalibr/rgb_right_thermal_right_20260529_233834__kalibr_20260529_235524/input-camchain.yaml
```

Left RGB values used in the configs:

```yaml
fx: 714.52028130
fy: 713.28393330
cx: 410.65414095
cy: 335.02442555
k1: -0.01740054
k2: 0.12156711
p1: 0.00560514
p2: -0.00334984
```

Right RGB values used in the configs:

```yaml
fx: 711.22882463
fy: 709.45355019
cx: 405.91403922
cy: 318.44975346
k1: -0.04747120
k2: 0.14071297
p1: 0.00090104
p2: -0.00210056
```

FAST-Calib only reads `k1`, `k2`, `p1`, and `p2`, then internally sets `k3=0`.
The selected Kalibr `radtan` model has the same four coefficients.

Detailed rationale:

```text
/home/zcy/fast_calib_ws/src/FAST-Calib/docs/lidar_thermal_ground_truth_strategy.md
```

## Target Parameters

All new configs set:

```yaml
marker_size: 0.16
```

This is the ArUco marker side length in meters. The rest of the board geometry
currently follows the FAST-Calib target definition already used by the package:

```yaml
delta_width_qr_center: 0.55
delta_height_qr_center: 0.35
delta_width_circles: 0.5
delta_height_circles: 0.4
circle_radius: 0.12
min_detected_markers: 3
```

## Prepared Config Files

Left RGB + Mid360:

```text
config/sensor_suite_rgb_left_mid360_scene01.yaml
config/sensor_suite_rgb_left_mid360_scene02.yaml
config/sensor_suite_rgb_left_mid360_scene03.yaml
```

Right RGB + Mid360:

```text
config/sensor_suite_rgb_right_mid360_scene01.yaml
config/sensor_suite_rgb_right_mid360_scene02.yaml
config/sensor_suite_rgb_right_mid360_scene03.yaml
```

The scene files differ only in input paths and the LiDAR distance filter. The
three scene filters are starting points:

- scene01: centered/front board placement
- scene02: board toward one side of the LiDAR FoV
- scene03: board toward the opposite side of the LiDAR FoV

If FAST-Calib fails to find four LiDAR circle centers, tune `x_min/x_max`,
`y_min/y_max`, and `z_min/z_max` for that scene.

## Prepared Launch Files

These launch files avoid overwriting the original `qr_params.yaml`:

```text
launch/calib_from_config.launch
launch/multi_calib_from_config.launch
```

They take a `config_file` argument and load that YAML with `subst_value="true"`,
so `$(find fast_calib)` paths inside config files are expanded.

## Simplified Command Wrapper

Use this wrapper for normal operation:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/fast_calib_mid360.sh <left|right> <action> [scene]
```

Common actions:

```bash
./scripts/fast_calib_mid360.sh left prepare
./scripts/fast_calib_mid360.sh left paths
./scripts/fast_calib_mid360.sh left reset
./scripts/fast_calib_mid360.sh left capture 1
./scripts/fast_calib_mid360.sh left image 1
./scripts/fast_calib_mid360.sh left bag 1
./scripts/fast_calib_mid360.sh left run 1
./scripts/fast_calib_mid360.sh left multi
```

Use `right` instead of `left` for the right RGB camera.

The wrapper sources the FAST-Calib workspace, resolves `rospack find
fast_calib`, picks the correct side-specific config file, and keeps left/right
output directories separate.

## Data Layout

Create these directories before collecting data:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/fast_calib_mid360.sh left prepare
./scripts/fast_calib_mid360.sh right prepare
```

Expected left files:

```text
calib_data/sensor_suite_rgb_left_mid360/scene01.bag
calib_data/sensor_suite_rgb_left_mid360/scene01.ppm
calib_data/sensor_suite_rgb_left_mid360/scene02.bag
calib_data/sensor_suite_rgb_left_mid360/scene02.ppm
calib_data/sensor_suite_rgb_left_mid360/scene03.bag
calib_data/sensor_suite_rgb_left_mid360/scene03.ppm
```

Expected right files:

```text
calib_data/sensor_suite_rgb_right_mid360/scene01.bag
calib_data/sensor_suite_rgb_right_mid360/scene01.ppm
calib_data/sensor_suite_rgb_right_mid360/scene02.bag
calib_data/sensor_suite_rgb_right_mid360/scene02.ppm
calib_data/sensor_suite_rgb_right_mid360/scene03.bag
calib_data/sensor_suite_rgb_right_mid360/scene03.ppm
```

## Capture Procedure

Start the sensor drivers first. Then verify topics:

```bash
rostopic list | grep -E '/rgb_left|/rgb_right|/livox/points'
```

The configs currently use:

```yaml
lidar_topic: "/livox/points"
```

The current ROS graph also exposes `/livox/lidar`, but that topic is
`livox_ros_driver2/CustomMsg`. This FAST-Calib checkout only handles the older
`livox_ros_driver/CustomMsg` type or generic `sensor_msgs/PointCloud2`, so use
`/livox/points` for this workspace unless the code is updated for driver2
custom messages.

For each scene, keep the target board static, save one RGB image, and record a
short LiDAR bag. The shortest command is `capture`, which saves one image and
then records a short Mid360 bag:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/fast_calib_mid360.sh left capture 1
```

Default bag duration is 8 seconds. Override it if needed:

```bash
BAG_SECONDS=12 ./scripts/fast_calib_mid360.sh left capture 1
```

Use separate `image` and `bag` commands only if you want manual control.

Left scene01 example:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/fast_calib_mid360.sh left image 1
```

Stop `image_saver` after it writes the image. Then record LiDAR for a few
seconds while the board remains still:

```bash
./scripts/fast_calib_mid360.sh left bag 1
```

Repeat for `scene02` and `scene03`. For the right camera, use `/rgb_right` and
the `sensor_suite_rgb_right_mid360` directory.

## Run Left RGB + Mid360

Clear the left output record before starting a new left calibration experiment.
This prevents `multi_fast_calib` from mixing old and new scene records:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/fast_calib_mid360.sh left reset
```

Run the three single-scene calibrations:

```bash
./scripts/fast_calib_mid360.sh left run 1
./scripts/fast_calib_mid360.sh left run 2
./scripts/fast_calib_mid360.sh left run 3
```

Each run should print:

```text
[Result] Single-scene calibration: extrinsic parameters T_cam_lidar =
[Record] Saved four pairs of circular hole centers ...
```

After all three scenes, run multi-scene calibration:

```bash
./scripts/fast_calib_mid360.sh left multi
```

Left result files:

```text
output/sensor_suite_rgb_left_mid360/circle_center_record.txt
output/sensor_suite_rgb_left_mid360/single_calib_result.txt
output/sensor_suite_rgb_left_mid360/multi_calib_result.txt
output/sensor_suite_rgb_left_mid360/colored_cloud.pcd
output/sensor_suite_rgb_left_mid360/qr_detect.ppm
```

Use `multi_calib_result.txt` as the final left RGB camera-from-Mid360 extrinsic
after the three scene runs.

## Run Right RGB + Mid360

Clear the right output record before starting a new right calibration
experiment:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/fast_calib_mid360.sh right reset
```

Run the three single-scene calibrations:

```bash
./scripts/fast_calib_mid360.sh right run 1
./scripts/fast_calib_mid360.sh right run 2
./scripts/fast_calib_mid360.sh right run 3
```

Then run multi-scene calibration:

```bash
./scripts/fast_calib_mid360.sh right multi
```

Right result files:

```text
output/sensor_suite_rgb_right_mid360/circle_center_record.txt
output/sensor_suite_rgb_right_mid360/single_calib_result.txt
output/sensor_suite_rgb_right_mid360/multi_calib_result.txt
output/sensor_suite_rgb_right_mid360/colored_cloud.pcd
output/sensor_suite_rgb_right_mid360/qr_detect.ppm
```

Use `multi_calib_result.txt` as the final right RGB camera-from-Mid360
extrinsic.

## Quality Checks

For every single-scene run:

- `qr_detect.ppm` should show the four ArUco markers and the projected circle
  centers on the RGB image.
- RViz should show filtered cloud, plane cloud, extracted centers, and colored
  point cloud.
- The terminal should report four QR centers and four LiDAR circle centers.
- RMSE should be small. If it is centimeters-level or worse, inspect detection
  and filtering before trusting the result.

For the final multi-scene run:

- Check that `circle_center_record.txt` contains exactly the three scenes you
  intended to use at the end of the file.
- Check that `multi_calib_result.txt` was updated.
- Compare left/right translation direction and magnitude against the physical
  sensor layout.

## Common Failure Modes

If QR/ArUco detection fails:

- confirm the RGB image is from the correct camera side.
- confirm `marker_size: 0.16`.
- confirm the board is not blurred or overexposed.
- confirm camera intrinsics are not swapped between left and right.

If LiDAR circle detection fails:

- tune the distance filter in the scene YAML.
- keep only the board plane and a little surrounding margin.
- use `scripts/distance_filter_tool.py` from the FAST-Calib package to help
  choose filter limits.

If multi-scene uses old data:

- delete `circle_center_record.txt` before starting a new side.
- do not run left and right calibrations into the same output directory.

## Intrinsics Guardrail

Before running, these checks should hold:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib/workspaces/fast_calib_ws/src/FAST-Calib

grep -n 'fx: 693.62863' config/sensor_suite_rgb_left_mid360_scene01.yaml
grep -n 'camera_name: rgb_left' /home/zcy/fast_calib_ws/src/FAST-Calib/camera/rgb/rgb_left/rgb_left.yaml

grep -n 'fx: 696.05781' config/sensor_suite_rgb_right_mid360_scene01.yaml
grep -n 'camera_name: rgb_right' /home/zcy/fast_calib_ws/src/FAST-Calib/camera/rgb/rgb_right/rgb_right.yaml

grep -n 'marker_size: 0.16' config/sensor_suite_rgb_*_mid360_scene*.yaml
```

If any of these fail, stop and fix the config before collecting data.
