# Mid360 to Thermal Ground-Truth Strategy

Goal: obtain a defensible reference transform from Livox Mid360 to each thermal
camera in order to evaluate LiDAR-thermal targetless calibration.

## Available Calibrations

1. RGB checkerboard intrinsics from ROS `camera_calibration`
   - Good target for RGB.
   - Does not solve thermal.
   - Files:
     - `/home/zcy/fast_calib_ws/src/FAST-Calib/camera/rgb/rgb_left/rgb_left.yaml`
     - `/home/zcy/fast_calib_ws/src/FAST-Calib/camera/rgb/rgb_right/rgb_right.yaml`

2. Thermal Apriltag Kalibr intrinsics/extrinsics
   - Good target for thermal.
   - Not usable for RGB because the board reflects badly in RGB.
   - Useful as an independent thermal sanity check, but not directly chained to
     RGB unless a matching RGB-thermal extrinsic is estimated with the same
     thermal intrinsics.

3. Heated asymmetric circle-hole Kalibr RGB-thermal calibration
   - Solves RGB intrinsics, thermal intrinsics, and RGB-thermal extrinsic in one
     bundle.
   - This is the only current calibration that directly connects RGB to thermal.
   - Selected left result:
     `/home/zcy/fast_calib_ws/src/FAST-Calib/output/kalibr/rgb_left_thermal_left_20260528_205814/input-camchain.yaml`
   - Selected right result:
     `/home/zcy/fast_calib_ws/src/FAST-Calib/output/kalibr/rgb_right_thermal_right_20260529_233834__kalibr_20260529_235524/input-camchain.yaml`

## Decision

For the current two-step ground-truth chain, use the heated circle-hole
RGB-thermal Kalibr camchains as the canonical RGB/thermal camera models.

That means:

- FAST-Calib RGB-Mid360 configs should use the RGB intrinsics from the selected
  RGB-thermal Kalibr camchain.
- RGB-thermal extrinsics should come from the same selected camchain.
- Thermal intrinsics for evaluation/visualization should also come from the
  same selected camchain.

Do not mix ROS checkerboard RGB intrinsics with the heated-circle RGB-thermal
extrinsic unless you re-estimate the RGB-thermal extrinsic with those RGB
intrinsics fixed. Mixing camera models makes the intermediate RGB frame
inconsistent and can bias the chained Mid360-to-thermal transform.

## Why Not Use the ROS Checkerboard RGB Intrinsics Here?

The ROS checkerboard RGB intrinsics are likely good for the RGB camera by
themselves. However, the final transform is a chain:

```text
T_thermal_mid360 = T_thermal_rgb * T_rgb_mid360
```

`T_thermal_rgb` was estimated by Kalibr together with a specific RGB pinhole
model. If `T_rgb_mid360` is estimated with a materially different RGB pinhole
model, the two transforms are not strictly in the same calibrated camera model.

The difference is not tiny:

- left ROS checkerboard RGB fx/fy: about `693.6 / 694.0`
- left selected Kalibr RGB fx/fy: about `714.5 / 713.3`
- right ROS checkerboard RGB fx/fy: about `696.1 / 696.5`
- right selected Kalibr RGB fx/fy: about `711.2 / 709.5`

For a ground-truth chain, model consistency is more important than choosing the
individually nicest-looking intrinsic file from a different calibration target.

## Selected RGB Intrinsics for FAST-Calib

Left RGB from selected left RGB-thermal Kalibr camchain:

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

Right RGB from selected right RGB-thermal Kalibr camchain:

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

These values are now used by the six `sensor_suite_rgb_*_mid360_scene*.yaml`
FAST-Calib configs.

## Chain Composition

After FAST-Calib:

- left RGB-Mid360 result:
  `FAST-Calib/output/sensor_suite_rgb_left_mid360/multi_calib_result.txt`
- right RGB-Mid360 result:
  `FAST-Calib/output/sensor_suite_rgb_right_mid360/multi_calib_result.txt`

Those files contain `T_cam_lidar`, i.e. RGB-camera-from-Mid360:

```text
T_rgb_mid360
```

Kalibr camchain contains the thermal camera transform relative to RGB. In the
selected camchains, `cam0` is RGB and `cam1` is thermal:

```text
T_thermal_rgb = T_cam1_cam0
```

Compose:

```text
T_thermal_mid360 = T_thermal_rgb * T_rgb_mid360
```

Use the left selected camchain for thermal-left, and the right selected
camchain for thermal-right.

## Validation Before Trusting as Ground Truth

Run at least two independent FAST-Calib captures for each side. Trust the chain
only if:

- FAST-Calib single-scene RMSE is low in all three scenes.
- FAST-Calib bags use `/livox/points` (`sensor_msgs/PointCloud2`) with this
  checkout, not `/livox/lidar` (`livox_ros_driver2/CustomMsg`).
- `circle_center_record.txt` contains exactly the intended latest three scenes.
- multi-scene `T_rgb_mid360` is stable across repeated captures.
- composed `T_thermal_mid360` is physically plausible for the rig.
- qualitative projection of Mid360 points into thermal roughly matches depth
  discontinuities or visible target/scene structure.

The result is a strong engineering reference, not a metrology-grade absolute
truth. The main remaining systematic risks are target manufacturing tolerances,
heating/thermal centroid bias, and any RGB-thermal Kalibr model bias from the
heated circle-hole board.
