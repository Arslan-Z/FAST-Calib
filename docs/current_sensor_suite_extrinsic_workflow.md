# Current Sensor Suite Extrinsic Workflow

Hardware on this NUC:

- RealSense D455
- FLIR Boson
- Robosense M1
- Robosense Helios

Use D455 color as the bridge frame.

## Step A: D455 color <-> FLIR Boson thermal

Use the Kalibr D455/Boson pipeline:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/record_kalibr_d455_boson.sh
./scripts/run_kalibr_d455_boson.sh
./scripts/target_plane_overlay_d455_boson.sh
```

Desktop shortcuts:

- `00 STOP Kalibr Recording`
- `31 Kalibr Record D455 Boson`
- `32 Kalibr STOP Record D455 Boson`
- `33 Kalibr Run D455 Boson`
- `34 Kalibr Open D455 Boson Folder`

Stop recording with the stop shortcut before moving on. The stop scripts send
Ctrl-C first so `rosbag record` can close and index the bag.

## Step B: LiDAR <-> D455

Run each LiDAR separately against the D455 image:

- M1 <-> D455: FAST-Calib single-scene and multi-scene entries.
- Helios <-> D455: FAST-Calib single-scene and multi-scene entries.

The desktop calibration menu groups these as "Step B: LiDAR <-> D455".

## Step C: LiDAR <-> thermal

Do not collect a separate thermal/LiDAR calibration unless needed. Compose
extrinsics through D455:

```text
T_thermal_lidar = T_thermal_d455 * T_d455_lidar
```

Keep every final transform file with its input bag/result directory so it is
clear which D455/Boson and D455/LiDAR runs were composed.


## Intrinsics Consistency

This is a hard rule for the current ground-truth pipeline.

For the LiDAR -> thermal ground-truth chain, use one RGB camera model throughout:

```text
T_thermal_lidar = T_thermal_rgb * T_rgb_lidar
```

The active FAST-Calib D455 configs use `cam0` from the selected D455/Boson
Kalibr result:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/d455_boson_20260604_220541/results/kalibr/calib_20260604_223257/input-camchain.yaml
```

The active FAST-Calib config files already contain those `cam0` intrinsics:

```text
config/sensor_suite_robosense_m1_d455.yaml
config/sensor_suite_robosense_helios_d455.yaml
config/sensor_suite_d455_common.yaml
```

The active desktop buttons and menu entries for M1/D455 and Helios/D455 load
the two `sensor_suite_robosense_*_d455.yaml` configs above. The generic
`config/qr_params.yaml` and `launch/calib*.launch` files are kept as upstream
author/reference material and are not the current sensor-suite RGB-LiDAR
ground-truth path.

Do not mix RealSense factory `/camera/color/camera_info` intrinsics with a
different Kalibr RGB/thermal camchain when composing LiDAR -> thermal extrinsics.
