# FAST-Calib Sensor Suite Workspace

This checkout is the active calibration workspace for this NUC.

Active hardware:

- RealSense D455
- FLIR Boson
- Robosense M1
- Robosense Helios

Kalibr is intentionally kept outside this ROS workspace:

```text
/home/zcy/kalibr_ws
```

Large bags and generated calibration outputs are intentionally kept outside the source tree:

```text
/home/zcy/calibrationdata/sensor_suite
```

## Active Workflows

### D455 + Boson Kalibr

Record:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/record_kalibr_d455_boson.sh
```

Calibrate:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/run_kalibr_d455_boson.sh \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag
```

Visualize:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/target_plane_overlay_d455_boson.sh \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/output/strict_sync_20260604/sync002_freq2/input-camchain.yaml \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag
```

### Robosense M1 + D455 FAST-Calib

Use the desktop shortcuts for record/run, or launch directly:

```bash
roslaunch fast_calib sensor_suite_robosense_m1_d455.launch
roslaunch fast_calib sensor_suite_robosense_m1_d455_multi.launch
```

### Robosense Helios + D455 FAST-Calib

Use the desktop shortcuts for record/run, or launch directly:

```bash
roslaunch fast_calib sensor_suite_robosense_helios_d455.launch
roslaunch fast_calib sensor_suite_robosense_helios_d455_multi.launch
```

## Repository Boundaries

- `config/sensor_suite_*.yaml` contains current machine M1/Helios/D455 configs.
- `launch/sensor_suite_*.launch` contains current machine M1/Helios launchers.
- `pipelines/kalibr_rgb_thermal/` contains the active D455/Boson Kalibr pipeline.
- `scripts/record_kalibr_d455_boson.sh`, `scripts/run_kalibr_d455_boson.sh`, and `scripts/target_plane_overlay_d455_boson.sh` are the active D455/Boson wrappers.
- `docs/reference/legacy_sensor_suites/` contains inactive sensor-suite reference material.

Do not put bags, generated Kalibr results, FAST-Calib output, or rebuilt Kalibr source inside this repo.

## Active Intrinsics Contract

Current RGB-LiDAR ground-truth work uses the D455 RGB intrinsics from the
selected D455/Boson Kalibr camchain, `cam0` in:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/d455_boson_20260604_220541/results/kalibr/calib_20260604_223257/input-camchain.yaml
```

Those intrinsics are written into the active FAST-Calib configs:

```text
config/sensor_suite_robosense_m1_d455.yaml
config/sensor_suite_robosense_helios_d455.yaml
config/sensor_suite_d455_common.yaml
```

Do not mix these active configs with `config/qr_params.yaml` when composing
LiDAR -> thermal ground-truth extrinsics; `qr_params.yaml` is retained as
upstream/reference configuration.

## Author Reference Files

Upstream FAST-Calib examples are intentionally kept in their original paths because they are useful references when writing new configs:

```text
config/qr_params.yaml
config/scene_torch.yaml
config/scene_torch.bkup.yaml
launch/calib.launch
launch/multi_calib.launch
launch/scene_torch.launch
launch/scene_torch_multi.launch
scripts/distance_filter_tool.py
```

Do not delete these during cleanup. They are examples/reference files, not this machine's desktop workflow.
