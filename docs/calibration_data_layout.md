# Calibration Data Layout

This machine uses one data root:

```text
/home/zcy/calibrationdata/sensor_suite/
```

The active sensors are:

- RealSense D455
- FLIR Boson
- Robosense M1
- Robosense Helios

## Kalibr: D455 Color <-> Boson Thermal

Each recording is one run:

```text
kalibr/d455_boson/runs/<run_id>/
  raw/input.bag
  run_manifest.env
  results/kalibr/calib_<YYYYmmdd_HHMMSS>/
    input.bag -> ../../raw/input.bag
    calibration_manifest.env
    input-camchain.yaml
    input-results-cam.txt
    input-report-cam.pdf
    target_plane_overlay/index.html
```

Use a readable run id when possible:

```bash
KALIBR_RUN_ID=d455_boson_20260604_static_board_01 ./scripts/record_kalibr_d455_boson.sh
```

If `KALIBR_RUN_ID` is not set, the script generates:

```text
d455_boson_<YYYYmmdd_HHMMSS>
```

`./scripts/run_kalibr_d455_boson.sh` finds the latest
`runs/*/raw/input.bag` automatically. It still accepts older top-level bags for
compatibility.

## FAST-Calib: LiDAR <-> D455

Use one run id for one multi-pose calibration attempt. Use one scene id per
static board pose:

```text
fast_calib/<sensor_pair>/runs/<run_id>/
  scenes/<scene_id>/
    raw/input.bag
    image/image.png
    metadata/image_sync_metadata.txt
    metadata/scene_manifest.env
  results/fast_calib/
    circle_center_record.txt
    FAST-Calib output files for this run
```

Current sensor pairs:

```text
fast_calib/robosense_m1_d455/
fast_calib/robosense_helios_d455/
```

Recommended names:

```bash
FAST_CALIB_RUN_ID=m1_d455_20260604_board01
FAST_CALIB_SCENE_ID=scene_01
FAST_CALIB_SCENE_ID=scene_02
FAST_CALIB_SCENE_ID=scene_03
```

The record scripts write:

```text
fast_calib/<sensor_pair>/.current.env
```

The run scripts read `.current.env`, so `Run FAST ...` uses the same
`input.bag`, `image.png`, and output directory produced by the latest matching
record button.

## Why Manifests Exist

Every run or scene writes a small manifest file. These files make the pairing
explicit:

- which bag produced a result;
- which topics were recorded;
- which image was extracted;
- which output directory belongs to that run.

Do not move a result folder without its manifest and linked input bag.


## Intrinsics Consistency For Chaining

Treat this as the active intrinsics contract for this machine.

The current ground-truth chain uses the D455 RGB frame as the bridge:

```text
T_thermal_lidar = T_thermal_rgb * T_rgb_lidar
```

Therefore RGB-LiDAR calibration must use the same RGB intrinsics as the selected
RGB/thermal Kalibr result. The active FAST-Calib configs use `cam0` from:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/d455_boson_20260604_220541/results/kalibr/calib_20260604_223257/input-camchain.yaml
```

The corresponding active FAST-Calib configs are:

```text
/home/zcy/fast_calib_ws/src/FAST-Calib/config/sensor_suite_robosense_m1_d455.yaml
/home/zcy/fast_calib_ws/src/FAST-Calib/config/sensor_suite_robosense_helios_d455.yaml
/home/zcy/fast_calib_ws/src/FAST-Calib/config/sensor_suite_d455_common.yaml
```

`qr_params.yaml` is retained for upstream-author reference and generic launch
compatibility. It is not the active current-suite RGB-LiDAR config for composing
LiDAR -> thermal ground truth.
