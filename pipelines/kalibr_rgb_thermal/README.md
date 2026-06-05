# Kalibr RGB/Thermal Pipeline

This directory is the standard entry point for RealSense D455 color + FLIR Boson thermal Kalibr work on this NUC.

Kalibr itself lives in a separate sibling ROS workspace:

```text
/home/zcy/kalibr_ws
```

Large bags and generated results stay outside the source tree, under:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson
```

## Naming Rules

New recordings use a run directory:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/<run_id>/
  raw/input.bag
  run_manifest.env
  results/kalibr/calib_<timestamp>/
    input.bag -> ../../raw/input.bag
    calibration_manifest.env
    input-camchain.yaml
    input-results-cam.txt
    target_plane_overlay/
      index.html
```

If `KALIBR_RUN_ID` is not set, the record script creates:

```text
d455_boson_<YYYYmmdd_HHMMSS>
```

Set `KALIBR_RUN_ID` when you want a human-readable name:

```bash
KALIBR_RUN_ID=d455_boson_20260604_static_board_01 ./scripts/record_kalibr_d455_boson.sh
```

The calibration script finds the latest `runs/*/raw/input.bag` automatically
and places results under the same run. Older top-level bags like
`d455_boson_20260603_220333.bag` are still accepted for compatibility.

## Structure

- `configs/d455_boson.env` - D455/Boson topic, model, target, and output settings.
- `scripts/record.sh` - record a calibration bag for the configured pair.
- `scripts/calibrate.sh` - run Kalibr and then target-plane visualization.
- `scripts/visualize.sh` - detect target points in RGB/thermal and project RGB detections into thermal with a camchain.
- `scripts/stability_sweep.sh` - run a sync/frequency sweep on one bag.

## Standard Commands

Record:

```bash
./pipelines/kalibr_rgb_thermal/scripts/record.sh d455_boson
```

Calibrate:

```bash
./pipelines/kalibr_rgb_thermal/scripts/calibrate.sh d455_boson \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag
```

Visualize an existing result:

```bash
./pipelines/kalibr_rgb_thermal/scripts/visualize.sh d455_boson \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/output/strict_sync_20260604/sync002_freq2/input-camchain.yaml \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag
```

Run a strict-sync sweep:

```bash
KALIBR_SWEEP="0.03:2.0 0.02:2.0 0.015:2.0" \
  ./pipelines/kalibr_rgb_thermal/scripts/stability_sweep.sh d455_boson \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/output/sweep_$(date +%Y%m%d_%H%M%S)
```


## Time Sync Check

Before trusting a new bag, run:

```bash
./pipelines/kalibr_rgb_thermal/scripts/time_sync_report.sh d455_boson   /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag
```

For the June 3 bag, `approx-sync=0.02` keeps 96.9% of thermal frames by header timestamp, and there is no obvious minute-scale drift. Keep RGB at 30Hz unless driver load causes drops; reducing RGB to 9/10Hz without hardware trigger does not guarantee better synchronization.

## Quality Check

The standard quality check is the target-plane projection report. It pairs RGB and thermal frames by timestamp, detects the circle board independently in each image, projects RGB target points into thermal through Kalibr's extrinsic, and reports per-point pixel error.

Good output contains:

- `index.html` - browser report.
- `*_summary.txt` - frame counts and quality counts.
- `*_alignment_report.csv` - per-frame timestamp gap, detections, and errors.
- `*_error_plot.png` - mean/median/max error over sampled frames.
- `*_poor_mosaic.png` - quick inspection of bad frames.

For D455/Boson, the current default visualization window is `0.02 s`, matching the recommended strict calibration sync.

## Legacy References

Inactive sensor-suite material is archived under docs/reference/legacy_sensor_suites/. It is reference-only and should not be used as the current machine workflow.
