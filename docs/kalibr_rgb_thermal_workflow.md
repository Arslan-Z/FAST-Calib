# Kalibr RGB/Thermal Workflow

This machine's active RGB/thermal calibration target is RealSense D455 color + FLIR Boson thermal.

Kalibr is kept in a sibling ROS workspace:

```text
/home/zcy/kalibr_ws
```

FAST-Calib keeps the wrapper scripts, configuration, documentation, and visualization tools:

```text
/home/zcy/fast_calib_ws/src/FAST-Calib
```

Large bags and generated calibration outputs live outside the source tree:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson
```

## Standard Commands

Record D455/Boson:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/record_kalibr_d455_boson.sh
```

Run Kalibr on a bag:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib
./scripts/run_kalibr_d455_boson.sh \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag
```

Run the current standard strict-sync setting explicitly:

```bash
KALIBR_APPROX_SYNC=0.02 KALIBR_BAG_FREQ=2.0 \
  ./scripts/run_kalibr_d455_boson.sh \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/output/strict_sync_$(date +%Y%m%d_%H%M%S)
```

Visualize an existing camchain:

```bash
./scripts/target_plane_overlay_d455_boson.sh \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/output/strict_sync_20260604/sync002_freq2/input-camchain.yaml \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag
```

## Current Recommended Result

Use the strict-sync result:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/output/strict_sync_20260604/sync002_freq2
```

Reason: it uses stricter RGB/thermal pairing (`approx-sync=0.02`, `bag-freq=2.0`) while keeping reprojection and target-plane overlay quality stable.


## Time Sync Check

Before trusting a new bag, run:

```bash
./pipelines/kalibr_rgb_thermal/scripts/time_sync_report.sh d455_boson   /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag
```

For the June 3 bag, `approx-sync=0.02` keeps 96.9% of thermal frames by header timestamp, and there is no obvious minute-scale drift. Keep RGB at 30Hz unless driver load causes drops; reducing RGB to 9/10Hz without hardware trigger does not guarantee better synchronization.

## Quality Visualization

The target-plane overlay is the standard extrinsic quality view. It pairs RGB and thermal frames by timestamp, detects the circle target independently in both images, projects RGB target points into thermal with the Kalibr camchain, and reports pixel error.

Expected output files:

- `index.html`
- `*_summary.txt`
- `*_alignment_report.csv`
- `*_error_plot.png`
- `*_poor_mosaic.png`

## Legacy References

Inactive sensor-suite references were moved out of the active workflow. They are kept only for parameter reference under:

```text
docs/reference/legacy_sensor_suites/
```
