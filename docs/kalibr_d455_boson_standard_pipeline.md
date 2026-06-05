# D455 + Boson Standard Kalibr Pipeline

Use the config-driven pipeline in:

```text
pipelines/kalibr_rgb_thermal/
```

The standard D455/Boson config is:

```text
pipelines/kalibr_rgb_thermal/configs/d455_boson.env
```

Kalibr itself is not stored inside FAST-Calib. Source/build Kalibr from the
sibling ROS workspace:

```text
/home/zcy/kalibr_ws
```

Current recommended settings:

```text
KALIBR_APPROX_SYNC=0.02
KALIBR_BAG_FREQ=2.0
KALIBR_VIS_MAX_DT=0.02
target=config/kalibr/targets/acircles_4x11_0p04.yaml
topics=/camera/color/image_raw /flir_boson/image_raw
```

The recommended result from the June 3, 2026 bag is:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/output/strict_sync_20260604/sync002_freq2/input-camchain.yaml
```

Visual quality reports are target-plane reports, not full-scene overlays. Full
scene overlays need depth assumptions; the target-plane report compares detected
thermal circle centers against RGB circle centers projected into thermal.

To regenerate the quality report:

```bash
cd /home/zcy/fast_calib_ws/src/FAST-Calib

./pipelines/kalibr_rgb_thermal/scripts/visualize.sh d455_boson \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/output/strict_sync_20260604/sync002_freq2/input-camchain.yaml \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/d455_boson_20260603_220333.bag \
  /home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/output/strict_sync_20260604/sync002_freq2/target_plane_overlay
```

## Data Layout

New recordings and calibration results follow the standard layout documented in:

```text
docs/calibration_data_layout.md
```

Kalibr D455/Boson runs now use:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/<run_id>/raw/input.bag
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/<run_id>/results/kalibr/calib_<timestamp>/
```

Older top-level bags remain compatible, including the June 3 bag listed above.
