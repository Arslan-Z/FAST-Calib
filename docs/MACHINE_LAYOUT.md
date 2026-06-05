# Machine Layout

This file defines the intended filesystem boundary for calibration work on this NUC.

## Source Workspaces

```text
/home/zcy/fast_calib_ws
/home/zcy/kalibr_ws
/home/zcy/sensor_ws
```

- `fast_calib_ws` contains this FAST-Calib source checkout and active wrappers.
- `kalibr_ws` contains the rebuilt Kalibr ROS workspace.
- `sensor_ws` contains sensor drivers and runtime ROS launch support.

## Data Roots

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson
/home/zcy/calibrationdata/sensor_suite/fast_calib/robosense_m1_d455
/home/zcy/calibrationdata/sensor_suite/fast_calib/robosense_helios_d455
```

Generated bags, camchains, reports, overlays, and FAST-Calib run outputs belong under these roots, not inside the source tree.

## Desktop Entrypoints

Desktop shortcuts under `/home/zcy/Desktop` should only expose workflows for:

- D455 + Boson Kalibr
- M1 + D455 FAST-Calib / Velo2Cam / RViz
- Helios + D455 FAST-Calib / Velo2Cam / RViz
- sensor start/stop/status utilities

Inactive sensor-suite shortcuts should not be reintroduced unless the matching hardware is physically installed.

## Author Examples Kept In Place

The upstream FAST-Calib examples are kept in their original paths for reference when writing new configs:

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

These files are examples, not this machine's desktop/default workflow. Do not delete them during cleanup.

## Legacy References

Inactive sensor-suite material is preserved under:

```text
docs/reference/legacy_sensor_suites
```

These files are reference-only and should not be wired into active desktop shortcuts unless the matching hardware is physically installed.
