# FAST-Calib Data Collection Pipeline

Current hardware:

- RealSense D455 color
- Robosense M1
- Robosense Helios

FAST-Calib consumes one camera image plus one LiDAR bag for each static target
pose. The recording pipeline now records both LiDAR and D455 image topics into
the same bag first, then extracts `image.png` from that bag after recording
stops.

## Why This Is Better

The old desktop script saved `image.png` before starting `rosbag record`. That
was convenient, but the saved image did not necessarily correspond to the LiDAR
messages in `input.bag`.

The current pipeline:

1. records LiDAR + D455 compressed image + D455 camera_info into `input.bag`;
2. when recording stops, selects the D455 image whose timestamp is closest to
   the median LiDAR timestamp in that same bag;
3. writes `image.png`;
4. writes `image_sync_metadata.txt` with the selected time delta.

`image.png` and `image_sync_metadata.txt` are written through temporary files
and then renamed into place, so FAST-Calib will not see a half-written image.
The stop scripts wait up to 30 seconds by default before force killing a
recording session, giving rosbag indexing and image extraction time to finish.

For a 10 Hz LiDAR and 30 Hz RGB stream, the nearest RGB frame is normally within
about 0.017 s if timestamps are comparable. Keep the board static while
recording each pose; the selected frame should represent the same stationary
pose as the LiDAR scan accumulation.

## Desktop Flow

Use the calibration menu or desktop buttons:

- `24 Record Calib M1 D455`
- `25 Record Calib Helios D455`
- `26 Stop Calib Recording`

After stopping, check the run directory:

```text
/home/zcy/calibrationdata/sensor_suite/fast_calib/<sensor>/runs/<run_id>/
  .current.env is stored at the sensor root and points to the latest scene
  scenes/<scene_id>/
    raw/input.bag
    image/image.png
    metadata/image_sync_metadata.txt
    metadata/scene_manifest.env
  results/fast_calib/
    circle_center_record.txt
    FAST-Calib output for this run
```

The metadata contains:

- LiDAR topic
- image topic
- LiDAR message count
- image message count
- selected image timestamp delta to median LiDAR timestamp

If the selected delta is much larger than `0.03 s`, do not trust that scene.
Check sensor timestamps and recollect the pose.

## Naming Rules

Use one `FAST_CALIB_RUN_ID` for a set of poses collected for the same physical
calibration attempt. Use a different `FAST_CALIB_SCENE_ID` for each static board
pose:

```bash
export FAST_CALIB_RUN_ID=m1_d455_20260604_table01
export FAST_CALIB_SCENE_ID=scene_01
```

The desktop menu has an environment helper for this. The record scripts write
the latest paths to:

```text
/home/zcy/calibrationdata/sensor_suite/fast_calib/<sensor>/.current.env
```

The FAST-Calib run scripts read `.current.env`, so the `Run FAST ...` buttons
use the same bag/image paths that the record button produced.

## Reference Material

Original author/reference config and launch files are kept for writing future
configs:

```text
config/qr_params.yaml
config/scene_torch.yaml
config/scene_torch.bkup.yaml
launch/calib.launch
launch/multi_calib.launch
launch/scene_torch.launch
launch/scene_torch_multi.launch
docs/reference/legacy_sensor_suites/
```

The archived legacy sensor-suite files are reference-only. The active NUC
pipeline is D455 + Boson + M1 + Helios.


## RGB Intrinsics Source

This is not optional for the LiDAR -> thermal ground-truth chain.

FAST-Calib M1/D455 and Helios/D455 configs must use the same D455 RGB intrinsics
as the RGB/thermal Kalibr result used in the final chain. The active source is
`cam0` from:

```text
/home/zcy/calibrationdata/sensor_suite/kalibr/d455_boson/runs/d455_boson_20260604_220541/results/kalibr/calib_20260604_223257/input-camchain.yaml
```

This keeps the intermediate RGB frame consistent when composing:

```text
T_thermal_lidar = T_thermal_rgb * T_rgb_lidar
```

The active configs have already been updated to this source:

```text
config/sensor_suite_robosense_m1_d455.yaml
config/sensor_suite_robosense_helios_d455.yaml
config/sensor_suite_d455_common.yaml
```

The active launch files are:

```text
launch/sensor_suite_robosense_m1_d455.launch
launch/sensor_suite_robosense_helios_d455.launch
launch/sensor_suite_robosense_m1_d455_multi.launch
launch/sensor_suite_robosense_helios_d455_multi.launch
```

These launch files load the active configs above. Do not use `qr_params.yaml`
for the M1/Helios ground-truth chain unless you intentionally create a separate
experimental pipeline and document that choice.
