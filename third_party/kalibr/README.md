# Kalibr Third-Party Patches

This directory does not contain the Kalibr ROS workspace.

The active Kalibr workspace for this machine is:

```text
/home/zcy/kalibr_ws
```

This directory only keeps small patch/reference files for the official Kalibr source:

```text
patches/
```

Calibration target YAML files are project configuration, not third-party source. They live closer to the active configs at:

```text
config/kalibr/targets/
```

The current D455/Boson pipeline target is:

```text
config/kalibr/targets/acircles_4x11_0p04.yaml
```

Do not put `official_ws`, Kalibr build directories, rosbags, reports, generated calibration output, or project target YAML files under this directory.
