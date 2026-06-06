#!/usr/bin/env python3
"""Visualize FAST-Calib LiDAR-camera extrinsics.

Outputs:
- LiDAR points projected onto the extracted RGB calibration image.
- Optional LiDAR points projected onto the nearest thermal image, using Kalibr
  RGB-thermal extrinsic composed with FAST-Calib RGB-LiDAR extrinsic.
- Matplotlib renderings of FAST-Calib colored_cloud_scene_*.pcd.

Conventions:
- FAST-Calib result T_cam_lidar is T_rgb_lidar for D455 RGB.
- Kalibr cam1.T_cn_cnm1 is T_thermal_rgb when topics are ordered RGB, thermal.
- Therefore T_thermal_lidar = T_thermal_rgb * T_rgb_lidar.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
from pathlib import Path
from typing import Optional

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import yaml
from cv_bridge import CvBridge


def msg_stamp(msg, bag_time) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and stamp.to_sec() > 0:
        return stamp.to_sec()
    return bag_time.to_sec()


def read_metadata(path: Optional[str]) -> dict[str, str]:
    if not path or not Path(path).is_file():
        return {}
    vals: dict[str, str] = {}
    for line in Path(path).read_text().splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            vals[key.strip()] = value.strip()
    return vals


def parse_fast_calib_result(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    text = Path(path).read_text()
    r_match = re.search(r"Rcl:\s*\[([^\]]+)\]", text, re.S)
    p_match = re.search(r"Pcl:\s*\[([^\]]+)\]", text, re.S)
    if not r_match or not p_match:
        raise RuntimeError(f"missing Rcl/Pcl in {path}")

    r_vals = [float(x) for x in re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", r_match.group(1))]
    t_vals = [float(x) for x in re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", p_match.group(1))]
    R = np.asarray(r_vals, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t_vals, dtype=np.float64).reshape(3, 1)

    intr = {}
    for key in ["cam_fx", "cam_fy", "cam_cx", "cam_cy", "cam_d0", "cam_d1", "cam_d2", "cam_d3"]:
        match = re.search(rf"^{key}:\s*([-+]?\d*\.\d+|[-+]?\d+)", text, re.M)
        if not match:
            raise RuntimeError(f"missing {key} in {path}")
        intr[key] = float(match.group(1))
    K = np.array([[intr["cam_fx"], 0.0, intr["cam_cx"]], [0.0, intr["cam_fy"], intr["cam_cy"]], [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.array([intr["cam_d0"], intr["cam_d1"], intr["cam_d2"], intr["cam_d3"]], dtype=np.float64)
    return R, t, K, D


def load_kalibr_camera(camchain_path: str, cam_key: str) -> tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
    with Path(camchain_path).open() as f:
        raw = yaml.safe_load(f)
    cam = raw[cam_key]
    fx, fy, cx, cy = [float(v) for v in cam["intrinsics"]]
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.asarray(cam.get("distortion_coeffs", []), dtype=np.float64)
    T = None
    if "T_cn_cnm1" in cam:
        T = np.asarray(cam["T_cn_cnm1"], dtype=np.float64)
    return K, D, T


def assert_rgb_intrinsics_match(
    fast_k: np.ndarray,
    fast_d: np.ndarray,
    kalibr_k: np.ndarray,
    kalibr_d: np.ndarray,
    tolerance: float,
) -> tuple[float, float]:
    if fast_d.shape[0] != kalibr_d.shape[0]:
        raise RuntimeError(
            f"RGB distortion length mismatch: FAST-Calib has {fast_d.shape[0]}, "
            f"Kalibr has {kalibr_d.shape[0]}"
        )
    k_delta = float(np.max(np.abs(fast_k - kalibr_k)))
    d_delta = float(np.max(np.abs(fast_d - kalibr_d))) if fast_d.size else 0.0
    if k_delta > tolerance or d_delta > tolerance:
        raise RuntimeError(
            "RGB intrinsics mismatch between FAST-Calib result and Kalibr camchain: "
            f"max_K_delta={k_delta:.9g}, max_D_delta={d_delta:.9g}, "
            f"tolerance={tolerance:.9g}"
        )
    return k_delta, d_delta


def decode_image(bridge: CvBridge, msg, topic: str) -> np.ndarray:
    if topic.endswith("/compressed") or getattr(msg, "_type", "") == "sensor_msgs/CompressedImage":
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to decode compressed image on {topic}")
        return image
    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def nearest_message(bag_path: str, topic: str, target_stamp: float):
    best = None
    best_dt = None
    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            dt = msg_stamp(msg, t) - target_stamp
            if best is None or abs(dt) < abs(best_dt):
                best = msg
                best_dt = dt
    if best is None:
        raise RuntimeError(f"no messages on {topic}")
    return best, float(best_dt)


def bag_has_topic(bag_path: str, topic: str) -> bool:
    with rosbag.Bag(bag_path) as bag:
        return topic in bag.get_type_and_topic_info().topics


def first_existing_topic(bag_path: str, topics: list[str]) -> Optional[str]:
    with rosbag.Bag(bag_path) as bag:
        available = bag.get_type_and_topic_info().topics
    for topic in topics:
        if topic in available:
            return topic
    return None


def bag_midpoint_stamp(bag_path: str) -> float:
    with rosbag.Bag(bag_path) as bag:
        return 0.5 * (bag.get_start_time() + bag.get_end_time())


def load_nearest_image_from_bag(bag_path: str, topic: str, target_stamp: float, out_path: Path, bridge: CvBridge) -> tuple[np.ndarray, float, Path]:
    msg, dt = nearest_message(bag_path, topic, target_stamp)
    image = decode_image(bridge, msg, topic)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), image)
    return image, dt, out_path


def cloud_points_from_message(msg, bounds: Optional[tuple[float, float, float, float, float, float]] = None) -> np.ndarray:
    points = []
    for p in pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True):
        x, y, z = [float(v) for v in p]
        if bounds is not None:
            x_min, x_max, y_min, y_max, z_min, z_max = bounds
            if not (x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
                continue
        points.append((x, y, z))
    if not points:
        raise RuntimeError("no LiDAR points remained after filtering")
    return np.asarray(points, dtype=np.float64)


def nearest_cloud_points(bag_path: str, topic: str, target_stamp: float, bounds: tuple[float, float, float, float, float, float]) -> tuple[np.ndarray, float]:
    msg, dt = nearest_message(bag_path, topic, target_stamp)
    return cloud_points_from_message(msg, bounds), dt


def cloud_points_and_intensity_from_message(msg) -> tuple[np.ndarray, np.ndarray]:
    field_names = [f.name for f in msg.fields]
    if "intensity" not in field_names:
        raise RuntimeError("PointCloud2 has no intensity field")
    points = []
    intensities = []
    for p in pc2.read_points(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True):
        x, y, z, intensity = [float(v) for v in p]
        points.append((x, y, z))
        intensities.append(intensity)
    if not points:
        raise RuntimeError("no LiDAR points found in full cloud")
    return np.asarray(points, dtype=np.float64), np.asarray(intensities, dtype=np.float64)


def nearest_full_cloud_points(bag_path: str, topic: str, target_stamp: float) -> tuple[np.ndarray, np.ndarray, float]:
    msg, dt = nearest_message(bag_path, topic, target_stamp)
    return (*cloud_points_and_intensity_from_message(msg), dt)


def project_points_with_values_and_indices(points_lidar: np.ndarray, values: Optional[np.ndarray], R_cam_lidar: np.ndarray, t_cam_lidar: np.ndarray, K: np.ndarray, D: np.ndarray, image_shape: tuple[int, int], min_depth: float = 0.0) -> tuple[np.ndarray, np.ndarray, Optional[np.ndarray], np.ndarray]:
    cam = (R_cam_lidar @ points_lidar.T + t_cam_lidar).T
    front = cam[:, 2] > min_depth
    front_indices = np.flatnonzero(front)
    points_front = points_lidar[front]
    cam_front = cam[front]
    values_front = values[front] if values is not None else None
    if len(points_front) == 0:
        empty_values = np.empty((0,)) if values is not None else None
        return np.empty((0, 2)), np.empty((0,)), empty_values, np.empty((0,), dtype=np.int64)
    rvec, _ = cv2.Rodrigues(R_cam_lidar)
    image_points, _ = cv2.projectPoints(points_front.reshape(-1, 1, 3), rvec, t_cam_lidar, K, D)
    uv = image_points.reshape(-1, 2)
    h, w = image_shape[:2]
    valid = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h) & np.isfinite(uv).all(axis=1)
    return uv[valid], cam_front[valid, 2], values_front[valid] if values_front is not None else None, front_indices[valid]


def project_points_with_values(points_lidar: np.ndarray, values: Optional[np.ndarray], R_cam_lidar: np.ndarray, t_cam_lidar: np.ndarray, K: np.ndarray, D: np.ndarray, image_shape: tuple[int, int], min_depth: float = 0.0) -> tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
    uv, depth, projected_values, _ = project_points_with_values_and_indices(points_lidar, values, R_cam_lidar, t_cam_lidar, K, D, image_shape, min_depth)
    return uv, depth, projected_values


def project_points(points_lidar: np.ndarray, R_cam_lidar: np.ndarray, t_cam_lidar: np.ndarray, K: np.ndarray, D: np.ndarray, image_shape: tuple[int, int], min_depth: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    uv, depth, _ = project_points_with_values(points_lidar, None, R_cam_lidar, t_cam_lidar, K, D, image_shape, min_depth)
    return uv, depth


def draw_projection(image: np.ndarray, uv: np.ndarray, depth: np.ndarray, title: str, max_draw: int, radius: int, alpha: float) -> np.ndarray:
    canvas = image.copy()
    if len(uv) > max_draw:
        rng = np.random.default_rng(7)
        idx = rng.choice(len(uv), max_draw, replace=False)
        uv = uv[idx]
        depth = depth[idx]
    if len(uv):
        p05, p95 = np.percentile(depth, [5, 95])
        denom = max(1e-6, p95 - p05)
        norm = np.clip((depth - p05) / denom, 0.0, 1.0)
        colors = cv2.applyColorMap((255 * (1.0 - norm)).astype(np.uint8), cv2.COLORMAP_TURBO).reshape(-1, 3)
        for (u, v), color in zip(uv, colors):
            cv2.circle(canvas, (int(round(u)), int(round(v))), radius, tuple(int(x) for x in color.tolist()), -1, cv2.LINE_AA)
    blended = cv2.addWeighted(canvas, alpha, image, 1.0 - alpha, 0)
    cv2.putText(blended, title, (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(blended, f"projected points: {len(uv)}  color: near=red/yellow far=blue", (20, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return blended


def draw_projection_by_value(image: np.ndarray, uv: np.ndarray, values: np.ndarray, title: str, value_name: str, max_draw: int, radius: int, alpha: float) -> np.ndarray:
    canvas = image.copy()
    if len(uv) > max_draw:
        rng = np.random.default_rng(11)
        idx = rng.choice(len(uv), max_draw, replace=False)
        uv = uv[idx]
        values = values[idx]
    if len(uv):
        finite = np.isfinite(values)
        uv = uv[finite]
        values = values[finite]
    if len(uv):
        p01, p99 = np.percentile(values, [1, 99])
        denom = max(1e-6, p99 - p01)
        norm = np.clip((values - p01) / denom, 0.0, 1.0)
        colors = cv2.applyColorMap((255 * norm).astype(np.uint8), cv2.COLORMAP_TURBO).reshape(-1, 3)
        for (u, v), color in zip(uv, colors):
            cv2.circle(canvas, (int(round(u)), int(round(v))), radius, tuple(int(x) for x in color.tolist()), -1, cv2.LINE_AA)
    blended = cv2.addWeighted(canvas, alpha, image, 1.0 - alpha, 0)
    cv2.putText(blended, title, (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(blended, f"projected points: {len(uv)}  color: low {value_name}=blue high {value_name}=red", (20, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return blended


def rgb_uint_to_colors(vals: np.ndarray) -> np.ndarray:
    vals = vals.astype(np.uint32)
    r = ((vals >> 16) & 255).astype(np.float32) / 255.0
    g = ((vals >> 8) & 255).astype(np.float32) / 255.0
    b = (vals & 255).astype(np.float32) / 255.0
    return np.stack([r, g, b], axis=1)


def read_ascii_pcd_sample(path: str, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    data_count = 0
    points_declared = None
    with Path(path).open("r", errors="replace") as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith("POINTS"):
                points_declared = int(stripped.split()[1])
            if stripped.lower().startswith("data"):
                break
        stride = max(1, int((points_declared or max_points) / max_points)) if max_points else 1
        pts = []
        rgbs = []
        for idx, line in enumerate(f):
            if idx % stride != 0:
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
            rgbs.append(int(float(parts[3])))
            data_count += 1
            if max_points and data_count >= max_points:
                break
    return np.asarray(pts, dtype=np.float64), rgb_uint_to_colors(np.asarray(rgbs, dtype=np.uint32))


def render_colored_cloud(pcd_path: str, out_dir: Path, max_points: int) -> list[str]:
    pts, colors = read_ascii_pcd_sample(pcd_path, max_points)
    outputs = []
    for name, a, b, xlabel, ylabel, title in [
        ("colored_cloud_xy_top.png", 0, 1, "LiDAR X", "LiDAR Y", "Colored cloud top view"),
        ("colored_cloud_xz_side.png", 0, 2, "LiDAR X", "LiDAR Z", "Colored cloud side view"),
    ]:
        fig, ax = plt.subplots(figsize=(12, 8), dpi=150)
        ax.scatter(pts[:, a], pts[:, b], s=0.15, c=colors, linewidths=0)
        ax.set_aspect("equal", adjustable="box")
        # Use robust limits so a few far returns do not collapse the useful scene.
        x_lo, x_hi = np.percentile(pts[:, a], [1, 99])
        y_lo, y_hi = np.percentile(pts[:, b], [1, 99])
        if np.isfinite([x_lo, x_hi, y_lo, y_hi]).all() and x_hi > x_lo and y_hi > y_lo:
            ax.set_xlim(x_lo, x_hi)
            ax.set_ylim(y_lo, y_hi)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, alpha=0.25)
        path = out_dir / name
        fig.tight_layout()
        fig.savefig(path)
        plt.close(fig)
        outputs.append(str(path))
    return outputs


def pack_rgb_uint(colors_rgb: np.ndarray) -> np.ndarray:
    colors = np.clip(colors_rgb, 0, 255).astype(np.uint32)
    return (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]


def write_ascii_xyzrgb_pcd(path: Path, points: np.ndarray, colors_rgb: np.ndarray) -> None:
    rgb = pack_rgb_uint(colors_rgb)
    with path.open("w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z rgb\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F U\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for (x, y, z), c in zip(points, rgb):
            f.write(f"{x:.9f} {y:.9f} {z:.9f} {int(c)}\n")


def thermal_colors_from_image(thermal_bgr: np.ndarray, uv: np.ndarray) -> np.ndarray:
    px = np.rint(uv).astype(np.int64)
    h, w = thermal_bgr.shape[:2]
    px[:, 0] = np.clip(px[:, 0], 0, w - 1)
    px[:, 1] = np.clip(px[:, 1], 0, h - 1)
    sampled_bgr = thermal_bgr[px[:, 1], px[:, 0]]
    return sampled_bgr[:, ::-1]


def colors_from_bgr_image(image_bgr: np.ndarray, uv: np.ndarray) -> np.ndarray:
    px = np.rint(uv).astype(np.int64)
    h, w = image_bgr.shape[:2]
    px[:, 0] = np.clip(px[:, 0], 0, w - 1)
    px[:, 1] = np.clip(px[:, 1], 0, h - 1)
    sampled_bgr = image_bgr[px[:, 1], px[:, 0]]
    return sampled_bgr[:, ::-1]


def default_target_stamp(metadata: dict[str, str], target_stamp: Optional[float], bag_path: str) -> tuple[float, str]:
    if target_stamp is not None:
        return target_stamp, "--target-stamp"
    for key in ["target_lidar_stamp", "selected_image_bag_time"]:
        if key in metadata:
            return float(metadata[key]), f"metadata:{key}"
    return bag_midpoint_stamp(bag_path), "bag midpoint"


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualize LiDAR projection onto RGB/thermal images and colored point cloud.")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--lidar-topic", required=True)
    parser.add_argument("--rgb-image", help="RGB image path. If omitted, --rgb-topic is decoded from the bag.")
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw/compressed")
    parser.add_argument("--target-stamp", type=float, help="Reference timestamp. Defaults to metadata timestamp, then bag midpoint.")
    parser.add_argument("--metadata")
    parser.add_argument("--fast-calib-result", required=True)
    parser.add_argument("--colored-pcd", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--kalibr-camchain")
    parser.add_argument("--thermal-topic", default="/flir_boson/image_raw/compressed")
    parser.add_argument("--rgb-cam", default="cam0")
    parser.add_argument("--thermal-cam", default="cam1")
    parser.add_argument("--intrinsics-tolerance", type=float, default=1e-4)
    parser.add_argument("--x-min", type=float, default=1.0)
    parser.add_argument("--x-max", type=float, default=3.5)
    parser.add_argument("--y-min", type=float, default=-1.5)
    parser.add_argument("--y-max", type=float, default=1.5)
    parser.add_argument("--z-min", type=float, default=-0.5)
    parser.add_argument("--z-max", type=float, default=1.8)
    parser.add_argument("--max-draw", type=int, default=45000)
    parser.add_argument("--full-max-draw", type=int, default=200000)
    parser.add_argument("--full-min-depth", type=float, default=0.0, help="Drop full-cloud projected points at or behind this camera-frame depth.")
    parser.add_argument("--point-radius", type=int, default=1)
    parser.add_argument("--alpha", type=float, default=0.82)
    parser.add_argument("--pcd-max-points", type=int, default=350000)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    metadata = read_metadata(args.metadata)
    target_stamp, target_source = default_target_stamp(metadata, args.target_stamp, args.bag)
    bridge = CvBridge()
    R_rgb_lidar, t_rgb_lidar, K_rgb, D_rgb = parse_fast_calib_result(args.fast_calib_result)
    K_kalibr_rgb = None
    D_kalibr_rgb = None
    T_thermal_rgb = None
    if args.kalibr_camchain:
        K_kalibr_rgb, D_kalibr_rgb, _ = load_kalibr_camera(args.kalibr_camchain, args.rgb_cam)
        k_delta, d_delta = assert_rgb_intrinsics_match(
            K_rgb,
            D_rgb,
            K_kalibr_rgb,
            D_kalibr_rgb,
            args.intrinsics_tolerance,
        )
    else:
        k_delta = None
        d_delta = None
    bounds = (args.x_min, args.x_max, args.y_min, args.y_max, args.z_min, args.z_max)
    points, cloud_dt = nearest_cloud_points(args.bag, args.lidar_topic, target_stamp, bounds)
    full_points, full_intensity, full_cloud_dt = nearest_full_cloud_points(args.bag, args.lidar_topic, target_stamp)

    summary = [
        "LiDAR-camera visual check",
        f"bag={args.bag}",
        f"lidar_topic={args.lidar_topic}",
        f"target_stamp={target_stamp:.9f}",
        f"target_stamp_source={target_source}",
        f"nearest_lidar_dt_s={cloud_dt:.9f}",
        f"filtered_lidar_points={len(points)}",
        f"full_lidar_points={len(full_points)}",
        f"full_intensity_min={float(np.min(full_intensity)):.9g}",
        f"full_intensity_max={float(np.max(full_intensity)):.9g}",
        f"nearest_full_lidar_dt_s={full_cloud_dt:.9f}",
        f"fast_calib_result={args.fast_calib_result}",
    ]
    if args.kalibr_camchain:
        summary += [
            f"kalibr_camchain={args.kalibr_camchain}",
            f"rgb_intrinsics_match=PASS",
            f"rgb_intrinsics_max_K_delta={k_delta:.9g}",
            f"rgb_intrinsics_max_D_delta={d_delta:.9g}",
            f"rgb_intrinsics_tolerance={args.intrinsics_tolerance:.9g}",
        ]

    if args.rgb_image:
        rgb = cv2.imread(args.rgb_image, cv2.IMREAD_COLOR)
        if rgb is None:
            raise RuntimeError(f"failed to read RGB image: {args.rgb_image}")
        rgb_source = args.rgb_image
        rgb_dt = None
    else:
        if not bag_has_topic(args.bag, args.rgb_topic):
            raise RuntimeError(f"bag has no RGB topic {args.rgb_topic}; pass --rgb-image or --rgb-topic")
        rgb, rgb_dt, rgb_frame_path = load_nearest_image_from_bag(args.bag, args.rgb_topic, target_stamp, out_dir / "extracted_rgb.png", bridge)
        rgb_source = str(rgb_frame_path)
    uv, depth = project_points(points, R_rgb_lidar, t_rgb_lidar, K_rgb, D_rgb, rgb.shape)
    rgb_overlay = draw_projection(rgb, uv, depth, "Helios LiDAR projected into D455 RGB", args.max_draw, args.point_radius, args.alpha)
    rgb_path = out_dir / "lidar_projected_on_rgb.png"
    cv2.imwrite(str(rgb_path), rgb_overlay)
    summary += [f"rgb_source={rgb_source}", f"rgb_overlay={rgb_path}", f"rgb_projected_points={len(uv)}"]
    uv_full, depth_full = project_points(full_points, R_rgb_lidar, t_rgb_lidar, K_rgb, D_rgb, rgb.shape, args.full_min_depth)
    rgb_full_overlay = draw_projection(rgb, uv_full, depth_full, "Full Helios LiDAR projected into D455 RGB", args.full_max_draw, args.point_radius, args.alpha)
    rgb_full_path = out_dir / "lidar_full_projected_on_rgb.png"
    cv2.imwrite(str(rgb_full_path), rgb_full_overlay)
    summary += [
        f"rgb_full_overlay={rgb_full_path}",
        f"rgb_full_projected_points={len(uv_full)}",
        f"rgb_full_min_depth={args.full_min_depth:.6f}",
    ]
    uv_full_i, _, intensity_full_i = project_points_with_values(full_points, full_intensity, R_rgb_lidar, t_rgb_lidar, K_rgb, D_rgb, rgb.shape, args.full_min_depth)
    rgb_full_intensity_overlay = draw_projection_by_value(rgb, uv_full_i, intensity_full_i, "Full Helios LiDAR intensity projected into D455 RGB", "intensity", args.full_max_draw, args.point_radius, args.alpha)
    rgb_full_intensity_path = out_dir / "lidar_full_intensity_projected_on_rgb.png"
    cv2.imwrite(str(rgb_full_intensity_path), rgb_full_intensity_overlay)
    summary += [
        f"rgb_full_intensity_overlay={rgb_full_intensity_path}",
        f"rgb_full_intensity_projected_points={len(uv_full_i)}",
    ]
    uv_rgb_color, _, _, rgb_color_indices = project_points_with_values_and_indices(
        full_points,
        None,
        R_rgb_lidar,
        t_rgb_lidar,
        K_rgb,
        D_rgb,
        rgb.shape,
        args.full_min_depth,
    )
    rgb_cloud_points = full_points[rgb_color_indices]
    rgb_cloud_colors = colors_from_bgr_image(rgb, uv_rgb_color)
    rgb_cloud_path = out_dir / "rgb_colored_cloud.pcd"
    write_ascii_xyzrgb_pcd(rgb_cloud_path, rgb_cloud_points, rgb_cloud_colors)
    summary += [
        f"rgb_colored_cloud={rgb_cloud_path}",
        f"rgb_colored_cloud_points={len(rgb_cloud_points)}",
    ]
    for rendered in render_colored_cloud(str(rgb_cloud_path), out_dir, args.pcd_max_points):
        rendered_path = Path(rendered)
        renamed = out_dir / f"rgb_{rendered_path.name}"
        rendered_path.replace(renamed)
        summary.append(f"rgb_colored_cloud_view={renamed}")
    if rgb_dt is not None:
        summary.append(f"nearest_rgb_dt_s={rgb_dt:.9f}")

    if args.kalibr_camchain and bag_has_topic(args.bag, args.thermal_topic):
        K_thermal, D_thermal, T_thermal_rgb = load_kalibr_camera(args.kalibr_camchain, args.thermal_cam)
        if T_thermal_rgb is None:
            raise RuntimeError(f"{args.thermal_cam} in camchain has no T_cn_cnm1")
        T_rgb_lidar = np.eye(4)
        T_rgb_lidar[:3, :3] = R_rgb_lidar
        T_rgb_lidar[:3, 3:4] = t_rgb_lidar
        T_thermal_lidar = T_thermal_rgb @ T_rgb_lidar
        R_th = T_thermal_lidar[:3, :3]
        t_th = T_thermal_lidar[:3, 3:4]
        th_msg, th_dt = nearest_message(args.bag, args.thermal_topic, target_stamp)
        thermal = decode_image(bridge, th_msg, args.thermal_topic)
        cv2.imwrite(str(out_dir / "extracted_thermal.png"), thermal)
        uv_th, depth_th = project_points(points, R_th, t_th, K_thermal, D_thermal, thermal.shape)
        th_overlay = draw_projection(thermal, uv_th, depth_th, "Helios LiDAR projected into Boson thermal", args.max_draw, args.point_radius, args.alpha)
        thermal_path = out_dir / "lidar_projected_on_thermal.png"
        cv2.imwrite(str(thermal_path), th_overlay)
        summary += [f"thermal_overlay={thermal_path}", f"thermal_projected_points={len(uv_th)}", f"nearest_thermal_dt_s={th_dt:.9f}"]
        uv_th_full, depth_th_full = project_points(full_points, R_th, t_th, K_thermal, D_thermal, thermal.shape, args.full_min_depth)
        th_full_overlay = draw_projection(thermal, uv_th_full, depth_th_full, "Full Helios LiDAR projected into Boson thermal", args.full_max_draw, args.point_radius, args.alpha)
        thermal_full_path = out_dir / "lidar_full_projected_on_thermal.png"
        cv2.imwrite(str(thermal_full_path), th_full_overlay)
        summary += [
            f"thermal_full_overlay={thermal_full_path}",
            f"thermal_full_projected_points={len(uv_th_full)}",
            f"thermal_full_min_depth={args.full_min_depth:.6f}",
        ]
        uv_th_full_i, _, intensity_th_full_i = project_points_with_values(full_points, full_intensity, R_th, t_th, K_thermal, D_thermal, thermal.shape, args.full_min_depth)
        th_full_intensity_overlay = draw_projection_by_value(thermal, uv_th_full_i, intensity_th_full_i, "Full Helios LiDAR intensity projected into Boson thermal", "intensity", args.full_max_draw, args.point_radius, args.alpha)
        thermal_full_intensity_path = out_dir / "lidar_full_intensity_projected_on_thermal.png"
        cv2.imwrite(str(thermal_full_intensity_path), th_full_intensity_overlay)
        summary += [
            f"thermal_full_intensity_overlay={thermal_full_intensity_path}",
            f"thermal_full_intensity_projected_points={len(uv_th_full_i)}",
        ]
        uv_th_color, _, _, th_color_indices = project_points_with_values_and_indices(full_points, None, R_th, t_th, K_thermal, D_thermal, thermal.shape, args.full_min_depth)
        thermal_cloud_points = full_points[th_color_indices]
        thermal_cloud_colors = thermal_colors_from_image(thermal, uv_th_color)
        thermal_cloud_path = out_dir / "thermal_colored_cloud.pcd"
        write_ascii_xyzrgb_pcd(thermal_cloud_path, thermal_cloud_points, thermal_cloud_colors)
        summary += [
            f"thermal_colored_cloud={thermal_cloud_path}",
            f"thermal_colored_cloud_points={len(thermal_cloud_points)}",
        ]
        for rendered in render_colored_cloud(str(thermal_cloud_path), out_dir, args.pcd_max_points):
            rendered_path = Path(rendered)
            renamed = out_dir / f"thermal_{rendered_path.name}"
            rendered_path.replace(renamed)
            summary.append(f"thermal_colored_cloud_view={renamed}")
    else:
        reason = "missing --kalibr-camchain" if not args.kalibr_camchain else f"bag has no {args.thermal_topic}"
        summary.append(f"thermal_overlay=SKIPPED ({reason})")

    fast_calib_colored_cloud_path = out_dir / "fast_calib_colored_cloud.pcd"
    shutil.copy2(args.colored_pcd, fast_calib_colored_cloud_path)
    summary.append(f"fast_calib_colored_cloud={fast_calib_colored_cloud_path}")
    for rendered in render_colored_cloud(str(fast_calib_colored_cloud_path), out_dir, args.pcd_max_points):
        summary.append(f"colored_cloud_view={rendered}")

    summary_path = out_dir / "visual_check_summary.txt"
    summary_path.write_text("\n".join(summary) + "\n")
    print(summary_path.read_text())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
