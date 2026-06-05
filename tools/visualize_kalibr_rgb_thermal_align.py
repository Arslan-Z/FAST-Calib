#!/usr/bin/env python3
"""Visual check for Kalibr RGB-thermal camera-chain results.

This tool does not estimate extrinsics. It validates an existing Kalibr
camchain by detecting the asymmetric circle grid in synchronized RGB/thermal
frames, solving the board pose in one camera, projecting the same 3D board
points into the other camera with Kalibr's extrinsic, and saving overlays plus
pixel-error reports.

Important limitation: a full image-to-image RGB/thermal warp is not possible
from camera intrinsics/extrinsics alone for arbitrary 3D scenes. That requires
depth, or an assumed plane. This script validates alignment on the calibration
target plane, which is the right sanity check for camera extrinsics.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml

cv2 = None


def require_cv2():
    global cv2
    if cv2 is None:
        try:
            import cv2 as cv2_module
        except ImportError as exc:
            raise RuntimeError("OpenCV Python bindings are required: install python3-opencv or source a ROS environment that provides cv2") from exc
        cv2 = cv2_module
    return cv2


@dataclass
class CameraModel:
    name: str
    resolution: Tuple[int, int]
    K: np.ndarray
    D: np.ndarray
    T_cn_cnm1: Optional[np.ndarray]


@dataclass
class ImagePair:
    rgb: np.ndarray
    thermal: np.ndarray
    rgb_stamp: float
    thermal_stamp: float
    index: int


def parse_pattern_size(value: str) -> Tuple[int, int]:
    try:
        cols_s, rows_s = value.lower().split("x", 1)
        cols, rows = int(cols_s), int(rows_s)
    except Exception as exc:  # pragma: no cover - argparse displays this.
        raise argparse.ArgumentTypeError("pattern size must look like 11x4") from exc
    if cols <= 0 or rows <= 0:
        raise argparse.ArgumentTypeError("pattern dimensions must be positive")
    return cols, rows


def load_camchain(path: Path, rgb_cam: str, thermal_cam: str) -> Tuple[CameraModel, CameraModel, np.ndarray]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if rgb_cam not in data:
        raise KeyError(f"{rgb_cam!r} not found in camchain")
    if thermal_cam not in data:
        raise KeyError(f"{thermal_cam!r} not found in camchain")

    rgb = parse_camera(rgb_cam, data[rgb_cam])
    thermal = parse_camera(thermal_cam, data[thermal_cam])
    T_thermal_rgb = resolve_transform(data, rgb_cam, thermal_cam)
    return rgb, thermal, T_thermal_rgb


def parse_camera(name: str, raw: dict) -> CameraModel:
    intrinsics = raw.get("intrinsics")
    if not intrinsics or len(intrinsics) != 4:
        raise ValueError(f"{name}: expected pinhole intrinsics [fx, fy, cx, cy]")

    fx, fy, cx, cy = [float(v) for v in intrinsics]
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.asarray(raw.get("distortion_coeffs", []), dtype=np.float64).reshape(-1, 1)

    resolution = raw.get("resolution")
    if not resolution or len(resolution) != 2:
        raise ValueError(f"{name}: missing resolution [width, height]")

    T = raw.get("T_cn_cnm1")
    T_arr = np.asarray(T, dtype=np.float64) if T is not None else None
    if T_arr is not None and T_arr.shape != (4, 4):
        raise ValueError(f"{name}: T_cn_cnm1 must be 4x4")

    return CameraModel(
        name=name,
        resolution=(int(resolution[0]), int(resolution[1])),
        K=K,
        D=D,
        T_cn_cnm1=T_arr,
    )


def camera_index(name: str) -> int:
    if not name.startswith("cam"):
        raise ValueError(f"camera name {name!r} is not in Kalibr camN form")
    return int(name[3:])


def resolve_transform(camchain: dict, rgb_cam: str, thermal_cam: str) -> np.ndarray:
    """Return T_thermal_rgb, mapping homogeneous RGB-camera points into thermal."""
    rgb_i = camera_index(rgb_cam)
    thermal_i = camera_index(thermal_cam)

    if thermal_i == rgb_i + 1:
        T = camchain[thermal_cam].get("T_cn_cnm1")
        if T is None:
            raise ValueError(f"{thermal_cam} has no T_cn_cnm1")
        return np.asarray(T, dtype=np.float64)

    if rgb_i == thermal_i + 1:
        T = camchain[rgb_cam].get("T_cn_cnm1")
        if T is None:
            raise ValueError(f"{rgb_cam} has no T_cn_cnm1")
        return np.linalg.inv(np.asarray(T, dtype=np.float64))

    raise ValueError(
        "Only adjacent Kalibr cameras are supported by this simple verifier. "
        "Run Kalibr with topics ordered as RGB then thermal, e.g. cam0/cam1."
    )


def object_points_acircles(cols: int, rows: int, square_m: float) -> np.ndarray:
    points = []
    for row in range(rows):
        for col in range(cols):
            points.append([(2.0 * col + (row % 2)) * square_m, row * square_m, 0.0])
    return np.asarray(points, dtype=np.float32)


def make_blob_detector() -> cv2.SimpleBlobDetector:
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 8.0
    params.maxArea = 200000.0
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False
    params.filterByColor = False
    params.minDistBetweenBlobs = 3.0
    return cv2.SimpleBlobDetector_create(params)


def to_gray8(image: np.ndarray) -> np.ndarray:
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if image.dtype == np.uint8:
        return image
    normalized = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
    return normalized.astype(np.uint8)


def display_image(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        gray = to_gray8(image)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if image.dtype != np.uint8:
        chans = cv2.split(image)
        norm = [cv2.normalize(c, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8) for c in chans]
        return cv2.merge(norm)
    return image.copy()


def detect_acircles(image: np.ndarray, pattern_size: Tuple[int, int], detector: cv2.SimpleBlobDetector) -> Tuple[bool, Optional[np.ndarray], str]:
    gray = to_gray8(image)
    flags = cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING

    found, centers = cv2.findCirclesGrid(gray, pattern_size, flags=flags, blobDetector=detector)
    if found:
        return True, centers.reshape(-1, 2).astype(np.float32), "normal"

    inverted = cv2.bitwise_not(gray)
    found, centers = cv2.findCirclesGrid(inverted, pattern_size, flags=flags, blobDetector=detector)
    if found:
        return True, centers.reshape(-1, 2).astype(np.float32), "inverted"

    return False, None, "not_found"


def project_points(obj: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    return projected.reshape(-1, 2)


def compose_pose(T_dst_src: np.ndarray, rvec_src_obj: np.ndarray, tvec_src_obj: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    R_src_obj, _ = cv2.Rodrigues(rvec_src_obj)
    R_dst_src = T_dst_src[:3, :3]
    t_dst_src = T_dst_src[:3, 3].reshape(3, 1)

    R_dst_obj = R_dst_src @ R_src_obj
    t_dst_obj = R_dst_src @ tvec_src_obj.reshape(3, 1) + t_dst_src
    rvec_dst_obj, _ = cv2.Rodrigues(R_dst_obj)
    return rvec_dst_obj, t_dst_obj


def mean_error_px(projected: np.ndarray, detected: np.ndarray) -> float:
    return float(np.mean(np.linalg.norm(projected - detected, axis=1)))


def draw_points(image: np.ndarray, detected: Optional[np.ndarray], projected: Optional[np.ndarray], title: str) -> np.ndarray:
    canvas = display_image(image)
    if detected is not None:
        for point in detected:
            cv2.circle(canvas, tuple(np.round(point).astype(int)), 5, (0, 220, 0), 2, cv2.LINE_AA)
    if projected is not None:
        for point in projected:
            cv2.drawMarker(
                canvas,
                tuple(np.round(point).astype(int)),
                (255, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=12,
                thickness=2,
                line_type=cv2.LINE_AA,
            )
    cv2.putText(canvas, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (30, 30, 255), 2, cv2.LINE_AA)
    return canvas


def hstack_different_sizes(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    height = max(left.shape[0], right.shape[0])
    width = left.shape[1] + right.shape[1]
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    canvas[: left.shape[0], : left.shape[1]] = left
    canvas[: right.shape[0], left.shape[1] : left.shape[1] + right.shape[1]] = right
    return canvas


def stamp_to_sec(stamp) -> float:
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    return float(stamp)


def image_msg_to_cv(msg) -> np.ndarray:
    encoding = msg.encoding.lower()
    channels = 1
    if encoding in ("bgr8", "rgb8"):
        dtype = np.uint8
        channels = 3
    elif encoding in ("bgra8", "rgba8"):
        dtype = np.uint8
        channels = 4
    elif encoding in ("mono8", "8uc1"):
        dtype = np.uint8
    elif encoding in ("mono16", "16uc1"):
        dtype = np.uint16
    elif encoding in ("32fc1",):
        dtype = np.float32
    else:
        raise ValueError(f"Unsupported ROS image encoding: {msg.encoding}")

    itemsize = np.dtype(dtype).itemsize
    row_values = msg.step // itemsize
    arr = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_values)
    needed = msg.width * channels
    arr = arr[:, :needed]
    if channels > 1:
        arr = arr.reshape(msg.height, msg.width, channels)
        if encoding == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        elif encoding == "bgra8":
            arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    else:
        arr = arr.reshape(msg.height, msg.width)
    return arr.copy()


def read_bag_pairs(
    bag_path: Path,
    rgb_topic: str,
    thermal_topic: str,
    max_dt: float,
    max_pairs: int,
    stride: int,
) -> List[ImagePair]:
    try:
        import rosbag  # type: ignore
    except ImportError as exc:
        raise RuntimeError("rosbag Python module is required for --bag mode; source ROS first") from exc

    rgb_stamps = []
    thermal_stamps = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[rgb_topic, thermal_topic]):
            stamp = stamp_to_sec(msg.header.stamp) if getattr(msg, "header", None) else stamp_to_sec(t)
            if topic == rgb_topic:
                rgb_stamps.append(stamp)
            elif topic == thermal_topic:
                thermal_stamps.append(stamp)

    if not rgb_stamps or not thermal_stamps:
        return []

    rgb_stamps.sort()
    thermal_stamps.sort()

    candidates: List[Tuple[float, float]] = []
    for r_stamp in rgb_stamps:
        insert_at = bisect.bisect_left(thermal_stamps, r_stamp)
        nearby = []
        for t_idx in (insert_at - 1, insert_at):
            if 0 <= t_idx < len(thermal_stamps):
                nearby.append(thermal_stamps[t_idx])
        if not nearby:
            continue
        t_stamp = min(nearby, key=lambda stamp: abs(stamp - r_stamp))
        if abs(t_stamp - r_stamp) > max_dt:
            continue
        candidates.append((r_stamp, t_stamp))

    if stride > 1:
        candidates = candidates[::stride]

    if max_pairs > 0 and len(candidates) > max_pairs:
        indices = np.linspace(0, len(candidates) - 1, max_pairs, dtype=int)
        candidates = [candidates[int(i)] for i in indices]

    def key(stamp: float) -> int:
        return int(round(stamp * 1_000_000_000))

    selected_rgb = {key(r_stamp): idx for idx, (r_stamp, _) in enumerate(candidates)}
    selected_thermal = {key(t_stamp): idx for idx, (_, t_stamp) in enumerate(candidates)}
    rgb_images: dict[int, np.ndarray] = {}
    thermal_images: dict[int, np.ndarray] = {}

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[rgb_topic, thermal_topic]):
            stamp = stamp_to_sec(msg.header.stamp) if getattr(msg, "header", None) else stamp_to_sec(t)
            stamp_key = key(stamp)
            if topic == rgb_topic and stamp_key in selected_rgb:
                rgb_images[selected_rgb[stamp_key]] = image_msg_to_cv(msg)
            elif topic == thermal_topic and stamp_key in selected_thermal:
                thermal_images[selected_thermal[stamp_key]] = image_msg_to_cv(msg)
            if len(rgb_images) == len(candidates) and len(thermal_images) == len(candidates):
                break

    pairs: List[ImagePair] = []
    for pair_index, (r_stamp, t_stamp) in enumerate(candidates):
        if pair_index not in rgb_images or pair_index not in thermal_images:
            continue
        pairs.append(
            ImagePair(
                rgb=rgb_images[pair_index],
                thermal=thermal_images[pair_index],
                rgb_stamp=r_stamp,
                thermal_stamp=t_stamp,
                index=pair_index,
            )
        )
    return pairs


def read_single_pair(rgb_image: Path, thermal_image: Path) -> List[ImagePair]:
    rgb = cv2.imread(str(rgb_image), cv2.IMREAD_UNCHANGED)
    thermal = cv2.imread(str(thermal_image), cv2.IMREAD_UNCHANGED)
    if rgb is None:
        raise FileNotFoundError(f"Could not read RGB image: {rgb_image}")
    if thermal is None:
        raise FileNotFoundError(f"Could not read thermal image: {thermal_image}")
    return [ImagePair(rgb=rgb, thermal=thermal, rgb_stamp=math.nan, thermal_stamp=math.nan, index=0)]


def verify_pair(
    pair: ImagePair,
    obj_points: np.ndarray,
    pattern_size: Tuple[int, int],
    rgb_cam: CameraModel,
    thermal_cam: CameraModel,
    T_thermal_rgb: np.ndarray,
    detector: cv2.SimpleBlobDetector,
    out_dir: Path,
    save_rejected: bool,
) -> dict:
    rgb_found, rgb_centers, rgb_mode = detect_acircles(pair.rgb, pattern_size, detector)
    thermal_found, thermal_centers, thermal_mode = detect_acircles(pair.thermal, pattern_size, detector)

    row = {
        "index": pair.index,
        "rgb_stamp": pair.rgb_stamp,
        "thermal_stamp": pair.thermal_stamp,
        "dt_sec": abs(pair.rgb_stamp - pair.thermal_stamp) if not math.isnan(pair.rgb_stamp) else math.nan,
        "rgb_detected": rgb_found,
        "thermal_detected": thermal_found,
        "rgb_detection_mode": rgb_mode,
        "thermal_detection_mode": thermal_mode,
        "rgb_reproj_px": "",
        "thermal_from_rgb_px": "",
        "rgb_from_thermal_px": "",
        "status": "rejected",
    }

    if not (rgb_found and thermal_found and rgb_centers is not None and thermal_centers is not None):
        if save_rejected:
            overlay = hstack_different_sizes(
                draw_points(pair.rgb, rgb_centers, None, f"RGB {rgb_mode}"),
                draw_points(pair.thermal, thermal_centers, None, f"Thermal {thermal_mode}"),
            )
            cv2.imwrite(str(out_dir / f"rejected_{pair.index:04d}.png"), overlay)
        return row

    ok_rgb, rvec_rgb, tvec_rgb = cv2.solvePnP(obj_points, rgb_centers, rgb_cam.K, rgb_cam.D, flags=cv2.SOLVEPNP_ITERATIVE)
    ok_th, rvec_th, tvec_th = cv2.solvePnP(obj_points, thermal_centers, thermal_cam.K, thermal_cam.D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not (ok_rgb and ok_th):
        return row

    rgb_reprojected = project_points(obj_points, rvec_rgb, tvec_rgb, rgb_cam.K, rgb_cam.D)
    row["rgb_reproj_px"] = f"{mean_error_px(rgb_reprojected, rgb_centers):.4f}"

    rvec_th_from_rgb, tvec_th_from_rgb = compose_pose(T_thermal_rgb, rvec_rgb, tvec_rgb)
    thermal_projected = project_points(obj_points, rvec_th_from_rgb, tvec_th_from_rgb, thermal_cam.K, thermal_cam.D)
    row["thermal_from_rgb_px"] = f"{mean_error_px(thermal_projected, thermal_centers):.4f}"

    T_rgb_thermal = np.linalg.inv(T_thermal_rgb)
    rvec_rgb_from_th, tvec_rgb_from_th = compose_pose(T_rgb_thermal, rvec_th, tvec_th)
    rgb_projected_from_th = project_points(obj_points, rvec_rgb_from_th, tvec_rgb_from_th, rgb_cam.K, rgb_cam.D)
    row["rgb_from_thermal_px"] = f"{mean_error_px(rgb_projected_from_th, rgb_centers):.4f}"
    row["status"] = "accepted"

    rgb_overlay = draw_points(pair.rgb, rgb_centers, rgb_projected_from_th, "RGB: green=detected magenta=from thermal+Kalibr")
    thermal_overlay = draw_points(pair.thermal, thermal_centers, thermal_projected, "Thermal: green=detected magenta=from RGB+Kalibr")
    combined = hstack_different_sizes(rgb_overlay, thermal_overlay)
    cv2.imwrite(str(out_dir / f"accepted_{pair.index:04d}.png"), combined)
    return row


def summarize(rows: Sequence[dict]) -> str:
    accepted = [r for r in rows if r["status"] == "accepted"]
    rejected = len(rows) - len(accepted)

    def avg(field: str) -> str:
        values = [float(r[field]) for r in accepted if r[field] not in ("", None)]
        return f"{float(np.mean(values)):.4f}" if values else "n/a"

    lines = [
        "Kalibr RGB-thermal alignment visual verification",
        "",
        f"frames checked: {len(rows)}",
        f"accepted: {len(accepted)}",
        f"rejected: {rejected}",
        f"avg rgb reprojection px: {avg('rgb_reproj_px')}",
        f"avg thermal-from-rgb px: {avg('thermal_from_rgb_px')}",
        f"avg rgb-from-thermal px: {avg('rgb_from_thermal_px')}",
        "",
        "Interpretation:",
        "- green circles are detected image measurements",
        "- magenta crosses are projected through the Kalibr extrinsic",
        "- low projected-vs-detected pixel error and visually overlapping marks support the extrinsic",
        "- this validates the calibration target plane, not arbitrary scene-wide image warping",
    ]
    return "\n".join(lines) + "\n"


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camchain", required=True, type=Path, help="Kalibr camchain YAML")
    parser.add_argument("--rgb-cam", default="cam0", help="Kalibr camera key for RGB, usually cam0")
    parser.add_argument("--thermal-cam", default="cam1", help="Kalibr camera key for thermal, usually cam1")
    parser.add_argument("--pattern-size", default=(11, 4), type=parse_pattern_size, help="Circle grid size, e.g. 11x4")
    parser.add_argument("--square-m", default=0.04, type=float, help="Circle-grid spacing in meters")
    parser.add_argument("--output-dir", required=True, type=Path, help="Directory for overlays and reports")

    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=Path, help="ROS bag containing raw RGB and thermal sensor_msgs/Image topics")
    source.add_argument("--rgb-image", type=Path, help="Single RGB image for image-pair mode")
    parser.add_argument("--thermal-image", type=Path, help="Single thermal image for image-pair mode")

    parser.add_argument("--rgb-topic", default="/rgb_left", help="RGB raw image topic in --bag mode")
    parser.add_argument("--thermal-topic", default="/thermal_left", help="Thermal raw image topic in --bag mode")
    parser.add_argument("--max-dt", default=0.05, type=float, help="Max RGB/thermal timestamp difference in seconds")
    parser.add_argument("--max-frames", default=30, type=int, help="Max paired frames to inspect")
    parser.add_argument("--stride", default=1, type=int, help="Use every Nth synchronized candidate before uniform max-frame sampling")
    parser.add_argument("--save-rejected", action="store_true", help="Save overlays for frames where detection failed")
    args = parser.parse_args(argv)

    if args.rgb_image and not args.thermal_image:
        parser.error("--thermal-image is required when --rgb-image is used")

    require_cv2()
    rgb_cam, thermal_cam, T_thermal_rgb = load_camchain(args.camchain, args.rgb_cam, args.thermal_cam)
    obj_points = object_points_acircles(args.pattern_size[0], args.pattern_size[1], args.square_m)
    detector = make_blob_detector()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.bag:
        pairs = read_bag_pairs(args.bag, args.rgb_topic, args.thermal_topic, args.max_dt, args.max_frames, args.stride)
    else:
        pairs = read_single_pair(args.rgb_image, args.thermal_image)

    if not pairs:
        raise RuntimeError("No synchronized image pairs found. Check topics, timestamps, and --max-dt.")

    rows = [
        verify_pair(
            pair=pair,
            obj_points=obj_points,
            pattern_size=args.pattern_size,
            rgb_cam=rgb_cam,
            thermal_cam=thermal_cam,
            T_thermal_rgb=T_thermal_rgb,
            detector=detector,
            out_dir=args.output_dir,
            save_rejected=args.save_rejected,
        )
        for pair in pairs
    ]

    csv_path = args.output_dir / "alignment_report.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = summarize(rows)
    (args.output_dir / "summary.txt").write_text(summary, encoding="utf-8")
    print(summary)
    print(f"wrote: {args.output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
