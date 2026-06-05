#!/usr/bin/env python3
"""Automatic target-plane overlay using Kalibr/aslam target detection.

This visualizes an existing Kalibr RGB->thermal camchain without assumed scene
depth. It detects the calibration board in RGB with Kalibr's own target
detector, estimates the board plane pose, projects that plane into the thermal
camera through the Kalibr extrinsic, and overlays only that target plane.

If the target is also detected in thermal, the script reports projected-vs-
detected target-point error. Thermal detection is useful for metrics but is not
required to produce the plane overlay.
"""

from __future__ import annotations

import argparse
import csv
import html
import math
import re
import sys
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np
import yaml

import visualize_kalibr_rgb_thermal_align as align

try:
    import aslam_cv as acv
    import kalibr_common.ConfigReader as cr
except ImportError as exc:  # pragma: no cover - environment-dependent.
    raise RuntimeError("Source scripts/source_kalibr_official.sh before running this tool") from exc


def safe_label(value: str) -> str:
    label = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("._")
    return label or "kalibr_overlay"


def detector_image(image: np.ndarray) -> np.ndarray:
    if image.ndim == 3:
        return cv2.cvtColor(align.display_image(image), cv2.COLOR_BGR2GRAY)
    if image.dtype == np.uint16:
        return (image / 256).astype("uint8")
    if image.dtype == np.uint8:
        return image
    return align.to_gray8(image)


def detector_image_variants(image: np.ndarray) -> list[tuple[str, np.ndarray]]:
    base = detector_image(image)
    variants = [("kalibr_gray", base)]
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(base)
    variants.extend(
        [
            ("clahe", clahe),
            ("equalized", cv2.equalizeHist(base)),
            ("inverted", cv2.bitwise_not(base)),
            ("inverted_clahe", cv2.bitwise_not(clahe)),
        ]
    )
    return variants


def roi_detector_image_variants(image: np.ndarray, projected_points: np.ndarray) -> list[tuple[str, np.ndarray]]:
    base = detector_image(image)
    if projected_points.size == 0:
        return []
    h, w = base.shape[:2]
    finite = projected_points[np.isfinite(projected_points).all(axis=1)]
    if finite.size == 0:
        return []
    x0, y0 = np.floor(finite.min(axis=0)).astype(int)
    x1, y1 = np.ceil(finite.max(axis=0)).astype(int)
    margin = 140
    x0 = max(0, x0 - margin)
    y0 = max(0, y0 - margin)
    x1 = min(w, x1 + margin)
    y1 = min(h, y1 + margin)
    if x1 <= x0 or y1 <= y0:
        return []

    roi = base[y0:y1, x0:x1]
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6)).apply(roi)
    strong_clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4)).apply(roi)
    eq = cv2.equalizeHist(roi)
    blurred = cv2.GaussianBlur(roi, (3, 3), 0)
    unsharp = cv2.addWeighted(roi, 1.7, blurred, -0.7, 0)
    unsharp_clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6)).apply(unsharp)
    adaptive = cv2.adaptiveThreshold(roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 3)
    _, otsu = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    variants = []
    for name, roi_variant in (
        ("roi_clahe", clahe),
        ("roi_strong_clahe", strong_clahe),
        ("roi_equalized", eq),
        ("roi_blur_clahe", cv2.createCLAHE(clipLimit=2.5, tileGridSize=(6, 6)).apply(blurred)),
        ("roi_unsharp_clahe", unsharp_clahe),
        ("roi_adaptive", adaptive),
        ("roi_adaptive_inv", cv2.bitwise_not(adaptive)),
        ("roi_otsu", otsu),
        ("roi_otsu_inv", cv2.bitwise_not(otsu)),
        ("roi_inverted_clahe", cv2.bitwise_not(clahe)),
    ):
        full = base.copy()
        full[y0:y1, x0:x1] = roi_variant
        variants.append((name, full))

        masked = np.full_like(base, int(np.median(base)))
        masked[y0:y1, x0:x1] = roi_variant
        variants.append((f"{name}_masked", masked))
    return variants


def make_detector(cam_raw: dict, target_yaml: Path):
    cam = cr.AslamCamera(
        cam_raw["camera_model"],
        cam_raw["intrinsics"],
        cam_raw["distortion_model"],
        cam_raw["distortion_coeffs"],
        cam_raw["resolution"],
    )
    target = cr.CalibrationTargetParameters(str(target_yaml))
    params = target.getTargetParams()
    if target.getTargetType() != "circlegrid":
        raise ValueError("This tool currently expects a Kalibr circlegrid target")

    circle_opts = acv.CirclegridOptions()
    circle_opts.useAsymmetricCirclegrid = params["asymmetricGrid"]
    grid = acv.GridCalibrationTargetCirclegrid(params["targetRows"], params["targetCols"], params["spacingMeters"], circle_opts)

    detector_opts = acv.GridDetectorOptions()
    detector_opts.filterCornerOutliers = True
    return acv.GridDetector(cam.geometry, grid, detector_opts)


def stamp_to_aslam_time(stamp: float):
    sec = int(stamp)
    nsec = int(round((stamp - sec) * 1e9))
    return acv.Time(sec, nsec)


def detect_from_variants(detector, stamp: float, variants: list[tuple[str, np.ndarray]]):
    for mode, image_variant in variants:
        for detector_method, method_name in (
            (detector.findTarget, "pose_checked"),
            (detector.findTargetNoTransformation, "corners_only"),
        ):
            success, obs = detector_method(stamp_to_aslam_time(stamp), image_variant)
            if not success:
                continue
            corners = np.asarray(obs.getCornersImageFrame(), dtype=np.float64).reshape(-1, 2)
            target = np.asarray(obs.getCornersTargetFrame(), dtype=np.float64).reshape(-1, 3)
            ids = np.asarray(obs.getCornersIdx(), dtype=np.int32).reshape(-1)
            if len(corners) < 8:
                continue
            return True, {"corners": corners, "target": target, "ids": ids, "mode": f"{mode}/{method_name}"}
    return False, None


def detect(detector, stamp: float, image: np.ndarray):
    return detect_from_variants(detector, stamp, detector_image_variants(image))


def draw_points(image: np.ndarray, points: np.ndarray, color: tuple[int, int, int], label: str) -> np.ndarray:
    canvas = align.display_image(image)
    for point in points:
        cv2.circle(canvas, tuple(np.round(point).astype(int)), 4, color, 2, cv2.LINE_AA)
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 32), (0, 0, 0), -1)
    cv2.putText(canvas, label, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
    return canvas


def thermal_color(image: np.ndarray) -> np.ndarray:
    gray = detector_image(image)
    return cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)


def pale_thermal_color(image: np.ndarray) -> np.ndarray:
    thermal = thermal_color(image)
    hsv = cv2.cvtColor(thermal, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * 0.28, 0, 255)
    hsv[:, :, 2] = np.clip(hsv[:, :, 2] * 0.80, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def enhance_rgb(image: np.ndarray) -> np.ndarray:
    bgr = align.display_image(image)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * 1.75, 0, 255)
    hsv[:, :, 2] = np.clip(hsv[:, :, 2] * 1.20, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def masked_plane_overlays(rgb: np.ndarray, thermal: np.ndarray, rgb_pts: np.ndarray, thermal_projected: np.ndarray, alpha: float) -> tuple[np.ndarray, np.ndarray]:
    thermal_canvas = pale_thermal_color(thermal)
    thermal_dark = np.clip(thermal_canvas.astype(np.float32) * 0.36, 0, 255).astype(np.uint8)
    rgb_canvas = enhance_rgb(rgb)

    H, inliers = cv2.findHomography(rgb_pts.astype(np.float32), thermal_projected.astype(np.float32), cv2.RANSAC, 5.0)
    if H is None:
        return thermal_canvas, thermal_canvas

    warped_rgb = cv2.warpPerspective(rgb_canvas, H, (thermal_canvas.shape[1], thermal_canvas.shape[0]), flags=cv2.INTER_LINEAR)
    hull = cv2.convexHull(thermal_projected.astype(np.float32)).astype(np.int32)
    mask = np.zeros(thermal_canvas.shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(mask, hull, 255)
    mask = cv2.dilate(mask, np.ones((25, 25), np.uint8), iterations=1)

    mixed = cv2.addWeighted(thermal_dark, 1.0 - alpha, warped_rgb, alpha, 0.0)
    out = thermal_canvas.copy()
    out[mask > 0] = mixed[mask > 0]
    cv2.polylines(out, [hull], True, (255, 255, 0), 2, cv2.LINE_AA)
    for point in thermal_projected:
        cv2.drawMarker(out, tuple(np.round(point).astype(int)), (255, 0, 255), cv2.MARKER_CROSS, 11, 2, cv2.LINE_AA)
    cv2.rectangle(out, (0, 0), (out.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(out, "Color overlay: bright RGB on dim thermal", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)

    gray_rgb = cv2.cvtColor(warped_rgb, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_rgb, 60, 140)
    edge_out = thermal_canvas.copy()
    edge_out[mask > 0] = np.clip(edge_out[mask > 0].astype(np.float32) * 0.52, 0, 255).astype(np.uint8)
    edge_out[(edges > 0) & (mask > 0)] = (255, 255, 0)
    cv2.polylines(edge_out, [hull], True, (255, 255, 255), 2, cv2.LINE_AA)
    for point in thermal_projected:
        cv2.drawMarker(edge_out, tuple(np.round(point).astype(int)), (255, 0, 255), cv2.MARKER_CROSS, 11, 2, cv2.LINE_AA)
    cv2.rectangle(edge_out, (0, 0), (edge_out.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(edge_out, "RGB edges on thermal", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)
    return out, edge_out


def hstack_resize(images: list[np.ndarray], target_h: int = 420) -> np.ndarray:
    resized = []
    for image in images:
        scale = target_h / image.shape[0]
        width = max(1, int(round(image.shape[1] * scale)))
        resized.append(cv2.resize(image, (width, target_h), interpolation=cv2.INTER_AREA))
    return np.hstack(resized)


def crop_zoom(image: np.ndarray, points: np.ndarray, output_h: int = 620) -> np.ndarray:
    if points.size == 0:
        return image
    h, w = image.shape[:2]
    finite = points[np.isfinite(points).all(axis=1)]
    if finite.size == 0:
        return image
    x0, y0 = np.floor(finite.min(axis=0)).astype(int)
    x1, y1 = np.ceil(finite.max(axis=0)).astype(int)
    margin = 70
    x0 = max(0, x0 - margin)
    y0 = max(0, y0 - margin)
    x1 = min(w, x1 + margin)
    y1 = min(h, y1 + margin)
    if x1 <= x0 or y1 <= y0:
        return image
    crop = image[y0:y1, x0:x1]
    scale = output_h / crop.shape[0]
    out_w = max(1, int(round(crop.shape[1] * scale)))
    return cv2.resize(crop, (out_w, output_h), interpolation=cv2.INTER_CUBIC)


def quality_label(mean_err: Optional[float], max_err: Optional[float]) -> str:
    if mean_err is None:
        return "no_metric"
    if mean_err <= 0.5 and (max_err is None or max_err <= 1.5):
        return "good"
    if mean_err <= 1.0 and (max_err is None or max_err <= 3.0):
        return "ok"
    return "poor"


def write_error_plot(out_dir: Path, rows: list[dict], filename: str) -> None:
    plot_w, plot_h = 1000, 360
    pad_l, pad_r, pad_t, pad_b = 70, 30, 30, 55
    canvas = np.full((plot_h, plot_w, 3), 245, dtype=np.uint8)
    measured = [r for r in rows if r["mean_common_error_px"] is not None]
    if not measured:
        cv2.putText(canvas, "No thermal detection metrics", (60, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.imwrite(str(out_dir / filename), canvas)
        return
    xs = np.arange(len(measured), dtype=np.float64)
    means = np.asarray([r["mean_common_error_px"] for r in measured], dtype=np.float64)
    medians = np.asarray([r["median_common_error_px"] for r in measured], dtype=np.float64)
    maxes = np.asarray([r["max_common_error_px"] for r in measured], dtype=np.float64)
    ymax = max(1.0, float(np.nanmax(maxes)) * 1.15)

    def map_xy(x: float, y: float) -> tuple[int, int]:
        px = pad_l + int(round((plot_w - pad_l - pad_r) * (x / max(1.0, len(measured) - 1))))
        py = plot_h - pad_b - int(round((plot_h - pad_t - pad_b) * (y / ymax)))
        return px, py

    cv2.line(canvas, (pad_l, pad_t), (pad_l, plot_h - pad_b), (80, 80, 80), 1)
    cv2.line(canvas, (pad_l, plot_h - pad_b), (plot_w - pad_r, plot_h - pad_b), (80, 80, 80), 1)
    for y in [0.5, 1.0, 2.0, 3.0]:
        if y <= ymax:
            _, py = map_xy(0, y)
            cv2.line(canvas, (pad_l, py), (plot_w - pad_r, py), (220, 220, 220), 1)
            cv2.putText(canvas, f"{y:.1f}px", (8, py + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1, cv2.LINE_AA)

    def draw_series(values: np.ndarray, color: tuple[int, int, int], name: str, yoff: int) -> None:
        pts = np.asarray([map_xy(i, float(v)) for i, v in enumerate(values)], dtype=np.int32)
        cv2.polylines(canvas, [pts], False, color, 2, cv2.LINE_AA)
        for pt in pts:
            cv2.circle(canvas, tuple(pt), 3, color, -1, cv2.LINE_AA)
        cv2.putText(canvas, name, (plot_w - 210, 35 + yoff), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    draw_series(means, (0, 120, 255), "mean", 0)
    draw_series(medians, (40, 170, 40), "median", 24)
    draw_series(maxes, (40, 40, 220), "max", 48)
    cv2.putText(canvas, "Projected-vs-detected thermal circle error per sampled frame", (pad_l, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.imwrite(str(out_dir / filename), canvas)


def write_quality_mosaic(out_dir: Path, rows: list[dict], quality: str, filename: str) -> bool:
    selected = [row for row in rows if row["quality"] == quality]
    if not selected:
        canvas = np.full((260, 900, 3), 245, dtype=np.uint8)
        cv2.putText(canvas, f"No {quality} frames in this sample", (45, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (30, 30, 30), 2, cv2.LINE_AA)
        cv2.imwrite(str(out_dir / filename), canvas)
        return False
    images = []
    for row in selected:
        image = cv2.imread(str(out_dir / row["zoom_image"]))
        if image is None:
            continue
        label = (
            f"seq {row['seq']} | mean {row['mean_common_error_px'] or 'n/a'} | "
            f"max {row['max_common_error_px'] or 'n/a'} | {row['thermal_detection_mode']}"
        )
        cv2.rectangle(image, (0, 0), (image.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(image, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        images.append(image)
    if not images:
        return False

    target_h = max(image.shape[0] for image in images)
    normalized = []
    for image in images:
        if image.shape[0] != target_h:
            width = max(1, int(round(image.shape[1] * target_h / image.shape[0])))
            image = cv2.resize(image, (width, target_h), interpolation=cv2.INTER_AREA)
        normalized.append(image)

    mosaic_rows = []
    for idx in range(0, len(normalized), 2):
        row_images = normalized[idx : idx + 2]
        if len(row_images) == 1:
            row_images.append(np.zeros_like(row_images[0]))
        max_h = max(image.shape[0] for image in row_images)
        row_images = [
            cv2.copyMakeBorder(image, 0, max_h - image.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))
            for image in row_images
        ]
        mosaic_rows.append(np.hstack(row_images))

    max_w = max(row.shape[1] for row in mosaic_rows)
    mosaic_rows = [
        cv2.copyMakeBorder(row, 0, 0, 0, max_w - row.shape[1], cv2.BORDER_CONSTANT, value=(20, 20, 20))
        for row in mosaic_rows
    ]
    cv2.imwrite(str(out_dir / filename), np.vstack(mosaic_rows))
    return True


def common_error_details(projected: np.ndarray, rgb_ids: np.ndarray, thermal_obs: Optional[dict]) -> list[tuple[np.ndarray, np.ndarray, float]]:
    if thermal_obs is None:
        return []
    th_by_id = {int(idx): point for idx, point in zip(thermal_obs["ids"], thermal_obs["corners"])}
    details = []
    for idx, point in zip(rgb_ids, projected):
        th = th_by_id.get(int(idx))
        if th is not None:
            details.append((point, th, float(np.linalg.norm(point - th))))
    return details


def common_error(projected: np.ndarray, rgb_ids: np.ndarray, thermal_obs: Optional[dict]) -> tuple[int, Optional[float], Optional[float], Optional[float]]:
    details = common_error_details(projected, rgb_ids, thermal_obs)
    errors = [err for _, _, err in details]
    if not errors:
        return 0, None, None, None
    return len(errors), float(np.mean(errors)), float(np.median(errors)), float(np.max(errors))


def draw_thermal_evaluation(
    thermal: np.ndarray,
    detected: np.ndarray,
    projected: np.ndarray,
    error_details: list[tuple[np.ndarray, np.ndarray, float]],
    label: str,
) -> np.ndarray:
    canvas = draw_points(thermal_color(thermal), detected, (0, 220, 0), label)
    for projected_point, detected_point, err in error_details:
        color = (255, 255, 0) if err <= 1.5 else (0, 0, 255)
        cv2.line(canvas, tuple(np.round(detected_point).astype(int)), tuple(np.round(projected_point).astype(int)), color, 2, cv2.LINE_AA)
    for point in projected:
        cv2.drawMarker(canvas, tuple(np.round(point).astype(int)), (255, 0, 255), cv2.MARKER_CROSS, 11, 2, cv2.LINE_AA)
    if error_details:
        errors = np.asarray([err for _, _, err in error_details], dtype=np.float64)
        text = f"green=thermal detected, magenta=RGB via extrinsic, cyan/red lines=error | mean {errors.mean():.2f}px med {np.median(errors):.2f}px max {errors.max():.2f}px"
    else:
        text = "green=thermal detected, magenta=RGB via extrinsic; no common detected ids"
    cv2.rectangle(canvas, (0, canvas.shape[0] - 30), (canvas.shape[1], canvas.shape[0]), (0, 0, 0), -1)
    cv2.putText(canvas, text, (8, canvas.shape[0] - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def common_detected_points(rgb_obs: dict, thermal_obs: Optional[dict]) -> tuple[np.ndarray, np.ndarray]:
    if thermal_obs is None:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0, 2), dtype=np.float64)
    th_by_id = {int(idx): point for idx, point in zip(thermal_obs["ids"], thermal_obs["corners"])}
    rgb_points = []
    thermal_points = []
    for idx, point in zip(rgb_obs["ids"], rgb_obs["corners"]):
        th = th_by_id.get(int(idx))
        if th is not None:
            rgb_points.append(point)
            thermal_points.append(th)
    if not rgb_points:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0, 2), dtype=np.float64)
    return np.asarray(rgb_points, dtype=np.float64), np.asarray(thermal_points, dtype=np.float64)


def detected_homography_overlay(rgb: np.ndarray, thermal: np.ndarray, rgb_points: np.ndarray, thermal_points: np.ndarray, alpha: float) -> np.ndarray:
    base = pale_thermal_color(thermal)
    if len(rgb_points) < 4 or len(thermal_points) < 4:
        cv2.rectangle(base, (0, 0), (base.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(base, "Detected-point homography: no thermal detection", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (255, 255, 255), 2, cv2.LINE_AA)
        return base

    H, inliers = cv2.findHomography(rgb_points.astype(np.float32), thermal_points.astype(np.float32), cv2.RANSAC, 3.0)
    if H is None:
        cv2.rectangle(base, (0, 0), (base.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(base, "Detected-point homography failed", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (255, 255, 255), 2, cv2.LINE_AA)
        return base

    warped_rgb = cv2.warpPerspective(enhance_rgb(rgb), H, (base.shape[1], base.shape[0]), flags=cv2.INTER_LINEAR)
    hull = cv2.convexHull(thermal_points.astype(np.float32)).astype(np.int32)
    mask = np.zeros(base.shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(mask, hull, 255)
    mask = cv2.dilate(mask, np.ones((25, 25), np.uint8), iterations=1)

    underlay = np.clip(base.astype(np.float32) * 0.36, 0, 255).astype(np.uint8)
    mixed = cv2.addWeighted(underlay, 1.0 - alpha, warped_rgb, alpha, 0.0)
    out = base.copy()
    out[mask > 0] = mixed[mask > 0]
    cv2.polylines(out, [hull], True, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.rectangle(out, (0, 0), (out.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(out, "Detected-point homography overlay", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def write_index(out_dir: Path, rows: list[dict], metadata: dict[str, str]) -> None:
    parts = [
        "<!doctype html><meta charset='utf-8'>",
        "<title>Kalibr Target-Plane Overlay</title>",
        "<style>body{font-family:sans-serif;background:#111;color:#eee} img{max-width:100%;height:auto;border:1px solid #333} code{color:#9ef} .good{background:#15351f}.ok{background:#38320f}.poor{background:#401717}.no_metric{background:#282828} table{border-collapse:collapse} td,th{border:1px solid #444}</style>",
        "<h1>Kalibr Target-Plane Overlay</h1>",
        "<p>No assumed scene depth is used. RGB target plane is detected, projected through the Kalibr extrinsic, and overlaid on thermal.</p>",
        "<table border='1' cellspacing='0' cellpadding='6'>",
        "<tr><th>source bag</th><td><code>{}</code></td></tr>".format(html.escape(metadata["bag"])),
        "<tr><th>camchain</th><td><code>{}</code></td></tr>".format(html.escape(metadata["camchain"])),
        "<tr><th>output prefix</th><td><code>{}</code></td></tr>".format(html.escape(metadata["prefix"])),
        "<tr><th>summary</th><td><a href='{0}'>{0}</a></td></tr>".format(html.escape(metadata["summary"])),
        "<tr><th>csv</th><td><a href='{0}'>{0}</a></td></tr>".format(html.escape(metadata["csv"])),
        "</table>",
        "<p><b>Legend:</b> green circles = detected target points; magenta crosses = RGB target points projected into thermal using the Kalibr extrinsic; cyan/red line segments = per-point projection error vectors.</p>",
        "<p><a href='{0}'>Open error plot</a></p>".format(html.escape(metadata["error_plot"])),
        "<img src='{}'>".format(html.escape(metadata["error_plot"])),
        "<p><a href='{0}'>Open poor-frame zoom mosaic</a> | <a href='{1}'>Open no-metric zoom mosaic</a></p>".format(
            html.escape(metadata["poor_mosaic"]),
            html.escape(metadata["no_metric_mosaic"]),
        ),
        "<table border='1' cellspacing='0' cellpadding='6'>",
        "<tr><th>seq</th><th>quality</th><th>rgb stamp</th><th>thermal stamp</th><th>rgb mode</th><th>thermal mode</th><th>rgb corners</th><th>thermal corners</th><th>mean px</th><th>median px</th><th>max px</th><th>full view</th><th>zoom</th></tr>",
    ]
    for row in rows:
        mean_err = "n/a" if row["mean_common_error_px"] is None else f"{row['mean_common_error_px']:.3f}"
        median_err = "n/a" if row["median_common_error_px"] is None else f"{row['median_common_error_px']:.3f}"
        max_err = "n/a" if row["max_common_error_px"] is None else f"{row['max_common_error_px']:.3f}"
        quality = html.escape(row["quality"])
        parts.append(
            f"<tr class='{quality}'>"
            f"<td>{row['seq']}</td><td>{quality}</td><td>{row['rgb_stamp']:.6f}</td><td>{row['thermal_stamp']:.6f}</td>"
            f"<td>{html.escape(row['rgb_detection_mode'])}</td><td>{html.escape(row['thermal_detection_mode'])}</td>"
            f"<td>{row['rgb_corners']}</td><td>{row['thermal_corners']}</td><td>{mean_err}</td><td>{median_err}</td><td>{max_err}</td>"
            f"<td><a href='{html.escape(row['image'])}'><img src='{html.escape(row['image'])}'></a></td>"
            f"<td><a href='{html.escape(row['zoom_image'])}'><img src='{html.escape(row['zoom_image'])}'></a></td>"
            "</tr>"
        )
    parts.append("</table>")
    (out_dir / "index.html").write_text("\n".join(parts) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    align.require_cv2()
    with args.camchain.open("r", encoding="utf-8") as handle:
        camchain = yaml.safe_load(handle)
    rgb_cam, thermal_cam, T_thermal_rgb = align.load_camchain(args.camchain, args.rgb_cam, args.thermal_cam)
    rgb_detector = make_detector(camchain[args.rgb_cam], args.target)
    thermal_detector = make_detector(camchain[args.thermal_cam], args.target)

    prefix = safe_label(args.output_prefix or args.bag.stem)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    pairs = align.read_bag_pairs(args.bag, args.rgb_topic, args.thermal_topic, args.max_dt, args.max_frames, args.stride)
    if not pairs:
        raise RuntimeError("No synchronized image pairs found")

    rows = []
    for seq, pair in enumerate(pairs):
        rgb_ok, rgb_obs = detect(rgb_detector, pair.rgb_stamp, pair.rgb)
        th_ok, th_obs = detect(thermal_detector, pair.thermal_stamp, pair.thermal)
        if not rgb_ok or rgb_obs is None or len(rgb_obs["corners"]) < 8:
            continue

        ok, rvec_rgb, tvec_rgb = cv2.solvePnP(
            rgb_obs["target"].astype(np.float32),
            rgb_obs["corners"].astype(np.float32),
            rgb_cam.K,
            rgb_cam.D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            continue

        rvec_th, tvec_th = align.compose_pose(T_thermal_rgb, rvec_rgb, tvec_rgb)
        thermal_projected = align.project_points(rgb_obs["target"].astype(np.float32), rvec_th, tvec_th, thermal_cam.K, thermal_cam.D)
        if not th_ok:
            th_ok, th_obs = detect_from_variants(
                thermal_detector,
                pair.thermal_stamp,
                roi_detector_image_variants(pair.thermal, thermal_projected),
            )
        common_count, mean_err, median_err, max_err = common_error(thermal_projected, rgb_obs["ids"], th_obs if th_ok else None)
        error_details = common_error_details(thermal_projected, rgb_obs["ids"], th_obs if th_ok else None)

        rgb_panel = draw_points(pair.rgb, rgb_obs["corners"], (0, 220, 0), f"RGB detected: {len(rgb_obs['corners'])} points ({rgb_obs['mode']})")
        thermal_points = th_obs["corners"] if th_ok else np.zeros((0, 2), dtype=np.float64)
        thermal_mode = th_obs["mode"] if th_ok else "not_detected"
        thermal_panel = draw_thermal_evaluation(
            pair.thermal,
            thermal_points,
            thermal_projected,
            error_details,
            f"Thermal detected: {len(thermal_points)} points ({thermal_mode})",
        )

        color_overlay, edge_overlay = masked_plane_overlays(pair.rgb, pair.thermal, rgb_obs["corners"], thermal_projected, args.alpha)
        common_rgb, common_thermal = common_detected_points(rgb_obs, th_obs if th_ok else None)
        measured_overlay = detected_homography_overlay(pair.rgb, pair.thermal, common_rgb, common_thermal, args.alpha)
        contact = hstack_resize([rgb_panel, thermal_panel, color_overlay, edge_overlay, measured_overlay], args.contact_height)
        image_name = f"{prefix}_frame_{seq:04d}.png"
        cv2.imwrite(str(args.output_dir / image_name), contact)
        zoom_points = np.vstack([thermal_projected, thermal_points]) if len(thermal_points) else thermal_projected
        zoom_image = crop_zoom(thermal_panel, zoom_points)
        zoom_name = f"{prefix}_zoom_{seq:04d}.png"
        cv2.imwrite(str(args.output_dir / zoom_name), zoom_image)
        quality = quality_label(mean_err, max_err)
        rows.append(
            {
                "seq": seq,
                "rgb_stamp": pair.rgb_stamp,
                "thermal_stamp": pair.thermal_stamp,
                "dt_sec": abs(pair.rgb_stamp - pair.thermal_stamp),
                "rgb_corners": int(len(rgb_obs["corners"])),
                "thermal_corners": int(len(thermal_points)),
                "common_points": int(common_count),
                "mean_common_error_px": mean_err,
                "median_common_error_px": median_err,
                "max_common_error_px": max_err,
                "rgb_detection_mode": str(rgb_obs["mode"]),
                "thermal_detection_mode": str(thermal_mode),
                "quality": quality,
                "image": image_name,
                "zoom_image": zoom_name,
            }
        )

    if not rows:
        raise RuntimeError("No RGB target detections were good enough for target-plane overlay")

    csv_name = f"{prefix}_alignment_report.csv"
    summary_name = f"{prefix}_summary.txt"
    error_plot_name = f"{prefix}_error_plot.png"
    poor_mosaic_name = f"{prefix}_poor_mosaic.png"
    no_metric_mosaic_name = f"{prefix}_no_metric_mosaic.png"

    with (args.output_dir / csv_name).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    measured = [row["mean_common_error_px"] for row in rows if row["mean_common_error_px"] is not None]
    medians = [row["median_common_error_px"] for row in rows if row["median_common_error_px"] is not None]
    maxes = [row["max_common_error_px"] for row in rows if row["max_common_error_px"] is not None]
    qualities = {name: sum(1 for row in rows if row["quality"] == name) for name in ("good", "ok", "poor", "no_metric")}
    summary = [
        "Kalibr target-plane overlay",
        "",
        f"source bag: {args.bag}",
        f"camchain: {args.camchain}",
        f"output prefix: {prefix}",
        "",
        f"frames sampled: {len(pairs)}",
        f"frames with RGB target-plane overlay: {len(rows)}",
        f"frames with thermal detection metric: {len(measured)}",
        f"avg thermal projected-vs-detected error px: {float(np.mean(measured)):.3f}" if measured else "avg thermal projected-vs-detected error px: n/a",
        f"median frame median error px: {float(np.median(medians)):.3f}" if medians else "median frame median error px: n/a",
        f"worst frame max error px: {float(np.max(maxes)):.3f}" if maxes else "worst frame max error px: n/a",
        f"quality counts: good={qualities['good']} ok={qualities['ok']} poor={qualities['poor']} no_metric={qualities['no_metric']}",
        "",
        "No assumed scene depth is used.",
        "The overlay is valid on the detected calibration-board plane only.",
        "Green circles are detected target points.",
        "Magenta crosses are RGB-board points projected into thermal through the Kalibr extrinsic.",
        "Cyan/red line segments are projected-vs-detected error vectors.",
    ]
    (args.output_dir / summary_name).write_text("\n".join(summary) + "\n", encoding="utf-8")
    write_error_plot(args.output_dir, rows, error_plot_name)
    write_quality_mosaic(args.output_dir, rows, "poor", poor_mosaic_name)
    write_quality_mosaic(args.output_dir, rows, "no_metric", no_metric_mosaic_name)
    write_index(
        args.output_dir,
        rows,
        {
            "bag": str(args.bag),
            "camchain": str(args.camchain),
            "prefix": prefix,
            "summary": summary_name,
            "csv": csv_name,
            "error_plot": error_plot_name,
            "poor_mosaic": poor_mosaic_name,
            "no_metric_mosaic": no_metric_mosaic_name,
        },
    )
    print("\n".join(summary))
    print(f"wrote: {args.output_dir}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camchain", required=True, type=Path)
    parser.add_argument("--target", required=True, type=Path)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--output-prefix", default=None, help="Prefix for generated images/reports; defaults to the rosbag filename stem")
    parser.add_argument("--rgb-cam", default="cam0")
    parser.add_argument("--thermal-cam", default="cam1")
    parser.add_argument("--rgb-topic", default="/rgb_left")
    parser.add_argument("--thermal-topic", default="/thermal_left")
    parser.add_argument("--max-dt", default=0.05, type=float)
    parser.add_argument("--max-frames", default=24, type=int)
    parser.add_argument("--stride", default=40, type=int)
    parser.add_argument("--alpha", default=0.84, type=float)
    parser.add_argument("--contact-height", default=420, type=int)
    args = parser.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
