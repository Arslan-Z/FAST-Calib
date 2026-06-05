#!/usr/bin/env python3
"""Create RGB/thermal image overlays from an existing Kalibr camchain.

This is a visualization tool, not a calibration tool. It uses Kalibr intrinsics
and extrinsics to warp RGB into the thermal image frame under either a
rotation-only approximation or an assumed fronto-parallel depth plane in the RGB
camera frame.
"""

from __future__ import annotations

import argparse
import html
import math
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

import visualize_kalibr_rgb_thermal_align as align


def parse_depths(value: str) -> list[float]:
    depths = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        depth = float(part)
        if depth <= 0:
            raise argparse.ArgumentTypeError("depths must be positive meters")
        depths.append(depth)
    if not depths:
        raise argparse.ArgumentTypeError("at least one depth is required")
    return depths


def undistort(image: np.ndarray, cam: align.CameraModel) -> np.ndarray:
    cv2 = align.require_cv2()
    width, height = cam.resolution
    return cv2.undistort(image, cam.K, cam.D, None, cam.K)[:height, :width]


def thermal_color(image: np.ndarray) -> np.ndarray:
    cv2 = align.require_cv2()
    gray = align.to_gray8(image)
    return cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)


def image_to_bgr(image: np.ndarray) -> np.ndarray:
    cv2 = align.require_cv2()
    if image.ndim == 2:
        return cv2.cvtColor(align.to_gray8(image), cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 3:
        return align.display_image(image)
    return cv2.cvtColor(align.display_image(image), cv2.COLOR_BGRA2BGR)


def plane_homography_rgb_to_thermal(
    rgb_cam: align.CameraModel,
    thermal_cam: align.CameraModel,
    T_thermal_rgb: np.ndarray,
    depth_m: Optional[float],
) -> np.ndarray:
    R = T_thermal_rgb[:3, :3]
    t = T_thermal_rgb[:3, 3].reshape(3, 1)
    if depth_m is None:
        motion = R
    else:
        n = np.array([[0.0, 0.0, 1.0]], dtype=np.float64)
        motion = R + (t @ n) / float(depth_m)
    return thermal_cam.K @ motion @ np.linalg.inv(rgb_cam.K)


def draw_label(image: np.ndarray, text: str) -> np.ndarray:
    cv2 = align.require_cv2()
    out = image.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(out, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def make_overlay(
    rgb_undist: np.ndarray,
    thermal_undist: np.ndarray,
    rgb_cam: align.CameraModel,
    thermal_cam: align.CameraModel,
    T_thermal_rgb: np.ndarray,
    depth_m: Optional[float],
    alpha: float,
) -> tuple[np.ndarray, np.ndarray]:
    cv2 = align.require_cv2()
    width, height = thermal_cam.resolution
    H = plane_homography_rgb_to_thermal(rgb_cam, thermal_cam, T_thermal_rgb, depth_m)
    rgb_bgr = image_to_bgr(rgb_undist)
    warped_rgb = cv2.warpPerspective(rgb_bgr, H, (width, height), flags=cv2.INTER_LINEAR)
    thermal_bgr = thermal_color(thermal_undist)
    valid_mask = (warped_rgb.sum(axis=2) > 0).astype(np.uint8)
    blended = thermal_bgr.copy()
    mixed = cv2.addWeighted(thermal_bgr, 1.0 - alpha, warped_rgb, alpha, 0.0)
    blended[valid_mask > 0] = mixed[valid_mask > 0]
    return warped_rgb, blended


def hstack_resize(images: list[np.ndarray], target_h: int = 360) -> np.ndarray:
    cv2 = align.require_cv2()
    resized = []
    for image in images:
        scale = target_h / image.shape[0]
        width = max(1, int(round(image.shape[1] * scale)))
        resized.append(cv2.resize(image, (width, target_h), interpolation=cv2.INTER_AREA))
    return np.hstack(resized)


def write_index(out_dir: Path, rows: list[dict], depths: list[float], include_rotation_only: bool) -> None:
    depth_labels = [f"depth_{d:.2f}m" for d in depths]
    if include_rotation_only:
        depth_labels = ["rotation_only"] + depth_labels

    parts = [
        "<!doctype html><meta charset='utf-8'>",
        "<title>Kalibr RGB/Thermal Overlay Preview</title>",
        "<style>body{font-family:sans-serif;background:#111;color:#eee} img{max-width:100%;height:auto;border:1px solid #333} code{color:#9ef}</style>",
        "<h1>Kalibr RGB/Thermal Overlay Preview</h1>",
        "<p>RGB is warped into the thermal frame using the Kalibr camchain. Depth overlays assume a fronto-parallel plane in the RGB camera frame.</p>",
        "<table border='1' cellspacing='0' cellpadding='6'>",
        "<tr><th>frame</th><th>rgb stamp</th><th>thermal stamp</th><th>dt sec</th><th>contact sheet</th></tr>",
    ]
    for row in rows:
        parts.append(
            "<tr>"
            f"<td>{row['index']}</td>"
            f"<td>{row['rgb_stamp']:.6f}</td>"
            f"<td>{row['thermal_stamp']:.6f}</td>"
            f"<td>{row['dt_sec']:.6f}</td>"
            f"<td><a href='{html.escape(row['contact_sheet'])}'><img src='{html.escape(row['contact_sheet'])}'></a></td>"
            "</tr>"
        )
    parts.append("</table>")
    parts.append("<h2>Overlay sets</h2><ul>")
    for label in depth_labels:
        parts.append(f"<li><code>{html.escape(label)}/</code></li>")
    parts.append("</ul>")
    (out_dir / "index.html").write_text("\n".join(parts) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    cv2 = align.require_cv2()
    rgb_cam, thermal_cam, T_thermal_rgb = align.load_camchain(args.camchain, args.rgb_cam, args.thermal_cam)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    if args.bag:
        pairs = align.read_bag_pairs(args.bag, args.rgb_topic, args.thermal_topic, args.max_dt, args.max_frames, args.stride)
    else:
        pairs = align.read_single_pair(args.rgb_image, args.thermal_image)
    if not pairs:
        raise RuntimeError("No synchronized image pairs found")

    overlay_modes: list[tuple[str, Optional[float]]] = []
    if args.rotation_only:
        overlay_modes.append(("rotation_only", None))
    overlay_modes.extend((f"depth_{depth:.2f}m", depth) for depth in args.depths)

    rows = []
    for out_name, _ in overlay_modes:
        (args.output_dir / out_name).mkdir(parents=True, exist_ok=True)
    (args.output_dir / "contact_sheets").mkdir(parents=True, exist_ok=True)

    for seq, pair in enumerate(pairs):
        rgb_undist = undistort(pair.rgb, rgb_cam)
        thermal_undist = undistort(pair.thermal, thermal_cam)
        thermal_base = thermal_color(thermal_undist)
        rgb_preview = image_to_bgr(rgb_undist)

        panels = [
            draw_label(cv2.resize(rgb_preview, thermal_cam.resolution), "RGB undistorted, resized only"),
            draw_label(thermal_base, "Thermal undistorted colormap"),
        ]

        for out_name, depth in overlay_modes:
            warped_rgb, blended = make_overlay(
                rgb_undist=rgb_undist,
                thermal_undist=thermal_undist,
                rgb_cam=rgb_cam,
                thermal_cam=thermal_cam,
                T_thermal_rgb=T_thermal_rgb,
                depth_m=depth,
                alpha=args.alpha,
            )
            label = "rotation only" if depth is None else f"assumed depth {depth:.2f} m"
            cv2.imwrite(str(args.output_dir / out_name / f"frame_{seq:04d}_warped_rgb.png"), draw_label(warped_rgb, f"RGB warped to thermal: {label}"))
            cv2.imwrite(str(args.output_dir / out_name / f"frame_{seq:04d}_overlay.png"), draw_label(blended, f"Overlay: {label}"))
            panels.append(draw_label(blended, f"Overlay: {label}"))

        contact = hstack_resize(panels, target_h=args.contact_height)
        contact_name = f"contact_sheets/frame_{seq:04d}.png"
        cv2.imwrite(str(args.output_dir / contact_name), contact)
        rows.append(
            {
                "index": pair.index,
                "rgb_stamp": pair.rgb_stamp,
                "thermal_stamp": pair.thermal_stamp,
                "dt_sec": abs(pair.rgb_stamp - pair.thermal_stamp) if not math.isnan(pair.rgb_stamp) else math.nan,
                "contact_sheet": contact_name,
            }
        )

    summary_lines = [
        "Kalibr RGB/thermal image overlay preview",
        "",
        f"frames: {len(rows)}",
        f"rgb topic: {args.rgb_topic}",
        f"thermal topic: {args.thermal_topic}",
        f"alpha: {args.alpha}",
        f"depths m: {', '.join(f'{d:.2f}' for d in args.depths)}",
        f"rotation only: {args.rotation_only}",
        "",
        "Interpretation:",
        "- These overlays use the Kalibr camchain; they do not estimate new calibration.",
        "- Depth overlays assume a fronto-parallel plane at the listed depth in the RGB camera frame.",
        "- If nearby objects align at one depth and far objects align at another, that is expected parallax.",
        "- Use this as a qualitative visual check, then confirm with repeat Kalibr runs/report errors.",
    ]
    (args.output_dir / "summary.txt").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    write_index(args.output_dir, rows, args.depths, args.rotation_only)
    print("\n".join(summary_lines))
    print(f"wrote: {args.output_dir}")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camchain", required=True, type=Path, help="Kalibr camchain YAML")
    parser.add_argument("--rgb-cam", default="cam0", help="Kalibr RGB camera key")
    parser.add_argument("--thermal-cam", default="cam1", help="Kalibr thermal camera key")
    parser.add_argument("--output-dir", required=True, type=Path, help="Output directory")
    parser.add_argument("--depths", default=[0.6, 0.8, 1.0, 1.5, 2.0], type=parse_depths, help="Comma-separated assumed plane depths in meters")
    parser.add_argument("--alpha", default=0.45, type=float, help="RGB overlay alpha")
    parser.add_argument("--rotation-only", action="store_true", help="Also output an infinite-depth / rotation-only overlay")
    parser.add_argument("--contact-height", default=360, type=int, help="Panel height in contact sheets")

    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=Path, help="ROS bag containing RGB and thermal images")
    source.add_argument("--rgb-image", type=Path, help="Single RGB image")
    parser.add_argument("--thermal-image", type=Path, help="Single thermal image when using --rgb-image")

    parser.add_argument("--rgb-topic", default="/rgb_left")
    parser.add_argument("--thermal-topic", default="/thermal_left")
    parser.add_argument("--max-dt", default=0.05, type=float)
    parser.add_argument("--max-frames", default=12, type=int)
    parser.add_argument("--stride", default=20, type=int)
    args = parser.parse_args(argv)

    if args.rgb_image and not args.thermal_image:
        parser.error("--thermal-image is required when --rgb-image is used")
    if not 0.0 <= args.alpha <= 1.0:
        parser.error("--alpha must be between 0 and 1")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
