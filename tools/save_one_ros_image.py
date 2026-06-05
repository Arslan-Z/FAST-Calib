#!/usr/bin/env python3
import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def convert_image(msg: Image):
    bridge = CvBridge()
    encoding = (msg.encoding or "").lower()
    try:
        if encoding in {"rgb8", "bgr8", "rgba8", "bgra8"}:
            return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError as exc:
        raise RuntimeError(f"cv_bridge conversion failed for encoding {msg.encoding}: {exc}") from exc


def write_portable_image(path: Path, image) -> None:
    tmp = path.with_name(path.name + ".tmp")
    if image.ndim == 2:
        if image.dtype != np.uint8:
            image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        header = f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
        payload = image.tobytes()
    else:
        if image.dtype != np.uint8:
            image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        if image.shape[2] == 4:
            image = image[:, :, :3]
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        header = f"P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n".encode("ascii")
        payload = rgb.tobytes()
    with tmp.open("wb") as handle:
        handle.write(header)
        handle.write(payload)

    check = cv2.imread(str(tmp), cv2.IMREAD_UNCHANGED)
    if check is None or check.size == 0:
        try:
            tmp.unlink()
        except OSError:
            pass
        raise RuntimeError(f"saved image did not pass cv2 readback: {tmp}")
    os.replace(str(tmp), str(path))


def main() -> int:
    parser = argparse.ArgumentParser(description="Save exactly one ROS sensor_msgs/Image as a valid image file.")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    rospy.init_node("save_one_ros_image", anonymous=True, disable_signals=True)
    msg = rospy.wait_for_message(args.topic, Image, timeout=args.timeout)
    image = convert_image(msg)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_portable_image(args.output, image)
    print(f"saved {args.output} shape={tuple(image.shape)} dtype={image.dtype} encoding={msg.encoding}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
