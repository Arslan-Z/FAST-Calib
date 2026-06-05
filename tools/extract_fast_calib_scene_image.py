#!/usr/bin/env python3
import argparse
import os
import statistics
import sys

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge


def msg_stamp(msg, bag_time):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and stamp.to_sec() > 0:
        return stamp.to_sec()
    return bag_time.to_sec()


def read_stamps(bag, topic):
    stamps = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        stamps.append(msg_stamp(msg, t))
    return stamps


def decode_image(bridge, msg, image_topic):
    if image_topic.endswith("/compressed") or msg._type == "sensor_msgs/CompressedImage":
        data = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("failed to decode compressed image")
        return image
    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main():
    parser = argparse.ArgumentParser(description="Extract a FAST-Calib image from the same bag as the LiDAR data.")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--lidar-topic", required=True)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--output-image", required=True)
    parser.add_argument("--metadata", required=True)
    args = parser.parse_args()

    if not os.path.isfile(args.bag):
        raise FileNotFoundError(args.bag)

    bridge = CvBridge()
    with rosbag.Bag(args.bag, "r") as bag:
        lidar_stamps = read_stamps(bag, args.lidar_topic)
        image_stamps = read_stamps(bag, args.image_topic)

        if not lidar_stamps:
            raise RuntimeError(f"no LiDAR messages on {args.lidar_topic}")
        if not image_stamps:
            raise RuntimeError(f"no image messages on {args.image_topic}")

        target_lidar_stamp = statistics.median(lidar_stamps)
        best = None
        best_dt = None
        best_bag_time = None

        for _, msg, t in bag.read_messages(topics=[args.image_topic]):
            stamp = msg_stamp(msg, t)
            dt = stamp - target_lidar_stamp
            if best is None or abs(dt) < abs(best_dt):
                best = msg
                best_dt = dt
                best_bag_time = t.to_sec()

        image = decode_image(bridge, best, args.image_topic)

    os.makedirs(os.path.dirname(args.output_image), exist_ok=True)
    image_root, image_ext = os.path.splitext(args.output_image)
    tmp_image = f"{image_root}.tmp{image_ext or '.png'}"
    if not cv2.imwrite(tmp_image, image):
        raise RuntimeError(f"failed to write {tmp_image}")
    os.replace(tmp_image, args.output_image)

    tmp_metadata = f"{args.metadata}.tmp"
    with open(tmp_metadata, "w") as f:
        f.write("fast_calib_scene_extraction\n")
        f.write(f"bag: {args.bag}\n")
        f.write(f"lidar_topic: {args.lidar_topic}\n")
        f.write(f"image_topic: {args.image_topic}\n")
        f.write(f"output_image: {args.output_image}\n")
        f.write(f"lidar_messages: {len(lidar_stamps)}\n")
        f.write(f"image_messages: {len(image_stamps)}\n")
        f.write(f"target_lidar_stamp: {target_lidar_stamp:.9f}\n")
        f.write(f"selected_image_stamp_delta_s: {best_dt:.9f}\n")
        f.write(f"selected_image_bag_time: {best_bag_time:.9f}\n")
        f.write(f"selected_image_shape: {image.shape[1]}x{image.shape[0]}\n")
    os.replace(tmp_metadata, args.metadata)

    print(f"wrote image: {args.output_image}")
    print(f"wrote metadata: {args.metadata}")
    print(f"selected image delta to median LiDAR stamp: {best_dt:.6f}s")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
