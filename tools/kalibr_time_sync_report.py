#!/usr/bin/env python3
import argparse
import bisect
import statistics
from collections import defaultdict

import rosbag


def summarize(values):
    if not values:
        return "n=0"
    values = sorted(values)

    def q(frac):
        idx = int((len(values) - 1) * frac)
        return values[idx]

    return (
        f"n={len(values)} mean={statistics.mean(values):.6f} "
        f"median={statistics.median(values):.6f} p05={q(0.05):.6f} "
        f"p95={q(0.95):.6f} min={values[0]:.6f} max={values[-1]:.6f}"
    )


def nearest_delta(source, target):
    out = []
    for stamp in source:
        idx = bisect.bisect_left(target, stamp)
        candidates = []
        if idx < len(target):
            candidates.append(target[idx] - stamp)
        if idx > 0:
            candidates.append(target[idx - 1] - stamp)
        if candidates:
            out.append(min(candidates, key=abs))
    return out


def read_stamps(bag_path, topics):
    header = {topic: [] for topic in topics}
    receive = {topic: [] for topic in topics}
    header_latency = {topic: [] for topic in topics}
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, recv_time in bag.read_messages(topics=topics):
            recv = recv_time.to_sec()
            receive[topic].append(recv)
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            if stamp is None:
                header[topic].append(recv)
            else:
                h = stamp.to_sec()
                header[topic].append(h)
                header_latency[topic].append(recv - h)
    return header, receive, header_latency


def print_limit_counts(title, values, limits):
    print(title)
    print(summarize(values))
    for limit in limits:
        count = sum(abs(value) <= limit for value in values)
        pct = 100.0 * count / len(values) if values else 0.0
        print(f"  within {limit:.3f}s: {count}/{len(values)} {pct:.1f}%")


def main():
    parser = argparse.ArgumentParser(description="Report RGB/thermal bag timestamp synchronization quality.")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--rgb-topic", required=True)
    parser.add_argument("--thermal-topic", required=True)
    parser.add_argument("--window", type=float, default=60.0)
    parser.add_argument("--strict-sync", type=float, default=0.02)
    args = parser.parse_args()

    topics = [args.rgb_topic, args.thermal_topic]
    header, receive, header_latency = read_stamps(args.bag, topics)
    rgb_header = header[args.rgb_topic]
    thermal_header = header[args.thermal_topic]
    rgb_receive = receive[args.rgb_topic]
    thermal_receive = receive[args.thermal_topic]

    print(f"bag: {args.bag}")
    print(f"rgb_topic: {args.rgb_topic}")
    print(f"thermal_topic: {args.thermal_topic}")
    print()

    print("receive_time - header_stamp [s]")
    for topic in topics:
        print(f"{topic}: {summarize(header_latency[topic])}")
    print()

    print("rates from header stamps")
    for topic, stamps in [(args.rgb_topic, rgb_header), (args.thermal_topic, thermal_header)]:
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
        rate = (len(stamps) - 1) / duration if duration > 0 else 0.0
        print(f"{topic}: rate={rate:.3f}Hz gaps {summarize(gaps)}")
    print()

    limits = [0.001, 0.002, 0.003, 0.005, 0.010, 0.015, 0.020, 0.030, 0.050]
    header_dt = nearest_delta(thermal_header, rgb_header)
    receive_dt = nearest_delta(thermal_receive, rgb_receive)
    print_limit_counts("thermal -> nearest RGB by header stamp, signed dt rgb-thermal [s]", header_dt, limits)
    print()
    print_limit_counts("thermal -> nearest RGB by bag receive time, signed dt rgb-thermal [s]", receive_dt, limits)
    print()

    print(f"header dt by {args.window:.0f}s window")
    if thermal_header and header_dt:
        start = thermal_header[0]
        buckets = defaultdict(list)
        for stamp, dt in zip(thermal_header, header_dt):
            buckets[int((stamp - start) // args.window)].append(dt)
        for bucket in sorted(buckets):
            values = buckets[bucket]
            abs95 = sorted(abs(value) for value in values)[int(0.95 * (len(values) - 1))]
            print(
                f"{bucket * args.window:06.1f}-{(bucket + 1) * args.window:06.1f}s "
                f"n={len(values)} median={statistics.median(values):.6f} "
                f"mean={statistics.mean(values):.6f} p95abs={abs95:.6f}"
            )
    print()

    in_sync = sum(abs(value) <= args.strict_sync for value in header_dt)
    pct = 100.0 * in_sync / len(header_dt) if header_dt else 0.0
    rgb_latency = statistics.median(header_latency[args.rgb_topic]) if header_latency[args.rgb_topic] else 0.0
    thermal_latency = statistics.median(header_latency[args.thermal_topic]) if header_latency[args.thermal_topic] else 0.0
    latency_delta = rgb_latency - thermal_latency

    print("recommendation")
    print(f"- strict_sync={args.strict_sync:.3f}s keeps {in_sync}/{len(header_dt)} thermal frames ({pct:.1f}%) by header stamp.")
    if pct >= 95.0:
        print("- Current 0.02s strict sync is reasonable for this bag.")
    else:
        print("- Current strict sync drops many frames; inspect drivers or use a larger approx-sync only for diagnostics.")
    print(f"- Median RGB receive-header latency is {rgb_latency:.3f}s; thermal is {thermal_latency:.3f}s; delta is {latency_delta:.3f}s.")
    if abs(latency_delta) > 0.02:
        print("- Header timestamp semantics differ across streams. For future data, prefer a driver/republisher setup that stamps both streams with comparable acquisition time or both with host receive time.")
    print("- For calibration accuracy, keep RGB at 30Hz unless the driver load causes drops; reducing RGB to 9Hz without hardware trigger does not guarantee better pairing.")
    print("- Continue collecting mostly stationary poses and avoid using frames while moving the target.")


if __name__ == "__main__":
    main()
