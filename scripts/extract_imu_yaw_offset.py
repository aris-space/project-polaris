#!/usr/bin/env python3
"""
Recover the yaw_offset_deg that imu_yaw_correction was running with at the
time a bag was recorded, by comparing matched-timestamp quaternions on
/imu/data (raw input) and /imu/data_corrected (rotated output).

imu_yaw_correction applies a rotation about +Z (CCW) of yaw_offset_deg to
the input quaternion. So for a matched pair:
    yaw(corrected) = yaw(raw) + yaw_offset_rad   (mod 2π)

Median of (yaw_corrected − yaw_raw) over many pairs gives the offset robustly
even if a few messages are dropped or out of order.

Use the printed value in offline_ekf_replay.launch.py via
`imu_yaw_offset_deg:=<value>` to recreate the on-bag heading frame when
running the offline imu_yaw_correction node.

Usage:
  python scripts/extract_imu_yaw_offset.py <bag_dir>

Dependencies: pip install rosbags numpy
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("pip install rosbags numpy", file=sys.stderr)
    raise

import numpy as np

TOPIC_RAW = "/imu/data"
TOPIC_CORR = "/imu/data_corrected"

# Two messages count as the "same instant" if their stamps are within this.
MATCH_TOLERANCE_S = 0.005  # 5 ms — well below the 10 ms IMU period at 100 Hz


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("bag_dir", help="Path to rosbag2 directory (containing metadata.yaml).")
    p.add_argument(
        "--max-pairs",
        type=int,
        default=10000,
        help="Stop after this many matched pairs (saves time on huge bags).",
    )
    args = p.parse_args()

    bag_path = Path(args.bag_dir)
    if not bag_path.exists():
        print(f"bag dir not found: {bag_path}", file=sys.stderr)
        return 2

    raw_yaws: list[tuple[float, float]] = []   # (stamp, yaw_rad)
    corr_yaws: list[tuple[float, float]] = []

    with AnyReader([bag_path]) as reader:
        topics = {c.topic for c in reader.connections}
        if TOPIC_RAW not in topics:
            print(f"missing topic in bag: {TOPIC_RAW}", file=sys.stderr)
            return 2
        if TOPIC_CORR not in topics:
            print(f"missing topic in bag: {TOPIC_CORR}", file=sys.stderr)
            return 2

        conns = [c for c in reader.connections if c.topic in (TOPIC_RAW, TOPIC_CORR)]
        for connection, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, connection.msgtype)
            t = _stamp_s(msg.header.stamp)
            q = msg.orientation
            yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
            if connection.topic == TOPIC_RAW:
                raw_yaws.append((t, yaw))
            else:
                corr_yaws.append((t, yaw))

    print(f"raw msgs: {len(raw_yaws)}  corrected msgs: {len(corr_yaws)}")
    if not raw_yaws or not corr_yaws:
        print("not enough samples to compare", file=sys.stderr)
        return 1

    raw_yaws.sort(key=lambda x: x[0])
    corr_yaws.sort(key=lambda x: x[0])
    raw_t = np.array([t for t, _ in raw_yaws])
    raw_y = np.array([y for _, y in raw_yaws])
    corr_t = np.array([t for t, _ in corr_yaws])
    corr_y = np.array([y for _, y in corr_yaws])

    deltas: list[float] = []
    j = 0
    for i, t in enumerate(corr_t):
        if len(deltas) >= args.max_pairs:
            break
        # advance j until raw_t[j] is within tolerance of t
        while j + 1 < len(raw_t) and raw_t[j + 1] <= t:
            j += 1
        # j now points to the latest raw_t <= t. Pick the closer of j and j+1.
        candidates = [j]
        if j + 1 < len(raw_t):
            candidates.append(j + 1)
        best = min(candidates, key=lambda k: abs(raw_t[k] - t))
        if abs(raw_t[best] - t) > MATCH_TOLERANCE_S:
            continue
        delta = _wrap_pi(corr_y[i] - raw_y[best])
        deltas.append(delta)

    if not deltas:
        print(
            f"no time-matched pairs within {MATCH_TOLERANCE_S * 1000:.1f} ms "
            f"— stamps too misaligned",
            file=sys.stderr,
        )
        return 1

    arr = np.array(deltas)
    med_rad = float(np.median(arr))
    med_deg = math.degrees(med_rad)
    p25_deg = math.degrees(float(np.percentile(arr, 25)))
    p75_deg = math.degrees(float(np.percentile(arr, 75)))
    std_deg = math.degrees(float(np.std(arr)))

    print()
    print(f"matched pairs analysed:        {len(deltas)}")
    print(f"yaw_offset_deg (median):       {med_deg:+.4f}°")
    print(f"yaw_offset_deg IQR (p25..p75): {p25_deg:+.4f}°  ..  {p75_deg:+.4f}°")
    print(f"yaw_offset_deg std:            {std_deg:.4f}°")
    print()

    if std_deg > 0.5:
        print(
            "WARNING: spread > 0.5° — the offset wasn't constant across the bag "
            "(possibly recalibrated mid-recording, or imu_yaw_correction's head_mot "
            "service was triggered). The median is still the best single value, "
            "but expect some residual yaw error after applying it."
        )

    print(f"\nUse:  imu_yaw_offset_deg:={med_deg:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
