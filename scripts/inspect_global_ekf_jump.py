"""
Find the moment of a sudden state jump in /odometry/filtered/global and print
the state immediately before and after.

This is the focused-debug companion to plot_global_ekf_divergence.py: when the
plot shows an instantaneous step from <1 km to tens of km, we want to know
what the position/velocity/orientation values were at the exact transition,
which tells us whether the jump came from a bad measurement, a predict-step
overshoot, or numerical drift in a specific state.

Usage:
    python3 scripts/inspect_global_ekf_jump.py <bag_dir> [--threshold 1000]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument(
        "--threshold",
        type=float,
        default=1000.0,
        help="Position-magnitude threshold (m) that defines the 'jump'. Default 1000.",
    )
    args = ap.parse_args()

    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    samples = []
    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == "/odometry/filtered/global"]
        for conn, _t, raw in reader.messages(connections=conns):
            ros = reader.deserialize(raw, conn.msgtype)
            t_ns = int(ros.header.stamp.sec) * 10**9 + int(ros.header.stamp.nanosec)
            p = ros.pose.pose.position
            o = ros.pose.pose.orientation
            v = ros.twist.twist.linear
            w = ros.twist.twist.angular
            cov = list(ros.pose.covariance)
            samples.append((
                t_ns,
                float(p.x), float(p.y), float(p.z),
                float(o.x), float(o.y), float(o.z), float(o.w),
                float(v.x), float(v.y), float(v.z),
                float(w.x), float(w.y), float(w.z),
                float(cov[0]), float(cov[7]),  # σ²_x, σ²_y
            ))

    samples.sort(key=lambda s: s[0])
    if not samples:
        print("ERROR: no /odometry/filtered/global in bag", file=sys.stderr)
        sys.exit(1)

    # Find the first index where magnitude crosses the threshold.
    crossing_idx = None
    for i, s in enumerate(samples):
        mag = math.sqrt(s[1] ** 2 + s[2] ** 2)
        if mag > args.threshold:
            crossing_idx = i
            break

    if crossing_idx is None:
        print(f"No sample exceeds threshold {args.threshold} m. Filter is healthy?")
        return

    # Show 5 samples on each side of the crossing.
    lo = max(0, crossing_idx - 5)
    hi = min(len(samples), crossing_idx + 6)
    t0_ns = samples[crossing_idx][0]

    print(
        f"Jump detected at sample {crossing_idx} of {len(samples)} "
        f"(threshold = {args.threshold} m).\n"
        f"Showing 5 samples before and after.\n"
    )
    header = (
        f"{'idx':>4} {'Δt(s)':>10} {'|p|(m)':>12} {'x':>12} {'y':>12} {'z':>8} "
        f"{'vx':>9} {'vy':>9} {'vz':>9} {'σ_x':>9} {'σ_y':>9} "
        f"{'wx':>9} {'wy':>9} {'wz':>9}"
    )
    print(header)
    print("-" * len(header))
    for i in range(lo, hi):
        s = samples[i]
        dt_s = (s[0] - t0_ns) * 1e-9
        mag = math.sqrt(s[1] ** 2 + s[2] ** 2)
        marker = " ←" if i == crossing_idx else ""
        print(
            f"{i:>4} {dt_s:>+10.4f} {mag:>12.3f} {s[1]:>12.3f} {s[2]:>12.3f} {s[3]:>8.3f} "
            f"{s[8]:>9.3f} {s[9]:>9.3f} {s[10]:>9.3f} {math.sqrt(s[14]):>9.3f} {math.sqrt(s[15]):>9.3f} "
            f"{s[11]:>9.3f} {s[12]:>9.3f} {s[13]:>9.3f}{marker}"
        )


if __name__ == "__main__":
    main()
