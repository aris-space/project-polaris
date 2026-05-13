#!/usr/bin/env python3
"""
Plot top ultrasonic distance vs pressure sensor depth.

Usage:
    python3 plot_ultrasonic_vs_pressure.py [BAG ...] [--out FILE]

    BAG       Path to a rosbag2 MCAP file.
    --out     Output PNG path (default: ultrasonic_vs_pressure.png next to this script).

Reads:
    /top/ultrasonic/distance   (std_msgs/Float32) — distance in m, 0.0 = invalid
    /sensors/pressure/pose_enu (geometry_msgs/PoseWithCovarianceStamped) — z_enu in m
    /imu/data                  (sensor_msgs/Imu) — orientation quaternion for pitch

Only ultrasonic samples with distance > 0 are kept. Shows the ultrasonic
distance vs the smooth pressure sensor depth, with ice draft in the middle
panel and pitch in the bottom panel.
"""

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    sys.exit(1)

TOPIC_US    = "/top/ultrasonic/distance"
TOPIC_DEPTH = "/sensors/pressure/pose_enu"
TOPIC_IMU   = "/imu/data"

MIN_DEPTH_M = 0.5   # ignore samples shallower than this
MAX_US_M    = 5.0   # drop clear outliers beyond this distance

DEFAULT_BAGS = [
    "/ros2_ws/recordings/zermatt_rectangle_06_2026_04_29-13_22_13_0.mcap",
]


def quaternion_to_pitch_deg(qx, qy, qz, qw):
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    return math.degrees(math.asin(sinp))


def extract_data(bag_paths):
    topics = [TOPIC_US, TOPIC_DEPTH, TOPIC_IMU]

    us_ts, us_dist = [], []
    depth_ts, depth_m = [], []
    pitch_ts, pitch_deg = [], []
    t_offset = None
    latest_depth = None

    for bag_path in bag_paths:
        bag_path = Path(bag_path)
        print(f"Reading {bag_path.name}...")
        with AnyReader([bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic in topics]
            available = {c.topic for c in connections}
            for t in [TOPIC_US, TOPIC_DEPTH, TOPIC_IMU]:
                if t not in available:
                    print(f"  WARNING: {t} not found in {bag_path.name}", file=sys.stderr)

            for conn, t_ns, raw in reader.messages(connections=connections):
                t_s = t_ns * 1e-9
                if t_offset is None:
                    t_offset = t_s
                t_rel = t_s - t_offset

                msg = reader.deserialize(raw, conn.msgtype)

                if conn.topic == TOPIC_DEPTH:
                    d = -float(msg.pose.pose.position.z)
                    latest_depth = d
                    if d >= MIN_DEPTH_M:
                        depth_ts.append(t_rel)
                        depth_m.append(d)

                elif conn.topic == TOPIC_US:
                    d_m = float(msg.data)
                    if d_m <= 0.0:
                        continue
                    if latest_depth is None or latest_depth < MIN_DEPTH_M:
                        continue
                    if d_m > MAX_US_M:
                        continue
                    us_ts.append(t_rel)
                    us_dist.append(d_m)

                elif conn.topic == TOPIC_IMU:
                    q = msg.orientation
                    pitch_ts.append(t_rel)
                    pitch_deg.append(quaternion_to_pitch_deg(q.x, q.y, q.z, q.w))

    return (
        np.array(us_ts), np.array(us_dist),
        np.array(depth_ts), np.array(depth_m),
        np.array(pitch_ts), np.array(pitch_deg),
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bags", nargs="*", metavar="BAG",
                        help="MCAP rosbag2 files. Defaults to zermatt_rectangle_06.")
    parser.add_argument("--out", default=None, metavar="FILE",
                        help="Output PNG path (default: ultrasonic_vs_pressure.png)")
    args = parser.parse_args()

    bag_paths = args.bags or DEFAULT_BAGS
    out_path = (
        Path(args.out) if args.out
        else Path(__file__).parent / "ultrasonic_vs_pressure.png"
    )

    us_ts, us_dist, depth_ts, depth_m, pitch_ts, pitch_deg = extract_data(bag_paths)

    if len(us_ts) == 0:
        print("No ultrasonic data found.", file=sys.stderr)
        sys.exit(1)

    print(f"\nTop ultrasonic:   {len(us_ts):,} samples  "
          f"mean={us_dist.mean():.3f} m  std={us_dist.std():.3f} m  "
          f"range=[{us_dist.min():.3f}, {us_dist.max():.3f}] m")
    if len(depth_m) > 0:
        print(f"Pressure depth:   {len(depth_m):,} samples  "
              f"mean={depth_m.mean():.3f} m  std={depth_m.std():.4f} m")
    if len(pitch_deg) > 0:
        print(f"Pitch (IMU):      {len(pitch_deg):,} samples  "
              f"mean={pitch_deg.mean():.2f} deg  std={pitch_deg.std():.2f} deg  "
              f"range=[{pitch_deg.min():.2f}, {pitch_deg.max():.2f}] deg")

    ice_draft = np.interp(us_ts, depth_ts, depth_m) - us_dist

    # ── Figure ────────────────────────────────────────────────────────────────
    fig, (ax_main, ax_draft, ax_pitch) = plt.subplots(
        3, 1, figsize=(14, 11), sharex=True,
        gridspec_kw={"height_ratios": [2, 1, 1], "hspace": 0.06},
        constrained_layout=True,
    )
    fig.patch.set_facecolor("white")

    def style(ax):
        ax.set_facecolor("#f7f9fc")
        ax.spines[:].set_color("#cccccc")
        ax.tick_params(colors="#333333", labelsize=9)
        ax.grid(True, color="#dddddd", linewidth=0.5, zorder=0)

    style(ax_main)
    style(ax_draft)
    style(ax_pitch)

    # ── Top: pressure depth + ultrasonic distance ─────────────────────────────
    ax_main.plot(
        depth_ts, depth_m,
        lw=1.2, color="#2c3e50", alpha=0.9, zorder=2,
        label=f"Pressure sensor depth  (n = {len(depth_m):,})",
    )
    ax_main.scatter(
        us_ts, us_dist,
        s=4, color="#e74c3c", alpha=0.6, zorder=3, rasterized=True,
        label=f"Top ultrasonic distance  (n = {len(us_ts):,})",
    )
    ax_main.set_ylabel("Distance / Depth (m)", color="#333333", fontsize=10)
    ax_main.set_title(
        "Top Ultrasonic Distance vs Pressure Sensor Depth — Zermatt",
        color="#111111", fontsize=12, pad=8,
    )
    ax_main.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333",
                   loc="upper right")
    all_vals = np.concatenate([depth_m, us_dist])
    margin = 0.1
    ax_main.set_ylim(all_vals.max() + margin, max(0.0, all_vals.min() - margin))

    # ── Middle: ice draft ─────────────────────────────────────────────────────
    ax_draft.scatter(
        us_ts, ice_draft,
        s=4, color="#8e44ad", alpha=0.6, zorder=2, rasterized=True,
        label="Ice draft = pressure depth − ultrasonic distance",
    )
    ax_draft.axhline(np.median(ice_draft), color="#555555", lw=1.2, ls="--", alpha=0.7,
                     label=f"Median = {np.median(ice_draft):.3f} m  |  std = {ice_draft.std():.3f} m")
    ax_draft.set_ylabel("Ice draft (m)", color="#333333", fontsize=10)
    ax_draft.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333")

    # ── Bottom: pitch ─────────────────────────────────────────────────────────
    if len(pitch_deg) > 0:
        ax_pitch.plot(
            pitch_ts, pitch_deg,
            lw=1.0, color="#16a085", alpha=0.9, zorder=2,
            label=f"Pitch (IMU)  std = {pitch_deg.std():.2f}°",
        )
        ax_pitch.axhline(0.0, color="#555555", lw=0.8, ls=":", alpha=0.6)
    ax_pitch.set_ylabel("Pitch (°)", color="#333333", fontsize=10)
    ax_pitch.set_xlabel("Time since bag start (s)", color="#333333", fontsize=10)
    ax_pitch.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333",
                    loc="upper right")

    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
