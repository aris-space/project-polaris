#!/usr/bin/env python3
"""
Plot ping sonar distance (confidence = 100 only) vs pressure sensor depth.

Usage:
    python3 plot_ping_sonar_reliability.py [BAG ...] [--out FILE]

    BAG       Path to a rosbag2 MCAP file.
    --out     Output PNG path (default: ping_sonar_reliability.png next to this script).

Reads:
    /ping_sonar/distance       (custom_msgs/Distance) — distance in mm, confidence 0-100
    /sensors/pressure/pose_enu (geometry_msgs/PoseWithCovarianceStamped) — z_enu in m

Only ping sonar samples with confidence == 100 are kept. Shows that even these
readings are noisy compared to the smooth pressure sensor depth.
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

TOPIC_PING = "/ping_sonar/distance"
TOPIC_DEPTH = "/sensors/pressure/pose_enu"
TOPIC_IMU = "/imu/data"

MIN_DEPTH_M = 0.5   # ignore samples shallower than this (ping sonar deadzone)
MAX_PING_M  = 10.0  # drop clear outliers beyond this distance


def quaternion_to_pitch_deg(qx, qy, qz, qw):
    """Tait-Bryan pitch (ZYX intrinsic) from quaternion, in degrees."""
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    return math.degrees(math.asin(sinp))

DEFAULT_BAGS = [
    "/ros2_ws/recordings/ping01_2026_04_01-14_46_23_0 (1).mcap",
]


def extract_data(bag_paths):
    """
    Returns:
        ping_ts   : timestamps (s, relative to first message across all bags)
        ping_dist : distances (m, converted from mm), confidence == 100 only
        depth_ts  : timestamps (s)
        depth_m   : depths (m, positive = down = -z_enu)
        pitch_ts  : timestamps (s)
        pitch_deg : pitch angle (deg) from /imu/data orientation
    """
    topics = [TOPIC_PING, TOPIC_DEPTH, TOPIC_IMU]

    ping_ts, ping_dist = [], []
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
            if TOPIC_PING not in available:
                print(f"  WARNING: {TOPIC_PING} not found in {bag_path.name}", file=sys.stderr)
            if TOPIC_DEPTH not in available:
                print(f"  WARNING: {TOPIC_DEPTH} not found in {bag_path.name}", file=sys.stderr)
            if TOPIC_IMU not in available:
                print(f"  WARNING: {TOPIC_IMU} not found in {bag_path.name}", file=sys.stderr)

            for conn, t_ns, raw in reader.messages(connections=connections):
                t_s = t_ns * 1e-9
                if t_offset is None:
                    t_offset = t_s
                t_rel = t_s - t_offset

                msg = reader.deserialize(raw, conn.msgtype)

                if conn.topic == TOPIC_DEPTH:
                    d = -float(msg.pose.pose.position.z)  # ENU z → depth positive down
                    latest_depth = d
                    if d >= MIN_DEPTH_M:
                        depth_ts.append(t_rel)
                        depth_m.append(d)

                elif conn.topic == TOPIC_PING:
                    if float(msg.confidence) != 100.0:
                        continue
                    if latest_depth is None or latest_depth < MIN_DEPTH_M:
                        continue
                    d_m = float(msg.distance) / 1000.0  # mm → m
                    if d_m > MAX_PING_M:
                        continue
                    ping_ts.append(t_rel)
                    ping_dist.append(d_m)

                elif conn.topic == TOPIC_IMU:
                    q = msg.orientation
                    pitch_ts.append(t_rel)
                    pitch_deg.append(quaternion_to_pitch_deg(q.x, q.y, q.z, q.w))

    return (
        np.array(ping_ts),
        np.array(ping_dist),
        np.array(depth_ts),
        np.array(depth_m),
        np.array(pitch_ts),
        np.array(pitch_deg),
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "bags", nargs="*", metavar="BAG",
        help="MCAP rosbag2 files. Defaults to ping01.",
    )
    parser.add_argument(
        "--out", default=None, metavar="FILE",
        help="Output PNG path (default: ping_sonar_reliability.png)",
    )
    args = parser.parse_args()

    bag_paths = args.bags or DEFAULT_BAGS
    out_path = (
        Path(args.out) if args.out
        else Path(__file__).parent / "ping_sonar_reliability.png"
    )

    ping_ts, ping_dist, depth_ts, depth_m, pitch_ts, pitch_deg = extract_data(bag_paths)

    if len(ping_ts) == 0:
        print("No ping sonar data with confidence = 100 found.", file=sys.stderr)
        sys.exit(1)

    print(f"\nPing sonar (conf=100): {len(ping_ts):,} samples  "
          f"mean={ping_dist.mean():.3f} m  std={ping_dist.std():.3f} m  "
          f"range=[{ping_dist.min():.3f}, {ping_dist.max():.3f}] m")
    if len(depth_m) > 0:
        print(f"Pressure depth:        {len(depth_m):,} samples  "
              f"mean={depth_m.mean():.3f} m  std={depth_m.std():.4f} m")
    if len(pitch_deg) > 0:
        print(f"Pitch (imu):           {len(pitch_deg):,} samples  "
              f"mean={pitch_deg.mean():.2f} deg  std={pitch_deg.std():.2f} deg  "
              f"range=[{pitch_deg.min():.2f}, {pitch_deg.max():.2f}] deg")

    # Interpolate pressure depth at ping sonar timestamps for ice draft
    ice_draft = np.interp(ping_ts, depth_ts, depth_m) - ping_dist

    # ── Figure: overlay + ice draft + pitch ──────────────────────────────────
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

    # ── Top: pressure depth + ping sonar distance ─────────────────────────────
    ax_main.plot(
        depth_ts, depth_m,
        lw=1.2, color="#2c3e50", alpha=0.9, zorder=2,
        label=f"Pressure sensor depth  (n = {len(depth_m):,})",
    )
    ax_main.scatter(
        ping_ts, ping_dist,
        s=4, color="#e74c3c", alpha=0.6, zorder=3, rasterized=True,
        label=f"Ping sonar distance — confidence = 100  (n = {len(ping_ts):,})",
    )
    ax_main.set_ylabel("Distance / Depth (m)", color="#333333", fontsize=10)
    ax_main.set_title(
        "Ping Sonar (confidence = 100) vs Pressure Sensor Depth — St. Moritz",
        color="#111111", fontsize=12, pad=8,
    )
    ax_main.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333",
                   loc="upper right")
    all_vals = np.concatenate([depth_m, ping_dist])
    margin = 0.2
    ax_main.set_ylim(all_vals.max() + margin, max(0.0, all_vals.min() - margin))

    # ── Middle: ice draft (pressure depth − ping distance) ───────────────────
    ax_draft.scatter(
        ping_ts, ice_draft,
        s=4, color="#8e44ad", alpha=0.6, zorder=2, rasterized=True,
        label="Ice draft = pressure depth − ping distance",
    )
    ax_draft.axhline(np.median(ice_draft), color="#555555", lw=1.2, ls="--", alpha=0.7,
                     label=f"Median = {np.median(ice_draft):.3f} m  |  std = {ice_draft.std():.3f} m")
    ax_draft.set_ylabel("Ice draft (m)", color="#333333", fontsize=10)
    ax_draft.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333")

    # ── Bottom: pitch from /imu/data ─────────────────────────────────────────
    if len(pitch_deg) > 0:
        ax_pitch.plot(
            pitch_ts, pitch_deg,
            lw=1.0, color="#16a085", alpha=0.9, zorder=2,
            label=f"Pitch (IMU)  std = {pitch_deg.std():.2f}°",
        )
        ax_pitch.axhline(0.0, color="#555555", lw=0.8, ls=":", alpha=0.6)
    else:
        ax_pitch.text(0.5, 0.5, "No /imu/data in bag",
                      ha="center", va="center", transform=ax_pitch.transAxes,
                      color="#666666", fontsize=10)
    ax_pitch.set_ylabel("Pitch (°)", color="#333333", fontsize=10)
    ax_pitch.set_xlabel("Time since bag start (s)", color="#333333", fontsize=10)
    ax_pitch.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333",
                    loc="upper right")

    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
