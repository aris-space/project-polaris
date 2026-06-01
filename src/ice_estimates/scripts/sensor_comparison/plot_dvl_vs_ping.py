#!/usr/bin/env python3
"""
Compare DVL altitude vs ping sonar distance (St. Moritz ice survey).

Usage:
    python3 plot_dvl_vs_ping.py [BAG ...] [--out FILE]

    BAG       Path to a rosbag2 MCAP file.
    --out     Output PNG path (default: dvl_vs_ping.png next to this script).

Reads:
    /sensors/dvl/velocity      (.altitude, metres)
    /ping_sonar/distance       (.distance in mm, .confidence 0-100)

Only ping sonar samples with confidence == 100 are kept. Both sensors are then
restricted to time windows where they are both actively publishing (nearest
sample from the other sensor within MAX_STALENESS_S).

If both sensors are reliable, dvl_altitude + ping_distance equals the total
water-column depth — a constant. The sum panel shows how far from constant it is.
"""

import argparse
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

TOPIC_DVL   = "/sensors/dvl/velocity"
TOPIC_PING  = "/ping_sonar/distance"
TOPIC_DEPTH = "/sensors/pressure/pose_enu"

MAX_PING_M      = 10.0  # drop clear outliers beyond this distance
MAX_STALENESS_S = 1.0   # only keep samples where the other sensor published within this window
MIN_DEPTH_M     = 0.5   # ignore samples when AUV is shallower than this (ping deadzone)


def extract_data(bag_paths):
    """Collect all DVL and ping (conf=100) samples across bags."""
    topics = [TOPIC_DVL, TOPIC_PING, TOPIC_DEPTH]

    dvl_ts, dvl_alt = [], []
    ping_ts, ping_dist = [], []
    t_offset = None
    latest_depth = None

    for bag_path in bag_paths:
        bag_path = Path(bag_path)
        print(f"Reading {bag_path.name}...")
        with AnyReader([bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic in topics]
            available = {c.topic for c in connections}
            if TOPIC_DVL not in available:
                print(f"  WARNING: {TOPIC_DVL} not found in {bag_path.name}", file=sys.stderr)
            if TOPIC_PING not in available:
                print(f"  WARNING: {TOPIC_PING} not found in {bag_path.name}", file=sys.stderr)
            if TOPIC_DEPTH not in available:
                print(f"  WARNING: {TOPIC_DEPTH} not found in {bag_path.name}", file=sys.stderr)

            for conn, t_ns, raw in reader.messages(connections=connections):
                t_s = t_ns * 1e-9
                if t_offset is None:
                    t_offset = t_s
                t_rel = t_s - t_offset

                msg = reader.deserialize(raw, conn.msgtype)

                if conn.topic == TOPIC_DEPTH:
                    latest_depth = -float(msg.pose.pose.position.z)  # ENU z → depth positive down

                elif conn.topic == TOPIC_DVL:
                    if latest_depth is None or latest_depth < MIN_DEPTH_M:
                        continue
                    if not bool(msg.beam_velocities_valid):
                        continue
                    dvl_ts.append(t_rel)
                    dvl_alt.append(float(msg.altitude))

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

    return (
        np.array(dvl_ts), np.array(dvl_alt),
        np.array(ping_ts), np.array(ping_dist),
    )


def coactive_mask(ts_query, ts_other):
    """
    Return a boolean mask for ts_query keeping only samples where the nearest
    timestamp in ts_other is within MAX_STALENESS_S.
    """
    idx = np.searchsorted(ts_other, ts_query).clip(0, len(ts_other) - 1)
    # Also check the preceding sample
    idx_prev = (idx - 1).clip(0, len(ts_other) - 1)
    nearest_dist = np.minimum(
        np.abs(ts_query - ts_other[idx]),
        np.abs(ts_query - ts_other[idx_prev]),
    )
    return nearest_dist <= MAX_STALENESS_S


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "bags", nargs="*", metavar="BAG",
        help="MCAP rosbag2 files. Defaults to archimedes bags.",
    )
    parser.add_argument(
        "--out", default=None, metavar="FILE",
        help="Output PNG path (default: dvl_vs_ping.png)",
    )
    parser.add_argument(
        "--location", default="St. Moritz", metavar="NAME",
        help="Location name shown in the plot title (default: 'St. Moritz')",
    )
    args = parser.parse_args()

    bag_paths = args.bags or [
        "/ros2_ws/recordings/archimedes_01_2026_04_01-13_31_25_0 (1).mcap",
        "/ros2_ws/recordings/archimedes_02_2026_04_01-13_35_15_0.mcap",
        "/ros2_ws/recordings/archimedes_03_2026_04_01-13_42_14_0.mcap",
    ]
    out_path = (
        Path(args.out) if args.out
        else Path(__file__).parent / "dvl_vs_ping.png"
    )

    dvl_ts, dvl_alt, ping_ts, ping_dist = extract_data(bag_paths)

    if len(dvl_ts) == 0:
        print("No DVL data found.", file=sys.stderr)
        sys.exit(1)
    if len(ping_ts) == 0:
        print("No ping sonar data with confidence = 100 found.", file=sys.stderr)
        sys.exit(1)

    # Restrict to co-active windows
    ping_mask = coactive_mask(ping_ts, dvl_ts)
    dvl_mask  = coactive_mask(dvl_ts, ping_ts)

    ping_ts_co   = ping_ts[ping_mask]
    ping_dist_co = ping_dist[ping_mask]
    dvl_ts_co    = dvl_ts[dvl_mask]
    dvl_alt_co   = dvl_alt[dvl_mask]

    print(f"\nAfter co-activity filter (±{MAX_STALENESS_S}s):")
    print(f"  DVL altitude:          {len(dvl_ts_co):,} / {len(dvl_ts):,} samples  "
          f"mean={dvl_alt_co.mean():.3f} m  std={dvl_alt_co.std():.3f} m")
    print(f"  Ping sonar (conf=100): {len(ping_ts_co):,} / {len(ping_ts):,} samples  "
          f"mean={ping_dist_co.mean():.3f} m  std={ping_dist_co.std():.3f} m")

    # Sum at ping timestamps (interpolate co-active DVL)
    dvl_at_ping = np.interp(ping_ts_co, dvl_ts_co, dvl_alt_co)
    total = dvl_at_ping + ping_dist_co
    total_median = np.median(total)
    total_std = total.std()

    print(f"\n  DVL + ping sum:        median={total_median:.3f} m  std={total_std:.3f} m  "
          f"range=[{total.min():.3f}, {total.max():.3f}] m")
    print(f"  (flat line expected if both reliable — actual std = {total_std:.3f} m)")

    # ── Figure ────────────────────────────────────────────────────────────────
    fig, (ax_main, ax_sum) = plt.subplots(
        2, 1, figsize=(14, 9), sharex=True,
        gridspec_kw={"height_ratios": [2, 1], "hspace": 0.06},
        constrained_layout=True,
    )
    fig.patch.set_facecolor("white")

    def style(ax):
        ax.set_facecolor("#f7f9fc")
        ax.spines[:].set_color("#cccccc")
        ax.tick_params(colors="#333333", labelsize=9)
        ax.grid(True, color="#dddddd", linewidth=0.5, zorder=0)

    style(ax_main)
    style(ax_sum)

    # ── Top: DVL and ping overlaid ────────────────────────────────────────────
    ax_main.plot(
        dvl_ts_co, dvl_alt_co,
        lw=1.2, color="#2c3e50", alpha=0.9, zorder=2,
        label=f"DVL altitude  (n = {len(dvl_ts_co):,})",
    )
    ax_main.scatter(
        ping_ts_co, ping_dist_co,
        s=4, color="#e74c3c", alpha=0.6, zorder=3, rasterized=True,
        label=f"Ping sonar distance — confidence = 100  (n = {len(ping_ts_co):,})",
    )
    ax_main.set_ylabel("Distance (m)", color="#333333", fontsize=10)
    ax_main.set_title(
        f"DVL Altitude vs Ping Sonar Distance — {args.location}\n"
        "If both sensors are reliable, their sum = total water-column depth (constant)",
        color="#111111", fontsize=11, pad=8,
    )
    ax_main.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333",
                   loc="upper right")

    # ── Bottom: sum (should be flat) ──────────────────────────────────────────
    ax_sum.scatter(
        ping_ts_co, total,
        s=4, color="#8e44ad", alpha=0.6, zorder=2, rasterized=True,
        label=f"DVL + ping sonar  (std = {total_std:.3f} m)",
    )
    ax_sum.axhline(total_median, color="#555555", lw=1.2, ls="--", alpha=0.8,
                   label=f"Median = {total_median:.3f} m")
    ax_sum.set_ylabel("DVL + ping (m)", color="#333333", fontsize=10)
    ax_sum.set_xlabel("Time since bag start (s)", color="#333333", fontsize=10)
    ax_sum.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333")

    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
