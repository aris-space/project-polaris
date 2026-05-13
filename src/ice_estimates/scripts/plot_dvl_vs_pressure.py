#!/usr/bin/env python3
"""
Compare DVL altitude vs pressure sensor depth (St. Moritz / Zermatt ice survey).

Usage:
    python3 plot_dvl_vs_pressure.py [BAG ...] [--out FILE]

    BAG       Path to a rosbag2 MCAP file.
    --out     Output PNG path (default: dvl_vs_pressure.png next to this script).

Reads:
    /sensors/dvl/velocity      (.altitude, metres)
    /sensors/pressure/pose_enu (geometry_msgs/PoseWithCovarianceStamped, z_enu in m)

Only samples where pressure depth >= 0.5 m are kept.

If both sensors are reliable, dvl_altitude + pressure_depth equals the total
water-column depth — a constant. Compare the sum std against the ping sonar
equivalent to show the pressure sensor is the more reliable reference.
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
TOPIC_DEPTH = "/sensors/pressure/pose_enu"

MAX_STALENESS_S = 1.0  # only keep samples where the other sensor published within this window
MIN_DEPTH_M     = 0.5  # ignore samples when AUV is shallower than this

DEFAULT_BAGS = [
    "/ros2_ws/recordings/zermatt_rectangle_06_2026_04_29-13_22_13_0.mcap",
]


def extract_data(bag_paths):
    """Collect all DVL and pressure depth samples, filtering depth < MIN_DEPTH_M."""
    topics = [TOPIC_DVL, TOPIC_DEPTH]

    dvl_ts, dvl_alt = [], []
    depth_ts, depth_m = [], []
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
            if TOPIC_DEPTH not in available:
                print(f"  WARNING: {TOPIC_DEPTH} not found in {bag_path.name}", file=sys.stderr)

            for conn, t_ns, raw in reader.messages(connections=connections):
                t_s = t_ns * 1e-9
                if t_offset is None:
                    t_offset = t_s
                t_rel = t_s - t_offset

                msg = reader.deserialize(raw, conn.msgtype)

                if conn.topic == TOPIC_DEPTH:
                    d = -float(msg.pose.pose.position.z)  # ENU z -> depth positive down
                    latest_depth = d
                    if d >= MIN_DEPTH_M:
                        depth_ts.append(t_rel)
                        depth_m.append(d)

                elif conn.topic == TOPIC_DVL:
                    if latest_depth is None or latest_depth < MIN_DEPTH_M:
                        continue
                    if not bool(msg.beam_velocities_valid):
                        continue
                    alt = float(msg.altitude)
                    if alt < 2.0:
                        continue
                    dvl_ts.append(t_rel)
                    dvl_alt.append(alt)

    return (
        np.array(dvl_ts), np.array(dvl_alt),
        np.array(depth_ts), np.array(depth_m),
    )


def coactive_mask(ts_query, ts_other):
    """Boolean mask keeping only samples with a neighbour in ts_other within MAX_STALENESS_S."""
    idx = np.searchsorted(ts_other, ts_query).clip(0, len(ts_other) - 1)
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
    parser.add_argument("bags", nargs="*", metavar="BAG",
                        help="MCAP rosbag2 files. Defaults to zermatt_rectangle_06.")
    parser.add_argument("--out", default=None, metavar="FILE",
                        help="Output PNG path (default: dvl_vs_pressure.png)")
    args = parser.parse_args()

    bag_paths = args.bags or DEFAULT_BAGS
    out_path = (
        Path(args.out) if args.out
        else Path(__file__).parent / "dvl_vs_pressure.png"
    )

    dvl_ts, dvl_alt, depth_ts, depth_m = extract_data(bag_paths)

    if len(dvl_ts) == 0:
        print("No DVL data found.", file=sys.stderr)
        sys.exit(1)
    if len(depth_ts) == 0:
        print("No pressure depth data found.", file=sys.stderr)
        sys.exit(1)

    # Restrict to co-active windows
    dvl_mask   = coactive_mask(dvl_ts, depth_ts)
    depth_mask = coactive_mask(depth_ts, dvl_ts)

    dvl_ts_co    = dvl_ts[dvl_mask]
    dvl_alt_co   = dvl_alt[dvl_mask]
    depth_ts_co  = depth_ts[depth_mask]
    depth_m_co   = depth_m[depth_mask]

    print(f"\nAfter co-activity filter (+-{MAX_STALENESS_S}s):")
    print(f"  DVL altitude:     {len(dvl_ts_co):,} / {len(dvl_ts):,} samples  "
          f"mean={dvl_alt_co.mean():.3f} m  std={dvl_alt_co.std():.3f} m")
    print(f"  Pressure depth:   {len(depth_ts_co):,} / {len(depth_ts):,} samples  "
          f"mean={depth_m_co.mean():.3f} m  std={depth_m_co.std():.3f} m")

    # Sum at DVL timestamps (interpolate pressure depth onto DVL timestamps)
    depth_at_dvl = np.interp(dvl_ts_co, depth_ts_co, depth_m_co)
    total = dvl_alt_co + depth_at_dvl
    total_median = np.median(total)
    total_std = total.std()

    print(f"\n  DVL + pressure sum:  median={total_median:.3f} m  std={total_std:.3f} m  "
          f"range=[{total.min():.3f}, {total.max():.3f}] m")
    print(f"  (flat line expected if both reliable -- actual std = {total_std:.3f} m)")

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

    # ── Top: DVL and pressure depth overlaid ──────────────────────────────────
    ax_main.plot(
        dvl_ts_co, dvl_alt_co,
        lw=1.0, color="#2c3e50", alpha=0.9, zorder=2,
        label=f"DVL altitude  (n = {len(dvl_ts_co):,})",
    )
    ax_main.plot(
        depth_ts_co, depth_m_co,
        lw=1.0, color="#27ae60", alpha=0.9, zorder=2,
        label=f"Pressure depth  (n = {len(depth_ts_co):,})",
    )
    ax_main.set_ylabel("Distance / Depth (m)", color="#333333", fontsize=10)
    ax_main.set_title(
        "DVL Altitude vs Pressure Sensor Depth — Zermatt\n"
        "If both sensors are reliable, their sum = total water-column depth (constant)",
        color="#111111", fontsize=11, pad=8,
    )
    ax_main.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333",
                   loc="upper right")

    # ── Bottom: sum (should be flat) ──────────────────────────────────────────
    ax_sum.plot(
        dvl_ts_co, total,
        lw=0.8, color="#8e44ad", alpha=0.85, zorder=2,
        label=f"DVL + pressure depth  (std = {total_std:.3f} m)",
    )
    ax_sum.axhline(total_median, color="#555555", lw=1.2, ls="--", alpha=0.8,
                   label=f"Median = {total_median:.3f} m")
    ax_sum.set_ylabel("DVL + depth (m)", color="#333333", fontsize=10)
    ax_sum.set_xlabel("Time since bag start (s)", color="#333333", fontsize=10)
    ax_sum.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333")

    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
