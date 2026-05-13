#!/usr/bin/env python3
"""
Extract and plot water temperature from one or more MCAP bags.

Usage:
    python3 plot_water_temperature.py BAG [BAG ...]  [--out FILE]

    BAG       Path to a rosbag2 directory (MCAP storage).
    --out     Output PNG path (default: water_temperature.png next to this script).

Reads /sensors/keller26x/water_temperature_degc (sensor_msgs/Temperature).
Prints mean, std, min, max across all bags.
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

TOPIC_TEMP = "/sensors/keller26x/water_temperature_degc"
TOPIC_DEPTH = "/sensors/pressure/pose_enu"
# Only keep temperature samples where depth (z, positive = up) is below this threshold.
# Set to None to disable.
MIN_DEPTH_M = -0.5

# Drop this many seconds from the start of each bag (sensor thermal equilibration).
TRIM_START_S = 45.0

# Drop this many samples from the end of each bag to remove end-of-mission spikes.
TRIM_END_SAMPLES = 30

DEFAULT_BAGS = [
    "/ros2_ws/recordings/zermatt_rectangle_01_2026_04_29-12_17_06_0.mcap",
    "/ros2_ws/recordings/zermatt_rectangle_02_2026_04_29-13_00_00_0.mcap",
    "/ros2_ws/recordings/zermatt_rectangle_04_2026_04_29-13_00_20_0.mcap",
    "/ros2_ws/recordings/zermatt_rectangle_06_2026_04_29-13_22_13_0.mcap",
    "/ros2_ws/recordings/zermatt_rectangle_07_2026_04_29-13_38_44_0.mcap",
    "/ros2_ws/recordings/zermatt_rectangle_08_2026_04_29-13_48_42_0.mcap",
]


def extract_temperature(bag_path: Path, min_depth_m=None):
    """Return (timestamps_s, temperatures_degc) arrays from a single bag.

    If min_depth_m is set, only samples where depth z < min_depth_m are kept
    (z is positive-up, so negative values mean submerged).
    """
    topics = [TOPIC_TEMP, TOPIC_DEPTH]

    all_msgs = []
    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        if not any(c.topic == TOPIC_TEMP for c in connections):
            print(
                f"  WARNING: {TOPIC_TEMP} not found in {bag_path.name}", file=sys.stderr
            )
            return np.array([]), np.array([])
        for conn, t_ns, raw in reader.messages(connections=connections):
            all_msgs.append((t_ns, conn.topic, raw, conn.msgtype))

    all_msgs.sort(key=lambda x: x[0])

    timestamps, temps = [], []
    current_depth = None
    current_depth_ts_ns = None
    MAX_DEPTH_AGE_S = 1.0  # reject temp sample if depth reading is older than this

    with AnyReader([bag_path]) as reader:
        for t_ns, topic, raw, msgtype in all_msgs:
            msg = reader.deserialize(raw, msgtype)
            if topic == TOPIC_DEPTH:
                current_depth = msg.pose.pose.position.z
                current_depth_ts_ns = t_ns
            elif topic == TOPIC_TEMP:
                if min_depth_m is not None:
                    depth_age_s = (
                        (t_ns - current_depth_ts_ns) * 1e-9
                        if current_depth_ts_ns is not None
                        else float("inf")
                    )
                    if (
                        current_depth is None
                        or current_depth > min_depth_m
                        or depth_age_s > MAX_DEPTH_AGE_S
                    ):
                        continue
                timestamps.append(t_ns * 1e-9)
                temps.append(msg.temperature)

    return np.array(timestamps), np.array(temps)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "bags",
        nargs="*",
        metavar="BAG",
        help="MCAP files. Defaults to the Zermatt rectangle recordings.",
    )
    parser.add_argument(
        "--out",
        default=None,
        metavar="FILE",
        help="Output PNG path (default: water_temperature.png)",
    )
    args = parser.parse_args()

    bag_paths = [Path(b) for b in (args.bags or DEFAULT_BAGS)]
    out_path = (
        Path(args.out) if args.out else Path(__file__).parent / "water_temperature.png"
    )

    fig, ax = plt.subplots(figsize=(12, 5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#f7f9fc")
    ax.spines[:].set_color("#cccccc")
    ax.tick_params(colors="#333333", labelsize=9)
    ax.grid(True, color="#dddddd", linewidth=0.5, zorder=0)

    all_temps = []
    colors = plt.get_cmap("tab10")

    for i, bag_path in enumerate(bag_paths):
        print(f"Reading {bag_path.name}...")
        ts, temps = extract_temperature(bag_path, min_depth_m=MIN_DEPTH_M)
        if len(temps) == 0:
            continue
        if TRIM_START_S and len(ts):
            mask = ts - ts[0] >= TRIM_START_S
            ts, temps = ts[mask], temps[mask]
        if TRIM_END_SAMPLES and len(ts) > TRIM_END_SAMPLES:
            ts, temps = ts[:-TRIM_END_SAMPLES], temps[:-TRIM_END_SAMPLES]
        if len(temps) == 0:
            continue
        all_temps.append(temps)
        t0 = ts[0]
        ax.plot(
            ts - t0,
            temps,
            lw=0.8,
            alpha=0.75,
            color=colors(i % 10),
            label=bag_path.name[-30:],
        )
        depth_str = f"  depth < {MIN_DEPTH_M} m" if MIN_DEPTH_M is not None else ""
        print(
            f"  {len(temps)} samples{depth_str}  |  mean={temps.mean():.3f}C  "
            f"std={temps.std():.3f}C  min={temps.min():.3f}C  max={temps.max():.3f}C"
        )

    if not all_temps:
        print("No temperature data found.", file=sys.stderr)
        sys.exit(1)

    combined = np.concatenate(all_temps)
    mean = combined.mean()
    std = combined.std()

    ax.axhline(
        mean,
        color="#c0392b",
        lw=1.8,
        ls="--",
        zorder=5,
        label=f"Overall mean = {mean:.3f}°C",
    )
    ax.axhspan(
        mean - std,
        mean + std,
        alpha=0.12,
        color="#c0392b",
        zorder=4,
        label=f"±1σ = {std*1000:.1f} m°C",
    )

    ax.set_xlabel("Time since first valid sample (s)", color="#333333", fontsize=10)
    ax.set_ylabel("Water temperature (°C)", color="#333333", fontsize=10)
    depth_label = f" (depth < {MIN_DEPTH_M} m)" if MIN_DEPTH_M is not None else ""
    ax.set_title(
        f"Water Temperature -- Keller 26x{depth_label}",
        color="#111111",
        fontsize=12,
        pad=8,
    )
    ax.legend(fontsize=8, facecolor="white", edgecolor="#cccccc", labelcolor="#333333")

    print(
        f"\nOverall  mean={mean:.4f}°C  std={std*1000:.2f} m°C  "
        f"n={len(combined):,}  range=[{combined.min():.3f}, {combined.max():.3f}]°C"
    )

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
