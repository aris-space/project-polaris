#!/usr/bin/env python3
"""
Plot top-mounted ultrasonic distance vs pressure-sensor depth (Zermatt grid survey).

Usage:
    python3 plot_ultrasonic_vs_depth.py [BAG ...] [--out FILE]

    BAG       Path to a rosbag2 directory (MCAP storage) or .mcap file.
    --out     Output PNG path (default: ultrasonic_vs_depth_zermatt.png next to this script).

Reads:
    /top/ultrasonic/distance     (std_msgs/Float32)             — distance to ice ceiling in m
    /sensors/pressure/pose_enu   (geometry_msgs/PoseWithCovarianceStamped) — z_enu in m
    /odometry/filtered/local     (nav_msgs/Odometry)            — roll/pitch from orientation quaternion

Ultrasonic samples reporting exactly 0.0 m are dropped (sensor fault sentinel
used by the touch-detection node).

Archimedes thickness (no touch required, AUV anywhere under the ice). Both
the sensor offset and the ultrasonic beam lie along body-z, so each picks up
a cos(pitch)cos(roll) factor when projected onto world-vertical:

    c                = cos(pitch) * cos(roll)
    depth_ultrasonic = depth_pressure - pressure_to_ultrasonic_z_m * c
    ice_draft        = depth_ultrasonic - d_ultrasonic * c
    T_ice            = ice_draft * rho_water / rho_ice

`pressure_to_ultrasonic_z_m` is the body-z distance from the pressure sensor
up to the ultrasonic sensor (positive when ultrasonic sits above pressure).
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

TOPIC_ULTRA = "/top/ultrasonic/distance"
TOPIC_DEPTH = "/sensors/pressure/pose_enu"
TOPIC_ODOM  = "/odometry/filtered/local"

MAX_STALENESS_S = 1.0   # only keep samples where the other sensor published within this window
MIN_DEPTH_M     = 0.5   # ignore samples when AUV is shallower than this (surface / pre-dive)
MAX_ULTRA_M     = 5.0   # drop clear outliers beyond this distance

# --- Archimedes thickness constants (mirror config.yaml) ---
PRESSURE_TO_ULTRASONIC_Z_M = 0.062   # ultrasonic sits this far ABOVE pressure sensor (body z, m)
RHO_WATER                  = 999.4   # kg/m^3
RHO_ICE                    = 887.5   # kg/m^3

DEFAULT_BAGS = [
    "/ros2_ws/recordings/zermatt_grid_01_2026_04_30-12_04_16",
    "/ros2_ws/recordings/zermatt_grid_02_2026_04_30-13_00_44",
]


def roll_pitch_from_quat(qx, qy, qz, qw):
    """Tait-Bryan roll, pitch (radians) from a unit quaternion (body -> world, ZYX)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def extract_data(bag_paths):
    """Collect ultrasonic, pressure-depth, and odom roll/pitch samples across bags."""
    topics = [TOPIC_ULTRA, TOPIC_DEPTH, TOPIC_ODOM]

    ultra_ts, ultra_dist = [], []
    depth_ts, depth_m = [], []
    odom_ts, odom_roll, odom_pitch = [], [], []
    t_offset = None
    latest_depth = None

    for bag_path in bag_paths:
        bag_path = Path(bag_path)
        print(f"Reading {bag_path.name}...")
        with AnyReader([bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic in topics]
            available = {c.topic for c in connections}
            if TOPIC_ULTRA not in available:
                print(f"  WARNING: {TOPIC_ULTRA} not found in {bag_path.name}", file=sys.stderr)
            if TOPIC_DEPTH not in available:
                print(f"  WARNING: {TOPIC_DEPTH} not found in {bag_path.name}", file=sys.stderr)
            if TOPIC_ODOM not in available:
                print(f"  WARNING: {TOPIC_ODOM} not found in {bag_path.name}", file=sys.stderr)

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

                elif conn.topic == TOPIC_ULTRA:
                    d_m = float(msg.data)
                    if d_m == 0.0:
                        continue  # sensor-fault sentinel used by touch-detection node
                    if d_m > MAX_ULTRA_M:
                        continue
                    if latest_depth is None or latest_depth < MIN_DEPTH_M:
                        continue
                    ultra_ts.append(t_rel)
                    ultra_dist.append(d_m)

                elif conn.topic == TOPIC_ODOM:
                    q = msg.pose.pose.orientation
                    norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
                    if norm_sq < 0.5:
                        continue  # quaternion not initialised
                    r, p = roll_pitch_from_quat(q.x, q.y, q.z, q.w)
                    odom_ts.append(t_rel)
                    odom_roll.append(r)
                    odom_pitch.append(p)

    return (
        np.array(ultra_ts), np.array(ultra_dist),
        np.array(depth_ts), np.array(depth_m),
        np.array(odom_ts), np.array(odom_roll), np.array(odom_pitch),
    )


def coactive_mask(ts_query, ts_other):
    """Boolean mask keeping only samples with a neighbour in ts_other within MAX_STALENESS_S."""
    if len(ts_other) == 0:
        return np.zeros(len(ts_query), dtype=bool)
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
                        help="rosbag2 directories or MCAP files. Defaults to the two zermatt_grid bags.")
    parser.add_argument("--out", default=None, metavar="FILE",
                        help="Output PNG path (default: ultrasonic_vs_depth_zermatt.png)")
    args = parser.parse_args()

    bag_paths = args.bags or DEFAULT_BAGS
    out_path = (
        Path(args.out) if args.out
        else Path(__file__).parent / "ultrasonic_vs_depth_zermatt.png"
    )

    (ultra_ts, ultra_dist,
     depth_ts, depth_m,
     odom_ts, odom_roll, odom_pitch) = extract_data(bag_paths)

    if len(ultra_ts) == 0:
        print("No ultrasonic samples found.", file=sys.stderr)
        sys.exit(1)
    if len(depth_ts) == 0:
        print("No pressure depth samples found.", file=sys.stderr)
        sys.exit(1)
    if len(odom_ts) == 0:
        print("WARNING: no odometry samples found — pitch/roll correction skipped.",
              file=sys.stderr)

    # Restrict to co-active windows
    ultra_mask = coactive_mask(ultra_ts, depth_ts)
    depth_mask = coactive_mask(depth_ts, ultra_ts)

    ultra_ts_co   = ultra_ts[ultra_mask]
    ultra_dist_co = ultra_dist[ultra_mask]
    depth_ts_co   = depth_ts[depth_mask]
    depth_m_co    = depth_m[depth_mask]

    print(f"\nAfter co-activity filter (+-{MAX_STALENESS_S}s):")
    print(f"  Ultrasonic distance: {len(ultra_ts_co):,} / {len(ultra_ts):,} samples  "
          f"mean={ultra_dist_co.mean():.3f} m  std={ultra_dist_co.std():.3f} m  "
          f"range=[{ultra_dist_co.min():.3f}, {ultra_dist_co.max():.3f}] m")
    print(f"  Pressure depth:      {len(depth_ts_co):,} / {len(depth_ts):,} samples  "
          f"mean={depth_m_co.mean():.3f} m  std={depth_m_co.std():.3f} m  "
          f"range=[{depth_m_co.min():.3f}, {depth_m_co.max():.3f}] m")

    # Archimedes ice thickness with pitch/roll correction:
    #   c                = cos(pitch) * cos(roll)
    #   depth_ultrasonic = depth_pressure - pressure_to_ultrasonic_z_m * c
    #   ice_draft        = depth_ultrasonic - d_ultrasonic * c
    #   T                = ice_draft * rho_water / rho_ice
    depth_at_ultra = np.interp(ultra_ts_co, depth_ts_co, depth_m_co)
    if len(odom_ts) > 0:
        roll_at_ultra  = np.interp(ultra_ts_co, odom_ts, odom_roll)
        pitch_at_ultra = np.interp(ultra_ts_co, odom_ts, odom_pitch)
        c              = np.cos(pitch_at_ultra) * np.cos(roll_at_ultra)
        pitch_deg = np.degrees(pitch_at_ultra)
        roll_deg  = np.degrees(roll_at_ultra)
        print(f"  Odometry roll:       n={len(odom_ts):,}  "
              f"mean={roll_deg.mean():.2f}°  std={roll_deg.std():.2f}°  "
              f"range=[{roll_deg.min():.2f}, {roll_deg.max():.2f}]°")
        print(f"  Odometry pitch:      n={len(odom_ts):,}  "
              f"mean={pitch_deg.mean():.2f}°  std={pitch_deg.std():.2f}°  "
              f"range=[{pitch_deg.min():.2f}, {pitch_deg.max():.2f}]°")
        print(f"  cos(pitch)*cos(roll) at ultrasonic samples:  "
              f"mean={c.mean():.4f}  min={c.min():.4f}  max={c.max():.4f}")
    else:
        c = np.ones_like(ultra_dist_co)

    ice_draft     = depth_at_ultra - (PRESSURE_TO_ULTRASONIC_Z_M + ultra_dist_co) * c
    ice_thickness = ice_draft * (RHO_WATER / RHO_ICE)

    draft_median = float(np.median(ice_draft))
    draft_std    = float(ice_draft.std())
    T_median     = float(np.median(ice_thickness))
    T_std        = float(ice_thickness.std())

    print(f"\n  Ice draft  (depth - ({PRESSURE_TO_ULTRASONIC_Z_M:.3f} m + ultrasonic) * cos(p)cos(r)):")
    print(f"    median={draft_median:.3f} m  std={draft_std:.3f} m  "
          f"range=[{ice_draft.min():.3f}, {ice_draft.max():.3f}] m")
    print(f"  Archimedes thickness  (draft * {RHO_WATER:.1f} / {RHO_ICE:.1f}):")
    print(f"    median={T_median:.3f} m  std={T_std:.3f} m  "
          f"range=[{ice_thickness.min():.3f}, {ice_thickness.max():.3f}] m")

    # -- Figure ----------------------------------------------------------------
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

    # -- Top: depth + ultrasonic overlaid -------------------------------------
    ax_main.plot(
        depth_ts_co, depth_m_co,
        lw=1.2, color="#2c3e50", alpha=0.9, zorder=2,
        label=f"Pressure sensor depth  (n = {len(depth_ts_co):,})",
    )
    ax_main.scatter(
        ultra_ts_co, ultra_dist_co,
        s=4, color="#e67e22", alpha=0.6, zorder=3, rasterized=True,
        label=f"Top ultrasonic distance to ice  (n = {len(ultra_ts_co):,})",
    )
    ax_main.set_ylabel("Distance / Depth (m)", color="#333333", fontsize=10)
    ax_main.set_title(
        "Top Ultrasonic Distance vs Pressure Sensor Depth — Zermatt grid\n"
        "Archimedes thickness  =  (depth − sensor_offset − ultrasonic) · ρ_water / ρ_ice",
        color="#111111", fontsize=11, pad=8,
    )
    ax_main.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333",
                   loc="upper right")
    all_vals = np.concatenate([depth_m_co, ultra_dist_co])
    margin = 0.2
    ax_main.set_ylim(all_vals.max() + margin, max(0.0, all_vals.min() - margin))

    # -- Bottom: Archimedes ice thickness -------------------------------------
    ax_sum.scatter(
        ultra_ts_co, ice_thickness,
        s=4, color="#8e44ad", alpha=0.6, zorder=2, rasterized=True,
        label=(f"Archimedes thickness   draft median = {draft_median:.3f} m   "
               f"std = {T_std:.3f} m"),
    )
    ax_sum.axhline(T_median, color="#555555", lw=1.2, ls="--", alpha=0.8,
                   label=f"Median T = {T_median:.3f} m   "
                         f"(offset = {PRESSURE_TO_ULTRASONIC_Z_M:.3f} m, "
                         f"ρ_w/ρ_i = {RHO_WATER:.1f}/{RHO_ICE:.1f})")
    ax_sum.set_ylabel("Ice thickness (m)", color="#333333", fontsize=10)
    ax_sum.set_xlabel("Time since bag start (s)", color="#333333", fontsize=10)
    ax_sum.legend(fontsize=9, facecolor="white", edgecolor="#cccccc", labelcolor="#333333")

    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
