#!/usr/bin/env python3
"""
Sanity-check that /imu/data_corrected's yaw is in true-ENU after head_mot
calibration, by comparing it to GNSS course-over-ground (which is true-ENU
by definition: 0° = east-going, 90° = north-going under ROS ENU).

Procedure:
  1. Read /imu/data_corrected quaternions and /ubx_nav_pvt headMotion
     (or /fix-derived course-over-ground from successive fixes).
  2. Filter to samples where the boat is moving (speed > MIN_SPEED), since
     CoG is undefined when stationary.
  3. Compute imu_yaw_enu - course_enu wrap-aligned. The median should be 0
     within a few degrees if the calibration is correct. A non-zero median
     means /imu/data_corrected's yaw is in some other rotational frame.

Usage:
  python scripts/verify_imu_yaw_vs_gnss_course.py <bag_dir>

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

TOPIC_IMU = "/imu/data_corrected"
TOPIC_UBX_PVT = "/ubx_nav_pvt"

MIN_SPEED_MM_S = 200  # 0.2 m/s — UBX gSpeed is in mm/s
MAX_HEAD_ACC_DEG = 5.0  # only trust headMotion when its accuracy is tight


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_to_yaw_enu(x: float, y: float, z: float, w: float) -> float:
    """Yaw in ROS ENU convention (rad)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _ubx_heading_to_enu_rad(head_motion_1e5_deg: int) -> float:
    """
    UBX-NAV-PVT headMotion: int32 in units of 1e-5 deg, where
    convention is heading-of-motion in degrees from true north,
    increasing clockwise (compass convention).
    Convert to ROS ENU yaw: 0° east, +90° north, CCW positive.
    """
    head_deg_compass = head_motion_1e5_deg * 1e-5  # degrees from true north, CW
    head_rad_compass = math.radians(head_deg_compass)
    # ENU yaw = π/2 - compass heading
    yaw_enu = (math.pi / 2.0) - head_rad_compass
    # wrap to [-π, π]
    return math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("bag_dir", help="Path to rosbag2 directory.")
    p.add_argument(
        "--match-tol-s",
        type=float,
        default=0.05,
        help="Stamp-match tolerance in seconds (default 50 ms).",
    )
    args = p.parse_args()

    bag_path = Path(args.bag_dir)
    if not bag_path.exists():
        print(f"bag dir not found: {bag_path}", file=sys.stderr)
        return 2

    imu_samples: list[tuple[float, float]] = []  # (stamp, yaw_enu_rad)
    pvt_samples: list[tuple[float, float, int, int]] = []
    # ^^^ (stamp, course_enu_rad, gSpeed_mm_s, head_acc_1e5_deg)

    with AnyReader([bag_path]) as reader:
        topics = {c.topic for c in reader.connections}
        if TOPIC_IMU not in topics:
            print(f"missing topic: {TOPIC_IMU}", file=sys.stderr)
            return 2
        if TOPIC_UBX_PVT not in topics:
            print(
                f"missing topic: {TOPIC_UBX_PVT} — cannot verify course-of-motion",
                file=sys.stderr,
            )
            return 2

        conns = [c for c in reader.connections if c.topic in (TOPIC_IMU, TOPIC_UBX_PVT)]
        for connection, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.topic == TOPIC_IMU:
                t = _stamp_s(msg.header.stamp)
                q = msg.orientation
                imu_samples.append((t, _quat_to_yaw_enu(q.x, q.y, q.z, q.w)))
            else:  # UBX-NAV-PVT
                t = _stamp_s(msg.header.stamp)
                pvt_samples.append(
                    (
                        t,
                        _ubx_heading_to_enu_rad(int(msg.head_mot)),
                        int(msg.g_speed),
                        int(msg.head_acc),
                    )
                )

    print(f"imu_samples: {len(imu_samples)}  pvt_samples: {len(pvt_samples)}")

    if not pvt_samples:
        print("no UBX PVT samples", file=sys.stderr)
        return 1

    # Diagnostic: distribution of g_speed and head_acc to pick sane gates.
    if pvt_samples:
        speeds = np.array([gs for _, _, gs, _ in pvt_samples])
        head_accs = np.array([ha for _, _, _, ha in pvt_samples])
        print(
            f"g_speed (mm/s) distribution: "
            f"min={speeds.min()} median={np.median(speeds):.0f} "
            f"p95={np.percentile(speeds, 95):.0f} max={speeds.max()}"
        )
        print(
            f"head_acc (1e-5 deg) distribution: "
            f"min={head_accs.min()} median={np.median(head_accs):.0f} "
            f"p95={np.percentile(head_accs, 95):.0f} max={head_accs.max()}"
        )

    # Filter PVT to moving + accurate
    pvt_filtered = [
        (t, c) for t, c, gs, ha in pvt_samples
        if gs >= MIN_SPEED_MM_S and 0 < ha < int(MAX_HEAD_ACC_DEG * 1e5)
    ]
    print(
        f"pvt_filtered (speed >= {MIN_SPEED_MM_S/1000:.2f} m/s, "
        f"head_acc < {MAX_HEAD_ACC_DEG}°): {len(pvt_filtered)}"
    )

    # Fallback: if no samples pass strict gate, relax head_acc since
    # head_mot accuracy is often large on small-aperture RTK devices.
    if not pvt_filtered:
        pvt_filtered = [
            (t, c) for t, c, gs, ha in pvt_samples
            if gs >= MIN_SPEED_MM_S
        ]
        print(
            f"strict gate empty — relaxed to speed-only: "
            f"{len(pvt_filtered)} samples (head_mot accuracy ignored, "
            f"results may be noisier)"
        )

    if not pvt_filtered:
        print(
            "no PVT samples meet movement+accuracy gate — "
            "this bag may be stationary or RTK heading is too noisy.",
            file=sys.stderr,
        )
        return 1

    # Match IMU to PVT by stamp.
    imu_samples.sort(key=lambda x: x[0])
    imu_t = np.array([t for t, _ in imu_samples])
    imu_y = np.array([y for _, y in imu_samples])

    deltas: list[tuple[float, float, float]] = []  # (t, imu_yaw, pvt_course)
    for t_pvt, c_pvt in pvt_filtered:
        idx = np.searchsorted(imu_t, t_pvt)
        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(imu_t):
            candidates.append(idx)
        if not candidates:
            continue
        best = min(candidates, key=lambda k: abs(imu_t[k] - t_pvt))
        if abs(imu_t[best] - t_pvt) > args.match_tol_s:
            continue
        deltas.append((t_pvt, imu_y[best], c_pvt))

    if not deltas:
        print("no time-matched IMU/PVT pairs", file=sys.stderr)
        return 1

    diffs = np.array([_wrap_pi(imu_y - pvt_y) for _, imu_y, pvt_y in deltas])
    diffs_deg = np.degrees(diffs)

    median_deg = float(np.median(diffs_deg))
    p25_deg = float(np.percentile(diffs_deg, 25))
    p75_deg = float(np.percentile(diffs_deg, 75))
    std_deg = float(np.std(diffs_deg))

    print()
    print(f"matched pairs (IMU yaw vs GNSS course-of-motion): {len(deltas)}")
    print(f"median (imu_yaw - course):       {median_deg:+.3f}°")
    print(f"IQR p25..p75:                    {p25_deg:+.3f}°  ..  {p75_deg:+.3f}°")
    print(f"std:                             {std_deg:.3f}°")
    print()

    if abs(median_deg) < 5.0 and std_deg < 15.0:
        print(
            "✓ /imu/data_corrected appears to be in TRUE-ENU within ~5°. "
            "navsat's UTM→odom rotation is approximately identity. "
            "Subtracting local_anchor in gps_odom_cov_floor and relabelling "
            "to map frame is geometrically valid."
        )
    elif abs(median_deg) >= 5.0:
        print(
            f"✗ /imu/data_corrected has a SYSTEMATIC YAW BIAS of "
            f"{median_deg:+.3f}° vs GNSS course-of-motion. "
            f"head_mot calibration is off (or imu_yaw_correction's rotation "
            f"is being applied with the wrong sign / convention). "
            f"navsat's UTM→odom rotation will have this rotational error, "
            f"and just subtracting local_anchor is NOT enough — we'd also "
            f"need to rotate /odometry/gps's position by {-median_deg:+.3f}° "
            f"to put it in true map frame."
        )
    else:
        print(
            f"≈ Median is acceptable but std={std_deg:.1f}° is high — "
            f"yaw is noisy under motion. Likely a magnetometer or vibration "
            f"issue rather than a calibration error."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
