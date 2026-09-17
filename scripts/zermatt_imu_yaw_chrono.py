"""Read each Zermatt bag and compute IMU-yaw stability + mean speed by segment.

Detects: per-bag yaw drift, yaw rms, big-acceleration epochs (which would
suggest external disturbance / collision), and surface-vs-submerged time
fraction (depth z range).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from mcap_ros2.reader import read_ros2_messages

ROOT = Path("recordings/rosbags")
BAGS = [
    ("rect_01", ROOT / "2026-04-29" / "zermatt_rectangle_01_2026_04_29-12_17_06"),
    ("rect_02", ROOT / "2026-04-29" / "zermatt_rectangle_02_2026_04_29-13_00_00"),
    ("rect_04", ROOT / "2026-04-29" / "zermatt_rectangle_04_2026_04_29-13_00_20"),
    ("rect_06", ROOT / "2026-04-29" / "zermatt_rectangle_06_2026_04_29-13_22_13"),
    ("rect_07", ROOT / "2026-04-29" / "zermatt_rectangle_07_2026_04_29-13_38_44"),
    ("rect_08", ROOT / "2026-04-29" / "zermatt_rectangle_08_2026_04_29-13_48_42"),
    ("grid_01", ROOT / "2026-04-30" / "zermatt_grid_01_2026_04_30-12_04_16"),
    ("grid_02", ROOT / "2026-04-30" / "zermatt_grid_02_2026_04_30-13_00_44"),
]

T_IMU = "/imu/data"
T_PRESSURE = "/sensors/pressure/pose_enu"
T_DVL_VEL = "/sensors/dvl/velocity"


def quat_to_yaw_deg(qx, qy, qz, qw):
    return math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                                    1.0 - 2.0 * (qy * qy + qz * qz)))


def stamp_ns(s):
    return int(s.sec) * 1_000_000_000 + int(s.nanosec)


def analyze(bag_dir: Path):
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        return None
    yaw = []
    yaw_t = []
    accel_norm = []
    z = []
    z_t = []
    dvl_alts = []
    dvl_lock = []
    dvl_t = []
    n_msgs = 0
    for msg in read_ros2_messages(str(mcap)):
        topic = msg.channel.topic
        if topic == T_IMU:
            try:
                t = stamp_ns(msg.ros_msg.header.stamp)
                if t == 0: continue
                q = msg.ros_msg.orientation
                yaw.append(quat_to_yaw_deg(float(q.x), float(q.y), float(q.z), float(q.w)))
                yaw_t.append(t)
                a = msg.ros_msg.linear_acceleration
                accel_norm.append(math.sqrt(float(a.x)**2 + float(a.y)**2 + float(a.z)**2))
                n_msgs += 1
            except AttributeError:
                pass
        elif topic == T_PRESSURE:
            try:
                t = stamp_ns(msg.ros_msg.header.stamp)
                if t > 0:
                    z.append(float(msg.ros_msg.pose.pose.position.z))
                    z_t.append(t)
            except AttributeError:
                pass
        elif topic == T_DVL_VEL:
            try:
                t = stamp_ns(msg.ros_msg.header.stamp)
                if t > 0:
                    dvl_t.append(t)
                    dvl_alts.append(float(msg.ros_msg.altitude))
                    dvl_lock.append(bool(msg.ros_msg.beam_velocities_valid))
            except AttributeError:
                pass
    if not yaw:
        return None
    yaw = np.array(yaw)
    yaw_t = np.array(yaw_t, dtype=np.int64)
    duration = (yaw_t[-1] - yaw_t[0]) / 1e9

    # Unwrap yaw to detect drift. Use first 5s as reference.
    yaw_unwrapped = np.unwrap(np.radians(yaw))
    yaw_deg_unwrapped = np.degrees(yaw_unwrapped)
    t_rel = (yaw_t - yaw_t[0]) / 1e9

    # Yaw drift: fit linear over the WHOLE bag (only meaningful if AUV is rotating little)
    # Better: take the difference between mean(first_5s) and mean(last_5s) as drift
    early = yaw_deg_unwrapped[t_rel < 5.0]
    late = yaw_deg_unwrapped[t_rel > t_rel[-1] - 5.0]
    drift = None
    if len(early) >= 5 and len(late) >= 5:
        drift = float(np.mean(late) - np.mean(early))

    # Average yaw rate (stability metric for non-rotating segments)
    yaw_rate = np.diff(yaw_unwrapped) / (np.diff(yaw_t) / 1e9)
    yaw_rate_p95 = float(np.percentile(np.abs(yaw_rate), 95))
    yaw_rate_median = float(np.median(np.abs(yaw_rate)))

    # Acceleration anomalies
    a = np.array(accel_norm)
    a_p95 = float(np.percentile(a, 95))
    a_max = float(np.max(a))

    # Depth (z) — relative to first sample
    z = np.array(z) if z else np.array([0.0])
    z_min = float(z.min()) if len(z) else None
    z_max = float(z.max()) if len(z) else None
    # surface time = z above some threshold
    z_t_arr = np.array(z_t, dtype=np.int64) if z_t else None
    surface_frac = None
    if z_t_arr is not None and len(z_t_arr) > 1:
        # "near surface" if z > -0.5 m (z is ENU; surface is z=0)
        near_surface = z > -0.5
        # time-weighted
        dt = np.diff(z_t_arr) / 1e9
        ns = near_surface[:-1]
        if dt.sum() > 0:
            surface_frac = float((dt * ns).sum() / dt.sum())

    # DVL altitude when locked
    dvl_alt_locked = None
    if dvl_alts:
        arr_alt = np.array(dvl_alts)
        arr_lock = np.array(dvl_lock)
        if arr_lock.any():
            dvl_alt_locked = float(np.median(arr_alt[arr_lock]))

    return {
        "duration_s": duration,
        "n_imu": n_msgs,
        "yaw_drift_deg_total": drift,
        "yaw_drift_deg_per_min": (drift / (duration / 60.0)) if drift is not None and duration > 0 else None,
        "yaw_rate_med_deg_s": math.degrees(yaw_rate_median),
        "yaw_rate_p95_deg_s": math.degrees(yaw_rate_p95),
        "accel_p95_m_s2": a_p95,
        "accel_max_m_s2": a_max,
        "z_min_m": z_min,
        "z_max_m": z_max,
        "surface_fraction": surface_frac,
        "dvl_altitude_locked_med_m": dvl_alt_locked,
    }


print(f"{'bag':10s} {'dur':>6s} {'imu_n':>7s} {'yaw_drift':>10s} {'yaw_drift/min':>14s} "
      f"{'yaw_rate_med':>13s} {'yaw_rate_p95':>13s} {'accel_p95':>10s} {'accel_max':>10s} "
      f"{'z_min':>6s} {'z_max':>6s} {'surf%':>6s} {'dvl_alt':>8s}")
for tag, p in BAGS:
    if not p.exists():
        print(f"{tag:10s}  MISSING {p}")
        continue
    print(f"{tag:10s} processing...", flush=True)
    r = analyze(p)
    if r is None:
        print(f"{tag:10s}  no IMU data")
        continue

    def f(v, dec=2):
        return f"{v:.{dec}f}" if v is not None else "-"

    print(f"{tag:10s} {f(r['duration_s'],0):>6s} "
          f"{r['n_imu']:>7d} "
          f"{f(r['yaw_drift_deg_total'],2):>10s} "
          f"{f(r['yaw_drift_deg_per_min'],3):>14s} "
          f"{f(r['yaw_rate_med_deg_s'],3):>13s} "
          f"{f(r['yaw_rate_p95_deg_s'],3):>13s} "
          f"{f(r['accel_p95_m_s2'],2):>10s} "
          f"{f(r['accel_max_m_s2'],2):>10s} "
          f"{f(r['z_min_m'],2):>6s} "
          f"{f(r['z_max_m'],2):>6s} "
          f"{f(r['surface_fraction']*100 if r['surface_fraction'] else None,1):>6s} "
          f"{f(r['dvl_altitude_locked_med_m'],2):>8s}")
