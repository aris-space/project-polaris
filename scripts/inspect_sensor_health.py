"""
Audit sensor data health on a bag, to test the two remaining candidate
mechanisms behind the residual global-EKF divergence:

  1. DVL bottom-lock loss.
     Reads /sensors/dvl/odometry_cov and reports the time-series of
     velocity-twist covariance (var(vx), var(vy), var(vz)).  The
     odometry_covariance_node sets these to `no_lock_variance` (default
     1.0e6 m²/s²) whenever DVL bottom-lock is lost or the velocity is
     stale.  If the bag shows even brief jumps to 1e6, that's a real
     contributor to global-EKF divergence: with DVL effectively disabled
     the EKF's velocity state is unconstrained and position drifts.

  2. IMU yaw drift over the bag.
     Reads /imu/data and reports the yaw at start / end and the inferred
     drift rate.  In VRU mode the IMU is gyro-only; bias-driven yaw
     drift of ~0.001 rad/s is normal but ~0.01 rad/s is huge and would
     compound to many radians of error over a 20-min bag, swamping any
     EKF correction.

Usage:
    python3 scripts/inspect_sensor_health.py <bag_dir>

The bag is read via the `rosbags` library (no ROS2 install required), so
this works on the Windows host. Same as plot_global_ekf_divergence.py.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


_TOPIC_DVL = "/sensors/dvl/odometry_cov"
_TOPIC_IMU = "/imu/data"
_TOPIC_GPS_FIX = "/fix"
_TOPIC_GPS_SELECTED = "/gps/selected"


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (rotation about z) from quaternion."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _percentile(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    idx = max(0, min(len(sorted_vals) - 1, int(p * (len(sorted_vals) - 1))))
    return sorted_vals[idx]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument(
        "--no-lock-threshold",
        type=float,
        default=1.0,
        help=(
            "Treat var(vx) >= this as 'DVL no-lock'. Default 1.0 m²/s² "
            "(distinguishes the 1e6 no-lock variance from sub-1.0 normal values)."
        ),
    )
    args = ap.parse_args()

    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    dvl_samples: list[tuple[int, float, float, float]] = []  # t_ns, var_vx, var_vy, var_vz
    imu_samples: list[tuple[int, float]] = []                # t_ns, yaw (rad)
    fix_samples: list[tuple[int, float, float, int]] = []    # t_ns, var_x, var_y, cov_type
    selected_samples: list[tuple[int, float, float, int]] = []  # /gps/selected — same fields

    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        wanted = {_TOPIC_DVL, _TOPIC_IMU, _TOPIC_GPS_FIX, _TOPIC_GPS_SELECTED}
        conns = [c for c in reader.connections if c.topic in wanted]
        for conn, _t, raw in reader.messages(connections=conns):
            ros = reader.deserialize(raw, conn.msgtype)
            try:
                t_ns = _stamp_ns(ros.header.stamp)
            except AttributeError:
                continue
            if conn.topic == _TOPIC_DVL:
                cov = list(ros.twist.covariance)
                # 6x6 row-major: indices 0,7,14 = var(vx), var(vy), var(vz)
                dvl_samples.append((t_ns, float(cov[0]), float(cov[7]), float(cov[14])))
            elif conn.topic == _TOPIC_IMU:
                q = ros.orientation
                imu_samples.append((t_ns, _quat_to_yaw(q.x, q.y, q.z, q.w)))
            elif conn.topic in (_TOPIC_GPS_FIX, _TOPIC_GPS_SELECTED):
                cov = list(ros.position_covariance)
                cov_type = int(getattr(ros, "position_covariance_type", 0))
                sample = (t_ns, float(cov[0]), float(cov[4]), cov_type)
                if conn.topic == _TOPIC_GPS_FIX:
                    fix_samples.append(sample)
                else:
                    selected_samples.append(sample)

    print("=" * 70)
    print(f"Sensor health audit — {bag_dir.name}")
    print("=" * 70)

    # --- DVL ---------------------------------------------------------------
    print(f"\n[1] /sensors/dvl/odometry_cov  ({len(dvl_samples)} messages)\n")
    if not dvl_samples:
        print("    (no DVL messages — skipping)")
    else:
        dvl_samples.sort(key=lambda s: s[0])
        var_vx = sorted(s[1] for s in dvl_samples)
        var_vy = sorted(s[2] for s in dvl_samples)
        var_vz = sorted(s[3] for s in dvl_samples)
        bag_dur_s = (dvl_samples[-1][0] - dvl_samples[0][0]) * 1e-9

        print(f"    bag duration:      {bag_dur_s:.1f} s")
        print(f"    var(vx) min/p50/p95/max:  "
              f"{min(var_vx):.3e} / {_percentile(var_vx, 0.5):.3e} / "
              f"{_percentile(var_vx, 0.95):.3e} / {max(var_vx):.3e}")
        print(f"    var(vy) min/p50/p95/max:  "
              f"{min(var_vy):.3e} / {_percentile(var_vy, 0.5):.3e} / "
              f"{_percentile(var_vy, 0.95):.3e} / {max(var_vy):.3e}")
        print(f"    var(vz) min/p50/p95/max:  "
              f"{min(var_vz):.3e} / {_percentile(var_vz, 0.5):.3e} / "
              f"{_percentile(var_vz, 0.95):.3e} / {max(var_vz):.3e}")

        no_lock = [s for s in dvl_samples if s[1] >= args.no_lock_threshold
                                          or s[2] >= args.no_lock_threshold
                                          or s[3] >= args.no_lock_threshold]
        n_no_lock = len(no_lock)
        pct = 100.0 * n_no_lock / len(dvl_samples)
        print(f"\n    'no-lock' samples (any of var(v*) >= {args.no_lock_threshold}):")
        print(f"      count   : {n_no_lock} of {len(dvl_samples)} ({pct:.1f}%)")
        if no_lock:
            t0 = dvl_samples[0][0]
            first_no_lock_s = (no_lock[0][0] - t0) * 1e-9
            last_no_lock_s = (no_lock[-1][0] - t0) * 1e-9
            print(f"      first at:  t = {first_no_lock_s:.1f} s")
            print(f"      last at:   t = {last_no_lock_s:.1f} s")
            print(f"\n    → DVL bottom-lock IS being lost. The EKF treats DVL as "
                  f"unusable during these windows.")
        else:
            print(f"      none — DVL bottom-lock is solid throughout the bag.")
            print(f"\n    → DVL is not the cause. Velocity input is constrained "
                  f"the whole time.")

    # --- IMU ---------------------------------------------------------------
    print(f"\n[2] /imu/data  ({len(imu_samples)} messages)\n")
    if len(imu_samples) < 2:
        print("    (not enough IMU messages — skipping)")
    else:
        imu_samples.sort(key=lambda s: s[0])
        bag_dur_s = (imu_samples[-1][0] - imu_samples[0][0]) * 1e-9
        yaw_start = imu_samples[0][1]
        yaw_end = imu_samples[-1][1]
        # Unwrap to nearest equivalent of yaw_end relative to yaw_start.
        delta = yaw_end - yaw_start
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi
        rate = delta / bag_dur_s

        print(f"    bag duration:      {bag_dur_s:.1f} s")
        print(f"    yaw at start:      {math.degrees(yaw_start):+.2f}° ({yaw_start:+.4f} rad)")
        print(f"    yaw at end:        {math.degrees(yaw_end):+.2f}° ({yaw_end:+.4f} rad)")
        print(f"    Δyaw (unwrapped):  {math.degrees(delta):+.2f}° ({delta:+.4f} rad)")
        print(f"    drift rate:        {math.degrees(rate):+.4f}°/s ({rate:+.5f} rad/s)")

        # Caveat: this includes both real rotation AND gyro bias drift.
        # For a vehicle that loops back (e.g. a rectangle that closes), Δyaw
        # over the full trajectory should be ~0 — any non-zero is bias.
        print()
        if abs(rate) < 0.0005:
            print(f"    → Drift rate < 0.0005 rad/s — IMU yaw is well-behaved.")
        elif abs(rate) < 0.005:
            print(f"    → Drift rate ~ {math.degrees(rate):.2f}°/s. Within typical "
                  f"VRU gyro spec but accumulates noticeably over long bags.")
        else:
            print(f"    → Drift rate > 0.005 rad/s ({math.degrees(rate):.2f}°/s). "
                  f"Large compared to typical VRU gyro spec; could materially "
                  f"affect EKF heading.")
        print(f"    Caveat: this Δyaw includes both gyro bias AND any real")
        print(f"    rotation across the bag. For a closed-loop trajectory the")
        print(f"    Δyaw should be ~0; any non-zero is bias.")

    # --- GPS ---------------------------------------------------------------
    def _report_gps(label: str, samples: list[tuple[int, float, float, int]]) -> None:
        print(f"\n[GPS] {label}  ({len(samples)} messages)\n")
        if not samples:
            print("    (no messages on this topic — skipping)")
            return
        var_x = sorted(s[1] for s in samples)
        var_y = sorted(s[2] for s in samples)
        sigma_x = sorted(math.sqrt(v) for v in var_x if v > 0)
        sigma_y = sorted(math.sqrt(v) for v in var_y if v > 0)
        print(f"    σ_x (m)  min/p50/p95/max:  "
              f"{min(sigma_x) if sigma_x else float('nan'):.3f} / "
              f"{_percentile(sigma_x, 0.5):.3f} / "
              f"{_percentile(sigma_x, 0.95):.3f} / "
              f"{max(sigma_x) if sigma_x else float('nan'):.3f}")
        print(f"    σ_y (m)  min/p50/p95/max:  "
              f"{min(sigma_y) if sigma_y else float('nan'):.3f} / "
              f"{_percentile(sigma_y, 0.5):.3f} / "
              f"{_percentile(sigma_y, 0.95):.3f} / "
              f"{max(sigma_y) if sigma_y else float('nan'):.3f}")
        # position_covariance_type: 0=UNKNOWN, 1=APPROXIMATED, 2=DIAGONAL_KNOWN, 3=KNOWN.
        cov_types = [s[3] for s in samples]
        type_counts: dict[int, int] = {}
        for ct in cov_types:
            type_counts[ct] = type_counts.get(ct, 0) + 1
        type_labels = {0: "UNKNOWN", 1: "APPROXIMATED", 2: "DIAGONAL_KNOWN", 3: "KNOWN"}
        type_summary = ", ".join(
            f"{type_labels.get(t, f'type{t}')}={c}" for t, c in sorted(type_counts.items())
        )
        print(f"    position_covariance_type counts:  {type_summary}")

    _report_gps("/fix (raw GPS)", fix_samples)
    _report_gps("/gps/selected (curated, EKF input)", selected_samples)
    print()
    print("    → If σ is mostly < 1 m, GPS is RTK-fix-quality. If > 5 m or zero,")
    print("      GPS is degraded and would limit EKF correction power. The")
    print("      position_covariance_type breakdown distinguishes 'covariance is")
    print("      meaningless (UNKNOWN)' from 'covariance is real and that's how")
    print("      bad it is (KNOWN).' If most are UNKNOWN, the high σ values are")
    print("      effectively no information — the GPS receiver just isn't")
    print("      reporting accuracy, not necessarily that accuracy is bad.")


if __name__ == "__main__":
    main()
