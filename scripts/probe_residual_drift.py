"""After applying the best constant rotation to /gps/filtered/global vs SBL,
analyse what the *residual* error looks like — does it grow linearly with
distance (constant bias), with time (time drift), in jumps (DVL unlock events),
or correlate with maneuvers (yaw rate)?

Helps distinguish whether the residual drift is at the DVL noise floor
(sensor-spec limit, ~0.1%) or whether something larger is going on (a
time-varying yaw bias, DVL covariance overconfidence, etc.) that an offline
EKF replay could fix.

Usage: python scripts/probe_residual_drift.py <bag_dir>
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ekf_sbl_overlay import (
    read_bag, gate_sbl, navsatfix_to_track, align_to_anchor, pair_errors,
)
from mcap_ros2.reader import read_ros2_messages


def rotate(E, N, anchor_E, anchor_N, angle_deg):
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    dE = E - anchor_E
    dN = N - anchor_N
    return anchor_E + c * dE - s * dN, anchor_N + s * dE + c * dN


def best_rotation(ekf_track, sbl_track, E_aligned, N_aligned):
    anchor_E = float(sbl_track.E[0])
    anchor_N = float(sbl_track.N_utm[0])
    best = None
    for ang in np.arange(-180.0, 180.0, 1.0):
        cE, cN = rotate(E_aligned, N_aligned, anchor_E, anchor_N, ang)
        es = pair_errors(ekf_track, sbl_track, cE, cN)
        if not es.n_pairs:
            continue
        m = float(np.mean(es.err_m))
        if best is None or m < best[0]:
            best = (m, ang, cE, cN, es)
    return best


def get_yaw_rate(bag_dir):
    """Return arrays t_ns, |yaw_rate| (rad/s) from the IMU."""
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        return None, None
    ts, qs = [], []
    for msg in read_ros2_messages(str(mcap)):
        if msg.channel.topic != "/imu/data":
            continue
        try:
            t = int(msg.ros_msg.header.stamp.sec) * 10**9 + int(msg.ros_msg.header.stamp.nanosec)
            if t == 0: continue
            q = msg.ros_msg.orientation
            ts.append(t)
            qs.append((float(q.x), float(q.y), float(q.z), float(q.w)))
        except AttributeError:
            pass
    if not ts: return None, None
    ts = np.array(ts, dtype=np.int64)
    yaws = np.array([math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
                      for (x, y, z, w) in qs])
    yaws_un = np.unwrap(yaws)
    dt = np.diff(ts) / 1e9
    rate = np.diff(yaws_un) / np.where(dt > 0, dt, 1.0)
    return ts[:-1], np.abs(rate)


def main():
    bag_dir = Path(sys.argv[1])
    print(f"Bag: {bag_dir.name}")
    bag = read_bag(bag_dir, want_local=True)
    sbl_msgs = gate_sbl(bag.sbl, max_sbl_std=5.0)
    sbl_track = navsatfix_to_track(sbl_msgs, "SBL")

    msgs = bag.anchored if bag.anchored else bag.local
    ekf_track = navsatfix_to_track(msgs, "ekf")
    E_aligned, N_aligned, *_ = align_to_anchor(ekf_track, sbl_track)
    es0 = pair_errors(ekf_track, sbl_track, E_aligned, N_aligned)

    print(f"Baseline:  mean={float(np.mean(es0.err_m)):.3f} m  max={float(np.max(es0.err_m)):.3f} m  "
          f"drift={es0.drift_rate_m_per_100m:.2f} m/100m")

    best = best_rotation(ekf_track, sbl_track, E_aligned, N_aligned)
    if best is None:
        print("No rotation found")
        return
    mean_after, ang, cE, cN, es = best
    print(f"Best rotation: {ang:+.1f}deg")
    print(f"After rot:  mean={float(np.mean(es.err_m)):.3f} m  max={float(np.max(es.err_m)):.3f} m  "
          f"drift={es.drift_rate_m_per_100m:.2f} m/100m")

    # --- Residual analysis ---
    # 1) Is residual error growing linearly with distance, time, or in jumps?
    err = es.err_m
    dist = es.dist_m
    t_rel = es.t_rel_s
    n = len(err)
    print(f"\nResidual ({n} pairs):")

    # Bin by distance — drift rate over windows
    if n >= 30 and dist.max() > 0:
        # Slope (m of residual error per m of distance)
        slope, intcpt = np.polyfit(dist, err, 1)
        print(f"  err ~ {slope*100:.2f} m/100m * d + {intcpt:.2f}  (linear fit on residual)")

    # 2) Bin error by time bucket of 60 s; show how mean error evolves
    if n >= 30:
        bin_edges = np.arange(0, t_rel.max() + 60, 60)
        bin_idx = np.digitize(t_rel, bin_edges) - 1
        print(f"\n  Mean error per 60 s window (post-rotation):")
        for b in range(len(bin_edges) - 1):
            mask = bin_idx == b
            if mask.sum() < 3: continue
            wm = float(np.mean(err[mask]))
            wn = mask.sum()
            t0 = bin_edges[b]
            t1 = bin_edges[b+1]
            print(f"    {t0:5.0f}-{t1:5.0f} s  n={wn:3d}  mean={wm:6.2f} m")

    # 3) Maneuver correlation: yaw rate vs error
    yaw_t, yaw_rate = get_yaw_rate(bag_dir)
    if yaw_t is not None:
        # For each pair (t_rel, err), find nearest yaw rate sample
        sbl_t0 = int(sbl_track.t_ns[0])
        pair_t_ns = (t_rel * 1e9 + sbl_t0).astype(np.int64)
        # Sample the yaw rate at the matching index
        nearest = np.searchsorted(yaw_t, pair_t_ns)
        nearest = np.clip(nearest, 0, len(yaw_t) - 1)
        rates = yaw_rate[nearest]  # rad/s
        rates_deg = np.degrees(rates)

        # Correlate yaw rate magnitude with residual error
        # Bucket: low yaw rate (straight-ish) vs high yaw rate (turning)
        median_rate = float(np.median(rates_deg))
        low = rates_deg < median_rate
        high = ~low
        if low.sum() > 5 and high.sum() > 5:
            print(f"\n  Residual error vs yaw-rate magnitude (median {median_rate:.1f} deg/s):")
            print(f"    LOW  yaw rate:  mean error {float(np.mean(err[low])):.2f} m  (n={low.sum()})")
            print(f"    HIGH yaw rate:  mean error {float(np.mean(err[high])):.2f} m  (n={high.sum()})")
            # Pearson correlation
            if rates_deg.std() > 0 and err.std() > 0:
                corr = float(np.corrcoef(rates_deg, err)[0, 1])
                print(f"    Pearson correlation (rate vs err): {corr:+.3f}")


if __name__ == "__main__":
    main()
