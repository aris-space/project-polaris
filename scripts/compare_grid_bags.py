"""Side-by-side comparison of grid_01 and grid_02 residuals after rotation
correction. Tests whether grid_02's larger residual is caused by:

  A) Time-varying yaw bias  (optimal rotation drifts over time)
  B) DVL micro-unlock accumulation  (bursts coincide with unlock clusters)
  C) SBL acoustic noise during error bursts  (error spike = SBL noise spike)
  D) Maneuvering coupling  (bursts correlate with high yaw rate)

Outputs a multi-panel PNG per bag and prints the numerical comparison.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ekf_sbl_overlay import (
    read_bag, gate_sbl, navsatfix_to_track, align_to_anchor, pair_errors,
)
from mcap_ros2.reader import read_ros2_messages


def rotate(E, N, anchor_E, anchor_N, angle_deg):
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    dE, dN = E - anchor_E, N - anchor_N
    return anchor_E + c * dE - s * dN, anchor_N + s * dE + c * dN


def best_rotation(ekf_track, sbl_track, E_aligned, N_aligned,
                  start_t_ns: int | None = None, end_t_ns: int | None = None):
    """Find optimal rotation; optionally restricted to a time window."""
    anchor_E = float(sbl_track.E[0])
    anchor_N = float(sbl_track.N_utm[0])
    sbl_t0 = int(sbl_track.t_ns[0])
    if start_t_ns is None:
        start_t_ns = sbl_t0
    if end_t_ns is None:
        end_t_ns = int(sbl_track.t_ns[-1])

    # Pre-compute SBL-window mask
    sbl_mask = (sbl_track.t_ns >= start_t_ns) & (sbl_track.t_ns <= end_t_ns)
    if sbl_mask.sum() < 5:
        return None

    best = None
    for ang in np.arange(-180.0, 180.0, 1.0):
        cE, cN = rotate(E_aligned, N_aligned, anchor_E, anchor_N, ang)
        es = pair_errors(ekf_track, sbl_track, cE, cN)
        if not es.n_pairs:
            continue
        # Filter pairs to time window
        pair_t_ns = (es.t_rel_s * 1e9 + sbl_t0).astype(np.int64)
        wmask = (pair_t_ns >= start_t_ns) & (pair_t_ns <= end_t_ns)
        if wmask.sum() < 5:
            continue
        m = float(np.mean(es.err_m[wmask]))
        if best is None or m < best[0]:
            best = (m, ang)
    return best


def get_dvl_lock(bag_dir):
    """Return (t_ns, locked_bool) from /sensors/dvl/velocity."""
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        return np.array([], dtype=np.int64), np.array([], dtype=bool)
    ts, lk = [], []
    for msg in read_ros2_messages(str(mcap)):
        if msg.channel.topic != "/sensors/dvl/velocity":
            continue
        try:
            t = int(msg.ros_msg.header.stamp.sec) * 10**9 + int(msg.ros_msg.header.stamp.nanosec)
            if t > 0:
                ts.append(t)
                lk.append(bool(msg.ros_msg.beam_velocities_valid))
        except AttributeError:
            pass
    return np.array(ts, dtype=np.int64), np.array(lk, dtype=bool)


def get_acoustic(bag_dir):
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        return np.array([], dtype=np.int64), np.array([])
    ts, vs = [], []
    for msg in read_ros2_messages(str(mcap)):
        if msg.channel.topic != "/waterlinked_ugps/locator_acoustic_quality":
            continue
        try:
            t = int(msg.ros_msg.header.stamp.sec) * 10**9 + int(msg.ros_msg.header.stamp.nanosec)
            if t > 0:
                ts.append(t)
                vs.append(float(msg.ros_msg.vector.x))
        except AttributeError:
            pass
    return np.array(ts, dtype=np.int64), np.array(vs)


def get_imu_yaw_rate(bag_dir):
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        return np.array([], dtype=np.int64), np.array([])
    ts, qs = [], []
    for msg in read_ros2_messages(str(mcap)):
        if msg.channel.topic != "/imu/data":
            continue
        try:
            t = int(msg.ros_msg.header.stamp.sec) * 10**9 + int(msg.ros_msg.header.stamp.nanosec)
            if t == 0:
                continue
            q = msg.ros_msg.orientation
            ts.append(t)
            qs.append((float(q.x), float(q.y), float(q.z), float(q.w)))
        except AttributeError:
            pass
    if not ts:
        return np.array([], dtype=np.int64), np.array([])
    ts = np.array(ts, dtype=np.int64)
    yaws = np.unwrap([math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
                      for (x, y, z, w) in qs])
    dt = np.diff(ts) / 1e9
    rate = np.diff(yaws) / np.where(dt > 0, dt, 1.0)
    return ts[:-1], np.abs(rate)


def get_imu_yaw_abs(bag_dir):
    """Return absolute IMU yaw angle (radians, unwrapped) over time."""
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        return np.array([], dtype=np.int64), np.array([])
    ts, qs = [], []
    for msg in read_ros2_messages(str(mcap)):
        if msg.channel.topic != "/imu/data":
            continue
        try:
            t = int(msg.ros_msg.header.stamp.sec) * 10**9 + int(msg.ros_msg.header.stamp.nanosec)
            if t == 0:
                continue
            q = msg.ros_msg.orientation
            ts.append(t)
            qs.append((float(q.x), float(q.y), float(q.z), float(q.w)))
        except AttributeError:
            pass
    if not ts:
        return np.array([], dtype=np.int64), np.array([])
    ts = np.array(ts, dtype=np.int64)
    yaws = np.unwrap([math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
                      for (x, y, z, w) in qs])
    return ts, yaws


def analyze(bag_dir, label, out_png):
    print(f"\n========== {label}: {bag_dir.name} ==========")
    bag = read_bag(bag_dir, want_local=True)
    sbl_msgs = gate_sbl(bag.sbl, max_sbl_std=5.0)
    sbl_track = navsatfix_to_track(sbl_msgs, "SBL")
    msgs = bag.anchored if bag.anchored else bag.local
    ekf_track = navsatfix_to_track(msgs, "ekf")
    E_aligned, N_aligned, *_ = align_to_anchor(ekf_track, sbl_track)

    sbl_t0 = int(sbl_track.t_ns[0])
    sbl_te = int(sbl_track.t_ns[-1])
    duration_s = (sbl_te - sbl_t0) / 1e9

    # Best overall rotation
    overall = best_rotation(ekf_track, sbl_track, E_aligned, N_aligned)
    print(f"Best overall rotation: {overall[1]:+.1f}deg, mean err {overall[0]:.3f} m")

    # Best rotation in early half vs late half
    half_t = (sbl_t0 + sbl_te) // 2
    early = best_rotation(ekf_track, sbl_track, E_aligned, N_aligned,
                          start_t_ns=sbl_t0, end_t_ns=half_t)
    late = best_rotation(ekf_track, sbl_track, E_aligned, N_aligned,
                         start_t_ns=half_t, end_t_ns=sbl_te)
    print(f"Best rotation early half: {early[1]:+.1f}deg, mean {early[0]:.3f} m")
    print(f"Best rotation late half:  {late[1]:+.1f}deg, mean {late[0]:.3f} m")
    print(f"Drift in best-rotation between halves: {late[1]-early[1]:+.2f}deg")

    # Sliding-window best rotation: chunks of 120s, step 60s
    chunk_s = 120.0
    step_s = 60.0
    chunks = []
    t_cur = sbl_t0
    while t_cur < sbl_te:
        t_end = min(t_cur + int(chunk_s * 1e9), sbl_te)
        b = best_rotation(ekf_track, sbl_track, E_aligned, N_aligned,
                          start_t_ns=t_cur, end_t_ns=t_end)
        if b is not None:
            chunks.append(((t_cur - sbl_t0) / 1e9, (t_end - sbl_t0) / 1e9,
                           b[1], b[0]))
        t_cur += int(step_s * 1e9)

    print(f"\nPer-window best rotation (window {chunk_s:.0f}s, step {step_s:.0f}s):")
    print(f"  {'t0':>6s}  {'t1':>6s}  {'rot':>8s}  {'mean':>7s}")
    for (a, b, r, m) in chunks[:20]:
        print(f"  {a:6.0f}  {b:6.0f}  {r:+8.1f}  {m:7.2f}")
    if len(chunks) > 20:
        print(f"  ... ({len(chunks)-20} more)")

    rots = np.array([c[2] for c in chunks])
    if len(rots) >= 3:
        rot_p10, rot_p50, rot_p90 = np.percentile(rots, [10, 50, 90])
        print(f"\nRotation across windows: p10={rot_p10:+.1f}  p50={rot_p50:+.1f}  p90={rot_p90:+.1f}  spread={rot_p90-rot_p10:.1f} deg")

    # Apply OVERALL best rotation; compute error over time
    cE, cN = rotate(E_aligned, N_aligned,
                    float(sbl_track.E[0]), float(sbl_track.N_utm[0]),
                    overall[1])
    es = pair_errors(ekf_track, sbl_track, cE, cN)

    # Get auxiliary data
    dvl_t, dvl_lock = get_dvl_lock(bag_dir)
    ac_t, ac_v = get_acoustic(bag_dir)
    yr_t, yr = get_imu_yaw_rate(bag_dir)
    iy_t, iy = get_imu_yaw_abs(bag_dir)

    # Plot multi-panel: error, DVL, acoustic, yaw rate, IMU yaw
    fig, axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True,
                              gridspec_kw={"hspace": 0.25,
                                           "height_ratios": [3, 1, 2, 2, 2]})

    ax = axes[0]
    ax.plot(es.t_rel_s, es.err_m, color="royalblue", lw=1.0)
    ax.axhline(3.0, color="orange", lw=0.5, ls=":")
    ax.set_ylabel("EKF↔SBL error (m)\n(post-rotation)")
    ax.set_title(f"{label} ({bag_dir.name})  —  best rotation {overall[1]:+.1f}°  →  mean {overall[0]:.2f} m")
    ax.grid(True, lw=0.3, alpha=0.5)

    ax = axes[1]
    if len(dvl_t):
        dvl_t_rel = (dvl_t - sbl_t0) / 1e9
        # Plot lock as horizontal lines
        ax.scatter(dvl_t_rel[dvl_lock], np.ones(dvl_lock.sum()),
                   c="green", s=2, label="locked")
        ax.scatter(dvl_t_rel[~dvl_lock], np.zeros((~dvl_lock).sum()),
                   c="red", s=2, label="unlocked")
        ax.set_yticks([0, 1])
        ax.set_yticklabels(["unlocked", "locked"])
    ax.set_ylabel("DVL lock")
    ax.set_ylim(-0.3, 1.3)
    ax.grid(True, lw=0.3, alpha=0.5)

    ax = axes[2]
    if len(ac_t):
        ac_t_rel = (ac_t - sbl_t0) / 1e9
        ax.plot(ac_t_rel, ac_v, color="darkviolet", lw=0.8)
        ax.axhline(2.0, color="orange", lw=0.5, ls=":", label="2 m threshold")
    ax.set_ylabel("SBL acoustic\nstd_m (m)")
    ax.set_yscale("log")
    ax.grid(True, lw=0.3, alpha=0.5)

    ax = axes[3]
    if len(yr_t):
        yr_t_rel = (yr_t - sbl_t0) / 1e9
        ax.plot(yr_t_rel, np.degrees(yr), color="darkorange", lw=0.5)
    ax.set_ylabel("|yaw rate|\n(deg/s)")
    ax.set_yscale("log")
    ax.grid(True, lw=0.3, alpha=0.5)

    ax = axes[4]
    if len(iy_t):
        iy_t_rel = (iy_t - sbl_t0) / 1e9
        ax.plot(iy_t_rel, np.degrees(iy), color="navy", lw=0.5)
        # Trend line: linear fit to detect drift
        if len(iy_t_rel) > 100 and iy_t_rel.max() > 0:
            slope, intcpt = np.polyfit(iy_t_rel, np.degrees(iy), 1)
            ax.plot(iy_t_rel, slope * iy_t_rel + intcpt, "r--", lw=0.8,
                    label=f"linear trend {slope:+.4f}°/s = {slope * 60:+.2f}°/min")
            ax.legend(fontsize=8, loc="upper left")
            print(f"\nIMU yaw linear trend: {slope*60:+.3f} deg/min over the bag")
    ax.set_ylabel("IMU yaw\n(deg, unwrapped)")
    ax.set_xlabel("Time (s, since first SBL sample)")
    ax.grid(True, lw=0.3, alpha=0.5)

    fig.savefig(out_png, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_png}")
    return chunks


def main():
    bags = [
        ("grid_01", Path("recordings/rosbags/2026-04-30/zermatt_grid_01_2026_04_30-12_04_16")),
        ("grid_02", Path("recordings/rosbags/2026-04-30/zermatt_grid_02_2026_04_30-13_00_44")),
    ]
    out_dir = Path("diagnosis/zermatt/grid_compare")
    out_dir.mkdir(parents=True, exist_ok=True)
    for label, p in bags:
        analyze(p, label, out_dir / f"{label}_compare.png")


if __name__ == "__main__":
    main()
