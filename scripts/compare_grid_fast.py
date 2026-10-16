"""Fast version: per-window optimal rotation in grid_02 + grid_01.
Uses analytical optimum (Procrustes) instead of brute-force angle sweep.
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
from sbl_accuracy_analysis import nearest_idx


def best_rotation_analytical(sbl_E, sbl_N, ekf_E, ekf_N, anchor_E, anchor_N):
    """Closed-form best rotation that minimizes sum of squared position errors,
    around a given anchor (Procrustes-like for 2D rotation only).
    Returns angle in degrees."""
    if len(sbl_E) < 2 or len(ekf_E) < 2:
        return None
    s_dE = sbl_E - anchor_E
    s_dN = sbl_N - anchor_N
    e_dE = ekf_E - anchor_E
    e_dN = ekf_N - anchor_N
    # Optimal rotation that maps EKF onto SBL: minimize sum(|R*ekf - sbl|^2)
    # For rotation by angle theta: rotated_E = c*e_dE - s*e_dN
    #                              rotated_N = s*e_dE + c*e_dN
    # Optimum: tan(theta) = (sum(e_dE*s_dN - e_dN*s_dE)) / (sum(e_dE*s_dE + e_dN*s_dN))
    num = np.sum(e_dE * s_dN - e_dN * s_dE)
    den = np.sum(e_dE * s_dE + e_dN * s_dN)
    return math.degrees(math.atan2(num, den))


def pair_within_window(ekf_track, sbl_track, E_aligned, N_aligned,
                       t_start_ns, t_end_ns, max_gap_s=1.0):
    """Return (sbl_E, sbl_N, ekf_E, ekf_N) for SBL pairs in [t_start, t_end]."""
    max_gap_ns = int(max_gap_s * 1e9)
    se, sn, ee, en = [], [], [], []
    for i in range(len(sbl_track.t_ns)):
        t = int(sbl_track.t_ns[i])
        if t < t_start_ns or t > t_end_ns:
            continue
        j = nearest_idx(ekf_track.t_ns, t)
        if abs(int(ekf_track.t_ns[j]) - t) > max_gap_ns:
            continue
        se.append(sbl_track.E[i])
        sn.append(sbl_track.N_utm[i])
        ee.append(E_aligned[j])
        en.append(N_aligned[j])
    return np.array(se), np.array(sn), np.array(ee), np.array(en)


def analyze(bag_dir, label):
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
    anchor_E = float(sbl_track.E[0])
    anchor_N = float(sbl_track.N_utm[0])

    # Sliding 120 s windows, step 60 s
    chunk_s = 120.0
    step_s = 60.0
    print(f"\nPer-window best rotation (analytical, window {chunk_s:.0f}s, step {step_s:.0f}s):")
    print(f"  {'t0':>5s}  {'t1':>5s}  {'n':>4s}  {'rot':>8s}")

    angles = []
    times = []
    t_cur = sbl_t0
    while t_cur < sbl_te:
        t_end = min(t_cur + int(chunk_s * 1e9), sbl_te)
        se, sn, ee, en = pair_within_window(
            ekf_track, sbl_track, E_aligned, N_aligned, t_cur, t_end
        )
        if len(se) >= 5:
            ang = best_rotation_analytical(se, sn, ee, en, anchor_E, anchor_N)
            angles.append(ang)
            times.append(((t_cur - sbl_t0) / 1e9, (t_end - sbl_t0) / 1e9))
            t0_rel = (t_cur - sbl_t0) / 1e9
            t1_rel = (t_end - sbl_t0) / 1e9
            print(f"  {t0_rel:5.0f}  {t1_rel:5.0f}  {len(se):4d}  {ang:+8.2f}")
        t_cur += int(step_s * 1e9)

    if len(angles) >= 3:
        a = np.array(angles)
        # Detect direction-of-drift change carefully (could wrap)
        a_unwrapped = np.unwrap(np.radians(a))
        a_deg_unwrapped = np.degrees(a_unwrapped)
        delta = a_deg_unwrapped[-1] - a_deg_unwrapped[0]
        print(f"\nWindow rotation summary:")
        print(f"  start: {a_deg_unwrapped[0]:+.2f}°")
        print(f"  end:   {a_deg_unwrapped[-1]:+.2f}°")
        print(f"  delta: {delta:+.2f}° over {duration_s:.0f} s = "
              f"{delta / (duration_s/60):+.3f}°/min")
        print(f"  median rate: {np.median(np.diff(a_deg_unwrapped) / np.diff([t[0] for t in times]) * 60):+.3f}°/min")


def main():
    bags = [
        ("grid_01", Path("recordings/rosbags/2026-04-30/zermatt_grid_01_2026_04_30-12_04_16")),
        ("grid_02", Path("recordings/rosbags/2026-04-30/zermatt_grid_02_2026_04_30-13_00_44")),
    ]
    for label, p in bags:
        analyze(p, label)


if __name__ == "__main__":
    main()
