"""Sweep constant rotation angles on rectangle_07's DR vs SBL to test whether
a single rotation correction can salvage the bag.

Also reports SBL straight-segment bearings vs anchored-EKF bearings so we can
see whether the offset is constant or time-varying.
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


BAG = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    "recordings/rosbags/2026-04-29/zermatt_rectangle_07_2026_04_29-13_38_44"
)


def rotate(E, N, anchor_E, anchor_N, angle_deg):
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    dE = E - anchor_E
    dN = N - anchor_N
    return anchor_E + c * dE - s * dN, anchor_N + s * dE + c * dN


def find_segments(track, min_chord=2.0, ratio_max=1.10):
    E = track.E
    N = track.N_utm
    n = len(E)
    segs = []
    cursor = 0
    while cursor < n - 1:
        e0, n0 = float(E[cursor]), float(N[cursor])
        last_e, last_n = e0, n0
        path = 0.0
        found = None
        for j in range(cursor + 1, n):
            e, ny = float(E[j]), float(N[j])
            path += math.hypot(e - last_e, ny - last_n)
            last_e, last_n = e, ny
            chord = math.hypot(e - e0, ny - n0)
            if chord <= 0:
                continue
            ratio = path / chord
            if chord >= min_chord and ratio <= ratio_max:
                found = j
            elif found is not None and ratio > ratio_max:
                break
        if found is not None:
            segs.append((cursor, found))
            cursor = found + 1
        else:
            break
    return segs


def main():
    print(f"Reading {BAG.name} ...")
    bag = read_bag(BAG, want_local=True)
    print(f"  /gps/filtered/global: {len(bag.anchored)}")
    print(f"  /gps/filtered:        {len(bag.local)}")
    print(f"  SBL navsatfix:        {len(bag.sbl)}")

    sbl_msgs = gate_sbl(bag.sbl, max_sbl_std=5.0)
    sbl_track = navsatfix_to_track(sbl_msgs, "SBL")

    # Use anchored EKF (same shape as local DR for this kind of node)
    src = "anchored" if bag.anchored else "local"
    msgs = bag.anchored if bag.anchored else bag.local
    ekf_track = navsatfix_to_track(msgs, src)
    print(f"Using {src} track: {len(ekf_track.t_ns)} samples")

    E_aligned, N_aligned, *_ = align_to_anchor(ekf_track, sbl_track)
    es0 = pair_errors(ekf_track, sbl_track, E_aligned, N_aligned)
    print(f"\nBaseline (no rotation):")
    print(f"  n_pairs  : {es0.n_pairs}")
    print(f"  mean err : {float(np.mean(es0.err_m)):.3f} m")
    print(f"  max err  : {float(np.max(es0.err_m)):.3f} m")
    print(f"  drift    : {es0.drift_rate_m_per_100m:.2f} m/100m")

    # Sweep angles
    print(f"\nRotation sweep around (sbl[0]) anchor:")
    print(f"  {'angle':>7s}  {'mean':>7s}  {'max':>7s}  {'drift':>7s}")
    anchor_E = float(sbl_track.E[0])
    anchor_N = float(sbl_track.N_utm[0])
    best = None
    for ang in np.linspace(-180, 180, 361):
        cE, cN = rotate(E_aligned, N_aligned, anchor_E, anchor_N, ang)
        es = pair_errors(ekf_track, sbl_track, cE, cN)
        if not es.n_pairs:
            continue
        m = float(np.mean(es.err_m))
        mx = float(np.max(es.err_m))
        dr = es.drift_rate_m_per_100m
        if best is None or m < best[1]:
            best = (ang, m, mx, dr)
        # Print every 15 degrees
        if abs(ang - round(ang)) < 0.01 and round(ang) % 15 == 0:
            print(f"  {ang:+7.2f}  {m:7.3f}  {mx:7.3f}  {dr:+7.2f}")
    print(f"\nBest single-rotation correction:")
    print(f"  angle    : {best[0]:+.2f}°")
    print(f"  mean err : {best[1]:.3f} m  (was {float(np.mean(es0.err_m)):.3f})")
    print(f"  max err  : {best[2]:.3f} m  (was {float(np.max(es0.err_m)):.3f})")
    print(f"  drift    : {best[3]:+.2f} m/100m")

    # Per-segment bearing comparison: SBL straight segments vs DR matching window
    print(f"\nPer-segment bearing comparison (SBL straight, chord >= 2 m):")
    print(f"  {'t0':>6s}  {'t1':>6s}  {'sbl_brg':>8s}  {'dr_brg':>8s}  {'delta':>8s}  {'sbl_chord':>10s}  {'dr_chord':>10s}")
    segs = find_segments(sbl_track, min_chord=2.0, ratio_max=1.15)
    deltas = []
    for (i0, i1) in segs:
        dEs = sbl_track.E[i1] - sbl_track.E[i0]
        dNs = sbl_track.N_utm[i1] - sbl_track.N_utm[i0]
        sbl_brg = math.degrees(math.atan2(dEs, dNs))
        sbl_chord = math.hypot(dEs, dNs)
        t0, t1 = int(sbl_track.t_ns[i0]), int(sbl_track.t_ns[i1])
        mask = (ekf_track.t_ns >= t0) & (ekf_track.t_ns <= t1)
        idx = np.where(mask)[0]
        if len(idx) < 2:
            continue
        dEd = E_aligned[idx[-1]] - E_aligned[idx[0]]
        dNd = N_aligned[idx[-1]] - N_aligned[idx[0]]
        dr_chord = math.hypot(dEd, dNd)
        if dr_chord < 0.5:
            continue
        dr_brg = math.degrees(math.atan2(dEd, dNd))
        delta = ((dr_brg - sbl_brg + 540) % 360) - 180
        deltas.append(delta)
        t0_rel = (t0 - int(sbl_track.t_ns[0])) / 1e9
        t1_rel = (t1 - int(sbl_track.t_ns[0])) / 1e9
        print(f"  {t0_rel:6.1f}  {t1_rel:6.1f}  {sbl_brg:+8.2f}  {dr_brg:+8.2f}  {delta:+8.2f}  {sbl_chord:10.2f}  {dr_chord:10.2f}")
    if deltas:
        arr = np.array(deltas)
        print(f"\n  n_segments: {len(deltas)}")
        print(f"  mean delta: {float(np.mean(arr)):+.2f}°")
        print(f"  std delta : {float(np.std(arr)):.2f}°")
        print(f"  median    : {float(np.median(arr)):+.2f}°")


if __name__ == "__main__":
    main()
