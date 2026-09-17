#!/usr/bin/env python3
"""
Decide whether the residual EKF↔SBL offset is:

  (a) a pure translation  — constant vector regardless of AUV position
                            (e.g. SBL surface-station GPS bias, antenna lever-arm)
  (b) a frame rotation    — scales linearly with AUV displacement from datum
                            (e.g. IMU yaw bias not absorbed by yaw_offset_deg /
                             magnetic_declination_radians)
  (c) a rotation + translation — both
  (d) something else      — drift, integration error, etc.

Reads ekf_offline_diagnostic CSV and fits the SBL residual ``sbl_err_xy`` against
the AUV position ``(x, y)``. The rigid-body model

    sbl_err = R(theta) · (x, y) - (x, y) + (tx, ty)
            = (cos θ - 1) (x,y) + (-sin θ) y, ..., + (tx, ty)

If we observe ``(sbl_x_proj, sbl_y_proj) = R · (x, y) + t`` then a 2D least
squares fit on (x, y) ==> (sbl_x, sbl_y) recovers (R, t) directly.

Reports:
- best-fit rotation angle θ° between map frame and SBL/UTM ENU
- best-fit translation (tx, ty) m
- residual after removing both (this is the "real" tracking error)

Usage:
    python scripts/analyze_diag_offset.py path/to/diag.csv [more.csv ...]
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


def load(path: Path, p_xx_max: float | None = None):
    xs, ys, sxs, sys_ = [], [], [], []
    # Also load global x,y unconditionally (even when SBL is missing) so we can
    # report path-length / bounding-box stats over the whole run.
    all_xs, all_ys = [], []
    all_sxs, all_sys = [], []
    # Diagnostic split: high-P vs low-P global samples. When the diagnostic
    # CSV interleaves messages from two different /odometry/filtered/global
    # publishers (e.g. a stale gnss_anchored_pose from a previous replay),
    # the bbox/path-length numbers double-count both streams. Splitting by
    # P_xx separates them.
    lo_xs, lo_ys = [], []  # P_xx <= p_xx_max  (the "good" stream if specified)
    hi_xs, hi_ys = [], []  # P_xx >  p_xx_max
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                x = float(row["x"]); y = float(row["y"])
                all_xs.append(x); all_ys.append(y)
                if p_xx_max is not None:
                    pxx = float(row.get("P_xx", "nan"))
                    if math.isfinite(pxx):
                        if pxx <= p_xx_max:
                            lo_xs.append(x); lo_ys.append(y)
                        else:
                            hi_xs.append(x); hi_ys.append(y)
            except (ValueError, KeyError):
                pass
            try:
                sx = float(row["sbl_x"]); sy = float(row["sbl_y"])
                all_sxs.append(sx); all_sys.append(sy)
            except (ValueError, KeyError):
                pass
            try:
                x = float(row["x"]); y = float(row["y"])
                sx = float(row["sbl_x"]); sy = float(row["sbl_y"])
            except (ValueError, KeyError):
                continue
            # If filtering, restrict the paired (rigid-body) fit to the low-P
            # stream so the fit isn't poisoned by the second publisher.
            if p_xx_max is not None:
                try:
                    pxx = float(row.get("P_xx", "nan"))
                except (ValueError, KeyError):
                    pxx = float("nan")
                if not math.isfinite(pxx) or pxx > p_xx_max:
                    continue
            xs.append(x); ys.append(y); sxs.append(sx); sys_.append(sy)
    return xs, ys, sxs, sys_, all_xs, all_ys, all_sxs, all_sys, lo_xs, lo_ys, hi_xs, hi_ys


def bbox_and_path(xs, ys):
    if not xs:
        return None
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    width = x_max - x_min
    height = y_max - y_min
    path_length = 0.0
    for i in range(1, len(xs)):
        path_length += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
    max_disp = max(math.hypot(x, y) for x, y in zip(xs, ys))
    return {
        "x_range": (x_min, x_max),
        "y_range": (y_min, y_max),
        "bbox_w": width,
        "bbox_h": height,
        "path_length": path_length,
        "max_disp_from_origin": max_disp,
        "n_samples": len(xs),
    }


def fit_rigid_2d(xs, ys, sxs, sys_):
    """Least-squares fit:  (sx, sy) = R(theta) (x, y) + (tx, ty).

    Closed form: minimise sum |sbl - (R x + t)|² over (theta, tx, ty).
    Translation is the centroid difference; rotation is the angle that
    aligns the centred clouds (Procrustes / Kabsch in 2D).
    """
    n = len(xs)
    if n < 2:
        return None
    cx = sum(xs) / n; cy = sum(ys) / n
    csx = sum(sxs) / n; csy = sum(sys_) / n
    # Centred clouds
    px = [v - cx for v in xs]; py = [v - cy for v in ys]
    qx = [v - csx for v in sxs]; qy = [v - csy for v in sys_]
    # 2x2 cross-cov matrix H = sum p_i^T q_i
    H_xx = sum(a * b for a, b in zip(px, qx))
    H_xy = sum(a * b for a, b in zip(px, qy))
    H_yx = sum(a * b for a, b in zip(py, qx))
    H_yy = sum(a * b for a, b in zip(py, qy))
    # In 2D, R(theta) that maximises trace(R H^T):
    # theta = atan2(H_yx - H_xy, H_xx + H_yy)
    theta = math.atan2(H_yx - H_xy, H_xx + H_yy)
    tx = csx - (math.cos(theta) * cx - math.sin(theta) * cy)
    ty = csy - (math.sin(theta) * cx + math.cos(theta) * cy)
    # Residuals after R+t
    res2 = []
    for x, y, sx, sy in zip(xs, ys, sxs, sys_):
        rx = math.cos(theta) * x - math.sin(theta) * y + tx
        ry = math.sin(theta) * x + math.cos(theta) * y + ty
        res2.append((sx - rx) ** 2 + (sy - ry) ** 2)
    n_res = len(res2)
    rms = math.sqrt(sum(res2) / n_res) if n_res else float("nan")
    return {
        "n": n,
        "theta_rad": theta,
        "theta_deg": math.degrees(theta),
        "tx": tx,
        "ty": ty,
        "centroid_global": (cx, cy),
        "centroid_sbl": (csx, csy),
        "rms_after_R_and_t": rms,
        "max_displacement_global": max(math.hypot(x, y) for x, y in zip(xs, ys)),
        "max_displacement_sbl": max(math.hypot(x, y) for x, y in zip(sxs, sys_)),
    }


def fit_translation_only(xs, ys, sxs, sys_):
    """For comparison: best pure translation only."""
    n = len(xs)
    if n == 0:
        return None
    tx = sum(s - x for s, x in zip(sxs, xs)) / n
    ty = sum(s - y for s, y in zip(sys_, ys)) / n
    res2 = []
    for x, y, sx, sy in zip(xs, ys, sxs, sys_):
        res2.append((sx - x - tx) ** 2 + (sy - y - ty) ** 2)
    rms = math.sqrt(sum(res2) / n)
    return {"tx": tx, "ty": ty, "rms_translation_only": rms}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csvs", nargs="+", type=Path)
    p.add_argument(
        "--rebase-sbl",
        action="store_true",
        help=(
            "Subtract the SBL value at the first paired row from every SBL "
            "sample. This rebases SBL onto whatever datum the global track "
            "is using — useful when the diagnostic's datum (first "
            "/gps/selected) and the anchored-pose's datum (first /gps/selected "
            "with h_acc<=0.5m) differ because the AUV moved while waiting for "
            "RTK quality to qualify. After rebasing, the residual = real "
            "EKF-vs-SBL tracking error, not a datum-pick artefact."
        ),
    )
    p.add_argument(
        "--p-xx-max",
        type=float,
        default=None,
        help=(
            "If set, only include rows where P_xx <= this value in the "
            "rigid-body fit. Reports separate bbox stats for the low-P "
            "stream vs the high-P stream so duplicate-publisher artefacts "
            "(e.g. a stale gnss_anchored_pose from a prior replay still "
            "publishing on /odometry/filtered/global) can be isolated. "
            "Try 1.0 to filter to gnss_anchored_pose's normal cov range."
        ),
    )
    args = p.parse_args(argv)

    for csvp in args.csvs:
        xs, ys, sxs, sys_, all_xs, all_ys, all_sxs, all_sys, lo_xs, lo_ys, hi_xs, hi_ys = load(
            csvp, p_xx_max=args.p_xx_max
        )
        if args.rebase_sbl and sxs and sys_:
            # First paired row: SBL at the moment the global track first
            # publishes (= anchor moment for anchored runs). The AUV is at
            # the global track's datum by definition; any non-zero SBL value
            # here is the inter-datum offset.
            sbl0_x = sxs[0]
            sbl0_y = sys_[0]
            sxs = [v - sbl0_x for v in sxs]
            sys_ = [v - sbl0_y for v in sys_]
            all_sxs = [v - sbl0_x for v in all_sxs]
            all_sys = [v - sbl0_y for v in all_sys]
            print(f"  rebased SBL by ({-sbl0_x:+.3f}, {-sbl0_y:+.3f}) m "
                  f"(magnitude {math.hypot(sbl0_x, sbl0_y):.3f} m)")
        if not xs:
            print(f"{csvp}: no valid rows with both global and sbl coords")
            continue
        rigid = fit_rigid_2d(xs, ys, sxs, sys_)
        trans = fit_translation_only(xs, ys, sxs, sys_)
        gbb = bbox_and_path(all_xs, all_ys)
        sbb = bbox_and_path(all_sxs, all_sys)
        lo_bb = bbox_and_path(lo_xs, lo_ys) if lo_xs else None
        hi_bb = bbox_and_path(hi_xs, hi_ys) if hi_xs else None
        print()
        print("=" * 90)
        print(f"file: {csvp}")
        print("=" * 90)
        print(f"  paired (both topics fresh) samples: {rigid['n']}")
        print()
        print("  GLOBAL track (whole run, both streams if duplicate publishers):")
        if gbb:
            print(f"    n_samples = {gbb['n_samples']}")
            print(f"    x range   = [{gbb['x_range'][0]:+8.2f}, {gbb['x_range'][1]:+8.2f}]  (width  = {gbb['bbox_w']:.2f} m)")
            print(f"    y range   = [{gbb['y_range'][0]:+8.2f}, {gbb['y_range'][1]:+8.2f}]  (height = {gbb['bbox_h']:.2f} m)")
            print(f"    path length              = {gbb['path_length']:>10.2f} m")
            print(f"    max distance from origin = {gbb['max_disp_from_origin']:>10.2f} m")
        if lo_bb is not None:
            print()
            print(f"  GLOBAL track (low-P stream, P_xx <= {args.p_xx_max}):")
            print(f"    n_samples = {lo_bb['n_samples']}")
            print(f"    x range   = [{lo_bb['x_range'][0]:+8.2f}, {lo_bb['x_range'][1]:+8.2f}]  (width  = {lo_bb['bbox_w']:.2f} m)")
            print(f"    y range   = [{lo_bb['y_range'][0]:+8.2f}, {lo_bb['y_range'][1]:+8.2f}]  (height = {lo_bb['bbox_h']:.2f} m)")
            print(f"    path length              = {lo_bb['path_length']:>10.2f} m")
            print(f"    max distance from origin = {lo_bb['max_disp_from_origin']:>10.2f} m")
        if hi_bb is not None and hi_bb["n_samples"] > 0:
            print()
            print(f"  GLOBAL track (high-P stream, P_xx > {args.p_xx_max} — likely stray publisher):")
            print(f"    n_samples = {hi_bb['n_samples']}")
            print(f"    x range   = [{hi_bb['x_range'][0]:+8.2f}, {hi_bb['x_range'][1]:+8.2f}]  (width  = {hi_bb['bbox_w']:.2f} m)")
            print(f"    y range   = [{hi_bb['y_range'][0]:+8.2f}, {hi_bb['y_range'][1]:+8.2f}]  (height = {hi_bb['bbox_h']:.2f} m)")
            print(f"    max distance from origin = {hi_bb['max_disp_from_origin']:>10.2f} m")
        print()
        print("  SBL track (whole run):")
        if sbb:
            print(f"    n_samples = {sbb['n_samples']}")
            print(f"    x range   = [{sbb['x_range'][0]:+8.2f}, {sbb['x_range'][1]:+8.2f}]  (width  = {sbb['bbox_w']:.2f} m)")
            print(f"    y range   = [{sbb['y_range'][0]:+8.2f}, {sbb['y_range'][1]:+8.2f}]  (height = {sbb['bbox_h']:.2f} m)")
            print(f"    path length              = {sbb['path_length']:>10.2f} m")
            print(f"    max distance from origin = {sbb['max_disp_from_origin']:>10.2f} m")
        print()
        print("  Pure-translation fit (sbl = global + t):")
        print(f"    t = ({trans['tx']:+.3f}, {trans['ty']:+.3f}) m")
        print(f"    |t| = {math.hypot(trans['tx'], trans['ty']):.3f} m")
        print(f"    RMS residual after t alone: {trans['rms_translation_only']:.3f} m")
        print()
        print("  Rigid-body fit (sbl = R(theta) global + t):")
        print(f"    theta = {rigid['theta_deg']:+.3f} deg ({rigid['theta_rad']:+.6f} rad)")
        print(f"    t = ({rigid['tx']:+.3f}, {rigid['ty']:+.3f}) m")
        print(f"    |t| = {math.hypot(rigid['tx'], rigid['ty']):.3f} m")
        print(f"    RMS residual after R + t: {rigid['rms_after_R_and_t']:.3f} m")
        print()
        if trans["rms_translation_only"] - rigid["rms_after_R_and_t"] > 0.5:
            print("  ==> ROTATION dominates: residual drops significantly when a rotation")
            print("    is included in the fit. The 'translation' you see in the diagnostic")
            print("    is mostly the linear effect of a frame-rotation between the EKF map")
            print("    frame and SBL/UTM ENU, scaled by AUV displacement from datum.")
        elif rigid["rms_after_R_and_t"] < 1.0:
            print("  ==> Mostly TRANSLATION (rotation negligible). The offset is a constant")
            print("    vector — likely SBL surface-station GPS bias or antenna lever-arm.")
        else:
            print("  ==> Residual after R + t is still large; the offset has a non-rigid")
            print("    component (drift, time-varying yaw, etc).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
