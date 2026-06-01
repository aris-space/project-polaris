#!/usr/bin/env python3
"""
Two-panel filter example for one gridpoint touch session: thickness vs time
on top, sensor depth vs time below, both coloured by whether the
depth-stability filter kept or rejected each sample. The depth panel
explains *why* a sample was rejected (vehicle drifted down, away from the
ice front), so the figure is not just "we threw values out" but "we threw
out the values where the vehicle wasn't actually pressed against the ice".

Reads measurements_unfiltered.csv produced by extract_zermatt_measurements.py.
"""
import argparse
import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np

# Match the dark theme used by plot_thickness_drift.py / failure-mode figures.
FIG_BG    = "#0e1117"
AX_BG     = "#1a1a2e"
GRID_COL  = "#2a2a4a"
SPINE_COL = "#444466"
LABEL_COL = "#aaaacc"

KEPT_COLOR = "#1f77b4"
REJ_COLOR  = "#d62728"


def _style_axes(ax):
    ax.set_facecolor(AX_BG)
    ax.grid(True, color=GRID_COL, linewidth=0.5, zorder=0)
    ax.spines[:].set_color(SPINE_COL)
    ax.tick_params(colors="white", labelsize=9)

# Lever arm from pressure sensor to ice-contact point (matches
# extract_zermatt_measurements.py / config.yaml).
PRESSURE_TO_CONTACT_Z_M = 0.210
PRESSURE_TO_CONTACT_X_M = 0.515


def corrected_depth(depth_m, pitch_deg, roll_deg):
    """Attitude-corrected depth of the ice-contact point — same quantity the
    depth-stability filter operates on."""
    pitch = np.radians(pitch_deg)
    roll  = np.radians(roll_deg)
    omega = (PRESSURE_TO_CONTACT_X_M * np.sin(pitch)
             + PRESSURE_TO_CONTACT_Z_M * np.cos(pitch) * np.cos(roll))
    return depth_m - omega


def load_gp(csv_path, gp_id):
    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            if int(r["grid_point_id"]) != gp_id:
                continue
            depth = float(r["depth_m"])
            pitch = float(r["pitch_deg"])
            roll  = float(r["roll_deg"])
            rows.append({
                "t":      float(r["timestamp_s"]),
                "thick":  float(r["ice_thickness_m"]),
                "depth":  depth,
                "dc":     float(corrected_depth(depth, pitch, roll)),
                "reason": r["rejection_reason"],
            })
    rows.sort(key=lambda r: r["t"])
    return rows


def split_sessions(rows, max_gap_s=2.0):
    if not rows:
        return []
    sessions = [[rows[0]]]
    for r in rows[1:]:
        if r["t"] - sessions[-1][-1]["t"] > max_gap_s:
            sessions.append([])
        sessions[-1].append(r)
    return sessions


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_unf = os.path.abspath(os.path.join(
        here, "../zermatt_results/measurements_unfiltered.csv"))
    default_out = os.path.abspath(os.path.join(
        here, "../zermatt_results/plots"))

    p = argparse.ArgumentParser()
    p.add_argument("--unfiltered", default=default_unf)
    p.add_argument("--gp", type=int, default=1)
    p.add_argument("--session", type=int, default=0,
                   help="0-based session index within the gridpoint")
    p.add_argument("--out", default=default_out)
    args = p.parse_args()

    rows = load_gp(args.unfiltered, args.gp)
    if not rows:
        raise SystemExit(f"No rows for grid_point_id={args.gp} in {args.unfiltered}")

    sessions = split_sessions(rows)
    if args.session >= len(sessions):
        raise SystemExit(
            f"gp {args.gp} has {len(sessions)} session(s); index {args.session} out of range"
        )
    sess = sessions[args.session]
    t0 = sess[0]["t"]

    kept = [(r["t"] - t0, r["thick"], r["dc"]) for r in sess if r["reason"] == ""]
    rej  = [(r["t"] - t0, r["thick"], r["dc"]) for r in sess if r["reason"] != ""]
    kept_mean = float(np.mean([y for _, y, _ in kept])) if kept else float("nan")
    kept_depth_min = float(np.min([d for _, _, d in kept])) if kept else float("nan")

    # Rolling minimum of the corrected depth over the same ±15 s window the
    # filter uses, computed from kept samples only. Visualises the threshold
    # line that drives each rejection.
    ts_kept = np.array([x for x, _, _ in kept])
    dc_kept = np.array([d for _, _, d in kept])
    half = 15.0
    threshold = 0.01
    ts_all = np.array([r["t"] - t0 for r in sess])
    rolling_min = np.empty_like(ts_all)
    for i, t in enumerate(ts_all):
        lo = np.searchsorted(ts_kept, t - half)
        hi = np.searchsorted(ts_kept, t + half, side="right")
        rolling_min[i] = dc_kept[lo:hi].min() if hi > lo else np.nan

    fig, (axT, axD) = plt.subplots(
        2, 1, figsize=(11, 7), sharex=True,
        gridspec_kw={"height_ratios": [2, 1]},
    )
    fig.patch.set_facecolor(FIG_BG)
    _style_axes(axT)
    _style_axes(axD)

    # --- Top: thickness, coloured by kept/rejected ---
    if rej:
        axT.scatter([x for x, _, _ in rej], [y for _, y, _ in rej],
                    s=8, c=REJ_COLOR, alpha=0.55,
                    label=f"rejected: vehicle drifted off ice  (n={len(rej):,})")
    if kept:
        axT.scatter([x for x, _, _ in kept], [y for _, y, _ in kept],
                    s=10, c=KEPT_COLOR, alpha=0.85,
                    label=f"kept: at ice front  (n={len(kept):,})")
    axT.axhline(kept_mean, color="#e67e22", linestyle="--", linewidth=1.2,
                label=f"filtered mean = {kept_mean:.3f} m")
    axT.set_ylabel("ice thickness (m)", color=LABEL_COL)
    axT.set_title(
        f"gridpoint {args.gp} session {args.session}: "
        f"{len(kept):,} of {len(sess):,} samples kept "
        f"({100*len(kept)/len(sess):.0f}%)",
        color="white", loc="left",
    )
    axT.legend(loc="upper right", fontsize=9, markerscale=2.0,
               facecolor=FIG_BG, edgecolor=SPINE_COL, labelcolor="white")

    # --- Bottom: attitude-corrected contact depth (what the filter sees) ---
    if rej:
        axD.scatter([x for x, _, _ in rej], [d for _, _, d in rej],
                    s=8, c=REJ_COLOR, alpha=0.55)
    if kept:
        axD.scatter([x for x, _, _ in kept], [d for _, _, d in kept],
                    s=10, c=KEPT_COLOR, alpha=0.85)
    axD.plot(ts_all, rolling_min + threshold, color="#e67e22",
             linestyle=":", linewidth=1.2,
             label=f"rolling-min + {threshold*100:.0f} cm (filter threshold)")
    axD.invert_yaxis()  # shallower depth (= pressed against ice) at the top
    axD.set_xlabel("time since session start (s)", color=LABEL_COL)
    axD.set_ylabel("contact depth (m)\n(shallow = at ice)", color=LABEL_COL)
    axD.legend(loc="lower right", fontsize=9,
               facecolor=FIG_BG, edgecolor=SPINE_COL, labelcolor="white")

    fig.tight_layout()

    os.makedirs(args.out, exist_ok=True)
    out_path = os.path.join(
        args.out, f"filter_example_gp{args.gp}_s{args.session}.png")
    fig.savefig(out_path, dpi=180, bbox_inches="tight", facecolor=FIG_BG)
    print(f"wrote {out_path}")
    print(f"  session_n={len(sess)}  kept={len(kept)}  rejected={len(rej)}")
    print(f"  filtered mean thickness = {kept_mean:.4f} m")


if __name__ == "__main__":
    main()
