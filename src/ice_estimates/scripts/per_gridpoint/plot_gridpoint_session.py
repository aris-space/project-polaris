#!/usr/bin/env python3
"""Per-grid-point session diagnostic plot.

For each requested grid point, render the candidate raw samples as a
time series (depth, pitch, roll, GNSS accuracy, ice thickness) with the
filter thresholds drawn in.

Two stages are produced from measurements_unfiltered.csv:
  before — pre-depth-filter view: every attitude/GNSS-clean sample, with
           the depth oscillations still in place. Motivates the filter.
  after  — same data, but samples flagged depth_oscillation are drawn in
           red. Shows what the filter actually removes.
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Gate thresholds (mirror extract_zermatt_measurements.py)
MIN_DEPTH_M         = 0.5
MAX_PITCH_DEG       = 25.0
MAX_ROLL_DEG        = 10.0
GNSS_MAX_ACC_M      = 4.0

DEFAULT_UNF = Path(__file__).resolve().parent.parent.parent / "zermatt_results" / "measurements_unfiltered.csv"
DEFAULT_OUT = Path(__file__).resolve().parent.parent.parent / "zermatt_results" / "gridpoint_sessions"


def load_unfiltered(path):
    """Return {gp_id: list[row]} sorted by timestamp within each gp.

    Includes both kept rows (rejection_reason == "") and depth_oscillation
    rejects — i.e. the pre-depth-filter sample set.
    """
    by_gp = defaultdict(list)
    with open(path) as f:
        for r in csv.DictReader(f):
            by_gp[int(r["grid_point_id"])].append({
                "t":        float(r["timestamp_s"]),
                "depth":    float(r["depth_m"]),
                "roll":     float(r["roll_deg"]),
                "pitch":    float(r["pitch_deg"]),
                "gnss_acc": float(r["gnss_accuracy_m"]),
                "T":        float(r["ice_thickness_m"]),
                "bag":      r["bag"],
                "reject":   r.get("rejection_reason", ""),
            })
    for gp in by_gp.values():
        gp.sort(key=lambda x: x["t"])
    return by_gp


def render_gp(gp_id, rows, out_path, stage):
    """stage ∈ {"before", "after"}.

    before: all rows drawn in the panel colour; the audience sees the
            depth oscillations untouched.
    after:  rows with rejection_reason == "depth_oscillation" are drawn
            in red on top, so the filter's effect is visible.
    """
    if not rows:
        print(f"gp{gp_id}: no samples"); return

    t0 = rows[0]["t"]
    t = np.array([r["t"] for r in rows]) - t0
    depth = np.array([r["depth"]    for r in rows])
    pitch = np.array([r["pitch"]    for r in rows])
    roll  = np.array([r["roll"]     for r in rows])
    acc   = np.array([r["gnss_acc"] for r in rows])
    T     = np.array([r["T"]        for r in rows])
    rej   = np.array([r["reject"] == "depth_oscillation" for r in rows])
    kept_mask = ~rej

    fig, axes = plt.subplots(5, 1, figsize=(11, 11), sharex=True,
                             gridspec_kw={"hspace": 0.12},
                             constrained_layout=True)
    fig.patch.set_facecolor("white")

    def style(ax):
        ax.set_facecolor("#f7f9fc")
        ax.grid(True, color="#dddddd", linewidth=0.5, zorder=0)
        ax.spines[:].set_color("#cccccc")
        ax.tick_params(colors="#333333", labelsize=9)

    # Break lines wherever consecutive samples are more than GAP_S seconds
    # apart — otherwise a sparse session draws a deceptive smooth line over an
    # interval where every sample was filtered out.
    GAP_S = 0.5
    dt = np.diff(t, prepend=t[0])
    seg = np.cumsum(dt > GAP_S)

    def seg_plot(ax, y, color):
        for s in np.unique(seg):
            m = seg == s
            if m.sum() == 1:
                ax.scatter(t[m], y[m], s=6, color=color, zorder=2)
            else:
                ax.plot(t[m], y[m], lw=0.8, color=color, zorder=2)
        ax.scatter(t, y, s=2, color=color, alpha=0.6, zorder=3, rasterized=True)

    REJ_COLOR = "#d62728"

    def overlay_rejects(ax, y):
        if stage == "after" and rej.any():
            ax.scatter(t[rej], y[rej], s=10, color=REJ_COLOR, alpha=0.85,
                       zorder=5, rasterized=True,
                       label="rejected: depth_oscillation")

    # depth
    ax = axes[0]
    style(ax)
    seg_plot(ax, depth, "#1f77b4")
    overlay_rejects(ax, depth)
    ax.axhline(MIN_DEPTH_M, color="#d62728", lw=0.8, ls="--", alpha=0.7,
               label=f"reject < {MIN_DEPTH_M} m")
    ax.set_ylabel("Depth (m)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)

    # pitch
    ax = axes[1]
    style(ax)
    seg_plot(ax, pitch, "#2ca02c")
    overlay_rejects(ax, pitch)
    ax.axhline(MAX_PITCH_DEG, color="#d62728", lw=0.8, ls="--", alpha=0.7)
    ax.axhline(-MAX_PITCH_DEG, color="#d62728", lw=0.8, ls="--", alpha=0.7,
               label=f"reject ±{MAX_PITCH_DEG:.0f}°")
    y_abs = max(abs(pitch).max() * 1.2, MAX_PITCH_DEG * 1.1, 10)
    ax.set_ylim(-y_abs, y_abs)
    ax.set_ylabel("Pitch (°)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)

    # roll
    ax = axes[2]
    style(ax)
    seg_plot(ax, roll, "#9467bd")
    overlay_rejects(ax, roll)
    ax.axhline(MAX_ROLL_DEG, color="#d62728", lw=0.8, ls="--", alpha=0.7)
    ax.axhline(-MAX_ROLL_DEG, color="#d62728", lw=0.8, ls="--", alpha=0.7,
               label=f"reject ±{MAX_ROLL_DEG:.0f}°")
    y_abs = max(abs(roll).max() * 1.2, MAX_ROLL_DEG * 1.1, 5)
    ax.set_ylim(-y_abs, y_abs)
    ax.set_ylabel("Roll (°)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)

    # gnss acc
    ax = axes[3]
    style(ax)
    seg_plot(ax, acc, "#ff7f0e")
    overlay_rejects(ax, acc)
    ax.axhline(GNSS_MAX_ACC_M, color="#d62728", lw=0.8, ls="--", alpha=0.7,
               label=f"reject ≥ {GNSS_MAX_ACC_M:.0f} m")
    ax.set_ylim(0, max(acc.max() * 1.2, GNSS_MAX_ACC_M * 1.1))
    ax.set_ylabel("GNSS σ (m)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)

    # thickness — median computed on kept samples only, since that's the
    # number the filter is trying to defend.
    ax = axes[4]
    style(ax)
    ax.scatter(t[kept_mask], T[kept_mask], s=4, color="#8e44ad", alpha=0.5,
               rasterized=True)
    if stage == "after" and rej.any():
        ax.scatter(t[rej], T[rej], s=10, color=REJ_COLOR, alpha=0.85,
                   zorder=5, rasterized=True)
    med = np.median(T[kept_mask]) if kept_mask.any() else float("nan")
    ax.axhline(med, color="#555555", lw=1.0, ls="--", alpha=0.8,
               label=f"median = {med:.3f} m")
    ax.set_ylabel("Ice thickness (m)", fontsize=10)
    ax.set_xlabel("Time since session start (s)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8)

    fig.suptitle(f"Grid point {gp_id}", fontsize=12, color="#111111")

    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"Saved {out_path}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gp", type=int, action="append", default=None,
                        help="Grid point id (repeatable). Default: all grid points present in the CSV.")
    parser.add_argument("--unfiltered", default=str(DEFAULT_UNF),
                        help=f"unfiltered CSV (default: {DEFAULT_UNF})")
    parser.add_argument("--out", default=str(DEFAULT_OUT),
                        help=f"output dir (default: {DEFAULT_OUT})")
    parser.add_argument("--stage", choices=("before", "after", "both"),
                        default="both",
                        help="Which stage(s) to render (default: both).")
    args = parser.parse_args()

    by_gp = load_unfiltered(args.unfiltered)

    gps = args.gp if args.gp is not None else sorted(by_gp)
    stages = ("before", "after") if args.stage == "both" else (args.stage,)
    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    for gp in gps:
        for st in stages:
            render_gp(gp, by_gp.get(gp, []), out_dir / f"gp{gp}_{st}.png", st)


if __name__ == "__main__":
    main()
