#!/usr/bin/env python3
"""Ice-thickness drift plot for a single grid-point session.

Plots the kept (post-attitude-filter) ice-thickness samples over time.

Usage:
    python3 plot_thickness_drift.py [--gp ID] [--unfiltered FILE] [--out FILE]
                                    [--channel thickness|depth]
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CSV_DEFAULT = (Path(__file__).resolve().parent.parent.parent
               / "zermatt_results" / "measurements_unfiltered.csv")
OUT_DEFAULT = (Path(__file__).resolve().parent.parent.parent
               / "zermatt_results" / "plots" / "thickness_drift_gp{gp}.png")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gp", type=int, default=1)
    ap.add_argument("--unfiltered", default=str(CSV_DEFAULT))
    ap.add_argument("--out", default=None)
    ap.add_argument("--channel", choices=("thickness", "depth"), default="thickness",
                    help="Which channel to plot on the y-axis (default: thickness).")
    ap.add_argument("--max-t", type=float, default=None,
                    help="Drop samples with t > MAX_T seconds since session start.")
    args = ap.parse_args()

    out_path = args.out or str(OUT_DEFAULT).format(gp=args.gp)

    df = pd.read_csv(args.unfiltered)
    df = df[df["grid_point_id"] == args.gp].copy()
    df = df.sort_values("timestamp_s").reset_index(drop=True)
    if df.empty:
        raise SystemExit(f"no rows for gp={args.gp}")

    t0 = df["timestamp_s"].iloc[0]
    df["t"] = df["timestamp_s"] - t0
    if args.max_t is not None:
        df = df[df["t"] <= args.max_t].copy()
        if df.empty:
            raise SystemExit(f"no rows survive --max-t {args.max_t}")

    col = "depth_m" if args.channel == "depth" else "ice_thickness_m"
    ylabel = "Depth (m)" if args.channel == "depth" else "Ice thickness (m)"

    t_all = df["t"].values
    y_all = df[col].values

    fig, ax = plt.subplots(figsize=(10, 4.5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    ax.grid(True, color="#eeeeee", linewidth=0.5, zorder=0)
    ax.spines[:].set_color("#cccccc")
    ax.tick_params(colors="black", labelsize=9)

    ch_label = "depth" if args.channel == "depth" else "ice thickness"

    ax.scatter(t_all, y_all, s=2,
               color="#1f77b4" if args.channel == "depth" else "#8e44ad",
               alpha=0.45, zorder=3, rasterized=True, label=ch_label)

    # Clip tails so the main action fills the frame
    lo, hi = np.percentile(y_all, 2), np.percentile(y_all, 98)
    pad = (hi - lo) * 0.15
    if args.channel == "depth":
        ax.set_ylim(hi + pad, lo - pad)   # inverted: deeper at bottom
    else:
        ax.set_ylim(lo - pad, hi + pad)

    ax.set_xlabel("Time since session start (s)", fontsize=10, color="#333333")
    ax.set_ylabel(ylabel, fontsize=10, color="#333333")
    ax.set_title(f"Grid point {args.gp} — {ch_label} over session",
                 fontsize=11, color="black", loc="left")

    ax.legend(loc="upper right", fontsize=9,
              facecolor="white", edgecolor="#cccccc", labelcolor="black")

    fig.tight_layout()
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight", facecolor="white")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
