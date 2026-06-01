#!/usr/bin/env python3
"""Per-grid-point ice-thickness distribution PNGs.

Splits the multi-panel distributions figure produced by
visualize_ice_measurements.py into one standalone PNG per measured
grid point.

Two stages, both sourced from measurements_unfiltered.csv:
  before — histogram over every attitude/GNSS-clean sample, i.e. the
           depth-oscillation rejects are still in. Shows the wide raw
           distribution that motivates the filter.
  after  — histogram over kept samples only (rejection_reason == "").
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

GRID_ROWS = 4

DEFAULT_UNF = Path(__file__).resolve().parent.parent / "zermatt_results" / "measurements_unfiltered.csv"
DEFAULT_AV  = Path(__file__).resolve().parent.parent / "zermatt_results" / "measurements_av.csv"
DEFAULT_OUT = Path(__file__).resolve().parent.parent / "zermatt_results" / "plots" / "per_gridpoint"


def load_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def unique_point_averages(av_rows):
    grouped = defaultdict(list)
    for r in av_rows:
        grouped[int(r["grid_point_id"])].append(r)

    result = {}
    for gp_id, rows in grouped.items():
        def avg(key):
            return np.mean([float(r[key]) for r in rows])
        result[gp_id] = {
            "gp_id": gp_id,
            "ice_thickness_m": avg("ice_thickness_m"),
            "distance_to_target_m": avg("distance_to_target_m"),
            "n_sessions": len(rows),
            "duration_s": sum(float(r["duration_s"]) for r in rows),
        }
    return result


def thickness_per_point(rows, include_rejected):
    """Pull ice_thickness_m per gp.

    include_rejected=False → only rejection_reason == "" (after-filter).
    include_rejected=True  → also include depth_oscillation rejects (before).
    """
    d = defaultdict(list)
    for r in rows:
        if not include_rejected and r.get("rejection_reason"):
            continue
        d[int(r["grid_point_id"])].append(float(r["ice_thickness_m"]))
    return {k: np.array(v) for k, v in d.items()}


def gp_row_col(gp_id):
    return gp_id % GRID_ROWS, gp_id // GRID_ROWS


def render_gp(gp_id, vals, av_info, out_path):
    """Render a single grid point's thickness distribution to out_path."""
    mean = vals.mean()
    std = vals.std()
    n_s = len(vals)

    fig, ax = plt.subplots(figsize=(7.5, 6))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    ax.spines[:].set_color("#cccccc")
    ax.tick_params(colors="black")

    bins = min(40, max(10, n_s // 30))
    ax.hist(
        vals,
        bins=bins,
        color="#5577ff",
        alpha=0.75,
        edgecolor="#3355cc",
        linewidth=0.4,
        density=True,
        zorder=2,
    )

    ax.axvline(mean, color="#ff9900", lw=2.0, zorder=3, label=f"mean {mean:.4f} m")
    ax.axvspan(
        mean - std,
        mean + std,
        alpha=0.18,
        color="#ffaa33",
        zorder=1,
        label=f"±1σ  {std*100:.2f} cm",
    )
    ax.axvline(mean - std, color="#ffaa33", lw=1.0, ls="--", zorder=3)
    ax.axvline(mean + std, color="#ffaa33", lw=1.0, ls="--", zorder=3)

    ax.set_title(
        f"Grid point {gp_id}",
        color="black",
        fontsize=14,
        pad=8,
    )
    ax.set_xlabel("Ice thickness (m)", color="#333333", fontsize=11)
    ax.set_ylabel("Density", color="#333333", fontsize=11)
    ax.xaxis.label.set_color("#333333")
    ax.yaxis.label.set_color("#333333")
    ax.tick_params(labelsize=10)

    ax.legend(
        fontsize=10,
        loc="upper left",
        facecolor="white",
        edgecolor="#cccccc",
        labelcolor="black",
    )

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--unfiltered", default=str(DEFAULT_UNF),
                   help=f"Unfiltered CSV (default: {DEFAULT_UNF})")
    p.add_argument("--av", default=str(DEFAULT_AV),
                   help=f"Averaged CSV (default: {DEFAULT_AV})")
    p.add_argument("--out", default=str(DEFAULT_OUT),
                   help=f"Output dir (default: {DEFAULT_OUT})")
    p.add_argument("--stage", choices=("before", "after", "both"),
                   default="both",
                   help="Which stage(s) to render (default: both).")
    p.add_argument("--gp", type=int, action="append",
                   help="Restrict to specific gp id (repeatable)")
    args = p.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = load_csv(args.unfiltered)
    av_pts = unique_point_averages(load_csv(args.av))

    stages = ("before", "after") if args.stage == "both" else (args.stage,)
    for stage in stages:
        rpg = thickness_per_point(rows, include_rejected=(stage == "before"))
        gp_ids = sorted(rpg.keys()) if not args.gp else sorted(set(args.gp) & set(rpg.keys()))
        if not gp_ids:
            print(f"[{stage}] no grid points to render.")
            continue
        for gp_id in gp_ids:
            vals = rpg[gp_id]
            out_path = out_dir / f"gp{gp_id:02d}_distribution_{stage}.png"
            render_gp(gp_id, vals, av_pts.get(gp_id), out_path)
            print(f"  [{stage}] Saved {out_path}  (n={len(vals):,})")
        print(f"  [{stage}] {len(gp_ids)} PNGs written")

    print(f"\nDone. Output dir: {out_dir}/")


if __name__ == "__main__":
    main()
