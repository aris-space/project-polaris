#!/usr/bin/env python3
"""
Two-panel comparison plot for a single gridpoint that visualises what the
depth-stability / attitude filter removes from a touch session.

Left panel  : every candidate sample (kept + rejected), colored by reason.
Right panel : only kept samples, with the filtered mean as a horizontal line.

Reads measurements_unfiltered.csv produced by extract_zermatt_measurements.py
(or regen_unfiltered_csv.py on the host). Outputs a PNG into
zermatt_results/plots/.
"""
import argparse
import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np


REASON_COLORS = {
    "":                  ("#1f77b4", "kept"),
    "depth_oscillation": ("#d62728", "depth oscillation"),
    "pitch":             ("#ff7f0e", "pitch > limit"),
    "roll":              ("#9467bd", "roll > limit"),
}


def load_gp(csv_path, gp_id):
    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            if int(r["grid_point_id"]) != gp_id:
                continue
            rows.append({
                "t":      float(r["timestamp_s"]),
                "thick":  float(r["ice_thickness_m"]),
                "depth":  float(r["depth_m"]),
                "reason": r["rejection_reason"],
            })
    rows.sort(key=lambda r: r["t"])
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_unf = os.path.abspath(os.path.join(
        here, "../zermatt_results/measurements_unfiltered.csv"))
    default_out = os.path.abspath(os.path.join(
        here, "../zermatt_results/plots"))

    p = argparse.ArgumentParser()
    p.add_argument("--unfiltered", default=default_unf)
    p.add_argument("--gp", type=int, default=1)
    p.add_argument("--out", default=default_out)
    args = p.parse_args()

    rows = load_gp(args.unfiltered, args.gp)
    if not rows:
        raise SystemExit(f"No rows for grid_point_id={args.gp} in {args.unfiltered}")

    t0 = rows[0]["t"]
    by_reason = defaultdict(list)
    for r in rows:
        by_reason[r["reason"]].append((r["t"] - t0, r["thick"]))

    kept = by_reason.get("", [])
    rejected_total = sum(len(v) for k, v in by_reason.items() if k)
    n_total = len(rows)
    kept_mean = float(np.mean([y for _, y in kept])) if kept else float("nan")
    naive_mean = float(np.mean([r["thick"] for r in rows]))

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 5), sharey=True)

    # --- Left: all candidate samples, colored by rejection reason ---
    for reason, pts in by_reason.items():
        color, label = REASON_COLORS.get(reason, ("#7f7f7f", reason or "kept"))
        xs = [x for x, _ in pts]
        ys = [y for _, y in pts]
        axL.scatter(xs, ys, s=4, c=color, alpha=0.5,
                    label=f"{label}  (n={len(pts):,})")
    axL.axhline(naive_mean, color="black", linestyle=":", linewidth=1.2,
                label=f"naive mean = {naive_mean:.3f} m")
    axL.set_xlabel("time since session start (s)")
    axL.set_ylabel("ice thickness (m)")
    axL.set_title(f"Before filtering — gp {args.gp}  ({n_total:,} samples)")
    axL.legend(loc="lower right", fontsize=9, markerscale=2.5, framealpha=0.9)
    axL.grid(alpha=0.3)

    # --- Right: only kept samples ---
    if kept:
        xs = [x for x, _ in kept]
        ys = [y for _, y in kept]
        axR.scatter(xs, ys, s=4, c=REASON_COLORS[""][0], alpha=0.6,
                    label=f"kept  (n={len(kept):,})")
    axR.axhline(kept_mean, color="black", linestyle="--", linewidth=1.4,
                label=f"filtered mean = {kept_mean:.3f} m")
    axR.set_xlabel("time since session start (s)")
    axR.set_title(f"After filtering — gp {args.gp}  "
                  f"({len(kept):,} kept, {rejected_total:,} dropped)")
    axR.legend(loc="lower right", fontsize=9, markerscale=2.5, framealpha=0.9)
    axR.grid(alpha=0.3)

    shift = kept_mean - naive_mean
    fig.suptitle(
        f"Depth-stability filter at gridpoint {args.gp}: "
        f"mean thickness shifts {shift*100:+.1f} cm "
        f"({naive_mean:.3f} m → {kept_mean:.3f} m)",
        fontsize=12,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    os.makedirs(args.out, exist_ok=True)
    out_path = os.path.join(args.out, f"filtering_comparison_gp{args.gp}.png")
    fig.savefig(out_path, dpi=150)
    print(f"wrote {out_path}")
    print(f"  total={n_total}  kept={len(kept)}  rejected={rejected_total}")
    for reason in sorted(by_reason):
        print(f"  reason={reason!r:22s}  n={len(by_reason[reason]):>6,}")
    print(f"  naive mean    : {naive_mean:.4f} m")
    print(f"  filtered mean : {kept_mean:.4f} m   (Δ = {shift*100:+.2f} cm)")


if __name__ == "__main__":
    main()
