#!/usr/bin/env python3
"""Horizontal positional drift for a single grid-point session.

Plots per-sample distance from the assigned grid-point target over time,
parallel to plot_thickness_drift.py.
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CSV_DEFAULT = (Path(__file__).resolve().parent.parent
               / "zermatt_results" / "measurements_unfiltered.csv")
AV_DEFAULT  = (Path(__file__).resolve().parent.parent
               / "zermatt_results" / "measurements_av.csv")
OUT_DEFAULT = (Path(__file__).resolve().parent.parent
               / "zermatt_results" / "plots" / "positional_drift_gp{gp}.png")

R_EARTH = 6_371_000.0


def latlon_to_meters(lat, lon, lat0, lon0):
    """Local flat-earth east/north (m) from (lat0, lon0)."""
    dlat = np.radians(lat - lat0)
    dlon = np.radians(lon - lon0)
    north = dlat * R_EARTH
    east  = dlon * R_EARTH * np.cos(np.radians(lat0))
    return east, north


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gp", type=int, default=1)
    ap.add_argument("--unfiltered", default=str(CSV_DEFAULT))
    ap.add_argument("--av", default=str(AV_DEFAULT))
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    out_path = args.out or str(OUT_DEFAULT).format(gp=args.gp)

    df = pd.read_csv(args.unfiltered)
    df = df[df["grid_point_id"] == args.gp].copy()
    df = df.sort_values("timestamp_s").reset_index(drop=True)
    if df.empty:
        raise SystemExit(f"no rows for gp={args.gp}")

    av = pd.read_csv(args.av)
    target = av[av["grid_point_id"] == args.gp].iloc[0]
    lat0 = float(target["grid_point_lat"])
    lon0 = float(target["grid_point_lon"])

    east, north = latlon_to_meters(df["latitude"].values, df["longitude"].values, lat0, lon0)
    dist = np.hypot(east, north)
    t = df["timestamp_s"].values - df["timestamp_s"].iloc[0]

    fig, ax = plt.subplots(figsize=(10, 4.5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    ax.grid(True, color="#eeeeee", linewidth=0.5, zorder=0)
    ax.spines[:].set_color("#cccccc")
    ax.tick_params(colors="black", labelsize=9)

    ax.scatter(t, dist, s=2, color="#2ca02c", alpha=0.45, zorder=3, rasterized=True,
               label="distance to target")

    lo, hi = np.percentile(dist, 2), np.percentile(dist, 98)
    pad = (hi - lo) * 0.15
    ax.set_ylim(max(0, lo - pad), hi + pad)

    ax.set_xlabel("Time since session start (s)", fontsize=10, color="#333333")
    ax.set_ylabel("Distance to target (m)", fontsize=10, color="#333333")
    ax.set_title(f"Grid point {args.gp} — horizontal drift from target",
                 fontsize=11, color="black", loc="left")
    ax.legend(loc="upper right", fontsize=9,
              facecolor="white", edgecolor="#cccccc", labelcolor="black")

    fig.tight_layout()
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight", facecolor="white")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
