#!/usr/bin/env python3
"""AUV trajectory across the ice surface for one grid point.

Single-panel top-down view: window-averaged AUV positions, coloured by the
measured ice thickness in each window. The target grid point is at the
origin. Thickness values are annotated at the first and last windows of
the initial contact session.

Usage:
    python3 plot_horizontal_drift.py [--gp ID] [--window S]
                                     [--unfiltered FILE] [--av FILE]
                                     [--out FILE]
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
               / "zermatt_results" / "plots" / "horizontal_drift_gp{gp}.png")

# Metres per degree at ~46° N. SBL accuracy dominates the horizontal error;
# WGS84 conversion-factor errors are << 1 cm and not worth correcting for.
LAT_M_PER_DEG = 111_000.0
LON_M_PER_DEG = 77_500.0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gp", type=int, default=8)
    ap.add_argument("--window", type=float, default=15.0,
                    help="Averaging window in seconds (default: 15).")
    ap.add_argument("--unfiltered", default=str(CSV_DEFAULT))
    ap.add_argument("--av", default=str(AV_DEFAULT))
    ap.add_argument("--max-t", type=float, default=None,
                    help="Drop samples with t > MAX_T seconds since session start.")
    ap.add_argument("--reassign", action="store_true",
                    help="Reassign each sample to its nearest grid point (using all "
                         "gps in --av) and keep only those whose nearest is --gp. "
                         "Compensates for the extractor's session-level gp assignment.")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    out_path = args.out or str(OUT_DEFAULT).format(gp=args.gp)

    av = pd.read_csv(args.av)
    av_gp = av[av["grid_point_id"] == args.gp]
    if av_gp.empty:
        raise SystemExit(f"no av rows for gp={args.gp}")
    tgt_lat = float(av_gp["grid_point_lat"].iloc[0])
    tgt_lon = float(av_gp["grid_point_lon"].iloc[0])

    df = pd.read_csv(args.unfiltered)
    df["rejection_reason"] = df["rejection_reason"].fillna("")
    df = df[(df["grid_point_id"] == args.gp) & (df["rejection_reason"] == "")].copy()
    df = df.sort_values("timestamp_s").reset_index(drop=True)
    if df.empty:
        raise SystemExit(f"no kept rows for gp={args.gp}")

    df["t"] = df["timestamp_s"] - df["timestamp_s"].iloc[0]
    if args.max_t is not None:
        df = df[df["t"] <= args.max_t].copy()
        if df.empty:
            raise SystemExit(f"no rows survive --max-t {args.max_t}")

    if args.reassign:
        gp_table = (av[["grid_point_id", "grid_point_lat", "grid_point_lon"]]
                    .drop_duplicates("grid_point_id"))
        gp_ids  = gp_table["grid_point_id"].to_numpy()
        gp_lats = gp_table["grid_point_lat"].to_numpy()
        gp_lons = gp_table["grid_point_lon"].to_numpy()
        sample_lat = df["latitude"].to_numpy()[:, None]
        sample_lon = df["longitude"].to_numpy()[:, None]
        dn = (sample_lat - gp_lats[None, :]) * LAT_M_PER_DEG
        de = (sample_lon - gp_lons[None, :]) * LON_M_PER_DEG
        d2 = dn * dn + de * de
        nearest = gp_ids[d2.argmin(axis=1)]
        before = len(df)
        df = df.loc[nearest == args.gp].copy()
        print(f"reassign: kept {len(df)}/{before} samples whose nearest gp is {args.gp}")
        if df.empty:
            raise SystemExit("no samples survive --reassign filter")

    df["east_m"]  = (df["longitude"] - tgt_lon) * LON_M_PER_DEG
    df["north_m"] = (df["latitude"]  - tgt_lat) * LAT_M_PER_DEG

    df["bin"] = (df["t"] // args.window).astype(int)
    agg = df.groupby("bin").agg(
        t       =("t",                "mean"),
        east_m  =("east_m",           "mean"),
        north_m =("north_m",          "mean"),
        thick_m =("ice_thickness_m",  "mean"),
    ).reset_index(drop=True)

    fig, ax = plt.subplots(figsize=(8.5, 7))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    ax.grid(True, color="#eeeeee", linewidth=0.5, zorder=0)
    ax.spines[:].set_color("#cccccc")
    ax.tick_params(colors="black", labelsize=9)

    # Faint arrows joining consecutive windows
    for i in range(len(agg) - 1):
        ax.annotate(
            "",
            xy=(agg["east_m"].iloc[i+1], agg["north_m"].iloc[i+1]),
            xytext=(agg["east_m"].iloc[i],   agg["north_m"].iloc[i]),
            arrowprops=dict(arrowstyle="->", color="#bbbbbb", lw=0.9, alpha=0.7),
            zorder=2,
        )

    sc = ax.scatter(
        agg["east_m"], agg["north_m"],
        c=agg["thick_m"] * 100,
        cmap="plasma",
        s=110, edgecolor="black", linewidth=0.6,
        zorder=4,
    )

    # Target marker + inline "target" label
    ax.scatter(0, 0, s=220, marker="x", color="black", linewidth=2.0, zorder=5)
    span = float(max(agg["east_m"].max() - agg["east_m"].min(),
                     agg["north_m"].max() - agg["north_m"].min(), 1.0))
    ax.text(0 + 0.03 * span, 0 + 0.03 * span, "target",
            color="black", fontsize=14, fontweight="bold",
            ha="left", va="bottom", zorder=6)

    # Thickness annotation on the topmost (highest-north) window
    if len(agg) >= 1:
        top = agg.loc[agg["north_m"].idxmax()]
        ax.annotate(
            f"{top['thick_m']*100:.2f} cm",
            xy=(top["east_m"], top["north_m"]),
            xytext=(top["east_m"] + 0.10 * span, top["north_m"] - 0.06 * span),
            fontsize=10, color="black",
            ha="left", va="center",
            arrowprops=dict(arrowstyle="-", color="#888888", lw=0.8),
            bbox=dict(boxstyle="round,pad=0.3", fc="white",
                      ec="#cccccc", alpha=0.95),
            zorder=7,
        )

    # Thickness annotation on the second window (matches the previous version)
    if len(agg) >= 2:
        second = agg.iloc[1]
        ax.annotate(
            f"{second['thick_m']*100:.2f} cm",
            xy=(second["east_m"], second["north_m"]),
            xytext=(second["east_m"] - 0.12 * span, second["north_m"]),
            fontsize=10, color="black",
            ha="right", va="center",
            arrowprops=dict(arrowstyle="-", color="#888888", lw=0.8),
            bbox=dict(boxstyle="round,pad=0.3", fc="white",
                      ec="#cccccc", alpha=0.95),
            zorder=7,
        )

    cbar = fig.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)
    cbar.set_label("Ice thickness (cm)", color="black", fontsize=10)
    cbar.ax.yaxis.set_tick_params(color="black")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="black")

    ax.set_xlabel("East of target (m)", color="#333333", fontsize=10)
    ax.set_ylabel("North of target (m)", color="#333333", fontsize=10)
    ax.set_title(
        f"Grid point {args.gp} — AUV trajectory across the ice surface",
        color="black", fontsize=12, fontweight="bold", loc="left", pad=10,
    )
    ax.set_aspect("equal", adjustable="datalim")

    fig.tight_layout()
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight", facecolor="white")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
