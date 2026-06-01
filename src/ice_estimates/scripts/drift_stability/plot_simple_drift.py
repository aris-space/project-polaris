#!/usr/bin/env python3
"""Single-panel plot of one touch session showing the AUV drifting off the ice.

Reads measurements_unfiltered.csv and renders attitude-corrected contact
depth over time for one grid-point session. The rolling local-minimum is
the "AUV is firmly on the ice" reference; samples that sit clearly below
it are moments where Pixhawk depth-hold pulled the AUV down off contact.
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CSV_DEFAULT = "/home/ridhch/project-polaris/src/ice_estimates/zermatt_results/measurements_unfiltered.csv"
OUT_DEFAULT = Path(__file__).with_name("simple_drift.png")

P2C_Z_M = 0.210
P2C_X_M = 0.515
WINDOW_S = 30.0
BAND_M   = 0.01


def corrected_depth(df):
    pitch = np.radians(df["pitch_deg"].values)
    roll  = np.radians(df["roll_deg"].values)
    omega = P2C_X_M * np.sin(pitch) + P2C_Z_M * np.cos(pitch) * np.cos(roll)
    return df["depth_m"].values - omega


def rolling_min(t, v, half):
    out = np.empty_like(v)
    for i, ti in enumerate(t):
        lo = np.searchsorted(t, ti - half)
        hi = np.searchsorted(t, ti + half, side="right")
        out[i] = v[lo:hi].min()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default=CSV_DEFAULT)
    ap.add_argument("--bag", default="zermatt_grid_01_2026_04_30-12_04_16_0.mcap")
    ap.add_argument("--gp", type=int, default=3)
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    df["rejection_reason"] = df["rejection_reason"].fillna("")
    df = df[(df["bag"] == args.bag) & (df["grid_point_id"] == args.gp)].copy()
    df = df.sort_values("timestamp_s").reset_index(drop=True)
    if df.empty:
        raise SystemExit(f"no rows for bag={args.bag} gp={args.gp}")

    t  = df["timestamp_s"].values - df["timestamp_s"].iloc[0]
    dc = corrected_depth(df)
    rmin = rolling_min(t, dc, WINDOW_S / 2.0)
    band_hi = rmin + BAND_M
    drifted = dc > band_hi

    fig, ax = plt.subplots(figsize=(10, 5.2))

    ax.fill_between(t, rmin, band_hi, color="#27ae60", alpha=0.20, lw=0,
                    label=f"on-ice band (rolling min, +{BAND_M*100:.0f} cm)")
    ax.plot(t, rmin, color="#2c3e50", lw=1.0, ls="--", alpha=0.7,
            label=f"{WINDOW_S:.0f}s rolling min (≈ ice surface)")
    ax.plot(t, dc, color="#1f77b4", lw=0.9, alpha=0.75, label="AUV contact depth")
    if drifted.any():
        ax.scatter(t[drifted], dc[drifted], s=10, color="#c0392b",
                   alpha=0.7, edgecolors="none",
                   label=f"drifted off ice ({drifted.sum()}/{len(dc)} samples, "
                         f"{100*drifted.mean():.0f}%)")

    i_peak = int(np.argmax(dc - rmin))
    peak_cm = (dc[i_peak] - rmin[i_peak]) * 100
    ax.annotate(f"AUV pulled {peak_cm:.0f} cm\nbelow ice contact",
                xy=(t[i_peak], dc[i_peak]),
                xytext=(t[i_peak] + 25, dc[i_peak] - 0.005),
                fontsize=9, ha="left", va="center",
                arrowprops=dict(arrowstyle="->", lw=0.8),
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.95))

    ax.invert_yaxis()
    ax.set_xlabel("time within touch session (s)")
    ax.set_ylabel("contact depth (m, downward positive)")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right", fontsize=9, framealpha=0.95)
    ax.set_title(
        f"Depth-hold drift — gp{args.gp}, {args.bag[:18]}\n"
        "Pixhawk setpoint deeper than the ice → AUV repeatedly loses contact",
        fontsize=11, loc="left", pad=8,
    )

    # Headroom so the legend doesn't sit on the data
    ymin, ymax = ax.get_ylim()
    ax.set_ylim(ymin, ymax - (ymin - ymax) * 0.20)

    fig.tight_layout()
    fig.savefig(args.out, dpi=180)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
