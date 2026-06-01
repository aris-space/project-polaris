#!/usr/bin/env python3
"""Pitch variability during a single touch session (dark theme).

Reads measurements_unfiltered.csv and renders pitch-over-time for one
grid-point session. Picks the session with the largest pitch std by default
— that's the visual example to show how much the AUV's attitude swings
while it's nominally "on the ice".
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CSV_DEFAULT = "/home/ridhch/project-polaris/src/ice_estimates/zermatt_results/measurements_unfiltered.csv"
OUT_DEFAULT = Path(__file__).with_name("pitch_variability.png")

BG       = "#0e1117"
FG       = "#e6edf3"
GRID     = "#30363d"
PITCH    = "#f5b942"
MEAN_C   = "#58a6ff"
BAND_C   = "#58a6ff"
def pick_session(df):
    """Rank sessions by pitch IQR — robust to the one-off detach spike at the
    end of a touch session, which inflates std but isn't representative of
    pitch variability while in contact."""
    g = df.groupby(["bag", "grid_point_id"])
    iqr = g["pitch_deg"].agg(lambda x: np.subtract(*np.percentile(x, [75, 25])))
    return iqr.idxmax()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--csv", default=CSV_DEFAULT)
    ap.add_argument("--bag", default=None)
    ap.add_argument("--gp", type=int, default=None)
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    df["rejection_reason"] = df["rejection_reason"].fillna("")

    if args.bag and args.gp is not None:
        bag, gp = args.bag, args.gp
    else:
        bag, gp = pick_session(df)
        print(f"Auto-picked session: bag={bag}  gp={gp}")

    sub = df[(df["bag"] == bag) & (df["grid_point_id"] == gp)].copy()
    if sub.empty:
        raise SystemExit(f"no rows for bag={bag} gp={gp}")
    sub = sub.sort_values("timestamp_s").reset_index(drop=True)

    t = sub["timestamp_s"].values - sub["timestamp_s"].iloc[0]
    p = sub["pitch_deg"].values

    mu    = float(np.mean(p))
    sigma = float(np.std(p, ddof=0))
    pmin, pmax = float(p.min()), float(p.max())
    prange = pmax - pmin
    iqr   = float(np.subtract(*np.percentile(p, [75, 25])))

    fig, ax = plt.subplots(figsize=(11, 5.4))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)

    ax.axhspan(mu - sigma, mu + sigma, color=BAND_C, alpha=0.15, lw=0,
               label=f"±1σ band  (σ = {sigma:.2f}°)")
    ax.axhline(mu, color=MEAN_C, lw=1.0, ls="--", alpha=0.85,
               label=f"mean = {mu:+.2f}°")

    ax.plot(t, p, color=PITCH, lw=1.1, alpha=0.95, label="pitch")

    i_peak = int(np.argmax(np.abs(p)))
    ax.scatter([t[i_peak]], [p[i_peak]], s=42, facecolor=PITCH,
               edgecolor="white", lw=1.0, zorder=6)
    # Place the annotation toward the centre of the x-axis so it does not
    # clip when the peak sits near a session edge.
    label_dx = (t[-1] - t[i_peak]) * 0.4 if t[i_peak] < t[-1] * 0.7 else -(t[i_peak]) * 0.25
    label_dy = 2.5 if p[i_peak] < 0 else -2.5
    ax.annotate(
        f"peak {p[i_peak]:+.1f}°",
        xy=(t[i_peak], p[i_peak]),
        xytext=(t[i_peak] + label_dx, p[i_peak] + label_dy * 1.5),
        color=FG, fontsize=10,
        ha="center" if label_dx < 0 else "left",
        va="center",
        arrowprops=dict(arrowstyle="->", color=FG, lw=0.9),
        bbox=dict(boxstyle="round,pad=0.35", fc=BG, ec=GRID, lw=0.8),
    )

    pad = max(1.5, 0.12 * prange)
    ax.set_ylim(pmin - pad, pmax + pad)
    ax.set_xlim(0, t[-1])

    ax.set_xlabel("time within touch session (s)", color=FG, fontsize=10)
    ax.set_ylabel("pitch (deg)", color=FG, fontsize=10)
    ax.tick_params(colors=FG, labelsize=9)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.grid(True, color=GRID, lw=0.5, alpha=0.7)
    ax.set_axisbelow(True)

    stats_text = (f"σ = {sigma:.2f}°    "
                  f"IQR = {iqr:.2f}°    "
                  f"range = {prange:.1f}° ({pmin:+.1f}° to {pmax:+.1f}°)    "
                  f"n = {len(p)}")
    ax.set_title(
        f"AUV pitch variability during one touch session — gp{gp}\n"
        f"{stats_text}",
        color=FG, fontsize=11, loc="left", pad=10,
    )

    leg = ax.legend(loc="upper right", fontsize=9, frameon=True,
                    facecolor=BG, edgecolor=GRID, labelcolor=FG)
    leg.get_frame().set_alpha(0.85)

    fig.tight_layout()
    fig.savefig(args.out, dpi=180, facecolor=fig.get_facecolor())
    print(f"wrote {args.out}")
    print(f"  σ={sigma:.2f}°  range={prange:.1f}°  mean={mu:+.2f}°  n={len(p)}")


if __name__ == "__main__":
    main()
