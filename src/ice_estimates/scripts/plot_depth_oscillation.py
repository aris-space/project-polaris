#!/usr/bin/env python3
"""
Visualize depth-hold oscillation and the accuracy gain from filtering it.

Reads measurements_unfiltered.csv (produced by extract_zermatt_measurements.py)
and renders a single 2x1 figure for one touch session:

  Top:    attitude-corrected contact depth over time, with the rolling 30 s
          minimum and the +1 cm "keep band" drawn explicitly. The depth-
          stability filter rejects any sample whose corrected depth lies
          above this band — making the filter rule visually obvious.

  Bottom: ice thickness over time, samples coloured the same way, with the
          unfiltered and filtered means drawn as lines and σ-before / σ-after
          shown in the legend.

The point: raw pressure-sensor depth is confused by pitch, so depth-hold
oscillations are hard to see on a depth-vs-time plot. The attitude-corrected
contact depth removes that confounder, so an off-ice bob looks like an
off-ice bob, and the filter's behaviour is easy to read.

Usage:
    python3 plot_depth_oscillation.py [--unfiltered CSV] [--out FILE]
                                       [--bag BAG] [--gp GP] [--window-s S]
                                       [--threshold-m M] [--config FILE]
                                       [--pressure-to-contact-z-m Z]
                                       [--pressure-to-contact-x-m X]

  --bag / --gp        Force a specific session. If omitted, the session with
                      the most depth_oscillation rejections is auto-picked.
  --window-s          Rolling window for the local-min reference. Must match
                      the value used in extract_zermatt_measurements.py for
                      the band shown to mirror the filter (default: 30).
  --threshold-m       Band height above the rolling min (default: 0.01 m).
"""

import argparse
import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


DEFAULT_UNFILTERED   = "/ros2_ws/measurements/measurements_unfiltered.csv"
DEFAULT_WINDOW_S     = 30.0
DEFAULT_THRESHOLD_M  = 0.01
DEFAULT_P2C_Z_M      = 0.210
DEFAULT_P2C_X_M      = 0.515

COLOR_KEPT   = "#1f77b4"   # blue
COLOR_OSC    = "#8e44ad"   # purple
COLOR_OTHER  = "#bdc3c7"   # grey (attitude-rejected)
COLOR_RMIN   = "#2c3e50"   # dark grey
COLOR_BAND   = "#27ae60"   # green


def corrected_depth(df, p2c_z_m, p2c_x_m):
    """depth_sensor - omega_corr, in metres. Invariant to pitch/roll about the contact point."""
    pitch = np.radians(df["pitch_deg"].values)
    roll  = np.radians(df["roll_deg"].values)
    omega = p2c_x_m * np.sin(pitch) + p2c_z_m * np.cos(pitch) * np.cos(roll)
    return df["depth_m"].values - omega


def pick_session(df):
    osc = df[df["rejection_reason"] == "depth_oscillation"]
    if osc.empty:
        sys.exit("No depth_oscillation rejections in CSV — nothing to show.")
    counts = osc.groupby(["bag", "grid_point_id"]).size().sort_values(ascending=False)
    return counts.index[0]


def load_offsets(cfg_path, z_arg, x_arg):
    z, x = DEFAULT_P2C_Z_M, DEFAULT_P2C_X_M
    if cfg_path:
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f) or {}
        z = float(cfg.get("pressure_to_contact_z_m", z))
        x = float(cfg.get("pressure_to_contact_x_m", x))
    if z_arg is not None:
        z = z_arg
    if x_arg is not None:
        x = x_arg
    return z, x


def rolling_min_at(query_t, ref_t, ref_v, half_window_s):
    """Centred rolling minimum of ref_v over a ±half window, evaluated at query_t."""
    out = np.empty_like(query_t, dtype=float)
    for i, ti in enumerate(query_t):
        lo = np.searchsorted(ref_t, ti - half_window_s)
        hi = np.searchsorted(ref_t, ti + half_window_s, side="right")
        out[i] = ref_v[lo:hi].min() if hi > lo else np.nan
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--unfiltered", default=DEFAULT_UNFILTERED,
                    help=f"Path to measurements_unfiltered.csv (default: {DEFAULT_UNFILTERED})")
    ap.add_argument("--out", default=None,
                    help="Output PNG (default: depth_oscillation_gp<GP>.png alongside this script)")
    ap.add_argument("--bag", default=None,
                    help="Bag basename to plot. Auto-picked if omitted.")
    ap.add_argument("--gp", type=int, default=None,
                    help="Grid-point id to plot. Auto-picked if omitted.")
    ap.add_argument("--window-s", type=float, default=DEFAULT_WINDOW_S,
                    help=f"Rolling-min window (s) for the keep band (default: {DEFAULT_WINDOW_S})")
    ap.add_argument("--threshold-m", type=float, default=DEFAULT_THRESHOLD_M,
                    help=f"Band height above rolling min (m) (default: {DEFAULT_THRESHOLD_M})")
    ap.add_argument("--config", default=None,
                    help="config.yaml with pressure_to_contact_{z,x}_m")
    ap.add_argument("--pressure-to-contact-z-m", type=float, default=None,
                    help=f"Override body-z offset (m). Default: {DEFAULT_P2C_Z_M}")
    ap.add_argument("--pressure-to-contact-x-m", type=float, default=None,
                    help=f"Override body-x offset (m). Default: {DEFAULT_P2C_X_M}")
    args = ap.parse_args()

    if not os.path.isfile(args.unfiltered):
        sys.exit(f"error: {args.unfiltered} not found")

    df_all = pd.read_csv(args.unfiltered)
    df_all["rejection_reason"] = df_all["rejection_reason"].fillna("")

    if args.bag and args.gp is not None:
        bag, gp = args.bag, args.gp
    else:
        bag, gp = pick_session(df_all)
        print(f"Auto-picked session: bag={bag}  gp={gp}")

    df = df_all[(df_all["bag"] == bag) & (df_all["grid_point_id"] == gp)].copy()
    if df.empty:
        sys.exit(f"No samples for bag={bag} gp={gp}")
    df = df.sort_values("timestamp_s").reset_index(drop=True)

    p2c_z, p2c_x = load_offsets(args.config,
                                args.pressure_to_contact_z_m,
                                args.pressure_to_contact_x_m)
    df["dc_m"] = corrected_depth(df, p2c_z, p2c_x)

    t0 = df["timestamp_s"].iloc[0]
    t  = df["timestamp_s"].values - t0

    kept_mask  = (df["rejection_reason"] == "").values
    osc_mask   = (df["rejection_reason"] == "depth_oscillation").values
    other_mask = ~(kept_mask | osc_mask)        # pitch / roll rejections

    # The rolling-min reference set in the actual filter is kept_idx (pre-osc):
    # i.e. samples that passed attitude. Match that here so the band drawn is
    # the same one the filter compared against.
    ref_mask = ~other_mask
    ref_t = t[ref_mask]
    ref_v = df["dc_m"].values[ref_mask]
    order = np.argsort(ref_t)
    ref_t, ref_v = ref_t[order], ref_v[order]
    rmin = rolling_min_at(t, ref_t, ref_v, args.window_s / 2.0)

    # --- Stats ----------------------------------------------------------
    T          = df["ice_thickness_m"].values
    sigma_unf  = T[ref_mask].std(ddof=0) * 100
    mu_unf     = T[ref_mask].mean()
    sigma_filt = T[kept_mask].std(ddof=0) * 100 if kept_mask.any() else np.nan
    mu_filt    = T[kept_mask].mean() if kept_mask.any() else np.nan
    reduction  = 100 * (1 - sigma_filt / sigma_unf) if sigma_unf > 0 else 0
    delta_mm   = 1000 * (mu_filt - mu_unf) if np.isfinite(mu_filt) else np.nan

    # --- Plot -----------------------------------------------------------
    fig, (ax_d, ax_t) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    # TOP — corrected depth
    ax_d.fill_between(t, rmin, rmin + args.threshold_m,
                      color=COLOR_BAND, alpha=0.18, lw=0,
                      label=f"keep band (rolling min + {args.threshold_m*100:.0f} cm)")
    ax_d.plot(t, rmin, color=COLOR_RMIN, lw=1.0, ls="--",
              label=f"{args.window_s:.0f}s rolling min")
    if other_mask.any():
        ax_d.scatter(t[other_mask], df["dc_m"].values[other_mask], s=10,
                     color=COLOR_OTHER, alpha=0.5, edgecolors="none",
                     label=f"attitude-rejected (n={other_mask.sum()})")
    ax_d.scatter(t[osc_mask], df["dc_m"].values[osc_mask], s=12,
                 color=COLOR_OSC, alpha=0.85, edgecolors="none",
                 label=f"depth_oscillation (n={osc_mask.sum()})")
    ax_d.scatter(t[kept_mask], df["dc_m"].values[kept_mask], s=8,
                 color=COLOR_KEPT, alpha=0.7, edgecolors="none",
                 label=f"kept (n={kept_mask.sum()})")
    ax_d.set_ylabel("attitude-corrected\ncontact depth [m]")
    ax_d.grid(alpha=0.3)
    ax_d.legend(loc="best", fontsize=9, framealpha=0.85)

    # BOTTOM — thickness
    if other_mask.any():
        ax_t.scatter(t[other_mask], T[other_mask], s=10, color=COLOR_OTHER,
                     alpha=0.5, edgecolors="none")
    ax_t.scatter(t[osc_mask], T[osc_mask], s=12, color=COLOR_OSC,
                 alpha=0.85, edgecolors="none")
    ax_t.scatter(t[kept_mask], T[kept_mask], s=8, color=COLOR_KEPT,
                 alpha=0.7, edgecolors="none")
    if np.isfinite(mu_unf):
        ax_t.axhline(mu_unf, color="black", lw=0.9, ls=":", alpha=0.7,
                     label=f"no osc filter:  mean = {mu_unf:.3f} m   σ = {sigma_unf:.2f} cm")
    if np.isfinite(mu_filt):
        ax_t.axhline(mu_filt, color="black", lw=1.4, ls="--", alpha=0.9,
                     label=f"with osc filter: mean = {mu_filt:.3f} m   σ = {sigma_filt:.2f} cm   "
                           f"(−{reduction:.0f}%, Δ {delta_mm:+.1f} mm)")
    ax_t.set_xlabel("time in session [s]")
    ax_t.set_ylabel("ice thickness [m]")
    ax_t.grid(alpha=0.3)
    ax_t.legend(loc="best", fontsize=9, framealpha=0.85)

    fig.suptitle(
        f"Depth-hold oscillation effect — gp{gp}   ({bag[-16:]})\n"
        f"σ {sigma_unf:.2f} cm  →  {sigma_filt:.2f} cm   (−{reduction:.0f}%)",
        fontsize=12, y=0.995,
    )
    fig.tight_layout()

    out = (Path(args.out) if args.out
           else Path(__file__).resolve().parent / f"depth_oscillation_gp{gp}.png")
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")
    print(f"  σ:    unfiltered = {sigma_unf:.2f} cm   filtered = {sigma_filt:.2f} cm   (−{reduction:.0f}%)")
    print(f"  mean: unfiltered = {mu_unf:.3f} m       filtered = {mu_filt:.3f} m       (Δ {delta_mm:+.1f} mm)")


if __name__ == "__main__":
    main()
