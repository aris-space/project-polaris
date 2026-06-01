#!/usr/bin/env python3
"""Slide 5 — recovery via post-processing.

Reads measurements_unfiltered.csv (produced by extract_zermatt_measurements.py)
and writes two PNGs:

  slide5_pitch.png         high-pitch event (gp2 in zermatt_grid_01) before/after
  slide5_oscillation.png   Pixhawk-hold oscillation, auto-picked from the data

Each PNG is a 1x2 panel: left = raw (kept + rejected samples), right = filtered
(kept only). Standard deviation of ice_thickness_m on each side is annotated and
the percentage reduction in σ is printed both on the plot and to stdout.

Usage:
    python3 plot_recovery_slide5.py [--unfiltered CSV] [--out-dir DIR]
                                    [--oscillation-bag BAG]
                                    [--oscillation-gp GP]
                                    [--show-context]

  --unfiltered       Path to measurements_unfiltered.csv (default:
                     /ros2_ws/measurements/measurements_unfiltered.csv).
  --out-dir          Where to drop the PNGs (default: alongside this script).
  --oscillation-bag  Force the oscillation example to this bag name.
  --oscillation-gp   Force the oscillation example to this grid point id.
  --show-context     Add a thin secondary subplot underneath each panel showing
                     pitch (pitch event) or depth (oscillation) over time.
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


DEFAULT_UNFILTERED = "/ros2_ws/measurements/measurements_unfiltered.csv"

# Fixed for the pitch event — the only session with notable attitude rejections.
PITCH_BAG = "zermatt_grid_01_2026_04_30-12_04_16_0.mcap"
PITCH_GP = 2

COLOR_KEPT      = "#1f77b4"   # blue
COLOR_PITCH     = "#c0392b"   # red
COLOR_ROLL      = "#e67e22"   # orange
COLOR_OSC       = "#8e44ad"   # purple
COLOR_CONTEXT   = "#7f8c8d"   # grey


def pick_oscillation_session(df):
    """Return (bag, gp) with the most depth_oscillation rejections.

    Ties are broken by the absolute count of rejected samples — i.e. the
    most visually oscillatory session.
    """
    osc = df[df["rejection_reason"] == "depth_oscillation"]
    if osc.empty:
        return None
    counts = osc.groupby(["bag", "grid_point_id"]).size().sort_values(ascending=False)
    return counts.index[0]


def session_stats(df_session):
    """σ before and after filtering, and per-reason counts."""
    kept = df_session["rejection_reason"] == ""
    sigma_raw_cm  = df_session["ice_thickness_m"].std(ddof=0) * 100
    sigma_filt_cm = df_session.loc[kept, "ice_thickness_m"].std(ddof=0) * 100
    counts = df_session["rejection_reason"].value_counts(dropna=False).to_dict()
    return {
        "sigma_raw_cm":   sigma_raw_cm,
        "sigma_filt_cm":  sigma_filt_cm,
        "reduction_pct":  100 * (1 - sigma_filt_cm / sigma_raw_cm) if sigma_raw_cm > 0 else 0,
        "n_total":        len(df_session),
        "n_kept":         int(kept.sum()),
        "mean_filt_m":    float(df_session.loc[kept, "ice_thickness_m"].mean()) if kept.any() else float("nan"),
        "counts":         counts,
    }


def plot_session(df_session, title, out_path, context_col=None, context_label=None):
    """Render a 1x2 (or 2x2 with context) before/after panel for one session."""
    df = df_session.sort_values("timestamp_s").reset_index(drop=True)
    t0 = df["timestamp_s"].iloc[0]
    t = df["timestamp_s"].values - t0

    stats = session_stats(df)
    kept_mask = (df["rejection_reason"] == "").values

    # Y axis range — same on both panels so before/after is a fair compare
    y = df["ice_thickness_m"].values
    y_kept = y[kept_mask]
    y_lo = min(y.min(), y_kept.min() if y_kept.size else y.min())
    y_hi = max(y.max(), y_kept.max() if y_kept.size else y.max())
    pad = max(0.02, 0.05 * (y_hi - y_lo))
    y_lim = (y_lo - pad, y_hi + pad)

    with_context = context_col is not None and context_col in df.columns
    if with_context:
        fig, axes = plt.subplots(2, 2, figsize=(12, 5.5), sharex="col",
                                 gridspec_kw={"height_ratios": [3, 1]})
        ax_raw, ax_filt = axes[0, 0], axes[0, 1]
        ctx_raw, ctx_filt = axes[1, 0], axes[1, 1]
    else:
        fig, (ax_raw, ax_filt) = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
        ctx_raw = ctx_filt = None

    # --- LEFT: raw -------------------------------------------------------
    # Plot rejection categories so the legend explains "why"
    rej_colors = {
        "pitch":             COLOR_PITCH,
        "roll":              COLOR_ROLL,
        "depth_oscillation": COLOR_OSC,
    }
    for reason, color in rej_colors.items():
        m = (df["rejection_reason"] == reason).values
        if m.any():
            ax_raw.scatter(t[m], y[m], s=10, color=color, alpha=0.7,
                           label=f"{reason} (n={m.sum()})", edgecolors="none")
    ax_raw.scatter(t[kept_mask], y[kept_mask], s=6, color=COLOR_CONTEXT,
                   alpha=0.4, label=f"kept (n={kept_mask.sum()})",
                   edgecolors="none")
    ax_raw.set_ylim(y_lim)
    ax_raw.set_ylabel("ice thickness [m]")
    ax_raw.set_title(f"raw   —   σ = {stats['sigma_raw_cm']:.1f} cm   "
                     f"(n = {stats['n_total']})")
    ax_raw.grid(alpha=0.3)
    ax_raw.legend(loc="best", fontsize=9, framealpha=0.85)

    # --- RIGHT: filtered -------------------------------------------------
    ax_filt.scatter(t[kept_mask], y[kept_mask], s=8, color=COLOR_KEPT,
                    alpha=0.7, edgecolors="none",
                    label=f"kept (n={kept_mask.sum()})")
    if kept_mask.any():
        ax_filt.axhline(stats["mean_filt_m"], color="black", lw=1.0,
                        ls="--", alpha=0.6,
                        label=f"mean = {stats['mean_filt_m']:.3f} m")
    ax_filt.set_ylim(y_lim)
    ax_filt.set_title(f"filtered   —   σ = {stats['sigma_filt_cm']:.2f} cm   "
                      f"(−{stats['reduction_pct']:.0f}%)")
    ax_filt.grid(alpha=0.3)
    ax_filt.legend(loc="best", fontsize=9, framealpha=0.85)
    if not with_context:
        ax_raw.set_xlabel("time in session [s]")
        ax_filt.set_xlabel("time in session [s]")

    # --- Optional context row -------------------------------------------
    if with_context:
        ctx = df[context_col].values
        # Highlight rejected regions visually too
        ctx_raw.plot(t, ctx, lw=0.8, color="#2c3e50")
        if not kept_mask.all():
            ctx_raw.scatter(t[~kept_mask], ctx[~kept_mask], s=8,
                            color=COLOR_PITCH, alpha=0.8, edgecolors="none")
        ctx_raw.set_ylabel(context_label)
        ctx_raw.set_xlabel("time in session [s]")
        ctx_raw.grid(alpha=0.3)

        ctx_filt.plot(t[kept_mask], ctx[kept_mask], lw=0.8, color="#2c3e50")
        ctx_filt.set_xlabel("time in session [s]")
        ctx_filt.grid(alpha=0.3)

    fig.suptitle(title, fontsize=13, y=1.00)
    fig.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return stats


def select_session(df, bag, gp, label):
    sub = df[(df["bag"] == bag) & (df["grid_point_id"] == gp)]
    if sub.empty:
        raise SystemExit(f"{label}: no samples for bag={bag}, gp={gp}")
    return sub


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--unfiltered", default=DEFAULT_UNFILTERED,
                        help=f"Path to measurements_unfiltered.csv (default: {DEFAULT_UNFILTERED})")
    parser.add_argument("--out-dir", default=str(Path(__file__).resolve().parent),
                        help="Directory for output PNGs (default: alongside this script)")
    parser.add_argument("--oscillation-bag", default=None,
                        help="Force the oscillation example to this bag")
    parser.add_argument("--oscillation-gp", type=int, default=None,
                        help="Force the oscillation example to this grid point id")
    parser.add_argument("--show-context", action="store_true",
                        help="Add a context subplot (pitch / depth) under each panel")
    args = parser.parse_args()

    if not os.path.isfile(args.unfiltered):
        print(f"error: {args.unfiltered} not found", file=sys.stderr)
        print("Run extract_zermatt_measurements.py first.", file=sys.stderr)
        sys.exit(1)

    df = pd.read_csv(args.unfiltered)
    df["rejection_reason"] = df["rejection_reason"].fillna("")
    os.makedirs(args.out_dir, exist_ok=True)

    # --- 1. Pitch event ------------------------------------------------
    pitch_df = select_session(df, PITCH_BAG, PITCH_GP, "pitch event")
    pitch_out = os.path.join(args.out_dir, "slide5_pitch.png")
    pitch_stats = plot_session(
        pitch_df,
        title=f"High-pitch event — gp{PITCH_GP}",
        out_path=pitch_out,
        context_col="pitch_deg" if args.show_context else None,
        context_label="pitch [deg]",
    )
    print(f"  wrote {pitch_out}")

    # --- 2. Oscillation ------------------------------------------------
    if args.oscillation_bag and args.oscillation_gp is not None:
        osc_bag, osc_gp = args.oscillation_bag, args.oscillation_gp
    else:
        picked = pick_oscillation_session(df)
        if picked is None:
            print("warning: no depth_oscillation rejections in CSV — "
                  "skipping oscillation plot", file=sys.stderr)
            return
        osc_bag, osc_gp = picked
    osc_df = select_session(df, osc_bag, osc_gp, "oscillation")
    osc_out = os.path.join(args.out_dir, "slide5_oscillation.png")
    osc_stats = plot_session(
        osc_df,
        title=f"Pixhawk-hold oscillation — gp{osc_gp}",
        out_path=osc_out,
        context_col="depth_m" if args.show_context else None,
        context_label="depth [m]",
    )
    print(f"  wrote {osc_out}")

    # --- Summary -------------------------------------------------------
    def _fmt(label, st):
        return (f"  {label:<20}  σ {st['sigma_raw_cm']:5.2f} cm "
                f"→ {st['sigma_filt_cm']:5.2f} cm   (−{st['reduction_pct']:4.1f}%)   "
                f"kept {st['n_kept']}/{st['n_total']}   "
                f"mean {st['mean_filt_m']:.3f} m")

    print()
    print(_fmt(f"pitch  gp{PITCH_GP}", pitch_stats))
    print(_fmt(f"oscil. gp{osc_gp}",   osc_stats))


if __name__ == "__main__":
    main()
