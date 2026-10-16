#!/usr/bin/env python3
"""
Side-by-side comparison of two Q-tuning campaigns.

Reads two `diagnostics_summary.json` files (old vs new) produced by
`scripts/ekf_residual_diagnostics.py` and emits:

  q_tuning_comparison.md       per-axis NIS_med delta table, verdict
                                 transitions, Q diagonal diff, cross-axis
                                 regression check.
  qfig3_comparison.png          per-bar batch consistency, old vs new.
  qfig7_comparison.png          axis x regime dashboard, two grids
                                 side-by-side.
  qfig5_comparison.png          per-regime boxplots, old (light) under
                                 new (saturated).

Usage:
  python scripts/q_tuning_compare.py <old_diagnostics_dir> <new_diagnostics_dir>
      [--output-dir DIR] [--axes csv] [--label-old LABEL] [--label-new LABEL]

`<old_diagnostics_dir>` and `<new_diagnostics_dir>` are folders containing
`diagnostics_summary.json` (e.g. `<campaign>/ekf_residual_analysis_good_bags/diagnostics_posterior`).

The script reads both summaries and produces a directional view: how does
each axis move from old → new? Useful answers: "did NIS bands move toward
green?", "did any previously-passing axis get pushed out of band?".
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patheffects as pe
    from matplotlib.patches import Patch, Rectangle
except ImportError:
    print("Install: pip install matplotlib", file=sys.stderr)
    raise

try:
    from scipy.stats import chi2
except ImportError:
    print("Install: pip install scipy", file=sys.stderr)
    raise


# ───────────────────────────────────────── theme + tokens
# Same light theme as q_tuning_figures.py.

plt.rcParams.update({
    "font.family":       "DejaVu Sans",
    "font.size":         9,
    "axes.titlesize":    10,
    "axes.labelsize":    9,
    "axes.linewidth":    0.8,
    "axes.spines.top":   False,
    "axes.spines.right": False,
    "axes.grid":         True,
    "grid.color":        "#dddddd",
    "grid.linewidth":    0.5,
    "xtick.labelsize":   8,
    "ytick.labelsize":   8,
    "xtick.direction":   "in",
    "ytick.direction":   "in",
    "legend.fontsize":   8,
    "legend.frameon":    True,
    "legend.framealpha": 0.9,
    "legend.edgecolor":  "#bbbbbb",
    "lines.linewidth":   1.4,
    "savefig.dpi":       300,
    "savefig.bbox":      "tight",
    "figure.facecolor":  "white",
    "axes.facecolor":    "white",
    "axes.edgecolor":    "black",
    "axes.labelcolor":   "black",
    "xtick.color":       "black",
    "ytick.color":       "black",
    "text.color":        "black",
    "legend.facecolor":  "white",
})

C_OLD = "#9aa3ad"   # neutral grey for old run
C_NEW = "#1f5fa3"   # blue for new run
C_BAD = "#c0392b"
C_OK  = "#1ea08a"
C_CHI = "#666666"

CHI2_LO  = float(chi2.ppf(0.025, 1))
CHI2_MED = float(chi2.median(1))
CHI2_HI  = float(chi2.ppf(0.975, 1))

ALL_AXES = (
    "pressure.z",
    "dvl.vx", "dvl.vy", "dvl.vz",
    "imu.roll", "imu.pitch", "imu.yaw",
    "imu.omega_x", "imu.omega_y", "imu.omega_z",
)
REGIMES_PLOT = ("idle", "surge", "reverse", "heave", "yaw", "step", "steady")

Q_LABELS = ["x", "y", "z", "roll", "pitch", "yaw",
            "vx", "vy", "vz", "ωx", "ωy", "ωz", "ax", "ay", "az"]


# ───────────────────────────────────────── helpers

def _load_summary(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text())


def _collect_field(summary: dict, axis: str, regime: str,
                   field: str) -> list[float]:
    out: list[float] = []
    for _, bag in summary.get("bags", {}).items():
        ax = bag.get("axes", {}).get(axis)
        if not ax:
            continue
        rg = ax.get(regime) if isinstance(ax.get(regime), dict) else None
        if rg is None:
            continue
        v = rg.get(field)
        if v is None or not isinstance(v, (int, float)):
            continue
        if not math.isfinite(v):
            continue
        out.append(float(v))
    return out


def _median_safe(values: list[float]) -> float:
    if not values:
        return float("nan")
    return float(np.median(values))


def _per_bar_chi2_band(n: int) -> tuple[float, float]:
    if n <= 0:
        return float("nan"), float("nan")
    return float(chi2.ppf(0.025, n) / n), float(chi2.ppf(0.975, n) / n)


def _bucket_for_axis(nis_med: float, n_med: int,
                     ratio: float | None = None) -> str:
    """Light reproduction of the verdict tree from
    Q_TUNING_IMPROVEMENT_PROCEDURE.md §2.3, scoped to what we can
    determine from a single diagnostics_summary.json."""
    if not math.isfinite(nis_med) or n_med == 0:
        return "no data"
    band_lo, band_hi = _per_bar_chi2_band(n_med)
    if math.isfinite(ratio if ratio is not None else float("nan")) \
            and ratio is not None and ratio >= 5.0:
        return "held back (collapse)"
    if band_lo <= nis_med <= band_hi:
        return "in band"
    if nis_med > band_hi:
        return "high"
    return "low"


# ───────────────────────────────────────── markdown report

def write_comparison_md(out_md: Path,
                        old_summary: dict, new_summary: dict,
                        label_old: str, label_new: str,
                        axes: list[str]) -> None:
    L: list[str] = []
    L.append(f"# Q-tuning comparison — {label_old} vs {label_new}")
    L.append("")
    L.append(f"- old: `{old_summary.get('bags_root', '<unknown>')}` "
             f"(n_bags={old_summary.get('n_bags_processed', '?')}, "
             f"mode=`{old_summary.get('residual_mode', '?')}`)")
    L.append(f"- new: `{new_summary.get('bags_root', '<unknown>')}` "
             f"(n_bags={new_summary.get('n_bags_processed', '?')}, "
             f"mode=`{new_summary.get('residual_mode', '?')}`)")
    L.append("")

    # Q diagonal diff.
    q_old = old_summary.get("Q_diagonal_ref") or []
    q_new = new_summary.get("Q_diagonal_ref") or []
    if q_old and q_new and len(q_old) == len(q_new):
        L.append("## Q diagonal diff (only entries that changed)")
        L.append("")
        L.append("| i | state | Q old | Q new | factor |")
        L.append("|---|---|---:|---:|---:|")
        any_change = False
        for i, lab in enumerate(Q_LABELS):
            v_old = float(q_old[i])
            v_new = float(q_new[i])
            if v_old == v_new:
                continue
            any_change = True
            factor = (v_new / v_old) if v_old != 0 else float("inf")
            L.append(f"| {i} | {lab} | {v_old:g} | {v_new:g} | "
                     f"{factor:.2f}× |")
        if not any_change:
            L.append("| — | — | (no Q entries changed) | — | — |")
        L.append("")

    # Per-axis NIS_med + bucket table.
    L.append("## Per-axis NIS_median  (median across bags, all-regime)")
    L.append("")
    L.append("| axis | NIS_med (old) | NIS_med (new) | delta | "
             "bucket (old) | bucket (new) | transition |")
    L.append("|---|---:|---:|---:|---|---|---|")
    transitions: dict[str, tuple[str, str]] = {}
    for axis in axes:
        nm_old = _collect_field(old_summary, axis, "all", "nis_median")
        nm_new = _collect_field(new_summary, axis, "all", "nis_median")
        n_old = _collect_field(old_summary, axis, "all", "n")
        n_new = _collect_field(new_summary, axis, "all", "n")
        med_old = _median_safe(nm_old)
        med_new = _median_safe(nm_new)
        nmed_old = int(_median_safe(n_old)) if n_old else 0
        nmed_new = int(_median_safe(n_new)) if n_new else 0
        b_old = _bucket_for_axis(med_old, nmed_old)
        b_new = _bucket_for_axis(med_new, nmed_new)
        transitions[axis] = (b_old, b_new)
        if not math.isfinite(med_old) or not math.isfinite(med_new):
            delta = "—"
        elif med_old == 0:
            delta = "∞"
        else:
            delta = f"{(med_new / med_old):.2g}×"
        nm_old_s = "—" if not math.isfinite(med_old) else f"{med_old:.3g}"
        nm_new_s = "—" if not math.isfinite(med_new) else f"{med_new:.3g}"
        arrow = b_old if b_old == b_new else f"{b_old} → {b_new}"
        L.append(f"| `{axis}` | {nm_old_s} | {nm_new_s} | {delta} | "
                 f"{b_old} | {b_new} | {arrow} |")
    L.append("")

    # Cross-axis regression check.
    L.append("## Cross-axis regression check")
    L.append("")
    regressions: list[str] = []
    for axis, (b_old, b_new) in transitions.items():
        if b_old == "in band" and b_new in ("high", "low"):
            regressions.append(
                f"- **REGRESSION**: `{axis}` was in band, now {b_new}.")
    if regressions:
        L.extend(regressions)
        L.append("")
        L.append("These axes had a passing bucket under the OLD run and "
                 "are out of band under the NEW one. The new Q is "
                 "coupling into them via EKF dynamics. Bisect the Q "
                 "change halfway back before any hardware deployment "
                 "(per Q_TUNING_IMPROVEMENT_PROCEDURE.md §4 step 5).")
    else:
        L.append("No regressions: no axis that was in band under the old "
                 "run was pushed out of band by the new one.")
    L.append("")

    # Improvements (toward band).
    L.append("## Improvements")
    L.append("")
    improvements: list[str] = []
    for axis, (b_old, b_new) in transitions.items():
        if b_old in ("high", "low") and b_new == "in band":
            improvements.append(
                f"- `{axis}`: {b_old} → in band.")
        elif b_old == "high" and b_new == "high":
            nm_old = _median_safe(_collect_field(old_summary, axis, "all", "nis_median"))
            nm_new = _median_safe(_collect_field(new_summary, axis, "all", "nis_median"))
            if math.isfinite(nm_old) and math.isfinite(nm_new) and nm_new < nm_old:
                improvements.append(
                    f"- `{axis}`: NIS_med {nm_old:.3g} → {nm_new:.3g} "
                    "(still high; closer to band).")
        elif b_old == "low" and b_new == "low":
            nm_old = _median_safe(_collect_field(old_summary, axis, "all", "nis_median"))
            nm_new = _median_safe(_collect_field(new_summary, axis, "all", "nis_median"))
            if math.isfinite(nm_old) and math.isfinite(nm_new) and nm_new > nm_old:
                improvements.append(
                    f"- `{axis}`: NIS_med {nm_old:.3g} → {nm_new:.3g} "
                    "(still low; closer to band).")
    if improvements:
        L.extend(improvements)
    else:
        L.append("No clear improvements detected at the bucket-transition "
                 "level. Inspect the qfig3 / qfig5 comparison figures for "
                 "finer-grained movement.")
    L.append("")

    # Exceedance change.
    L.append("## Exceedance change  (P(NIS > 5.024), median across bags)")
    L.append("")
    L.append("| axis | exceed (old) | exceed (new) | delta pp |")
    L.append("|---|---:|---:|---:|")
    for axis in axes:
        ex_old = _median_safe(_collect_field(old_summary, axis, "all",
                                              "exceedance_rate_gt_3p84"))
        ex_new = _median_safe(_collect_field(new_summary, axis, "all",
                                              "exceedance_rate_gt_3p84"))
        if math.isfinite(ex_old) and math.isfinite(ex_new):
            ex_old_s = f"{ex_old*100:.1f}%"
            ex_new_s = f"{ex_new*100:.1f}%"
            dpp = f"{(ex_new - ex_old) * 100:+.1f}pp"
        else:
            ex_old_s = "—"; ex_new_s = "—"; dpp = "—"
        L.append(f"| `{axis}` | {ex_old_s} | {ex_new_s} | {dpp} |")
    L.append("")

    L.append("## Files")
    L.append("")
    L.append("- `qfig3_comparison.png` — per-bar batch consistency, old (grey) and new (blue) overlaid.")
    L.append("- `qfig5_comparison.png` — per-regime NIS boxplots, old (light) under new (saturated).")
    L.append("- `qfig7_comparison.png` — axis × regime health dashboards, two grids side-by-side.")
    L.append("")
    out_md.write_text("\n".join(L), encoding="utf-8")


# ───────────────────────────────────────── qfig3 comparison

def fig3_comparison(old_summary: dict, new_summary: dict,
                    label_old: str, label_new: str,
                    axes: list[str], out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(11.0, 4.4))

    # Aggregate per-axis NIS_med across bags for each summary.
    pos_axis = np.arange(len(axes))
    width = 0.36

    def _summarise(summary: dict, axis: str) -> tuple[float, float, int]:
        nm = _collect_field(summary, axis, "all", "nis_median")
        n_  = _collect_field(summary, axis, "all", "n")
        med = _median_safe(nm)
        return (med, _median_safe(n_) if n_ else float("nan"),
                len(nm))

    olds: list[float] = []
    news: list[float] = []
    band_lo_old: list[float] = []
    band_hi_old: list[float] = []
    band_lo_new: list[float] = []
    band_hi_new: list[float] = []
    for axis in axes:
        m_old, n_old, _ = _summarise(old_summary, axis)
        m_new, n_new, _ = _summarise(new_summary, axis)
        olds.append(m_old)
        news.append(m_new)
        lo_o, hi_o = (_per_bar_chi2_band(int(n_old))
                       if math.isfinite(n_old) else (float("nan"), float("nan")))
        lo_n, hi_n = (_per_bar_chi2_band(int(n_new))
                       if math.isfinite(n_new) else (float("nan"), float("nan")))
        band_lo_old.append(lo_o); band_hi_old.append(hi_o)
        band_lo_new.append(lo_n); band_hi_new.append(hi_n)

    # Per-bar acceptance bands drawn behind the bars.
    for i, axis in enumerate(axes):
        for x0, lo, hi in [(pos_axis[i] - width/2, band_lo_old[i], band_hi_old[i]),
                            (pos_axis[i] + width/2, band_lo_new[i], band_hi_new[i])]:
            if math.isfinite(lo) and math.isfinite(hi):
                ax.add_patch(Rectangle((x0 - width*0.45, lo),
                                        width*0.9, hi - lo,
                                        facecolor="#d4e6f1",
                                        edgecolor="#7aa9d4",
                                        lw=0.4, alpha=0.45, zorder=1))

    # Bars.
    ax.bar(pos_axis - width/2, olds, width=width,
           color=C_OLD, edgecolor="white", alpha=0.85,
           label=label_old, zorder=2)
    ax.bar(pos_axis + width/2, news, width=width,
           color=C_NEW, edgecolor="white", alpha=0.85,
           label=label_new, zorder=2)

    # Highlight transitions with arrows.
    for i, (o, n) in enumerate(zip(olds, news)):
        if math.isfinite(o) and math.isfinite(n) and o > 0 and n > 0:
            color = C_OK if (band_lo_new[i] <= n <= band_hi_new[i]) else (
                    C_BAD if (n > band_hi_new[i] or n < band_lo_new[i]) else C_CHI)
            ax.annotate("", xy=(pos_axis[i] + width/2, n),
                        xytext=(pos_axis[i] - width/2, o),
                        arrowprops=dict(arrowstyle="->",
                                        color=color, alpha=0.6, lw=0.7),
                        zorder=3)

    ax.axhline(1.0, color=C_OK, lw=1.0, label=r"target $\bar{NIS}=1$")
    ax.axhline(CHI2_MED, color=C_CHI, ls=":", lw=0.8,
               label=f"$\\chi^2_1$ median = {CHI2_MED:.2f}")
    ax.set_yscale("log")
    ax.set_ylim(1e-4, 1e3)
    ax.set_xticks(pos_axis)
    ax.set_xticklabels(axes, rotation=20)
    ax.set_ylabel("NIS_median (median across bags)")
    ax.set_title(
        f"Batch NIS consistency — {label_old} vs {label_new}  "
        "(per-bar $\\chi^2(n)/n$ bands shaded)",
        loc="left", pad=4)
    ax.legend(loc="upper left", fontsize=7.5)
    fig.savefig(out_path)
    plt.close(fig)


# ───────────────────────────────────────── qfig5 comparison (boxplots)

def fig5_comparison(old_summary: dict, new_summary: dict,
                    label_old: str, label_new: str,
                    axes: list[str], regimes: list[str],
                    out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(11.0, 4.2))
    width = 0.36

    for j, reg in enumerate(regimes):
        positions = (np.arange(len(axes))
                     + (j - (len(regimes) - 1) / 2) * (width * 1.1))
        old_data: list[np.ndarray] = []
        new_data: list[np.ndarray] = []
        for axis in axes:
            old_per_bag = _collect_field(old_summary, axis, reg, "nis_median")
            new_per_bag = _collect_field(new_summary, axis, reg, "nis_median")
            old_data.append(np.array(old_per_bag) if old_per_bag
                            else np.array([np.nan]))
            new_data.append(np.array(new_per_bag) if new_per_bag
                            else np.array([np.nan]))
        # Old (lighter)
        ax.boxplot(old_data, positions=positions - width/3,
                   widths=width*0.35, showfliers=False, patch_artist=True,
                   medianprops=dict(color="black", lw=0.8),
                   boxprops=dict(facecolor=C_OLD, alpha=0.35,
                                 edgecolor="black", lw=0.4),
                   whiskerprops=dict(color="black", lw=0.4),
                   capprops=dict(color="black", lw=0.4))
        ax.boxplot(new_data, positions=positions + width/3,
                   widths=width*0.35, showfliers=False, patch_artist=True,
                   medianprops=dict(color="black", lw=0.9),
                   boxprops=dict(facecolor=C_NEW, alpha=0.7,
                                 edgecolor="black", lw=0.5),
                   whiskerprops=dict(color="black", lw=0.5),
                   capprops=dict(color="black", lw=0.5))

    ax.axhspan(CHI2_LO, CHI2_HI, color="#d4e6f1", alpha=0.6, zorder=0,
               label=r"$\chi^2_1$ 95% band")
    ax.axhline(CHI2_MED, color=C_CHI, ls=":", lw=0.9,
               label=f"$\\chi^2_1$ median = {CHI2_MED:.2f}")
    ax.set_yscale("log")
    ax.set_xticks(np.arange(len(axes)))
    ax.set_xticklabels(axes, rotation=20)
    ax.set_ylabel("NIS_median (per-bag)")
    ax.set_title(
        f"Per-regime NIS  —  {label_old} (light) vs {label_new} "
        "(saturated)  — boxes per regime",
        loc="left", pad=4)
    legend_handles = [Patch(facecolor=C_OLD, alpha=0.4, label=label_old),
                      Patch(facecolor=C_NEW, alpha=0.7, label=label_new)]
    band_h, band_l = ax.get_legend_handles_labels()
    ax.legend(handles=legend_handles + band_h,
              labels=[label_old, label_new] + band_l,
              loc="upper left", fontsize=7.5, ncol=2)
    fig.savefig(out_path)
    plt.close(fig)


# ───────────────────────────────────────── qfig7 comparison (dashboards)

def fig7_comparison(old_summary: dict, new_summary: dict,
                    label_old: str, label_new: str,
                    axes: list[str], regimes: list[str],
                    out_path: Path) -> None:

    def _grid(summary: dict) -> np.ndarray:
        g = np.full((len(axes), len(regimes)), np.nan)
        for i, axis in enumerate(axes):
            for j, reg in enumerate(regimes):
                vals = _collect_field(summary, axis, reg, "nis_median")
                if vals:
                    g[i, j] = float(np.median(vals))
        return g

    g_old = _grid(old_summary)
    g_new = _grid(new_summary)

    fig, axs = plt.subplots(1, 2, figsize=(13.5, 4.4))
    for ax, grid, title in zip(axs, [g_old, g_new],
                                [label_old, label_new]):
        log_grid = np.log10(np.where(grid > 0, grid, np.nan))
        score = log_grid - np.log10(CHI2_MED)
        im = ax.imshow(score, cmap="RdBu_r", vmin=-3, vmax=3, aspect="auto")
        for i in range(len(axes)):
            for j in range(len(regimes)):
                v = grid[i, j]
                if not math.isfinite(v):
                    ax.add_patch(Rectangle((j - 0.5, i - 0.5), 1, 1,
                                            facecolor="#eeeeee",
                                            edgecolor="white",
                                            hatch="////", lw=0.5,
                                            alpha=0.9))
                    ax.text(j, i, "n=0", ha="center", va="center",
                            fontsize=6.5, color="#666666")
                    continue
                if v < 0.001:
                    txt = f"{v:.0e}"
                elif v < 1:
                    txt = f"{v:.2f}"
                elif v < 100:
                    txt = f"{v:.1f}"
                else:
                    txt = f"{v:.0f}"
                ax.text(j, i, txt, ha="center", va="center", fontsize=6.8,
                        color="black",
                        path_effects=[pe.Stroke(linewidth=1.3,
                                                foreground="white"),
                                      pe.Normal()])
        ax.set_xticks(range(len(regimes)))
        ax.set_xticklabels(regimes, rotation=20)
        ax.set_yticks(range(len(axes)))
        ax.set_yticklabels(axes)
        ax.set_title(title, loc="left", pad=4)
        fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02)

    fig.suptitle(
        "Axis × regime NIS dashboards  —  "
        f"{label_old} (left)  vs  {label_new} (right)",
        x=0.02, ha="left", fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path)
    plt.close(fig)


# ───────────────────────────────────────── CLI

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("old_dir", type=Path,
                    help="Old diagnostics_<mode>/ folder (baseline).")
    ap.add_argument("new_dir", type=Path,
                    help="New diagnostics_<mode>/ folder (post-Q-change).")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Default: <new_dir>/../../q_tuning_comparison/")
    ap.add_argument("--axes", default=None,
                    help="Comma list of axes (default: all 10)")
    ap.add_argument("--label-old", default="old",
                    help="Label for the old run in figures/markdown.")
    ap.add_argument("--label-new", default="new",
                    help="Label for the new run in figures/markdown.")
    args = ap.parse_args(argv)

    old_dir = args.old_dir.resolve()
    new_dir = args.new_dir.resolve()
    old_summary = _load_summary(old_dir / "diagnostics_summary.json")
    new_summary = _load_summary(new_dir / "diagnostics_summary.json")

    axes = ([a.strip() for a in args.axes.split(",") if a.strip()]
            if args.axes else list(ALL_AXES))

    # Default output dir: a sibling of the new diagnostics dir at the
    # campaign level.
    if args.output_dir is None:
        # new_dir is typically  <campaign>/ekf_residual_analysis_*/diagnostics_<mode>
        # We climb two levels and write q_tuning_comparison there.
        camp_root = new_dir.parent.parent
        out_dir = camp_root / "q_tuning_comparison"
    else:
        out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    label_old = args.label_old
    label_new = args.label_new

    print(f"old: {old_dir}  (label='{label_old}')")
    print(f"new: {new_dir}  (label='{label_new}')")
    print(f"out: {out_dir}")
    print(f"axes: {axes}")

    print("\n== qfig3_comparison ==")
    fig3_comparison(old_summary, new_summary, label_old, label_new,
                    axes, out_dir / "qfig3_comparison.png")

    print("== qfig5_comparison ==")
    fig5_comparison(old_summary, new_summary, label_old, label_new,
                    axes, list(REGIMES_PLOT),
                    out_dir / "qfig5_comparison.png")

    print("== qfig7_comparison ==")
    fig7_comparison(old_summary, new_summary, label_old, label_new,
                    axes, list(REGIMES_PLOT),
                    out_dir / "qfig7_comparison.png")

    md_path = out_dir / "q_tuning_comparison.md"
    write_comparison_md(md_path, old_summary, new_summary,
                        label_old, label_new, axes)
    print(f"\nWrote markdown: {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
