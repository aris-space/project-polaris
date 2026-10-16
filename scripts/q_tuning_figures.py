#!/usr/bin/env python3
"""
Canonical Q-tuning figure generator (qfig1..qfig8) + per-campaign
verdict markdown.

Visual ground truth: POLARIS/research/q_tuning_figure_examples/qfig*.png
and its generator make_q_tuning_examples.py. Theme, palette,
annotations and panel layouts are copied verbatim where possible.

Three correctness rules baked in:
  Rule 1 — Per-bar chi^2(N)/N acceptance bands in qfig3 / qfig8(c);
           never an aggregate band over many bars with different N.
  Rule 2 — Whiteness (qfig4) on standardised innovations nu = r/sqrt(S),
           over the dominant regime per bag. prior_approx residuals
           used when posterior↔prior_approx ratio < 5×; deferred
           (greyed-out) when ratio >= 5×.
  Rule 3 — Monotonic time on every per-sample residual array before
           plotting. Single-bag only for timeline figures; never
           concatenate across bags as a single polyline.

Inputs:
  <diagnostics_dir>/diagnostics_summary.json
  <diagnostics_dir>/<bag>/<bag>_diagnostics_series_<mode>.npz
  recordings/rosbags/<campaign>/<bag>/  (for on-demand recompute)

Output (default <diagnostics_dir>/../q_tuning_figures/):
  qfig3_batch_nis_consistency.png
  qfig4_innovation_whiteness.png
  qfig5_regime_boxplot.png
  qfig6_q_sweep.png                          # placeholder
  qfig7_consistency_dashboard.png
  qfig8_q_tuning_report_panel.png
  q_tuning_summary.md
  pressure_z/qfig1_residual_band.png
  dvl_{vx,vy,vz}/qfig1_residual_band.png + qfig2_nis_distribution.png
  imu_{roll,pitch,yaw}/qfig1_residual_band.png  # paired post+prior

qfig6 (Q-sweep) is a placeholder; needs offline EKF replay (procedure
§8.3).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

# UTF-8 console for Windows.
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
    from matplotlib.gridspec import GridSpec
except ImportError:
    print("Install: pip install matplotlib", file=sys.stderr)
    raise

try:
    from scipy.stats import chi2
except ImportError:
    print("Install: pip install scipy", file=sys.stderr)
    raise

# Make the existing residual machinery importable for on-demand recompute.
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))
import ekf_residual_analysis as era  # noqa: E402
import ekf_residual_diagnostics as erd  # noqa: E402


# ───────────────────────────────────────── theme + tokens
# Verbatim from POLARIS/research/q_tuning_figure_examples/make_q_tuning_examples.py
# (lines 37–80).

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
    # Explicit light theme — override any dark-theme rcParams that
    # `import ekf_residual_analysis` may have applied at module import.
    "figure.facecolor":  "white",
    "axes.facecolor":    "white",
    "axes.edgecolor":    "black",
    "axes.labelcolor":   "black",
    "xtick.color":       "black",
    "ytick.color":       "black",
    "text.color":        "black",
    "legend.facecolor":  "white",
})

C_RES = "#1f5fa3"
C_ENV = "#d6263d"
C_NIS = "#2a8a64"
C_CHI = "#666666"
C_BAD = "#c0392b"
C_OK  = "#1ea08a"
C_REG = {
    "idle":       "#cfd8dc",
    "steady":     "#d4e6f1",
    "step":       "#fbe5d6",
    "pump":       "#e8daef",
    "surge":      "#fadbd8",
    "reverse":    "#f5cba7",
    "heave":      "#d5f5e3",
    "yaw":        "#d6eaf8",
    "roll_pitch": "#fdebd0",
    "mixed":      "#fcf3cf",
    "sway":       "#a78bfa",
}

CHI2_LO  = float(chi2.ppf(0.025, 1))   # 0.000982
CHI2_MED = float(chi2.median(1))       # 0.4549
CHI2_HI  = float(chi2.ppf(0.975, 1))   # 5.024


# ───────────────────────────────────────── axis classes + Q metadata

ALL_AXES = (
    "pressure.z",
    "dvl.vx", "dvl.vy", "dvl.vz",
    "imu.roll", "imu.pitch", "imu.yaw",
    "imu.omega_x", "imu.omega_y", "imu.omega_z",
)
DIAGNOSTIC_AXES   = ("imu.omega_x", "imu.omega_y", "imu.omega_z")
ORIENTATION_AXES  = ("imu.roll", "imu.pitch", "imu.yaw")
TUNE_AXES         = ("dvl.vx", "dvl.vy", "dvl.vz")
CONFIRM_AXES      = ("pressure.z",)
HELD_BACK_AXES    = ORIENTATION_AXES

REGIMES_PLOT = ("idle", "surge", "reverse", "heave", "yaw", "step", "steady")

Q_INDEX = {
    "pressure.z": 2,
    "imu.roll": 3, "imu.pitch": 4, "imu.yaw": 5,
    "dvl.vx": 6, "dvl.vy": 7, "dvl.vz": 8,
    "imu.omega_x": 9, "imu.omega_y": 10, "imu.omega_z": 11,
}

AXIS_UNIT = {
    "pressure.z": "m",
    "dvl.vx": "m/s", "dvl.vy": "m/s", "dvl.vz": "m/s",
    "imu.roll": "rad", "imu.pitch": "rad", "imu.yaw": "rad",
    "imu.omega_x": "rad/s", "imu.omega_y": "rad/s", "imu.omega_z": "rad/s",
}


# ───────────────────────────────────────── helpers

def _safe_axis_dir(axis: str) -> str:
    return axis.replace(".", "_")


def _clean_series(t: np.ndarray, *fields: np.ndarray
                  ) -> tuple[np.ndarray, ...]:
    """Rule 3 pre-plot pipeline.

    Sort by t, drop duplicate timestamps, assert strict monotonicity.
    Returns (t_clean, *fields_clean).
    """
    t = np.asarray(t, dtype=np.float64)
    arrs = [np.asarray(f) for f in fields]
    if t.size == 0:
        return (t, *arrs)
    order = np.argsort(t, kind="mergesort")
    t = t[order]
    arrs = [a[order] for a in arrs]
    if t.size > 1:
        keep = np.concatenate([[True], np.diff(t) > 0])
        t = t[keep]
        arrs = [a[keep] for a in arrs]
    assert t.size <= 1 or np.all(np.diff(t) > 0), \
        "Rule 3 violation: t still not strictly increasing"
    return (t, *arrs)


def _load_summary(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def _load_npz_axis(diag_dir: Path, bag_name: str, axis: str, mode: str
                   ) -> dict[str, np.ndarray] | None:
    """Return (t, r, R, P, S, NIS, regime, dt) for one axis from the
    NPZ sidecar; None if the axis is absent."""
    npz_path = diag_dir / bag_name / f"{bag_name}_diagnostics_series_{mode}.npz"
    if not npz_path.is_file():
        return None
    with np.load(npz_path) as data:
        ax_key = _safe_axis_dir(axis)
        keys = [f"{ax_key}__{f}" for f in
                ("t", "r", "R", "P", "S", "NIS", "regime", "dt")]
        if not all(k in data.files for k in keys):
            return None
        return {f: data[f"{ax_key}__{f}"][:]
                for f in ("t", "r", "R", "P", "S", "NIS", "regime", "dt")}


def _recompute_axis_from_bag(bags_root: Path, bag_name: str, axis: str,
                             mode: str, max_gap_s: float = 0.05,
                             max_prior_age_s: float = 0.10
                             ) -> dict[str, np.ndarray] | None:
    """On-demand recompute of (t, r, R, P, S, NIS, regime) for an
    axis whose NPZ entry is missing (e.g. yaw bag × dvl.vy).

    Calls era._read_bag + era._compute_residual + erd regime helpers.
    Returns the same dict shape as _load_npz_axis (minus 'dt' for now).
    """
    bag_dir = bags_root / bag_name
    if not (bag_dir / "metadata.yaml").is_file():
        return None
    bag = era._read_bag(bag_dir)
    if bag.ekf_t_ns.size == 0:
        return None
    rep = era._run_sanity(bag)
    sensors_reliable = rep.sensors_reliable

    max_gap_ns = int(max_gap_s * 1e9)
    max_prior_age_ns = int(max_prior_age_s * 1e9)
    res = era._compute_residual(axis, bag, max_gap_ns, sensors_reliable,
                                mode=mode,
                                max_prior_age_ns=max_prior_age_ns)
    if res is None or res.resid.size == 0:
        return None

    # Recover regime labels per matched sample by reusing erd helpers.
    cmd = erd._read_manual_control(bag_dir)
    t0 = int(bag.ekf_t_ns[0])
    ekf_t_rel = (bag.ekf_t_ns - t0) * 1e-9
    pressure_regime = erd._classify_pressure_regime(ekf_t_rel, bag.ekf_z)
    cmd_regime_at_ekf = erd._resolve_cmd_regimes_at(cmd, bag.ekf_t_ns)

    # Build an AxisSeries-like object so we can call _resolve_axis_regimes.
    class _S:
        pass
    s = _S()
    s.t_rel_s = res.t_rel_s
    s.resid = res.resid
    s.R = res.R
    s.P = res.P
    s.S = res.R + np.where(np.isfinite(res.P) & (res.P > 0), res.P, 0.0)
    nis = np.zeros_like(res.resid)
    mask = np.isfinite(s.S) & (s.S > 0)
    nis[mask] = (res.resid[mask] ** 2) / s.S[mask]
    nis[~mask] = np.nan
    s.NIS = nis

    regime_at = erd._resolve_axis_regimes(s, axis, ekf_t_rel,
                                          pressure_regime,
                                          cmd_regime_at_ekf)
    return {
        "t": res.t_rel_s.astype(np.float64),
        "r": res.resid.astype(np.float64),
        "R": res.R.astype(np.float64),
        "P": res.P.astype(np.float64),
        "S": s.S.astype(np.float64),
        "NIS": s.NIS.astype(np.float64),
        "regime": np.asarray([str(x) for x in regime_at], dtype="U16"),
        "dt": res.dt_s.astype(np.float64) if hasattr(res, "dt_s")
              else np.zeros_like(res.resid),
    }


def _dominant_regime(regime_labels: np.ndarray) -> tuple[str, np.ndarray]:
    """Longest contiguous run of one label; return (label, mask).

    For Rule 2: whiteness ACF is run over this dominant regime per bag.
    """
    if regime_labels.size == 0:
        return "", np.zeros(0, dtype=bool)
    best_label = ""
    best_len = 0
    best_start = 0
    cur_label = str(regime_labels[0])
    cur_start = 0
    for i in range(1, regime_labels.size):
        if str(regime_labels[i]) != cur_label:
            if (i - cur_start) > best_len:
                best_len = i - cur_start
                best_start = cur_start
                best_label = cur_label
            cur_label = str(regime_labels[i])
            cur_start = i
    if (regime_labels.size - cur_start) > best_len:
        best_len = regime_labels.size - cur_start
        best_start = cur_start
        best_label = cur_label
    mask = np.zeros(regime_labels.size, dtype=bool)
    mask[best_start:best_start + best_len] = True
    return best_label, mask


def _per_bar_chi2_band(n: int) -> tuple[float, float]:
    """Per-bar chi^2(n)/n 95 % acceptance band. Rule 1."""
    if n <= 0:
        return float("nan"), float("nan")
    return float(chi2.ppf(0.025, n) / n), float(chi2.ppf(0.975, n) / n)


def _collect_field(summary: dict, axis: str, regime: str,
                   field: str) -> dict[str, float]:
    out: dict[str, float] = {}
    for name, bag in summary.get("bags", {}).items():
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
        out[name] = float(v)
    return out


def _collect_field_list(summary, axis, regime, field):
    return list(_collect_field(summary, axis, regime, field).values())


def _post_prior_ratio(post: dict, prior: dict | None,
                      axis: str) -> float:
    if prior is None:
        return float("nan")
    p_med = _collect_field_list(post, axis, "all", "nis_median")
    pr_med = _collect_field_list(prior, axis, "all", "nis_median")
    if not p_med or not pr_med:
        return float("nan")
    p = float(np.median(p_med))
    pr = float(np.median(pr_med))
    if not math.isfinite(p) or p <= 0:
        return float("nan")
    return pr / p


# ───────────────────────────────────────── regime shading helper

def _shade_regimes(ax, t: np.ndarray, regime_labels: np.ndarray) -> None:
    if t.size == 0 or regime_labels.size != t.size:
        return
    starts = [0]
    for i in range(1, regime_labels.size):
        if str(regime_labels[i]) != str(regime_labels[i - 1]):
            starts.append(i)
    starts.append(regime_labels.size)
    for k in range(len(starts) - 1):
        i0 = starts[k]
        i1 = starts[k + 1]
        label = str(regime_labels[i0])
        color = C_REG.get(label, "#eeeeee")
        t0 = float(t[i0])
        t1 = float(t[min(i1, t.size - 1)])
        ax.axvspan(t0, t1, color=color, alpha=0.45, lw=0, zorder=0)


def _regime_legend(used_labels: Iterable[str]) -> list[Patch]:
    seen: list[str] = []
    for u in used_labels:
        s = str(u)
        if s not in seen:
            seen.append(s)
    return [Patch(facecolor=C_REG.get(s, "#eeeeee"), edgecolor="#999999",
                  alpha=0.7, label=s) for s in seen]


# ───────────────────────────────────────── qfig1 — per-axis residual band timeline

def fig1_per_axis(axis: str, bag_name: str,
                  data: dict[str, np.ndarray],
                  drive_signal: tuple[np.ndarray, np.ndarray, str] | None,
                  q_value: float | None,
                  mode: str,
                  out_path: Path) -> None:
    """4-panel timeline (a/b/c/d) for one axis on one showcase bag.

    Visual reference: q_tuning_figure_examples/qfig1_timeline_dvl_vz.png.
    """
    t, r, S, regime = _clean_series(
        data["t"], data["r"], data["S"], data["regime"])
    nis = data["NIS"]
    # Reorder NIS to match (we lost the reorder above; recompute).
    # For consistency, rebuild NIS from r and S after _clean_series.
    nis_clean = np.where(S > 0, (r * r) / np.maximum(S, 1e-30), np.nan)

    fig, axes = plt.subplots(4, 1, figsize=(8.0, 7.6), sharex=True,
                             gridspec_kw=dict(
                                 height_ratios=[1.2, 1, 1, 0.9],
                                 hspace=0.18,
                             ))

    # (a) residual ± 2 sqrt(S) band
    ax = axes[0]
    _shade_regimes(ax, t, regime)
    env = 2.0 * np.sqrt(np.maximum(S, 0.0))
    ax.fill_between(t, -env, env, color=C_ENV, alpha=0.18,
                    label=r"$\pm 2\sqrt{S}$ (filter $95\,\%$ band)")
    ax.plot(t, r, color=C_RES, lw=0.7, label="residual r")
    ax.axhline(0, color="black", lw=0.5, alpha=0.6)
    ax.set_ylabel(f"{axis} residual [{AXIS_UNIT.get(axis, '')}]")
    ax.set_title("(a) Residual within filter $95\\,\\%$ band  —  "
                 "if 5–10 % of points fall outside, S (thus Q) is too small",
                 loc="left", pad=4)
    # If env is essentially invisible (Q very tight), annotate a median band.
    finite_env = env[np.isfinite(env)]
    if finite_env.size > 0:
        med_env = float(np.median(finite_env))
        max_abs_r = float(np.max(np.abs(r))) if r.size else 0.0
        if max_abs_r > 0 and med_env < 0.05 * max_abs_r:
            unit = AXIS_UNIT.get(axis, "")
            ax.text(0.99, 0.02,
                    f"median band: ±{med_env:.3g} {unit}"
                    + (f"  (Q[{Q_INDEX.get(axis, '?')}] = {q_value:g})"
                       if q_value is not None else ""),
                    transform=ax.transAxes, ha="right", va="bottom",
                    fontsize=7.5, color=C_CHI,
                    bbox=dict(boxstyle="round,pad=0.3",
                              facecolor="white",
                              edgecolor="#bbbbbb", lw=0.4))
    ax.legend(loc="upper right", ncol=2)

    # (b) S log-y
    ax = axes[1]
    _shade_regimes(ax, t, regime)
    ax.plot(t, S, color="#7a3a91", lw=1.0)
    ax.set_yscale("log")
    unit2 = AXIS_UNIT.get(axis, "")
    ax.set_ylabel(rf"$S = HPH^\top + R$  [{unit2}²]")
    ax.set_title("(b) Innovation covariance S (log)", loc="left", pad=4)

    # (c) NIS scatter + chi^2 band
    ax = axes[2]
    _shade_regimes(ax, t, regime)
    nis_pos = np.where(np.isfinite(nis_clean) & (nis_clean > 0),
                        nis_clean, np.nan)
    ax.scatter(t, nis_pos, s=4, c=C_NIS, alpha=0.65,
               rasterized=True, label="NIS = r²/S")
    ax.axhline(CHI2_HI,  color=C_BAD, ls="--", lw=1.0,
               label=f"$\\chi^2_{{1,0.975}} = {CHI2_HI:.2f}$")
    ax.axhline(CHI2_MED, color=C_CHI, ls=":",  lw=1.0,
               label=f"$\\chi^2_{{1,0.5}} = {CHI2_MED:.2f}$")
    ax.axhline(CHI2_LO,  color=C_BAD, ls="--", lw=1.0)
    ax.set_yscale("log")
    ax.set_ylabel("NIS")
    ax.set_title("(c) NIS samples with $\\chi^2_1$ acceptance band  —  "
                 "well-tuned: median $\\approx 0.45$; over-confident: "
                 "median $\\gg 1$",
                 loc="left", pad=4)
    ax.legend(loc="upper right", fontsize=7.5, ncol=3)

    # (d) driving signal
    ax = axes[3]
    _shade_regimes(ax, t, regime)
    if drive_signal is not None:
        td, xd, lbl = drive_signal
        td_a, xd_a = _clean_series(td, xd)
        ax.plot(td_a, xd_a, color=C_RES, lw=1.2, label=lbl)
        ax.set_ylabel(lbl)
    else:
        ax.text(0.5, 0.5, "(no driving signal)", ha="center", va="center",
                color=C_CHI, transform=ax.transAxes)
        ax.set_ylabel("—")
    ax.set_xlabel("t − t₀ [s]")
    ax.set_title("(d) Driving signal — confirms regime labelling",
                 loc="left", pad=4)

    # Bottom regime legend.
    used = []
    for v in regime:
        s = str(v)
        if s not in used:
            used.append(s)
    handles = _regime_legend(used)
    if handles:
        fig.legend(handles=handles, loc="lower center",
                   ncol=min(len(handles), 7),
                   bbox_to_anchor=(0.5, -0.02),
                   frameon=False, fontsize=8)

    q_str = f"  (Q[{Q_INDEX.get(axis, '?')}] = {q_value:g})" if q_value is not None else ""
    fig.suptitle(f"{axis}  —  {bag_name}{q_str}  {mode} mode",
                 x=0.02, ha="left", fontweight="bold", fontsize=11)
    fig.savefig(out_path)
    plt.close(fig)


def fig1_paired(axis: str, bag_name: str,
                post: dict[str, np.ndarray],
                prior: dict[str, np.ndarray] | None,
                drive_signal: tuple[np.ndarray, np.ndarray, str] | None,
                q_value: float | None,
                out_path: Path) -> None:
    """Held-back axes: one PNG with two columns (posterior / prior_approx),
    each with the 4-panel layout. figsize=(12, 7.6)."""
    if prior is None:
        # No prior available: degrade gracefully to single column.
        fig1_per_axis(axis, bag_name, post, drive_signal, q_value,
                      "posterior", out_path)
        return

    fig, axes = plt.subplots(4, 2, figsize=(13.0, 8.6), sharex=False,
                             gridspec_kw=dict(
                                 height_ratios=[1.2, 1, 1, 0.9],
                                 hspace=0.30, wspace=0.25,
                             ))

    for col, (label, data) in enumerate([("posterior", post),
                                         ("prior_approx", prior)]):
        t, r, S, regime = _clean_series(
            data["t"], data["r"], data["S"], data["regime"])
        nis_clean = np.where(S > 0, (r * r) / np.maximum(S, 1e-30), np.nan)

        # (a) residual + band
        ax = axes[0][col]
        _shade_regimes(ax, t, regime)
        env = 2.0 * np.sqrt(np.maximum(S, 0.0))
        ax.fill_between(t, -env, env, color=C_ENV, alpha=0.18)
        ax.plot(t, r, color=C_RES, lw=0.7)
        ax.axhline(0, color="black", lw=0.5, alpha=0.6)
        ax.set_ylabel(f"r [{AXIS_UNIT.get(axis, '')}]")
        ax.set_title(f"({['a','e'][col]}) {label}: residual ± 2√S",
                     loc="left", pad=4)

        # (b) S
        ax = axes[1][col]
        _shade_regimes(ax, t, regime)
        ax.plot(t, S, color="#7a3a91", lw=1.0)
        ax.set_yscale("log")
        ax.set_ylabel("S")
        ax.set_title(f"({['b','f'][col]}) {label}: S (log)", loc="left", pad=4)

        # (c) NIS
        ax = axes[2][col]
        _shade_regimes(ax, t, regime)
        nis_pos = np.where(np.isfinite(nis_clean) & (nis_clean > 0),
                            nis_clean, np.nan)
        ax.scatter(t, nis_pos, s=3, c=C_NIS, alpha=0.6, rasterized=True)
        ax.axhline(CHI2_HI,  color=C_BAD, ls="--", lw=0.9)
        ax.axhline(CHI2_MED, color=C_CHI, ls=":",  lw=0.9)
        ax.axhline(CHI2_LO,  color=C_BAD, ls="--", lw=0.9)
        ax.set_yscale("log")
        ax.set_ylabel("NIS")
        ax.set_title(f"({['c','g'][col]}) {label}: NIS with $\\chi^2_1$ band",
                     loc="left", pad=4)

        # (d) driving signal
        ax = axes[3][col]
        _shade_regimes(ax, t, regime)
        if drive_signal is not None:
            td, xd, lbl = drive_signal
            td_a, xd_a = _clean_series(td, xd)
            ax.plot(td_a, xd_a, color=C_RES, lw=1.0)
            ax.set_ylabel(lbl)
        ax.set_xlabel("t − t₀ [s]")
        ax.set_title(f"({['d','h'][col]}) {label}: driving signal",
                     loc="left", pad=4)

    q_str = f"  (Q[{Q_INDEX.get(axis, '?')}] = {q_value:g})" if q_value is not None else ""
    fig.suptitle(
        f"{axis}  —  {bag_name}{q_str}  paired posterior vs prior_approx  "
        "(post-update collapse signature)",
        x=0.02, ha="left", fontweight="bold", fontsize=11)
    fig.savefig(out_path)
    plt.close(fig)


# ───────────────────────────────────────── qfig2 — per-axis NIS distribution

def fig2_per_axis(axis: str,
                  diag_post: Path, summary_post: dict,
                  diag_prior: Path | None, summary_prior: dict | None,
                  out_path: Path) -> None:
    """Three-panel NIS distribution: posterior across bags, prior_approx
    across bags, per-regime overlay (posterior).

    Visual reference: q_tuning_figure_examples/qfig2_nis_distribution.png.
    """
    fig, axs = plt.subplots(1, 3, figsize=(11.0, 3.4))

    def _gather(diag_dir: Path, summary: dict, mode: str):
        all_nis: list[float] = []
        per_regime: dict[str, list[float]] = defaultdict(list)
        for bag_name in summary.get("bags", {}):
            blob = _load_npz_axis(diag_dir, bag_name, axis, mode)
            if blob is None:
                continue
            nis = blob["NIS"]
            reg = blob["regime"]
            mask = np.isfinite(nis) & (nis > 0)
            for v, rname in zip(nis[mask], reg[mask]):
                all_nis.append(float(v))
                per_regime[str(rname)].append(float(v))
        return np.asarray(all_nis), per_regime

    post_all, post_by_regime = _gather(diag_post, summary_post, "posterior")
    prior_all = np.array([])
    if diag_prior is not None and summary_prior is not None:
        prior_all, _ = _gather(diag_prior, summary_prior, "prior_approx")

    titles = [f"{axis} — posterior  (all bags)",
              f"{axis} — prior_approx  (all bags)",
              f"{axis} — per-regime  (posterior)"]
    series_for_panel = [post_all, prior_all]
    for ax, data, title in zip(axs[:2], series_for_panel, titles[:2]):
        if data.size == 0:
            ax.text(0.5, 0.5, "(no data)", ha="center", va="center",
                    color=C_CHI, transform=ax.transAxes)
            ax.set_title(title, loc="left", pad=4)
            ax.set_xscale("log"); ax.set_yscale("log")
            ax.set_ylim(1e-6, 5)
            continue
        lo_x = max(1e-6, float(np.nanmin(data)))
        hi_x = max(CHI2_HI * 5, float(np.nanmax(data)))
        if not (np.isfinite(lo_x) and np.isfinite(hi_x)) or hi_x <= lo_x:
            lo_x, hi_x = 1e-6, 1e3
        bins = np.logspace(np.log10(lo_x), np.log10(hi_x), 60)
        ax.hist(data, bins=bins, density=True, color=C_NIS,
                alpha=0.55, edgecolor="white", lw=0.4,
                label="empirical")
        xx = np.logspace(np.log10(lo_x), np.log10(hi_x), 400)
        ax.plot(xx, chi2.pdf(xx, 1), color=C_BAD, lw=1.4,
                label=r"$\chi^2_1$ PDF")
        ax.axvline(CHI2_HI, color=C_BAD, ls="--", lw=0.9, alpha=0.8)
        ax.axvline(CHI2_MED, color=C_CHI, ls=":", lw=0.9, alpha=0.8)
        ax.axvline(CHI2_LO, color=C_BAD, ls="--", lw=0.9, alpha=0.8)
        ax.set_xscale("log"); ax.set_yscale("log")
        ax.set_xlabel("NIS"); ax.set_ylabel("density")
        ax.set_title(title, loc="left", pad=4)
        med = float(np.median(data))
        exc = float(np.mean(data > CHI2_HI))
        ax.text(0.98, 0.95,
                f"median = {med:.3g}\nP(NIS > {CHI2_HI:.2f}) = {exc*100:.1f}%",
                transform=ax.transAxes, ha="right", va="top",
                fontsize=7.5,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                          edgecolor="#bbbbbb", lw=0.5, alpha=0.9))
        ax.legend(loc="lower left", fontsize=7.5)
        ax.set_ylim(1e-6, 5)

    # Panel 3 — per-regime overlay (posterior)
    ax = axs[2]
    if not post_by_regime:
        ax.text(0.5, 0.5, "(no data)", ha="center", va="center",
                color=C_CHI, transform=ax.transAxes)
    else:
        all_concat = np.concatenate(list(post_by_regime.values()))
        lo_x = max(1e-6, float(np.nanmin(all_concat)))
        hi_x = max(CHI2_HI * 5, float(np.nanmax(all_concat)))
        if not (np.isfinite(lo_x) and np.isfinite(hi_x)) or hi_x <= lo_x:
            lo_x, hi_x = 1e-6, 1e3
        bins = np.logspace(np.log10(lo_x), np.log10(hi_x), 50)
        for reg, vals in sorted(post_by_regime.items()):
            color = C_REG.get(reg, "#888888")
            ax.hist(vals, bins=bins, density=True, color=color,
                    alpha=0.55, edgecolor="white", lw=0.3,
                    label=f"{reg}  (n={len(vals)})")
        xx = np.logspace(np.log10(lo_x), np.log10(hi_x), 400)
        ax.plot(xx, chi2.pdf(xx, 1), color=C_BAD, lw=1.0,
                label=r"$\chi^2_1$")
        ax.axvline(CHI2_MED, color=C_CHI, ls=":", lw=0.9)
        ax.axvline(CHI2_HI, color=C_BAD, ls="--", lw=0.9, alpha=0.7)
        ax.set_xscale("log"); ax.set_yscale("log")
        ax.set_xlabel("NIS"); ax.set_ylabel("density")
        ax.legend(loc="lower left", fontsize=7)
        ax.set_ylim(1e-6, 5)
    ax.set_title(titles[2], loc="left", pad=4)

    fig.suptitle(f"{axis} — NIS distribution vs $\\chi^2_1$",
                 x=0.02, ha="left", fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path)
    plt.close(fig)


# ───────────────────────────────────────── qfig3 — batch consistency (Rule 1)

def fig3_batch_consistency(summary: dict, axes: list[str],
                           out_path: Path) -> dict[str, Any]:
    """One bar per (axis × bag), each with its OWN chi^2(n_i)/n_i band.

    Rule 1 — bands are per-bar, never aggregate.
    """
    fig, ax = plt.subplots(figsize=(11.0, 4.4))

    bag_names = sorted(summary.get("bags", {}).keys())
    bar_records: list[dict[str, Any]] = []
    pos_axis = np.arange(len(axes))
    bar_width = 0.85 / max(1, len(bag_names))
    half = (len(bag_names) - 1) / 2

    for j, bag_name in enumerate(bag_names):
        for i, axis in enumerate(axes):
            ax_block = (summary["bags"][bag_name].get("axes", {})
                        .get(axis, {}).get("all"))
            if not isinstance(ax_block, dict):
                continue
            n_i = ax_block.get("n", 0)
            nis_med = ax_block.get("nis_median")
            if not (isinstance(nis_med, (int, float))
                    and math.isfinite(nis_med) and n_i > 0):
                continue
            lo, hi = _per_bar_chi2_band(int(n_i))
            x_pos = pos_axis[i] + (j - half) * bar_width
            ok = bool(lo <= nis_med <= hi)
            color = C_OK if ok else C_BAD
            bar_records.append(dict(
                axis=axis, bag=bag_name, x=x_pos, h=float(nis_med),
                lo=lo, hi=hi, ok=ok, n=int(n_i),
            ))
            # Bar
            ax.bar([x_pos], [float(nis_med)], width=bar_width * 0.85,
                   color=color, alpha=0.80, edgecolor="white", lw=0.4)
            # Per-bar band (Rule 1) — drawn as a thin floating box
            # centred at y=1, occupying the bar's x-range.
            band_x0 = x_pos - bar_width * 0.45
            band_w = bar_width * 0.9
            ax.add_patch(Rectangle((band_x0, lo), band_w, hi - lo,
                                   facecolor="#d4e6f1",
                                   edgecolor="#7aa9d4", lw=0.4,
                                   alpha=0.45, zorder=1))

    ax.axhline(1.0, color=C_OK, lw=1.0, label=r"target $\bar{NIS}=1$")
    ax.axhline(CHI2_MED, color=C_CHI, ls=":", lw=0.8,
               label=f"$\\chi^2_1$ median = {CHI2_MED:.2f}")
    ax.set_yscale("log")
    ax.set_ylim(1e-3, 1e3)
    ax.set_xticks(pos_axis)
    ax.set_xticklabels(axes, rotation=20)
    ax.set_ylabel(r"per-bag time-averaged NIS")
    ax.set_title("Batch NIS consistency  —  per-bar $\\chi^2(n_i)/n_i$ bands  "
                 "(green = inside band, red = outside)",
                 loc="left", pad=4)

    # Pass-count companion bars at the foot of each axis group.
    pass_counts: dict[str, tuple[int, int]] = {}
    for axis in axes:
        recs = [r for r in bar_records if r["axis"] == axis]
        ok = sum(1 for r in recs if r["ok"])
        tot = len(recs)
        pass_counts[axis] = (ok, tot)
    # Annotate pass count below each axis label.
    y_ann = 1e-3 * 1.5
    for i, axis in enumerate(axes):
        ok, tot = pass_counts[axis]
        if tot == 0:
            label = "n=0"
            color = "#888888"
        else:
            label = f"{ok}/{tot} pass"
            color = C_OK if ok == tot else (C_BAD if ok == 0 else "#bb7733")
        ax.text(pos_axis[i], y_ann, label, ha="center", va="bottom",
                fontsize=7, color=color, fontweight="bold")

    # Off-axis triangle markers if any bar is clipped by ylim.
    for r in bar_records:
        if r["h"] > 1e3:
            ax.plot(r["x"], 1e3 * 0.92, marker="^",
                    color=C_BAD, markersize=6)
        elif r["h"] < 1e-3:
            ax.plot(r["x"], 1e-3 * 1.08, marker="v",
                    color=C_BAD, markersize=6)

    ax.legend(loc="upper left", fontsize=7.5)
    fig.savefig(out_path)
    plt.close(fig)
    return {axis: pass_counts[axis] for axis in axes}


# ───────────────────────────────────────── qfig4 — whiteness (Rule 2)

def fig4_whiteness(diag_post: Path, summary_post: dict,
                   diag_prior: Path | None, summary_prior: dict | None,
                   axes: list[str], out_path: Path) -> dict[str, Any]:
    """Innovation autocorrelation on standardised innovations
    nu = r/sqrt(S), in the dominant regime per bag.

    Rule 2: prior_approx residuals when posterior↔prior ratio < 5×;
    deferred (greyed-out) when ratio >= 5×.
    """
    DEFER_THRESHOLD = 5.0
    n_axes = len(axes)
    n_cols = 4
    n_rows = int(math.ceil(n_axes / n_cols))
    fig, axes_grid = plt.subplots(n_rows, n_cols,
                                  figsize=(13.5, 3.0 * n_rows))
    flat = np.array(axes_grid).reshape(-1)

    max_lag = 30
    verdicts: dict[str, Any] = {}
    for idx, axis in enumerate(axes):
        ax = flat[idx]
        ratio = _post_prior_ratio(summary_post, summary_prior, axis)
        deferred = (math.isfinite(ratio) and ratio >= DEFER_THRESHOLD)

        if deferred:
            ax.set_facecolor("#f0f0f0")
            ax.text(0.5, 0.5,
                    f"{axis}\n\nratio = {ratio:.1f}× ≥ {DEFER_THRESHOLD}×\n"
                    "deferred — see procedure §8.3",
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=8, color="#555555",
                    bbox=dict(boxstyle="round,pad=0.4",
                              facecolor="white",
                              edgecolor="#bbbbbb", lw=0.5))
            ax.set_xticks([]); ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_visible(False)
            verdicts[axis] = {
                "verdict": "DEFERRED", "ratio": ratio, "lags_outside": None,
            }
            continue

        # Pick mode based on ratio.
        use_prior = (math.isfinite(ratio) and ratio < DEFER_THRESHOLD
                     and diag_prior is not None and summary_prior is not None)
        diag_dir = diag_prior if use_prior else diag_post
        summary_use = summary_prior if use_prior else summary_post
        mode = "prior_approx" if use_prior else "posterior"

        # Pool standardised innovations across bags, weighted by bag length.
        weighted_acf = np.zeros(max_lag + 1)
        weight_total = 0.0
        n_total = 0
        for bag_name in summary_use.get("bags", {}):
            blob = _load_npz_axis(diag_dir, bag_name, axis, mode)
            if blob is None:
                continue
            t, r, S, regime = _clean_series(
                blob["t"], blob["r"], blob["S"], blob["regime"])
            if t.size < max_lag + 5:
                continue
            valid = (S > 0) & np.isfinite(S) & np.isfinite(r)
            if valid.sum() < max_lag + 5:
                continue
            nu = np.zeros_like(r)
            nu[valid] = r[valid] / np.sqrt(S[valid])
            # dominant regime
            _, dom_mask = _dominant_regime(regime)
            mask = dom_mask & valid
            if mask.sum() < max_lag + 5:
                # Fall back to all-valid if dominant regime too short
                mask = valid
            nu_seg = nu[mask] - np.mean(nu[mask])
            v = float(np.dot(nu_seg, nu_seg))
            n = nu_seg.size
            if v <= 0 or n < max_lag + 5:
                continue
            acf = np.zeros(max_lag + 1)
            for k in range(max_lag + 1):
                acf[k] = (np.dot(nu_seg[: n - k], nu_seg[k:]) /
                          max(1, (n - k)) / (v / n))
            weighted_acf += acf * n
            weight_total += n
            n_total += n
        if weight_total <= 0:
            ax.text(0.5, 0.5, f"{axis}\nno data", color=C_CHI,
                    transform=ax.transAxes, ha="center", va="center")
            ax.set_xticks([]); ax.set_yticks([])
            verdicts[axis] = {"verdict": "NA", "ratio": ratio,
                              "lags_outside": None}
            continue
        mean_acf = weighted_acf / weight_total
        ci = 1.96 / np.sqrt(weight_total)
        # Whiteness verdict from lags 1..max_lag
        lags_outside = int(np.sum(np.abs(mean_acf[1:]) > ci))
        verdict = "white" if lags_outside == 0 else "coloured"
        col = C_OK if verdict == "white" else C_BAD
        verdicts[axis] = {"verdict": verdict, "ratio": ratio,
                          "lags_outside": lags_outside,
                          "ci_95": float(ci),
                          "mode_used": mode}

        lags = np.arange(max_lag + 1)
        ax.axhspan(-ci, ci, color="#d4e6f1", alpha=0.55,
                   label=f"$\\pm 1.96/\\sqrt{{N}} = {ci:.2g}$")
        ax.vlines(lags, 0, mean_acf, color=col, lw=1.0)
        ax.scatter(lags, mean_acf, s=6, color=col)
        ax.axhline(0, color="black", lw=0.4)
        ax.set_xlim(-1, max_lag + 1)
        ax.set_ylim(-1.05, 1.05)
        ax.set_xlabel("lag k")
        ax.set_ylabel(r"$\hat{\rho}(k)$")
        ax.set_title(
            f"{axis}  —  {verdict}"
            + (f"  ({mode})" if use_prior else ""),
            loc="left", pad=4, fontsize=8.5, color=col)
        ax.legend(loc="upper right", fontsize=7)

    for k in range(n_axes, n_rows * n_cols):
        flat[k].axis("off")

    fig.suptitle(
        "Innovation autocorrelation  —  Mehra whiteness test  "
        "(standardised innovations $\\nu = r/\\sqrt{S}$, dominant regime per bag)",
        x=0.02, ha="left", fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_path)
    plt.close(fig)
    return verdicts


# ───────────────────────────────────────── qfig5 — per-regime box

def fig5_regime_boxplot(summary: dict, axes: list[str],
                        regimes: list[str], out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(10.0, 4.2))

    width = 0.85 / max(1, len(regimes))
    cols = ["#1f5fa3", "#d6263d", "#e07a3a", "#1ea08a", "#7a3a91",
            "#9ad17b", "#bba16a"]
    n_zero_annotations: list[tuple[float, float]] = []
    off_band_marks: list[tuple[float, float, float]] = []  # (x, y, value)

    for j, reg in enumerate(regimes):
        positions = np.arange(len(axes)) + (j - (len(regimes) - 1) / 2) * width
        bp_data: list[np.ndarray] = []
        for i, axis in enumerate(axes):
            per_bag = _collect_field_list(summary, axis, reg, "nis_median")
            if not per_bag:
                bp_data.append(np.array([np.nan]))
                n_zero_annotations.append((float(positions[i]), 1.0))
            else:
                arr = np.array(per_bag)
                bp_data.append(arr)
                med = float(np.median(arr))
                if math.isfinite(med) and (med > CHI2_HI or med < CHI2_LO):
                    off_band_marks.append((float(positions[i]), med, med))
        col = cols[j % len(cols)]
        ax.boxplot(bp_data, positions=positions, widths=width * 0.85,
                   showfliers=False, patch_artist=True,
                   medianprops=dict(color="black", lw=1.0),
                   boxprops=dict(facecolor=col, alpha=0.65,
                                 edgecolor="black", lw=0.5),
                   whiskerprops=dict(color="black", lw=0.5),
                   capprops=dict(color="black", lw=0.5))

    ax.axhspan(CHI2_LO, CHI2_HI, color="#d4e6f1", alpha=0.6, zorder=0,
               label=r"$\chi^2_1$ 95% band")
    ax.axhline(CHI2_MED, color=C_CHI, ls=":", lw=0.9,
               label=f"$\\chi^2_1$ median = {CHI2_MED:.2f}")

    # n=0 annotations.
    for x, y in n_zero_annotations:
        ax.text(x, y, "n=0", ha="center", va="center", fontsize=6.5,
                color="#888888")

    # Off-band median triangles.
    for x, y, val in off_band_marks:
        if y > CHI2_HI:
            ax.plot(x, CHI2_HI * 1.1, marker="^", color=C_BAD,
                    markersize=6, zorder=10)
        else:
            ax.plot(x, CHI2_LO * 0.9, marker="v", color=C_BAD,
                    markersize=6, zorder=10)

    ax.set_yscale("log")
    ax.set_xticks(np.arange(len(axes)))
    ax.set_xticklabels(axes, rotation=20)
    ax.set_ylabel("NIS_median (per-bag)")
    ax.set_title("Per-regime NIS  —  reveals where Q is wrong vs where the model is",
                 loc="left", pad=4)
    handles = [Patch(facecolor=cols[j % len(cols)], alpha=0.7, label=r)
               for j, r in enumerate(regimes)]
    band_h, band_l = ax.get_legend_handles_labels()
    ax.legend(handles=handles + band_h, labels=regimes + band_l,
              loc="upper left", fontsize=7.5, ncol=4)
    fig.savefig(out_path)
    plt.close(fig)


# ───────────────────────────────────────── qfig6 — Q-sweep placeholder

def fig6_q_sweep_placeholder(summary: dict, out_path: Path,
                             worst_axis: str = "dvl.vz") -> None:
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    ax.set_facecolor("#f5f5f5")
    ax.text(0.5, 0.62,
            "Q-sweep curve  (qfig6)\n\n"
            "requires offline EKF replay\n"
            "—  see procedure §8.3  —",
            transform=ax.transAxes, ha="center", va="center",
            fontsize=12, color="#555555",
            bbox=dict(boxstyle="round,pad=0.6", facecolor="white",
                      edgecolor="#bbbbbb", lw=0.6))

    # Optional: the single measured point + moment-match prediction for the worst axis.
    nis_med_list = _collect_field_list(summary, worst_axis, "all", "nis_median")
    if nis_med_list:
        nis_med = float(np.median(nis_med_list))
        # Fish out Q[i] for this axis.
        q_diag = summary.get("Q_diagonal_ref")
        q_idx = Q_INDEX.get(worst_axis)
        q_current = (float(q_diag[q_idx]) if q_diag and q_idx is not None
                     and q_idx < len(q_diag) else None)
        q_proposed = (q_current * nis_med) if q_current else None
        if q_current and q_proposed:
            ax.text(0.5, 0.18,
                    f"single measured point + moment-match prediction:\n"
                    f"  axis = {worst_axis}\n"
                    f"  Q[{q_idx}]_current = {q_current:g}, NIS_med = {nis_med:.3g}\n"
                    f"  predicted Q[{q_idx}]_new ≈ {q_proposed:.3g} "
                    f"(at NIS_med = {CHI2_MED:.3g})\n"
                    "  full sweep deferred",
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=8, color="#444444",
                    bbox=dict(boxstyle="round,pad=0.4",
                              facecolor="white",
                              edgecolor="#bbbbbb", lw=0.4))
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_title("Q-sweep — placeholder (deferred)",
                 loc="left", pad=4)
    fig.savefig(out_path)
    plt.close(fig)


# ───────────────────────────────────────── qfig7 — dashboard

def fig7_dashboard(summary: dict, axes: list[str],
                   regimes: list[str], out_path: Path) -> dict[str, dict]:
    grid = np.full((len(axes), len(regimes)), np.nan)
    for i, axis in enumerate(axes):
        for j, reg in enumerate(regimes):
            vals = _collect_field_list(summary, axis, reg, "nis_median")
            if vals:
                grid[i, j] = float(np.median(vals))

    fig, ax = plt.subplots(figsize=(9.5, 4.2))
    log_grid = np.log10(np.where(grid > 0, grid, np.nan))
    score = log_grid - np.log10(CHI2_MED)
    im = ax.imshow(score, cmap="RdBu_r", vmin=-3, vmax=3, aspect="auto")

    # Cell text and NaN-cell hatch.
    for i in range(len(axes)):
        for j in range(len(regimes)):
            v = grid[i, j]
            if not math.isfinite(v):
                ax.add_patch(Rectangle((j - 0.5, i - 0.5), 1, 1,
                                       facecolor="#eeeeee",
                                       edgecolor="white",
                                       hatch="////", lw=0.5, alpha=0.9))
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
            ax.text(j, i, txt, ha="center", va="center", fontsize=7.0,
                    color="black",
                    path_effects=[pe.Stroke(linewidth=1.4, foreground="white"),
                                  pe.Normal()])
    ax.set_xticks(range(len(regimes)))
    ax.set_xticklabels(regimes, rotation=20)
    ax.set_yticks(range(len(axes)))
    ax.set_yticklabels(axes)
    cbar = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02)
    cbar.set_label(
        "log₁₀(NIS_median / 0.45)\n"
        "(blue = under-confident, red = over-confident)")
    ax.set_title("Axis × regime NIS dashboard  —  the at-a-glance Q-tuning health card",
                 loc="left", pad=4)
    fig.savefig(out_path)
    plt.close(fig)
    return {axes[i]: {regimes[j]: (None if not math.isfinite(grid[i, j])
                                   else float(grid[i, j]))
                      for j in range(len(regimes))}
            for i in range(len(axes))}


# ───────────────────────────────────────── qfig8 — composite report panel

def fig8_composite(diag_post: Path, summary_post: dict,
                   showcase: dict[str, str], axes: list[str],
                   bags_root: Path,
                   out_path: Path) -> None:
    """Composite report panel mirroring qfig8_q_tuning_report_panel.png."""
    fig = plt.figure(figsize=(13.5, 9.0))
    gs = GridSpec(3, 3, figure=fig, hspace=0.55, wspace=0.40,
                  height_ratios=[1.0, 1.0, 1.1])

    worst_axis = "dvl.vz"
    showcase_bag = showcase.get(worst_axis, "")
    blob = _load_npz_axis(diag_post, showcase_bag, worst_axis, "posterior")

    # (a) residual band — Rule 3 enforced
    axA = fig.add_subplot(gs[0, :2])
    if blob is not None:
        t, r, S = _clean_series(blob["t"], blob["r"], blob["S"])
        env = 2.0 * np.sqrt(np.maximum(S, 0.0))
        axA.fill_between(t, -env, env, color=C_ENV, alpha=0.5, zorder=4,
                         label=r"$\pm 2\sqrt{S}$")
        axA.plot(t, r, color=C_RES, lw=0.4, alpha=0.6,
                 rasterized=True, zorder=2,
                 label=f"r ({worst_axis})")
        axA.axhline(0, color="black", lw=0.5)
        axA.set_ylabel(f"r [{AXIS_UNIT.get(worst_axis, '')}]")
        axA.set_xlabel("t [s]")
        axA.set_title(
            f"(a) {worst_axis}  ({showcase_bag}) — residual outside "
            "±2√S ⇒ over-confident",
            loc="left", pad=4)
        axA.legend(loc="upper right")
        # 5-second zoom inset
        if t.size > 5:
            from mpl_toolkits.axes_grid1.inset_locator import inset_axes
            ax_in = inset_axes(axA, width="32%", height="38%",
                                loc="lower right", borderpad=0.6)
            mid = t.size // 2
            t0 = max(0.0, t[mid] - 2.5)
            t1 = t[mid] + 2.5
            mask = (t >= t0) & (t <= t1)
            if mask.sum() >= 3:
                t_zoom = t[mask]
                r_zoom = r[mask]
                env_zoom = env[mask]
                ax_in.fill_between(t_zoom, -env_zoom, env_zoom,
                                   color=C_ENV, alpha=0.5)
                ax_in.plot(t_zoom, r_zoom, color=C_RES, lw=0.7)
                ax_in.scatter(t_zoom, r_zoom, color=C_RES, s=4)
                ax_in.axhline(0, color="black", lw=0.4)
                ax_in.set_xticks([t_zoom[0], t_zoom[-1]])
                ax_in.tick_params(labelsize=6)
                ax_in.set_title("5-s zoom", fontsize=7, pad=2)
    else:
        axA.text(0.5, 0.5, "no NPZ available", color=C_CHI,
                 transform=axA.transAxes, ha="center", va="center")

    # (b) NIS distribution
    axB = fig.add_subplot(gs[0, 2])
    if blob is not None:
        nis = blob["NIS"]
        nis_pos = nis[(nis > 0) & np.isfinite(nis)]
        if nis_pos.size > 0:
            lo_x = max(1e-6, float(np.nanmin(nis_pos)))
            hi_x = max(CHI2_HI * 5, float(np.nanmax(nis_pos)))
            bins = np.logspace(np.log10(lo_x), np.log10(hi_x), 50)
            axB.hist(nis_pos, bins=bins, density=True,
                     color=C_NIS, alpha=0.55, edgecolor="white", lw=0.4)
            xx = np.logspace(np.log10(lo_x), np.log10(hi_x), 400)
            axB.plot(xx, chi2.pdf(xx, 1), color=C_BAD, lw=1.4)
            axB.axvline(CHI2_HI,  color=C_BAD, ls="--", lw=0.9)
            axB.axvline(CHI2_MED, color=C_CHI, ls=":")
            axB.axvline(CHI2_LO,  color=C_BAD, ls="--", lw=0.9)
            axB.set_xscale("log"); axB.set_yscale("log")
            axB.set_ylim(1e-6, 5)
            axB.set_xlabel("NIS"); axB.set_ylabel("density")
            axB.set_title(
                f"(b) NIS dist vs $\\chi^2_1$  med={np.median(nis_pos):.2f}",
                loc="left", pad=4)

    # (c) Batch consistency — Rule 1 inherited
    axC = fig.add_subplot(gs[1, :])
    bag_names = sorted(summary_post.get("bags", {}).keys())
    bar_width = 0.85 / max(1, len(bag_names))
    half = (len(bag_names) - 1) / 2
    pos_axis = np.arange(len(axes))
    for j, bag_name in enumerate(bag_names):
        for i, axis in enumerate(axes):
            ax_block = (summary_post["bags"][bag_name].get("axes", {})
                        .get(axis, {}).get("all"))
            if not isinstance(ax_block, dict):
                continue
            n_i = ax_block.get("n", 0)
            nis_med = ax_block.get("nis_median")
            if not (isinstance(nis_med, (int, float))
                    and math.isfinite(nis_med) and n_i > 0):
                continue
            lo, hi = _per_bar_chi2_band(int(n_i))
            x_pos = pos_axis[i] + (j - half) * bar_width
            ok = bool(lo <= nis_med <= hi)
            color = C_OK if ok else C_BAD
            axC.bar([x_pos], [float(nis_med)],
                    width=bar_width * 0.85, color=color,
                    alpha=0.85, edgecolor="white", lw=0.4)
            band_x0 = x_pos - bar_width * 0.45
            band_w = bar_width * 0.9
            axC.add_patch(Rectangle((band_x0, lo), band_w, hi - lo,
                                    facecolor="#d4e6f1",
                                    edgecolor="#7aa9d4", lw=0.3,
                                    alpha=0.45, zorder=1))
    axC.axhline(1.0, color=C_OK, lw=1.0)
    axC.axhline(CHI2_MED, color=C_CHI, ls=":", lw=0.7)
    axC.set_yscale("log")
    axC.set_ylim(1e-3, 1e3)
    axC.set_xticks(pos_axis)
    axC.set_xticklabels(axes, rotation=20)
    axC.set_ylabel(r"$\bar{NIS}$")
    axC.set_title("(c) Batch NIS consistency  (per-bar $\\chi^2(n_i)/n_i$ bands)",
                  loc="left", pad=4)

    # (d) Q-sweep placeholder
    axD = fig.add_subplot(gs[2, 0])
    axD.set_facecolor("#f5f5f5")
    axD.text(0.5, 0.5,
             "Q-sweep curve\n(qfig6)\n\nrequires offline EKF replay\n"
             "—procedure §8.3—",
             transform=axD.transAxes, ha="center", va="center",
             fontsize=9, color=C_CHI,
             bbox=dict(boxstyle="round,pad=0.5", facecolor="white",
                       edgecolor="#bbbbbb", lw=0.5))
    axD.set_title("(d) Q-sweep (deferred)", loc="left", pad=4)
    axD.set_xticks([]); axD.set_yticks([])

    # (e) per-regime box for dvl.vz only — n=0 regimes annotated
    axE = fig.add_subplot(gs[2, 1])
    regs = ("idle", "surge", "heave", "step", "steady")
    box_data: list[np.ndarray] = []
    for r in regs:
        per_bag = _collect_field_list(summary_post, worst_axis, r, "nis_median")
        if per_bag:
            box_data.append(np.array(per_bag))
        else:
            box_data.append(np.array([np.nan]))
    bp = axE.boxplot(box_data, showfliers=False, patch_artist=True,
                     medianprops=dict(color="black"),
                     boxprops=dict(facecolor=C_BAD, alpha=0.5,
                                   edgecolor="black"))
    for i, (r, data) in enumerate(zip(regs, box_data)):
        if not np.any(np.isfinite(data)):
            axE.text(i + 1, 1.0, "n=0", ha="center", va="center",
                     fontsize=7, color="#888888")
    axE.axhspan(CHI2_LO, CHI2_HI, color="#d4e6f1", alpha=0.6)
    axE.axhline(CHI2_MED, color=C_CHI, ls=":")
    axE.set_yscale("log")
    axE.set_xticks(np.arange(1, len(regs) + 1))
    axE.set_xticklabels(regs, rotation=20)
    axE.set_ylabel("NIS")
    axE.set_title(f"(e) {worst_axis} NIS by regime", loc="left", pad=4)

    # (f) mini dashboard
    axF = fig.add_subplot(gs[2, 2])
    grid = np.full((len(axes), len(regs)), np.nan)
    for i, a in enumerate(axes):
        for j, r in enumerate(regs):
            v = _collect_field_list(summary_post, a, r, "nis_median")
            if v:
                grid[i, j] = float(np.median(v))
    score = np.log10(np.where(grid > 0, grid, np.nan)) - np.log10(CHI2_MED)
    im = axF.imshow(score, cmap="RdBu_r", vmin=-3, vmax=3, aspect="auto")
    for i in range(len(axes)):
        for j in range(len(regs)):
            if not math.isfinite(grid[i, j]):
                axF.add_patch(Rectangle((j - 0.5, i - 0.5), 1, 1,
                                        facecolor="#eeeeee",
                                        edgecolor="white",
                                        hatch="////", lw=0.5, alpha=0.9))
    axF.set_xticks(range(len(regs)))
    axF.set_xticklabels(regs, rotation=30, fontsize=8)
    axF.set_yticks(range(len(axes)))
    axF.set_yticklabels([a.replace("imu.", "").replace("dvl.", "")
                         .replace("pressure.", "p_")
                         for a in axes], fontsize=8)
    axF.set_title("(f) Health dashboard", loc="left", pad=4)
    fig.colorbar(im, ax=axF, fraction=0.04, pad=0.02)

    fig.suptitle(
        f"POLARIS — Q tuning report  "
        f"(St. Moritz {summary_post.get('bags_root', '')[-10:]}, "
        f"{summary_post.get('n_bags_processed', '?')} bags)",
        x=0.02, ha="left", fontweight="bold", fontsize=12)
    fig.savefig(out_path)
    plt.close(fig)


# ───────────────────────────────────────── driving signal helper

def _drive_signal_for_axis(axis: str, bag_data: dict[str, np.ndarray]
                           ) -> tuple[np.ndarray, np.ndarray, str] | None:
    """Driving signal for the qfig1 (d) panel. We use the per-axis NIS
    array's own time grid to display a synthetic 'driving' curve based
    on cumulative residual energy as a fallback. Caller should pass a
    proper drive when available."""
    return None


def _bag_drive_signal(axis: str, bag_dir: Path
                      ) -> tuple[np.ndarray, np.ndarray, str] | None:
    """Re-read the bag for an authoritative driving signal (z, Euler,
    velocity)."""
    if not (bag_dir / "metadata.yaml").is_file():
        return None
    bag = era._read_bag(bag_dir)
    if bag.ekf_t_ns.size == 0:
        return None
    t0 = int(bag.ekf_t_ns[0])
    t = (bag.ekf_t_ns - t0) * 1e-9
    if axis == "pressure.z":
        return t, bag.ekf_z, "ekf z [m]"
    if axis == "imu.roll":
        return t, bag.ekf_euler[:, 0], "ekf roll [rad]"
    if axis == "imu.pitch":
        return t, bag.ekf_euler[:, 1], "ekf pitch [rad]"
    if axis == "imu.yaw":
        return t, bag.ekf_euler[:, 2], "ekf yaw [rad]"
    if axis == "dvl.vx":
        return t, bag.ekf_v[:, 0], "ekf vx [m/s]"
    if axis == "dvl.vy":
        return t, bag.ekf_v[:, 1], "ekf vy [m/s]"
    if axis == "dvl.vz":
        return t, bag.ekf_v[:, 2], "ekf vz [m/s]"
    if axis.startswith("imu.omega"):
        idx = {"imu.omega_x": 0, "imu.omega_y": 1, "imu.omega_z": 2}[axis]
        return t, bag.ekf_omega[:, idx], f"ekf {axis.split('.')[-1]} [rad/s]"
    return None


# ───────────────────────────────────────── verdict + markdown summary

def _verdict_for_axis(axis: str,
                      summary_post: dict, summary_prior: dict | None,
                      dashboard_grid: dict[str, dict],
                      whiteness_verdict: dict[str, Any]) -> dict[str, Any]:
    """Mirror of the §2.3 decision tree from
    Q_TUNING_IMPROVEMENT_PROCEDURE.md."""
    nis_med_all = _collect_field_list(summary_post, axis, "all", "nis_median")
    excd_all = _collect_field_list(summary_post, axis, "all",
                                    "exceedance_rate_gt_3p84")
    n_total = _collect_field_list(summary_post, axis, "all", "n")
    nis_med = (float(np.median(nis_med_all)) if nis_med_all
               else float("nan"))
    excd = (float(np.median(excd_all)) if excd_all else float("nan"))
    n_med = (int(np.median(n_total)) if n_total else 0)
    ratio = _post_prior_ratio(summary_post, summary_prior, axis)
    q_idx = Q_INDEX.get(axis, -1)

    def _make(bucket, action, q_factor=None):
        return dict(axis=axis, bucket=bucket, action=action,
                    nis_median=nis_med, exceedance=excd, n=n_med,
                    ratio=ratio, q_factor=q_factor)

    if axis in DIAGNOSTIC_AXES:
        return _make("Diagnostic only",
                     "Gyro is not fused. Residual reflects the EKF's "
                     "constant-velocity propagation; not a Q knob.")

    collapse = 5.0
    if axis in ORIENTATION_AXES and math.isfinite(ratio) and ratio >= collapse:
        return _make("Held back (post-update collapse)",
                     f"posterior↔prior_approx ratio = {ratio:.1f}× ≥ "
                     f"{collapse}×. NIS not interpretable until offline "
                     "replay exposes the prior P (procedure §8.3).")

    # Per-bag pass count from qfig3 — if none of the bars pass,
    # filter is failing in some way for the axis.
    if not math.isfinite(nis_med) or n_med == 0:
        return _make("Needs more dynamic data",
                     f"No usable samples (n_med = {n_med}).")

    # Use χ²(n_i)/n_i band on the median sample count as a heuristic.
    band_lo, band_hi = _per_bar_chi2_band(n_med)

    if band_lo <= nis_med <= band_hi:
        return _make("Tunable now: confirm + freeze",
                     f"NIS_median {nis_med:.3g} in band [{band_lo:.3g}, "
                     f"{band_hi:.3g}]. Filter is consistent. "
                     f"Leave Q[{q_idx}] = current value.")

    grid = dashboard_grid.get(axis, {})
    fail = [r for r, v in grid.items()
            if v is not None and (v > CHI2_HI or v < CHI2_LO)]
    pass_r = [r for r, v in grid.items()
              if v is not None and CHI2_LO <= v <= CHI2_HI]
    if (fail and pass_r and len(fail) == 1
            and not (math.isfinite(ratio) and ratio >= collapse)):
        return _make("Needs more diagnosis",
                     f"NIS fails only in '{fail[0]}' regime "
                     f"(NIS_med = {grid[fail[0]]:.3g}); other regimes pass. "
                     "Likely regime-specific model mismatch; investigate "
                     "before a blanket Q change.")

    if nis_med > band_hi and (math.isnan(ratio) or ratio < collapse):
        return _make("Tunable now: raise Q",
                     f"NIS_median {nis_med:.3g} > band upper "
                     f"{band_hi:.3g}. Moment-match: "
                     f"Q[{q_idx}]_new = Q[{q_idx}]_current × "
                     f"{nis_med:.2g}.",
                     q_factor=nis_med)

    if nis_med < band_lo and (math.isnan(ratio) or ratio < collapse):
        return _make("Tunable now: lower Q (low confidence)",
                     f"NIS_median {nis_med:.3g} < band lower "
                     f"{band_lo:.3g}. Q[{q_idx}] could be lowered, but "
                     "confirm prior_approx ratio < 5× before committing.")

    if math.isfinite(ratio) and ratio >= collapse:
        return _make("Held back (post-update collapse)",
                     f"posterior↔prior_approx ratio = {ratio:.1f}× ≥ "
                     f"{collapse}×; NIS not interpretable.")

    if n_med < 100:
        return _make("Needs more dynamic data",
                     f"Median sample count {n_med} < 100; aggregates unstable.")

    return _make("Needs more diagnosis",
                 "Inconclusive; examine residual band and ACF.")


def write_summary_md(out_path: Path,
                     summary_post: dict, summary_prior: dict | None,
                     verdicts: list[dict[str, Any]],
                     whiteness_verdict: dict[str, Any],
                     dashboard_grid: dict[str, dict],
                     showcase: dict[str, str],
                     showcase_substitutions: dict[str, dict[str, str]],
                     pass_counts: dict[str, tuple[int, int]],
                     out_dir: Path) -> None:
    bags_root = summary_post.get("bags_root", "<unknown>")
    n_bags = summary_post.get("n_bags_processed", 0)
    q_ref = summary_post.get("Q_diagonal_ref")
    q_ref_src = summary_post.get("Q_diagonal_ref_source", "<unspecified>")
    imu_topic = summary_post.get("imu_topic", "<unspecified>")

    L: list[str] = []
    L.append(f"# Q-tuning summary  —  {Path(bags_root).name}")
    L.append("")
    L.append(f"- bags_root: `{bags_root}`")
    L.append(f"- n_bags: {n_bags}")
    L.append(f"- imu_topic: `{imu_topic}`")
    L.append(f"- Q_diagonal_ref source: `{q_ref_src}`")
    if q_ref:
        labels = ["x", "y", "z", "roll", "pitch", "yaw",
                  "vx", "vy", "vz", "ωx", "ωy", "ωz",
                  "ax", "ay", "az"]
        L.append("- Q diagonal at recording time:")
        L.append("")
        L.append("  | i | state | Q[i] |")
        L.append("  |---|---|---:|")
        for i, (lab, v) in enumerate(zip(labels, q_ref)):
            L.append(f"  | {i} | {lab} | {v:g} |")
    L.append("")
    if showcase_substitutions:
        L.append("## Showcase bag substitutions")
        L.append("")
        L.append("Some axes were assigned a showcase bag whose data was not "
                 "directly available; the actual bag used is documented "
                 "below for traceability.")
        L.append("")
        L.append("| axis | requested showcase | actual showcase | reason |")
        L.append("|---|---|---|---|")
        for axis, info in showcase_substitutions.items():
            L.append(f"| `{axis}` | `{info['requested']}` | "
                     f"`{info['actual']}` | {info['reason']} |")
        L.append("")
    L.append("## Per-axis verdicts")
    L.append("")
    L.append("| axis | bucket | NIS_med | exceed | n | post↔prior ratio | "
             "pass count | Q proposed × |")
    L.append("|---|---|---:|---:|---:|---:|:---:|---:|")
    for v in verdicts:
        nm = v["nis_median"]
        ex = v["exceedance"]
        n = v["n"]
        ratio = v["ratio"]
        q_f = v["q_factor"]
        nm_s = "—" if nm is None or not math.isfinite(nm) else f"{nm:.3g}"
        ex_s = ("—" if ex is None or not math.isfinite(ex)
                else f"{ex*100:.1f}%")
        ratio_s = ("—" if ratio is None or not math.isfinite(ratio)
                   else f"{ratio:.1f}")
        q_s = "—" if q_f is None else f"{q_f:.2g}×"
        ok, tot = pass_counts.get(v["axis"], (0, 0))
        pc = f"{ok}/{tot}" if tot else "—"
        L.append(f"| `{v['axis']}` | {v['bucket']} | {nm_s} | "
                 f"{ex_s} | {n} | {ratio_s} | {pc} | {q_s} |")
    L.append("")
    L.append("## Per-axis details")
    L.append("")
    for v in verdicts:
        L.append(f"### `{v['axis']}` — {v['bucket']}")
        L.append("")
        L.append(v["action"])
        wv = whiteness_verdict.get(v["axis"], {})
        if wv.get("verdict") and wv.get("verdict") != "NA":
            verdict_word = wv["verdict"]
            if verdict_word == "DEFERRED":
                L.append(f"- whiteness: **deferred** "
                         f"(post↔prior ratio = {wv.get('ratio', float('nan')):.1f}× "
                         "≥ 5×).")
            else:
                lags = wv.get("lags_outside")
                ci = wv.get("ci_95", float("nan"))
                mode_used = wv.get("mode_used", "—")
                L.append(f"- whiteness: **{verdict_word}** "
                         f"(mode={mode_used}, lags outside CI: {lags}, "
                         f"CI ±{ci:.3g})")
        # Per-axis figure references.
        d = _safe_axis_dir(v["axis"])
        if v["axis"] in TUNE_AXES:
            L.append(f"- figures: [`{d}/qfig1_residual_band.png`]({d}/qfig1_residual_band.png), "
                     f"[`{d}/qfig2_nis_distribution.png`]({d}/qfig2_nis_distribution.png)")
        elif v["axis"] in HELD_BACK_AXES:
            L.append(f"- figure: [`{d}/qfig1_residual_band.png`]({d}/qfig1_residual_band.png) "
                     "(paired posterior + prior_approx)")
        elif v["axis"] in CONFIRM_AXES:
            L.append(f"- figure: [`{d}/qfig1_residual_band.png`]({d}/qfig1_residual_band.png)")
        else:  # diagnostic only
            L.append("- diagnostic only — appears in qfig5 and qfig7 only.")
        L.append("")
    L.append("## qfig6 (Q-sweep) — deferred")
    L.append("")
    L.append("qfig6 is a placeholder; full Q-sweep requires offline EKF "
             "replay (procedure §8.3). The placeholder optionally shows "
             "the moment-match prediction for `dvl.vz` "
             "(`Q_new ≈ Q_current × NIS_med`).")
    L.append("")
    L.append("## Files")
    L.append("")
    L.append("- `qfig3_batch_nis_consistency.png`")
    L.append("- `qfig4_innovation_whiteness.png`")
    L.append("- `qfig5_regime_boxplot.png`")
    L.append("- `qfig6_q_sweep.png` (placeholder)")
    L.append("- `qfig7_consistency_dashboard.png`")
    L.append("- `qfig8_q_tuning_report_panel.png`")
    L.append("- `q_tuning_summary.md` (this file)")
    L.append("- per-axis subdirs: `pressure_z/`, `dvl_{vx,vy,vz}/`, "
             "`imu_{roll,pitch,yaw}/`")
    L.append("")
    out_path.write_text("\n".join(L), encoding="utf-8")


# ───────────────────────────────────────── CLI

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("diagnostics_dir", type=Path,
                    help="Path to a posterior diagnostics_<mode>/ folder.")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Default: <diagnostics_dir>/../q_tuning_figures/")
    ap.add_argument("--bags-root", type=Path, default=None,
                    help="Root of raw rosbags. Default: inferred from "
                         "summary['bags_root'].")
    ap.add_argument("--prior-approx-summary", type=Path, default=None,
                    help="Path to prior_approx diagnostics_summary.json. "
                         "Default: sibling diagnostics_prior_approx/.")
    ap.add_argument("--axes", default=None,
                    help="Comma list of axes (default: all 10)")
    ap.add_argument("--report-md", type=Path, default=None,
                    help="Markdown summary path. Default: "
                         "<output-dir>/q_tuning_summary.md")
    args = ap.parse_args(argv)

    diag_post = args.diagnostics_dir.resolve()
    summary_post = _load_summary(diag_post / "diagnostics_summary.json")
    print(f"diagnostics_dir: {diag_post}")
    print(f"  n_bags: {summary_post.get('n_bags_processed', '?')}")

    diag_prior = None
    summary_prior = None
    if args.prior_approx_summary is not None:
        p = args.prior_approx_summary.resolve()
        if p.is_file():
            summary_prior = _load_summary(p)
            diag_prior = p.parent
    else:
        sibling = diag_post.parent / "diagnostics_prior_approx" / "diagnostics_summary.json"
        if sibling.is_file():
            summary_prior = _load_summary(sibling)
            diag_prior = sibling.parent
    if summary_prior is not None:
        print(f"  prior_approx: n_bags={summary_prior.get('n_bags_processed', '?')}")
    else:
        print("  prior_approx: NOT FOUND — ratios will be n/a")

    bags_root = args.bags_root or Path(summary_post.get("bags_root",
                                                          "."))
    bags_root = bags_root.resolve()
    print(f"  bags_root: {bags_root}")

    if args.axes:
        axes = [a.strip() for a in args.axes.split(",") if a.strip()]
    else:
        axes = list(ALL_AXES)
    print(f"  axes: {axes}")

    out_dir = args.output_dir or (diag_post.parent / "q_tuning_figures")
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"  output dir: {out_dir}")

    # Showcase mapping with substitutions.
    bag_names = list(summary_post.get("bags", {}).keys())

    def _has(bag): return bag in bag_names

    showcase: dict[str, str] = {}
    substitutions: dict[str, dict[str, str]] = {}

    showcase["pressure.z"] = "heave_05_2026_05_07-14_27_55"
    showcase["dvl.vz"] = "heave_05_2026_05_07-14_27_55"
    # dvl.vx — user wanted 15:19:49 (no metadata); fall back to 14:08:19.
    requested_vx = "surge_02_2026_05_07-15_19_49"
    actual_vx = "surge_02_2026_05_07-14_08_19"
    showcase["dvl.vx"] = actual_vx
    if requested_vx != actual_vx:
        substitutions["dvl.vx"] = {
            "requested": requested_vx, "actual": actual_vx,
            "reason": "the 15:19 take has no metadata.yaml and was excluded "
                      "from the good-bag set; the 14:08 take is the surge_02 "
                      "we have.",
        }
    # dvl.vy — yaw bag, but yaw bags do not include dvl.vy in the script's
    # per-bag-type axis selection. Use the same yaw bag and recompute on
    # demand (handled in qfig1 path below).
    showcase["dvl.vy"] = "yaw_02_2026_05_07-14_00_38"
    substitutions["dvl.vy"] = {
        "requested": "yaw_02_2026_05_07-14_00_38",
        "actual": "yaw_02_2026_05_07-14_00_38 (residual recomputed on demand)",
        "reason": "yaw bags do not include dvl.vy in the per-bag-type axis "
                  "selection of ekf_residual_diagnostics.py; "
                  "q_tuning_figures.py calls era._compute_residual on the "
                  "raw bag to populate the showcase.",
    }
    showcase["imu.roll"] = "roll_01_2026_05_07-14_17_07"
    showcase["imu.pitch"] = "pitch_01_2026_05_07-14_18_58"
    showcase["imu.yaw"] = "yaw_02_2026_05_07-14_00_38"

    # ----------------------------------------------------------------
    # qfig3 — batch consistency
    # ----------------------------------------------------------------
    print("\n== qfig3 batch NIS consistency ==")
    pass_counts = fig3_batch_consistency(
        summary_post, axes, out_dir / "qfig3_batch_nis_consistency.png")
    for axis, (ok, tot) in pass_counts.items():
        print(f"  {axis:<14}  pass {ok}/{tot}")

    # ----------------------------------------------------------------
    # qfig4 — whiteness
    # ----------------------------------------------------------------
    print("\n== qfig4 innovation whiteness (Rule 2) ==")
    whiteness_verdict = fig4_whiteness(
        diag_post, summary_post,
        diag_prior, summary_prior,
        axes, out_dir / "qfig4_innovation_whiteness.png")
    for axis, info in whiteness_verdict.items():
        print(f"  {axis:<14}  {info.get('verdict', 'NA')}")

    # ----------------------------------------------------------------
    # qfig5 — per-regime boxplot
    # ----------------------------------------------------------------
    print("\n== qfig5 per-regime boxplot ==")
    fig5_regime_boxplot(summary_post, axes, list(REGIMES_PLOT),
                        out_dir / "qfig5_regime_boxplot.png")

    # ----------------------------------------------------------------
    # qfig6 — placeholder
    # ----------------------------------------------------------------
    print("\n== qfig6 placeholder ==")
    fig6_q_sweep_placeholder(summary_post, out_dir / "qfig6_q_sweep.png",
                             worst_axis="dvl.vz")

    # ----------------------------------------------------------------
    # qfig7 — dashboard
    # ----------------------------------------------------------------
    print("\n== qfig7 dashboard ==")
    dashboard_grid = fig7_dashboard(
        summary_post, axes, list(REGIMES_PLOT),
        out_dir / "qfig7_consistency_dashboard.png")

    # ----------------------------------------------------------------
    # qfig8 — composite
    # ----------------------------------------------------------------
    print("\n== qfig8 composite (worst axis: dvl.vz) ==")
    fig8_composite(diag_post, summary_post, showcase, axes, bags_root,
                   out_dir / "qfig8_q_tuning_report_panel.png")

    # ----------------------------------------------------------------
    # qfig1 / qfig2 — per axis
    # ----------------------------------------------------------------
    per_axis_plan = {
        "pressure.z":    {"qfig1": True,  "qfig2": False, "paired": False},
        "dvl.vx":        {"qfig1": True,  "qfig2": True,  "paired": False},
        "dvl.vy":        {"qfig1": True,  "qfig2": True,  "paired": False},
        "dvl.vz":        {"qfig1": True,  "qfig2": True,  "paired": False},
        "imu.roll":      {"qfig1": True,  "qfig2": False, "paired": True},
        "imu.pitch":     {"qfig1": True,  "qfig2": False, "paired": True},
        "imu.yaw":       {"qfig1": True,  "qfig2": False, "paired": True},
    }

    for axis, plan in per_axis_plan.items():
        bag_name = showcase[axis]
        print(f"\n== qfig1/qfig2 for {axis} on {bag_name} ==")
        sub_dir = out_dir / _safe_axis_dir(axis)
        sub_dir.mkdir(parents=True, exist_ok=True)

        # Resolve posterior series (NPZ first, recompute fallback).
        post_blob = _load_npz_axis(diag_post, bag_name, axis, "posterior")
        if post_blob is None:
            print(f"    posterior NPZ missing for {axis} × {bag_name}; recomputing")
            post_blob = _recompute_axis_from_bag(bags_root, bag_name, axis,
                                                  "posterior")
        if post_blob is None:
            print(f"    [skip] no data for {axis} × {bag_name}")
            continue

        # Driving signal — re-read bag once.
        bag_dir = bags_root / bag_name
        drive = _bag_drive_signal(axis, bag_dir)

        # Q value for the suptitle.
        q_diag = summary_post.get("Q_diagonal_ref") or []
        q_idx = Q_INDEX.get(axis, -1)
        q_value = (float(q_diag[q_idx]) if 0 <= q_idx < len(q_diag) else None)

        # qfig1
        if plan["paired"]:
            prior_blob = None
            if diag_prior is not None:
                prior_blob = _load_npz_axis(diag_prior, bag_name, axis,
                                              "prior_approx")
            fig1_paired(axis, bag_name, post_blob, prior_blob,
                        drive, q_value,
                        sub_dir / "qfig1_residual_band.png")
        else:
            fig1_per_axis(axis, bag_name, post_blob, drive, q_value,
                          "posterior",
                          sub_dir / "qfig1_residual_band.png")

        # qfig2
        if plan["qfig2"]:
            fig2_per_axis(axis, diag_post, summary_post,
                          diag_prior, summary_prior,
                          sub_dir / "qfig2_nis_distribution.png")

    # ----------------------------------------------------------------
    # Verdicts + markdown summary
    # ----------------------------------------------------------------
    print("\n== verdicts ==")
    verdicts = []
    for axis in axes:
        v = _verdict_for_axis(axis, summary_post, summary_prior,
                              dashboard_grid, whiteness_verdict)
        verdicts.append(v)
        print(f"  {axis:<14}  {v['bucket']}")

    report_md = args.report_md or (out_dir / "q_tuning_summary.md")
    write_summary_md(report_md, summary_post, summary_prior, verdicts,
                     whiteness_verdict, dashboard_grid, showcase,
                     substitutions, pass_counts, out_dir)
    print(f"\nWrote markdown summary: {report_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
