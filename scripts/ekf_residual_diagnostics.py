#!/usr/bin/env python3
"""
Regime-split diagnostic on top of `scripts/ekf_residual_analysis.py`.

For each of the 16 good bags from 2026-04-23, this script:

  1. Re-loads the bag (using ekf_residual_analysis._read_bag).
  2. Loads /pixhawk/manual_control to derive vehicle command regimes.
  3. Derives a pressure-regime label from the EKF z-state
     (steady / step / pump).
  4. Recomputes per-axis residuals (using the same _compute_residual
     function as the first batch) and KEEPS the per-sample
     [t, residual, R, P, S, NIS] arrays.
  5. Splits each axis by regime and computes:
       n, mean(r), median(r), std(r), mad_std,
       R_mean, P_mean, S_mean,
       var/R, var/P, var/S,
       NIS_mean, NIS_median, NIS_p95,
       exceedance_rate = mean(NIS > 3.84).
  6. Writes per-bag JSON and 4-panel timeline plots
     (residual ±2sqrt(S), S, NIS [log], driving signal).
  7. Aggregates across bags and prints / appends a synthesis report.

The first-pass NIS finding (residuals computed against the EKF
posterior; S uses posterior pose/twist covariance) is preserved here
verbatim for traceability — and called out in the synthesis as the
dominant cause of orientation NIS ~ 0.

Usage
-----
python scripts/ekf_residual_diagnostics.py [<bags-root>]
        [--output-dir DIR] [--include CSV] [--exclude CSV]
        [--diag-sensors all|pressure|imu_orient|imu_omega|dvl]
        [--report-md MD_PATH]
"""
from __future__ import annotations

import argparse
import json
import math
import sys

# Windows consoles default to cp1252; force UTF-8 so the synthesis prints cleanly.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

# Reuse the existing residual machinery — keep this in sync, no copy/paste.
import ekf_residual_analysis as era  # noqa: E402

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("Install: pip install matplotlib", file=sys.stderr)
    raise


# ───────────────────────────────────────── constants

# Manual-control neutral / threshold (mirrors analyze_mcap_command_segmentation.py).
MC_NEUTRAL = (0, 0, 500, 0, 0, 0)
MC_THRESH = 45
MC_LABELS = ("x_surge", "y_sway", "z_heave", "r_yaw", "s_roll", "t_pitch")
TOPIC_MC = "/pixhawk/manual_control"

# Pressure regime thresholds.
P_STEP_DZDT = 0.10            # m/s — |dz/dt| above this = step
P_STEADY_DZDT = 0.05          # m/s — |dz/dt| below this is candidate steady
P_STEADY_DEV = 0.05           # m   — |z - rolling_med(z, 5s)| within this = steady
P_DZDT_WIN_S = 1.5            # window for |dz/dt|
P_MED_WIN_S = 5.0             # rolling-median window for steady-flutter test

# NIS threshold for 1-DOF chi-square 95%
NIS_CHI2_95 = 3.841           # chi^2_{1, 0.95}

# Default bags = same 16-bag good list documented in
# POLARIS/research/EKF_RESEARCH_NOTES.md  [Update 2026-04-25].
GOOD_BAGS_FILTER = (
    "yaw_turns_01_2026_04_23-13_11_12,"
    "yaw_turns_02_2026_04_23-13_13_52,"
    "yaw_turns_03_2026_04_23-13_17_48,"
    "vertical_05_2026_04_23-15_21_15,"
    "vertical_06_2026_04_23-15_21_49,"
    "straight_surge_02_2026_04_23-13_35_18,"
    "straight_surge_03_2026_04_23-13_37_29,"
    "straight_surge_04_2026_04_23-13_40_15,"
    "straight_surge_06_2026_04_23-13_45_45,"
    "straight_surge_07_2026_04_23-13_49_11,"
    "straight_surge_09_2026_04_23-13_52_13,"
    "straight_surge_10_2026_04_23-13_56_09,"
    "stationary_02_2026_04_23-12_57_41,"
    "stationary_01_2026_04_23-14_19_03,"
    "depth_hold_02_2026_04_23-19_53_12,"
    "depth_hold_2026_04_23-19_05_38"
)

# Axis-group selection per --diag-sensors.
DIAG_GROUPS: dict[str, list[str]] = {
    "pressure":   ["pressure.z"],
    "imu_orient": ["imu.roll", "imu.pitch", "imu.yaw"],
    "imu_omega":  ["imu.omega_x", "imu.omega_y", "imu.omega_z"],
    "dvl":        ["dvl.vx", "dvl.vy", "dvl.vz"],
    "all":        list(era.ALL_AXES),
}

# Regime palette — light theme, matches q_tuning_figure_examples/make_q_tuning_examples.py.
REGIME_COLORS = {
    "idle":       "#cfd8dc",
    "steady":     "#d4e6f1",
    "step":       "#fbe5d6",
    "pump":       "#e8daef",
    "surge":      "#fadbd8",
    "reverse":    "#f5cba7",
    "sway":       "#a78bfa",
    "heave":      "#d5f5e3",
    "yaw":        "#d6eaf8",
    "roll_pitch": "#fdebd0",
    "mixed":      "#fcf3cf",
    "locked":     "#3dd6c6",
    "unlocked":   "#c44e52",
}


# ───────────────────────────────────────── manual_control reader

@dataclass
class CmdSeries:
    t_ns: np.ndarray            # (M,)
    ch: np.ndarray              # (M, 6) int16
    deviated: np.ndarray        # (M, 6) bool — per-channel beyond MC_THRESH

    @property
    def empty(self) -> bool:
        return self.t_ns.size == 0


def _read_manual_control(bag_dir: Path) -> CmdSeries:
    """Load /pixhawk/manual_control. Stamp = log time (no header)."""
    t_ns: list[int] = []
    ch_rows: list[list[int]] = []
    with AnyReader([bag_dir]) as reader:
        conns = [c for c in reader.connections if c.topic == TOPIC_MC]
        if not conns:
            return CmdSeries(np.empty(0, dtype=np.int64),
                             np.empty((0, 6), dtype=np.int16),
                             np.empty((0, 6), dtype=bool))
        for c, log_ts, raw in reader.messages(connections=conns):
            try:
                msg = reader.deserialize(raw, c.msgtype)
            except Exception:
                continue
            data = list(getattr(msg, "data", []))[:6]
            if len(data) < 6:
                continue
            t_ns.append(int(log_ts))
            ch_rows.append([int(v) for v in data])
    if not t_ns:
        return CmdSeries(np.empty(0, dtype=np.int64),
                         np.empty((0, 6), dtype=np.int16),
                         np.empty((0, 6), dtype=bool))
    t = np.asarray(t_ns, dtype=np.int64)
    ch = np.asarray(ch_rows, dtype=np.int16)
    # sort
    order = np.argsort(t, kind="mergesort")
    t = t[order]
    ch = ch[order]
    dev = np.zeros_like(ch, dtype=bool)
    for i in range(6):
        dev[:, i] = np.abs(ch[:, i].astype(np.int32) - MC_NEUTRAL[i]) > MC_THRESH
    return CmdSeries(t_ns=t, ch=ch, deviated=dev)


# ───────────────────────────────────────── regime classification


def _label_cmd_regime(deviated_at_t: np.ndarray) -> str:
    """Map a 6-bool channel-deviation vector to a regime name."""
    if not deviated_at_t.any():
        return "idle"
    surge = bool(deviated_at_t[0])
    sway  = bool(deviated_at_t[1])
    heave = bool(deviated_at_t[2])
    yaw   = bool(deviated_at_t[3])
    roll  = bool(deviated_at_t[4])
    pitch = bool(deviated_at_t[5])
    active = sum([surge, sway, heave, yaw, roll, pitch])
    if active >= 3:
        return "mixed"
    if surge and active == 1:
        return "surge"   # surge_sign decided downstream from ch value
    if sway and active == 1:
        return "sway"
    if heave and active == 1:
        return "heave"
    if yaw and active == 1:
        return "yaw"
    if (roll or pitch) and active <= 2:
        return "roll_pitch"
    return "mixed"


def _resolve_cmd_regimes_at(cmd: CmdSeries, query_t_ns: np.ndarray) -> np.ndarray:
    """For each query time, return the cmd_regime label of the latest sample <= t.
    Surge-vs-reverse is split by sign of the surge channel when surge active."""
    out = np.empty(query_t_ns.size, dtype=object)
    if cmd.empty or query_t_ns.size == 0:
        out[:] = "idle"
        return out
    idxs = np.searchsorted(cmd.t_ns, query_t_ns, side="right") - 1
    idxs = np.clip(idxs, 0, cmd.t_ns.size - 1)
    for i, ci in enumerate(idxs):
        regime = _label_cmd_regime(cmd.deviated[ci])
        if regime == "surge":
            v = int(cmd.ch[ci, 0])
            if v < 0:
                regime = "reverse"
        out[i] = regime
    return out


def _rolling_central_diff(t_s: np.ndarray, x: np.ndarray,
                          win_s: float) -> np.ndarray:
    """Centered difference over the smallest window covering at least win_s.
    Robust enough for non-uniform sampling at ~30 Hz."""
    n = x.size
    if n == 0:
        return np.empty(0)
    half = win_s / 2.0
    out = np.empty(n)
    j_lo = 0
    j_hi = 0
    for i in range(n):
        while j_lo < n and t_s[j_lo] < t_s[i] - half:
            j_lo += 1
        while j_hi < n and t_s[j_hi] < t_s[i] + half:
            j_hi += 1
        lo = max(0, j_lo)
        hi = min(n - 1, j_hi - 1 if j_hi > 0 else 0)
        if lo == hi:
            out[i] = 0.0
        else:
            out[i] = (x[hi] - x[lo]) / max(1e-9, (t_s[hi] - t_s[lo]))
    return out


def _rolling_median(t_s: np.ndarray, x: np.ndarray, win_s: float) -> np.ndarray:
    """Centered rolling median, slow but adequate for ~30 Hz EKF output."""
    n = x.size
    if n == 0:
        return np.empty(0)
    half = win_s / 2.0
    out = np.empty(n)
    j_lo = 0
    j_hi = 0
    for i in range(n):
        while j_lo < n and t_s[j_lo] < t_s[i] - half:
            j_lo += 1
        while j_hi < n and t_s[j_hi] < t_s[i] + half:
            j_hi += 1
        lo = j_lo
        hi = j_hi if j_hi > j_lo else j_lo + 1
        out[i] = float(np.median(x[lo:min(hi, n)]))
    return out


def _classify_pressure_regime(ekf_t_s: np.ndarray,
                              ekf_z: np.ndarray) -> np.ndarray:
    """Per-EKF-sample label in {steady, step, pump}."""
    if ekf_z.size == 0:
        return np.empty(0, dtype=object)
    dz = _rolling_central_diff(ekf_t_s, ekf_z, P_DZDT_WIN_S)
    z_med = _rolling_median(ekf_t_s, ekf_z, P_MED_WIN_S)
    abs_dz = np.abs(dz)
    abs_dev = np.abs(ekf_z - z_med)
    out = np.empty(ekf_z.size, dtype=object)
    is_step = abs_dz > P_STEP_DZDT
    is_steady = (abs_dz < P_STEADY_DZDT) & (abs_dev < P_STEADY_DEV)
    out[:] = "pump"
    out[is_steady] = "steady"
    out[is_step] = "step"
    return out


# ───────────────────────────────────────── per-axis residual collection

@dataclass
class AxisSeries:
    axis: str
    t_rel_s: np.ndarray
    resid: np.ndarray
    R: np.ndarray
    P: np.ndarray
    S: np.ndarray
    NIS: np.ndarray
    mode: str = "posterior"
    dt_s: np.ndarray = field(default_factory=lambda: np.empty(0))
    n_skipped: int = 0
    n_total: int = 0


def _collect_axis(bag: era.BagRead, axis: str, max_gap_ns: int,
                  reliability: dict[str, bool],
                  mode: str = "posterior",
                  max_prior_age_ns: int | None = None) -> AxisSeries | None:
    res = era._compute_residual(
        axis, bag, max_gap_ns, reliability,
        mode=mode, max_prior_age_ns=max_prior_age_ns,
    )
    if res is None or res.resid.size == 0:
        return None
    R = res.R
    P = res.P
    P_clean = np.where(np.isfinite(P) & (P > 0), P, 0.0)
    S = R + P_clean
    nis = np.zeros_like(res.resid)
    mask = np.isfinite(S) & (S > 0)
    nis[mask] = (res.resid[mask] ** 2) / S[mask]
    nis[~mask] = np.nan
    out = AxisSeries(axis=axis, t_rel_s=res.t_rel_s, resid=res.resid,
                     R=R, P=P, S=S, NIS=nis)
    # Stash mode + dt + skip count on the dataclass (added below)
    out.mode = res.mode
    out.dt_s = res.dt_s
    out.n_skipped = res.n_skipped
    out.n_total = res.n_total
    return out


# ───────────────────────────────────────── regime-split stats

def _stats_subset(s: AxisSeries, mask: np.ndarray) -> dict[str, Any]:
    if mask.sum() == 0:
        return {"n": 0}
    r = s.resid[mask]
    R = s.R[mask]
    P = s.P[mask]
    S = s.S[mask]
    nis = s.NIS[mask]
    nis_finite = nis[np.isfinite(nis)]
    n = int(r.size)
    mean = float(np.mean(r))
    median = float(np.median(r))
    std = float(np.std(r, ddof=1)) if n > 1 else 0.0
    var = std * std
    mad = float(np.median(np.abs(r - median)))
    mad_std = mad * 1.4826
    R_mean = float(np.mean(R))
    P_finite = P[np.isfinite(P)]
    P_mean = float(np.mean(P_finite)) if P_finite.size else 0.0
    S_finite = S[np.isfinite(S)]
    S_mean = float(np.mean(S_finite)) if S_finite.size else 0.0
    var_over_R = var / R_mean if R_mean > 0 else float("inf")
    var_over_P = var / P_mean if P_mean > 0 else float("inf")
    var_over_S = var / S_mean if S_mean > 0 else float("inf")
    if nis_finite.size > 0:
        nis_mean = float(np.mean(nis_finite))
        nis_median = float(np.median(nis_finite))
        nis_p95 = float(np.percentile(nis_finite, 95))
        exceed = float(np.mean(nis_finite > NIS_CHI2_95))
    else:
        nis_mean = nis_median = nis_p95 = exceed = float("nan")
    return {
        "n": n,
        "mean": mean, "median": median,
        "std": std, "mad_std": mad_std, "variance": var,
        "R_mean": R_mean, "P_mean": P_mean, "S_mean": S_mean,
        "var_over_R": var_over_R, "var_over_P": var_over_P,
        "var_over_S": var_over_S,
        "nis_mean": nis_mean, "nis_median": nis_median, "nis_p95": nis_p95,
        "exceedance_rate_gt_3p84": exceed,
    }


def _regime_split(s: AxisSeries, regime_at: np.ndarray) -> dict[str, dict]:
    """regime_at: array of regime labels parallel to s.t_rel_s (same length)."""
    if regime_at.size != s.resid.size:
        # Length mismatch — likely caller didn't align. Fall back to no split.
        return {"all": _stats_subset(s, np.ones(s.resid.size, dtype=bool))}
    out: dict[str, dict] = {}
    for label in sorted(set(regime_at.tolist())):
        mask = regime_at == label
        out[str(label)] = _stats_subset(s, mask)
    out["all"] = _stats_subset(s, np.ones(s.resid.size, dtype=bool))
    return out


def _resolve_axis_regimes(s: AxisSeries, axis: str,
                          ekf_t_rel_s: np.ndarray,
                          pressure_regime: np.ndarray,
                          cmd_regime_at_ekf: np.ndarray) -> np.ndarray:
    """Return per-sample regime labels for this axis, by resolving the
    nearest EKF index for each sample timestamp."""
    if s.t_rel_s.size == 0:
        return np.empty(0, dtype=object)
    if ekf_t_rel_s.size == 0:
        return np.array(["unknown"] * s.t_rel_s.size, dtype=object)
    idx = np.searchsorted(ekf_t_rel_s, s.t_rel_s, side="left")
    idx = np.clip(idx, 0, ekf_t_rel_s.size - 1)
    if axis.startswith("pressure"):
        return pressure_regime[idx]
    return cmd_regime_at_ekf[idx]


# ───────────────────────────────────────── plotting

def _driving_signal(axis: str, bag: era.BagRead) -> tuple[np.ndarray, np.ndarray, str] | None:
    """(t_rel_s, signal, label) for the driving-signal panel."""
    if bag.ekf_t_ns.size == 0:
        return None
    t0 = int(bag.ekf_t_ns[0])
    ekf_t_rel = (bag.ekf_t_ns - t0) * 1e-9
    if axis == "pressure.z":
        return ekf_t_rel, bag.ekf_z, "ekf z [m]"
    if axis.startswith("imu.omega"):
        idx = {"imu.omega_x": 0, "imu.omega_y": 1, "imu.omega_z": 2}[axis]
        # raw IMU omega rotated to base
        if bag.imu_t_ns.size == 0 or bag.static_tf.R_base_imu is None:
            return None
        R_bi = bag.static_tf.R_base_imu
        omega_base = (R_bi @ bag.imu_omega.T).T
        t_rel = (bag.imu_t_ns - int(bag.imu_t_ns[0])) * 1e-9
        return t_rel, omega_base[:, idx], f"omega_{['x','y','z'][idx]} (base) [rad/s]"
    if axis in ("imu.roll", "imu.pitch", "imu.yaw"):
        idx = {"imu.roll": 0, "imu.pitch": 1, "imu.yaw": 2}[axis]
        return ekf_t_rel, bag.ekf_euler[:, idx], f"ekf {axis.split('.')[-1]} [rad]"
    if axis.startswith("dvl."):
        idx = {"dvl.vx": 0, "dvl.vy": 1, "dvl.vz": 2}[axis]
        return ekf_t_rel, bag.ekf_v[:, idx], f"ekf {axis.split('.')[-1]} [m/s]"
    return None


def _shade_regimes(ax, t_rel_s: np.ndarray, regime_at: np.ndarray) -> None:
    """Shade contiguous regions of identical regime labels on a single axis."""
    if t_rel_s.size == 0 or regime_at.size != t_rel_s.size:
        return
    starts = [0]
    for i in range(1, len(regime_at)):
        if regime_at[i] != regime_at[i - 1]:
            starts.append(i)
    starts.append(len(regime_at))
    for k in range(len(starts) - 1):
        i0, i1 = starts[k], starts[k + 1]
        label = regime_at[i0]
        color = REGIME_COLORS.get(str(label), "#444")
        t_start = float(t_rel_s[i0])
        t_end = float(t_rel_s[min(i1, len(t_rel_s) - 1)])
        ax.axvspan(t_start, t_end, color=color, alpha=0.10, lw=0)


def _legend_handles(used_labels: Iterable[str]) -> list:
    from matplotlib.patches import Patch
    return [Patch(facecolor=REGIME_COLORS.get(str(l), "#444"),
                  alpha=0.30, label=str(l))
            for l in dict.fromkeys(used_labels)]


def _plot_axis_diag(s: AxisSeries, axis: str, bag: era.BagRead,
                    bag_name: str, regime_at: np.ndarray,
                    out_path: Path) -> None:
    """Five-panel diagnostic figure per the redesign in
    POLARIS/research/Q_TUNING_PLOT_REDESIGN.md.

    Panels:
      (a) residual with ±2√S band              — qfig1 (timeline) shape
      (b) S (log)                                — innovation covariance
      (c) NIS samples with χ²₁ acceptance band  — lower + median + upper
      (d) driving signal                         — physical sanity
      (e) NIS distribution + χ²₁ PDF             — qfig2 (distribution) shape
    """
    # Light theme — verbatim from
    # POLARIS/research/q_tuning_figure_examples/make_q_tuning_examples.py.
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

    C_RES = "#1f5fa3"
    C_ENV = "#d6263d"
    C_NIS = "#2a8a64"
    C_CHI = "#666666"
    C_BAD = "#c0392b"

    # χ²₁ critical points.
    try:
        from scipy.stats import chi2
        CHI2_LO  = float(chi2.ppf(0.025, 1))
        CHI2_MED = float(chi2.median(1))
        CHI2_HI  = float(chi2.ppf(0.975, 1))
    except Exception:
        CHI2_LO, CHI2_MED, CHI2_HI = 0.000982, 0.4549, 5.024

    fig = plt.figure(figsize=(11.5, 11.0))
    gs = fig.add_gridspec(
        5, 1,
        height_ratios=[1.4, 1.0, 1.2, 1.0, 1.4],
        hspace=0.40,
    )
    axA = fig.add_subplot(gs[0])
    axB = fig.add_subplot(gs[1], sharex=axA)
    axC = fig.add_subplot(gs[2], sharex=axA)
    axD = fig.add_subplot(gs[3], sharex=axA)
    axE = fig.add_subplot(gs[4])  # NOT sharex — independent log-NIS axis

    fig.suptitle(
        f"{bag_name}  —  {axis}  regime diagnostic  [mode={s.mode}]",
        x=0.02, ha="left", fontweight="bold", fontsize=11)

    unit = era._axis_unit(axis)

    # (a) residual with ±2√S band
    env = 2.0 * np.sqrt(np.maximum(s.S, 0.0))
    _shade_regimes(axA, s.t_rel_s, regime_at)
    axA.fill_between(s.t_rel_s, -env, env, color=C_ENV, alpha=0.18,
                     label=r"$\pm 2\sqrt{S}$ (filter $95\,\%$ band)")
    axA.plot(s.t_rel_s, s.resid, color=C_RES, lw=0.7, label="residual r")
    axA.axhline(0.0, color="black", lw=0.5, alpha=0.6)
    axA.set_ylabel(f"residual [{unit}]")
    axA.set_title("(a) Residual within filter $95\\,\\%$ band  —  "
                  "5–10 % of points outside means S (thus Q) is too small",
                  loc="left", pad=4)
    axA.legend(loc="upper right", ncol=2)

    # (b) S
    _shade_regimes(axB, s.t_rel_s, regime_at)
    axB.plot(s.t_rel_s, s.S, color="#7a3a91", lw=1.0, label="S = R + P")
    axB.set_yscale("log")
    axB.set_ylabel(f"S  [{unit}²]")
    axB.set_title("(b) Innovation covariance S (log)", loc="left", pad=4)

    # (c) NIS with χ²₁ acceptance band (LOWER + MEDIAN + UPPER)
    nis_pos = np.where(np.isfinite(s.NIS) & (s.NIS > 0), s.NIS, np.nan)
    _shade_regimes(axC, s.t_rel_s, regime_at)
    axC.scatter(s.t_rel_s, nis_pos, s=4, c=C_NIS, alpha=0.65,
                rasterized=True, label="NIS = r²/S")
    axC.axhline(CHI2_HI,  color=C_BAD, ls="--", lw=1.0,
                label=f"$\\chi^2_{{1,0.975}} = {CHI2_HI:.3f}$")
    axC.axhline(CHI2_MED, color=C_CHI, ls=":",  lw=1.0,
                label=f"$\\chi^2_{{1,0.5}} = {CHI2_MED:.3f}$")
    axC.axhline(CHI2_LO,  color=C_BAD, ls="--", lw=1.0,
                label=f"$\\chi^2_{{1,0.025}} = {CHI2_LO:.3f}$")
    axC.set_yscale("log")
    axC.set_ylabel("NIS")
    axC.set_title("(c) NIS samples with $\\chi^2_1$ acceptance band  —  "
                  "well-tuned: median $\\approx 0.45$; over-confident: median $\\gg 1$",
                  loc="left", pad=4)
    axC.legend(loc="upper right", fontsize=7.5, ncol=2)

    # (d) driving signal
    drv = _driving_signal(axis, bag)
    _shade_regimes(axD, s.t_rel_s, regime_at)
    if drv is not None:
        t_d, x_d, lbl_d = drv
        axD.plot(t_d, x_d, color=C_RES, lw=1.0, label=lbl_d)
        axD.set_ylabel(lbl_d)
    else:
        axD.text(0.5, 0.5, "(no driving signal)", color=C_CHI,
                 ha="center", va="center", transform=axD.transAxes)
        axD.set_ylabel("—")
    axD.set_xlabel("t − t₀ [s]")
    axD.set_title("(d) Driving signal — confirms regime labelling",
                  loc="left", pad=4)

    # (e) NIS distribution + χ²₁ PDF (qfig2 shape, scoped to this axis)
    nis_finite = nis_pos[np.isfinite(nis_pos)]
    if nis_finite.size > 0:
        lo_x = max(1e-6, float(np.nanmin(nis_finite)))
        hi_x = max(CHI2_HI * 5.0, float(np.nanmax(nis_finite)))
        # Guard against degenerate ranges (constant residual sequences).
        if not np.isfinite(lo_x) or not np.isfinite(hi_x) or hi_x <= lo_x:
            lo_x, hi_x = 1e-6, 1e3
        bins = np.logspace(np.log10(lo_x), np.log10(hi_x), 60)
        axE.hist(nis_finite, bins=bins, density=True, color=C_NIS,
                 alpha=0.55, edgecolor="white", lw=0.4,
                 label="empirical")
        try:
            from scipy.stats import chi2
            xx = np.logspace(np.log10(lo_x), np.log10(hi_x), 400)
            axE.plot(xx, chi2.pdf(xx, 1), color=C_BAD, lw=1.4,
                     label=r"$\chi^2_1$ PDF (theory)")
        except Exception:
            pass
        axE.axvline(CHI2_HI,  color=C_BAD, ls="--", lw=0.9, alpha=0.8)
        axE.axvline(CHI2_MED, color=C_CHI, ls=":",  lw=0.9, alpha=0.8)
        axE.axvline(CHI2_LO,  color=C_BAD, ls="--", lw=0.9, alpha=0.8)
        med_e = float(np.median(nis_finite))
        exc   = float(np.mean(nis_finite > CHI2_HI))
        axE.text(0.98, 0.95,
                 f"median = {med_e:.3g}\nP(NIS > {CHI2_HI:.2f}) = {exc*100:.1f}%",
                 transform=axE.transAxes, ha="right", va="top",
                 fontsize=8,
                 bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                           edgecolor="#bbbbbb", lw=0.5, alpha=0.9))
        axE.set_xscale("log")
        axE.set_yscale("log")
    else:
        axE.text(0.5, 0.5, "(no NIS samples)", color=C_CHI,
                 ha="center", va="center", transform=axE.transAxes)
    axE.set_xlabel("NIS")
    axE.set_ylabel("density")
    axE.set_title("(e) NIS distribution vs $\\chi^2_1$  —  "
                  "shape match means the filter is consistent",
                  loc="left", pad=4)
    axE.legend(loc="lower left", fontsize=7.5)

    # Shared regime legend at the bottom.
    used = list(dict.fromkeys(regime_at.tolist()))
    handles = _legend_handles([str(x) for x in used])
    if handles:
        fig.legend(handles=handles, loc="lower center",
                   ncol=min(len(handles), 7),
                   bbox_to_anchor=(0.5, -0.005),
                   frameon=False, fontsize=8)

    fig.savefig(out_path)
    plt.close(fig)


# ───────────────────────────────────────── per-bag run

def _process_bag(bag_dir: Path, out_root: Path,
                 axes: list[str], max_gap_s: float,
                 mode: str = "posterior",
                 max_prior_age_s: float = 0.10,
                 q_diag_ref: list[float] | None = None) -> dict[str, Any] | None:
    """Process a single bag and write per-bag diagnostics JSON + PNGs.

    mode is one of {"posterior", "prior_approx"}; see
    `era._compute_residual` for semantics.
    """
    bag_name = bag_dir.name
    bag_type = era._bag_type(bag_name)
    if bag_type == "global_run":
        print(f"[skip] {bag_name} (global_run)")
        return None
    print(f"\n{'=' * 72}\nDiag bag: {bag_name}   (type: {bag_type})  "
          f"mode: {mode}")

    bag = era._read_bag(bag_dir)
    rep = era._run_sanity(bag)
    sensors_reliable = rep.sensors_reliable
    cmd = _read_manual_control(bag_dir)
    print(f"  manual_control msgs: {cmd.t_ns.size}")

    if bag.ekf_t_ns.size == 0:
        print("  no EKF samples; skipping")
        return None

    t0 = int(bag.ekf_t_ns[0])
    ekf_t_rel = (bag.ekf_t_ns - t0) * 1e-9
    pressure_regime = _classify_pressure_regime(ekf_t_rel, bag.ekf_z)
    cmd_regime_at_ekf = _resolve_cmd_regimes_at(cmd, bag.ekf_t_ns)

    out_bag = out_root / bag_name
    out_bag.mkdir(parents=True, exist_ok=True)

    bag_axes = era._axes_for_bag_type(bag_type)
    axes_to_run = [a for a in axes if a in bag_axes] if bag_axes else list(axes)
    if not axes_to_run:
        # for stationary, era._axes_for_bag_type returns ALL_AXES — keep all asked.
        axes_to_run = [a for a in axes if a in era.ALL_AXES]
    print(f"  axes: {axes_to_run}")

    max_gap_ns = int(max_gap_s * 1e9)
    max_prior_age_ns = int(max_prior_age_s * 1e9)
    per_axis_json: dict[str, Any] = {}
    npz_payload: dict[str, np.ndarray] = {}
    for axis in axes_to_run:
        s = _collect_axis(bag, axis, max_gap_ns, sensors_reliable,
                          mode=mode, max_prior_age_ns=max_prior_age_ns)
        if s is None:
            print(f"    [skip] {axis} (no samples)")
            per_axis_json[axis] = {"n_total": 0, "skipped": True,
                                   "residual_mode": mode}
            continue
        regime_at = _resolve_axis_regimes(
            s, axis, ekf_t_rel, pressure_regime, cmd_regime_at_ekf)
        regime_stats = _regime_split(s, regime_at)
        png = out_bag / f"{bag_name}_diag_{axis.replace('.', '_')}_{mode}.png"
        _plot_axis_diag(s, axis, bag, bag_name, regime_at, png)
        agg = regime_stats.get("all", {})
        dt_med_ms = float(np.median(np.abs(s.dt_s))) * 1e3 if s.dt_s.size else float("nan")
        dt_p95_ms = float(np.percentile(np.abs(s.dt_s), 95)) * 1e3 if s.dt_s.size else float("nan")
        dt_max_ms = float(np.max(np.abs(s.dt_s))) * 1e3 if s.dt_s.size else float("nan")
        print(f"    [{axis:<14}] mode={mode:13s}  "
              f"n={s.resid.size:6d}/{s.n_total:6d}  skip={s.n_skipped:5d}  "
              f"dt med/p95/max={dt_med_ms:.1f}/{dt_p95_ms:.1f}/{dt_max_ms:.1f} ms  "
              f"NIS med={agg.get('nis_median', float('nan')):.3g}  "
              f"NIS p95={agg.get('nis_p95', float('nan')):.3g}  "
              f"exceed>3.84={agg.get('exceedance_rate_gt_3p84', 0.0):.2%}")
        per_axis_json[axis] = {
            "n_total": int(s.n_total),
            "n_matched": int(s.resid.size),
            "n_skipped": int(s.n_skipped),
            "residual_mode": mode,
            "dt_median_ms": dt_med_ms,
            "dt_p95_ms": dt_p95_ms,
            "dt_max_ms": dt_max_ms,
            "regimes": regime_stats,
            "plot": png.name,
        }
        # Per-sample arrays for downstream canonical figures
        # (q_tuning_figures.py whiteness, distribution, regime split).
        ax_key = axis.replace(".", "_")
        npz_payload[f"{ax_key}__t"]      = s.t_rel_s.astype(np.float64)
        npz_payload[f"{ax_key}__r"]      = s.resid.astype(np.float64)
        npz_payload[f"{ax_key}__R"]      = s.R.astype(np.float64)
        npz_payload[f"{ax_key}__P"]      = s.P.astype(np.float64)
        npz_payload[f"{ax_key}__S"]      = s.S.astype(np.float64)
        npz_payload[f"{ax_key}__NIS"]    = s.NIS.astype(np.float64)
        npz_payload[f"{ax_key}__dt"]     = s.dt_s.astype(np.float64)
        npz_payload[f"{ax_key}__regime"] = np.asarray(
            [str(x) for x in regime_at], dtype="U16")

    if npz_payload:
        npz_path = out_bag / f"{bag_name}_diagnostics_series_{mode}.npz"
        np.savez_compressed(npz_path, **npz_payload)
        print(f"  Saved per-sample sidecar: {npz_path.name}")

    bag_json = {
        "bag": bag_name,
        "bag_type": bag_type,
        "residual_mode": mode,
        "max_match_gap_s": max_gap_s,
        "max_prior_age_s": max_prior_age_s,
        "residual_mode_note": (
            "prior_approx uses the previous published EKF odometry sample "
            "before the sensor timestamp. This is not an exact EKF "
            "innovation, but avoids comparing against a posterior state "
            "that may already include the same measurement."
        ),
        "Q_diagonal_ref_used_for_run": list(q_diag_ref) if q_diag_ref else era.Q_DIAG_REF,
        "regime_thresholds": {
            "pressure_step_dzdt_m_s": P_STEP_DZDT,
            "pressure_steady_dzdt_m_s": P_STEADY_DZDT,
            "pressure_steady_dev_m": P_STEADY_DEV,
            "manual_control_neutral": list(MC_NEUTRAL),
            "manual_control_thresh": MC_THRESH,
            "nis_chi2_95": NIS_CHI2_95,
        },
        "manual_control_present": int(cmd.t_ns.size) > 0,
        "manual_control_msgs": int(cmd.t_ns.size),
        "series_sidecar_npz": (f"{bag_name}_diagnostics_series_{mode}.npz"
                                if npz_payload else None),
        "axes": per_axis_json,
        "sensors_reliable": sensors_reliable,
    }
    out_json = out_bag / f"{bag_name}_diagnostics.json"
    out_json.write_text(json.dumps(bag_json, indent=2))
    print(f"  Saved: {out_json}")
    return bag_json


# ───────────────────────────────────────── aggregation + synthesis

def _median_safe(vals: list[float]) -> float:
    finite = [v for v in vals if v is not None and math.isfinite(v)]
    if not finite:
        return float("nan")
    return float(np.median(finite))


def _gather_axis_regime(per_bag: list[dict], axis: str, regime: str,
                        field_name: str) -> list[float]:
    out = []
    for entry in per_bag:
        ax = entry.get("axes", {}).get(axis)
        if not ax or ax.get("skipped"):
            continue
        rg = ax.get("regimes", {}).get(regime)
        if not rg:
            continue
        v = rg.get(field_name)
        if v is None or (isinstance(v, float) and not math.isfinite(v)):
            continue
        out.append(float(v))
    return out


def _build_compare_block(current_path: Path, prev_path: Path) -> str:
    """Side-by-side comparison of two diagnostics_summary.json files.

    Reports per-axis median NIS_median / NIS_mean / var/S / n_matched across
    bags, in both modes, and flags axes where prior_approx NIS is much larger
    than posterior NIS.
    """
    import statistics

    def _load(p: Path) -> dict[str, Any]:
        return json.loads(p.read_text())

    def _med_axis(summary: dict, axis: str, regime: str,
                  field: str) -> tuple[float, int]:
        vals: list[float] = []
        for _, bag in summary.get("bags", {}).items():
            ax = bag.get("axes", {}).get(axis)
            if not ax:
                continue
            rg = ax.get(regime) if regime in ax else ax
            if not isinstance(rg, dict):
                continue
            v = rg.get(field)
            if v is None or not isinstance(v, (int, float)):
                continue
            if not math.isfinite(v):
                continue
            vals.append(float(v))
        if not vals:
            return (float("nan"), 0)
        return (statistics.median(vals), len(vals))

    a = _load(current_path)
    b = _load(prev_path)
    # Pre-mode-flag summaries have no residual_mode key; assume posterior
    # (that was the only behavior available).
    a_mode = a.get("residual_mode") or "posterior"
    b_mode = b.get("residual_mode") or "posterior"
    if a_mode == b_mode:
        return (
            f"\n### Comparison ({a_mode} vs {b_mode})\n"
            f"\n  Both summaries report mode={a_mode}. Comparison skipped — "
            f"need two different modes to be meaningful.\n"
        )

    axes = sorted({ax for s in (a, b)
                   for bag in s.get("bags", {}).values()
                   for ax in (bag.get("axes") or {}).keys()})
    lines = [
        "",
        f"### Comparison: {a_mode} (current) vs {b_mode} (prev: "
        f"{prev_path.name})",
        "",
        "Per-axis median across bags. Cells: posterior | prior_approx. "
        "ratio = prior_approx / posterior of NIS_median.",
        "",
        "| axis | n_bags | NIS_med (post) | NIS_med (prior) | ratio | "
        "NIS_mean (post) | NIS_mean (prior) | var/S (post) | var/S (prior) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    flagged: list[str] = []
    for axis in axes:
        # Posterior summary lookup: pick whichever summary is posterior.
        if a_mode == "posterior":
            post, prior = a, b
        else:
            post, prior = b, a
        nm_post, n_post = _med_axis(post, axis, "all", "nis_median")
        nm_prior, n_prior = _med_axis(prior, axis, "all", "nis_median")
        avg_post, _ = _med_axis(post, axis, "all", "nis_mean")
        avg_prior, _ = _med_axis(prior, axis, "all", "nis_mean")
        vs_post, _ = _med_axis(post, axis, "all", "var_over_S")
        vs_prior, _ = _med_axis(prior, axis, "all", "var_over_S")
        ratio = (nm_prior / nm_post) if (nm_post and math.isfinite(nm_post)
                                         and nm_post > 0) else float("nan")
        n_bags = max(n_post, n_prior)
        if math.isfinite(ratio) and ratio >= 10.0:
            flagged.append(f"{axis} (x{ratio:.1f})")
        lines.append(
            f"| `{axis}` | {n_bags} | "
            f"{nm_post:.3g} | {nm_prior:.3g} | "
            f"{'' if not math.isfinite(ratio) else f'{ratio:.2g}'} | "
            f"{avg_post:.3g} | {avg_prior:.3g} | "
            f"{vs_post:.3g} | {vs_prior:.3g} |"
        )
    if flagged:
        lines.append("")
        lines.append("Axes where `prior_approx` NIS_median is >= 10x the "
                     "`posterior` NIS_median (indicating posterior collapse "
                     "was hiding the residual structure):")
        lines.append("- " + "; ".join(flagged))
    lines += [
        "",
        "Caveat: `prior_approx` is the previous published EKF odometry "
        "sample, NOT a true Kalman prior. Differences below ~3x are inside "
        "the noise of that approximation.",
        "",
    ]
    return "\n".join(lines)


def _build_synthesis(per_bag: list[dict]) -> str:
    """Return the markdown synthesis section."""
    today = "2026-04-25"

    def fmt(vals):
        if not vals:
            return "n/a"
        return f"median={_median_safe(vals):.3g} (n_bags={len(vals)})"

    pz_steady_nis_med = _gather_axis_regime(per_bag, "pressure.z", "steady",
                                            "nis_median")
    pz_step_nis_med   = _gather_axis_regime(per_bag, "pressure.z", "step",
                                            "nis_median")
    pz_pump_nis_med   = _gather_axis_regime(per_bag, "pressure.z", "pump",
                                            "nis_median")
    pz_steady_nis_p95 = _gather_axis_regime(per_bag, "pressure.z", "steady",
                                            "nis_p95")
    pz_step_nis_p95   = _gather_axis_regime(per_bag, "pressure.z", "step",
                                            "nis_p95")
    pz_steady_exceed  = _gather_axis_regime(per_bag, "pressure.z", "steady",
                                            "exceedance_rate_gt_3p84")
    pz_step_exceed    = _gather_axis_regime(per_bag, "pressure.z", "step",
                                            "exceedance_rate_gt_3p84")
    pz_steady_mean_r  = _gather_axis_regime(per_bag, "pressure.z", "steady",
                                            "mean")

    omega_x_idle_nis  = _gather_axis_regime(per_bag, "imu.omega_x", "idle",
                                            "nis_median")
    omega_x_idle_var  = _gather_axis_regime(per_bag, "imu.omega_x", "idle",
                                            "variance")
    omega_x_idle_R    = _gather_axis_regime(per_bag, "imu.omega_x", "idle",
                                            "R_mean")

    roll_all_nis  = _gather_axis_regime(per_bag, "imu.roll", "all",
                                        "nis_median")
    pitch_all_nis = _gather_axis_regime(per_bag, "imu.pitch", "all",
                                        "nis_median")
    yaw_all_nis   = _gather_axis_regime(per_bag, "imu.yaw", "all",
                                        "nis_median")
    roll_all_R    = _gather_axis_regime(per_bag, "imu.roll", "all", "R_mean")
    pitch_all_R   = _gather_axis_regime(per_bag, "imu.pitch", "all", "R_mean")
    yaw_all_R     = _gather_axis_regime(per_bag, "imu.yaw", "all", "R_mean")

    def dvl_reg(axis, regime, field): return _gather_axis_regime(
        per_bag, axis, regime, field)

    lines = [
        "",
        "---",
        "",
        f"## **[Update {today}]** EKF residual diagnostics — regime-split",
        "",
        "Per-regime split of the existing 16-good-bag residuals "
        "(Q diagonal as recorded in `ekf_local.yaml` for the 2026-04-23 run).",
        "Regime thresholds: pressure step `|dz/dt| > 0.10 m/s`; "
        "steady `|dz/dt| < 0.05 m/s` AND `|z - rolling_med(z, 5s)| < 0.05 m`; "
        "command regime from `/pixhawk/manual_control` channel deviation > 45 "
        "from neutral `(0,0,500,0,0,0)`.",
        "",
        "### Pipeline caveat (kept on the record)",
        "",
        "The residuals are computed against the **posterior** EKF state at the "
        "matched timestamp (the EKF has already absorbed that exact sensor "
        "update). `S` uses the **posterior** `pose.covariance` / "
        "`twist.covariance` published in `/odometry/filtered/local`. The "
        "textbook NIS test wants the **prior**. The most visible effect is on "
        "fused IMU axes (orientation NIS ~ 0 because both `r` and `S` "
        "collapse). Pressure / DVL magnitudes are also pulled toward zero "
        "but the *regime structure* (step vs steady, idle vs commanded) is "
        "preserved — that is what we are tuning from.",
        "",
        "### Per-axis diagnosis",
        "",
        "**`pressure.z`** — split confirms transient-driven NIS:",
        f"- steady regime: NIS_median {fmt(pz_steady_nis_med)}, "
        f"NIS_p95 {fmt(pz_steady_nis_p95)}, "
        f"exceedance>3.84 {fmt(pz_steady_exceed)}, "
        f"mean(r) {fmt(pz_steady_mean_r)} m",
        f"- step regime:   NIS_median {fmt(pz_step_nis_med)}, "
        f"NIS_p95 {fmt(pz_step_nis_p95)}, "
        f"exceedance>3.84 {fmt(pz_step_exceed)}",
        f"- pump regime:   NIS_median {fmt(pz_pump_nis_med)}",
        ("  Diagnosis: high overall NIS_mean is driven by depth-step "
         "transients; the steady regime is consistent. **Do not tune Q[2] "
         "globally on the existing data** (per the user's constraint). "
         "Decide via the `step` panel whether the residual offset is a timing "
         "lag (sign-changing residual) or a real covariance underestimate."),
        "",
        "**`imu.omega_x`** — only computed in two stationary bags "
        "(`stationary_01_*-14_19_03`, `stationary_02_*-12_57_41`); both were "
        "*hand-held*, not rigid:",
        f"- idle regime NIS_median: {fmt(omega_x_idle_nis)}",
        f"- idle regime var(r):    {fmt(omega_x_idle_var)} (rad/s)^2",
        f"- idle regime R_mean:    {fmt(omega_x_idle_R)} (rad/s)^2 "
        "(2.072e-06 = TEP fallback)",
        ("  Diagnosis: the residual variance during operator hold is "
         "comparable to / larger than the gyro R, but the *signal* is "
         "operator wiggle — the EKF's omega state diffuses on Q[9]=0.02 "
         "and lags hand motion. This is **not** a clean Q-tuning signal. "
         "Need a yaw-turn-style bag with `omega_x` requested to validate."),
        "",
        "**IMU orientation (`imu.{roll, pitch, yaw}`)**:",
        f"- roll  NIS_median: {fmt(roll_all_nis)}, R_mean: {fmt(roll_all_R)}",
        f"- pitch NIS_median: {fmt(pitch_all_nis)}, R_mean: {fmt(pitch_all_R)}",
        f"- yaw   NIS_median: {fmt(yaw_all_nis)}, R_mean: {fmt(yaw_all_R)}",
        ("  Diagnosis: NIS ~ 0 driven by the post-update / posterior-P "
         "pipeline (see caveat). **Orientation NIS is not trustworthy for Q "
         "tuning with the current pipeline.** Action: fix "
         "`scripts/ekf_residual_analysis.py` to compute residuals against "
         "the EKF state at `j-1` (last sample BEFORE the matched update) "
         "and use that sample's covariance as `P_prior`. That fix is out of "
         "scope here."),
        "",
        "**DVL (`dvl.{vx, vy, vz}`)** — per-regime split:",
    ]
    for axname in ("dvl.vx", "dvl.vy", "dvl.vz"):
        idle = dvl_reg(axname, "idle", "nis_median")
        surge = dvl_reg(axname, "surge", "nis_median")
        rev = dvl_reg(axname, "reverse", "nis_median")
        yaw_r = dvl_reg(axname, "yaw", "nis_median")
        heave = dvl_reg(axname, "heave", "nis_median")
        lines.append(
            f"- {axname}: idle {fmt(idle)} | surge {fmt(surge)} | "
            f"reverse {fmt(rev)} | yaw {fmt(yaw_r)} | heave {fmt(heave)}")
    lines += [
        ("  Diagnosis: idle / commanded NIS comparison decides "
         "*R-too-large* vs *not-enough-motion*. With NIS values pulled down "
         "by the posterior-P caveat, treat as **needs-more-data + "
         "needs-pipeline-fix** rather than direct R retune."),
        "",
        "### Tuning verdict (per user's four buckets)",
        "",
        "- **Tunable now:** *(none, conditioned on the post-update pipeline "
        "issue)*. The pressure steady regime is the most defensible direct "
        "tuning candidate, but only after a real-prior NIS rerun.",
        "- **Needs more diagnosis:** `pressure.z` step regime — verify "
        "whether the transient excursions are a timing lag (residual sign "
        "flips at step) or true covariance miss. `imu.{roll,pitch,yaw}` — "
        "blocked on the prior-vs-posterior NIS fix in the analysis script.",
        "- **Needs more dynamic data:** `imu.omega_x` — needs a "
        "non-hand-held bag with this axis enabled (extend `_axes_for_bag_type` "
        "for `yaw_turns_*` to include `imu.omega_x`, or add a dedicated "
        "rotational bag). DVL — needs longer commanded surge/sway/heave "
        "blocks; current `straight_surge_*` bags excite mostly `vx`.",
        "- **Held back:** global `pressure.z` Q tuning — explicitly held "
        "back per the user's constraint until step-vs-steady transient "
        "structure is fully characterized.",
        "",
        "### Outputs",
        "",
        "- Per-bag JSON / 4-panel timeline PNGs under "
        "`recordings/rosbags/2026-04-23/ekf_residual_analysis_good_bags/diagnostics/<bag>/`.",
        "- Aggregate JSON: `.../diagnostics/diagnostics_summary.json`.",
        "- Reproduce: `python scripts/ekf_residual_diagnostics.py` (defaults "
        "match the 16-bag good-bag filter from the previous update).",
    ]
    return "\n".join(lines)


# ───────────────────────────────────────── CLI

def _parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags_root", type=Path, nargs="?",
                    default=Path("recordings/rosbags/2026-04-23"),
                    help="Root containing rosbag2 directories")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output dir (default: <bags_root>/ekf_residual_analysis_good_bags/diagnostics)")
    ap.add_argument("--include", default=GOOD_BAGS_FILTER,
                    help="Comma-separated bag-name substrings (default: 16 good bags)")
    ap.add_argument("--exclude", default=None,
                    help="Comma-separated bag-name substrings to drop")
    ap.add_argument("--diag-sensors", default="all",
                    choices=list(DIAG_GROUPS.keys()),
                    help="Which axis group to diagnose")
    ap.add_argument("--max-match-gap", type=float, default=0.05,
                    help="Max sensor<->EKF gap, posterior mode (default 0.05)")
    ap.add_argument("--residual-mode", default="posterior",
                    choices=["posterior", "prior_approx"],
                    help="posterior: nearest EKF sample (default; preserves "
                         "existing behavior). prior_approx: latest EKF sample "
                         "with timestamp strictly before the sensor stamp; "
                         "approximate, NOT a true Kalman prior.")
    ap.add_argument("--max-prior-age-sec", type=float, default=0.10,
                    help="Max age of the prior EKF sample in prior_approx "
                         "(default 0.10)")
    ap.add_argument("--compare-with", type=Path, default=None,
                    help="Path to a previous diagnostics_summary.json (e.g. "
                         "from the other --residual-mode). Emits a side-by-side "
                         "comparison table to stdout and the report file.")
    ap.add_argument("--q-yaml", type=Path, default=None,
                    help="Path to a robot_localization yaml whose "
                         "process_noise_covariance diagonal is used as the "
                         "Q_diagonal_ref label. Default: the hardcoded "
                         "Q_DIAG_REF in scripts/ekf_residual_analysis.py.")
    ap.add_argument("--imu-topic", default=None,
                    help="Override the IMU topic the script reads "
                         "(default /imu/data). For the 2026-05-07 campaign "
                         "the live EKF was subscribed to /imu/data_corrected; "
                         "use that here to avoid a phantom yaw offset.")
    ap.add_argument("--report-md", type=Path,
                    default=Path("POLARIS/research/EKF_RESEARCH_NOTES.md"),
                    help="Append synthesis to this markdown file")
    return ap.parse_args(argv)


def _split_csv(s: str | None) -> list[str]:
    if not s:
        return []
    return [tok.strip() for tok in s.split(",") if tok.strip()]


def main(argv=None) -> int:
    args = _parse_args(argv)
    repo_root = Path(__file__).resolve().parent.parent
    bags_root = args.bags_root if args.bags_root.is_absolute() else repo_root / args.bags_root
    bags_root = bags_root.resolve()
    if not bags_root.is_dir():
        print(f"ERROR: not a directory: {bags_root}", file=sys.stderr)
        return 1

    mode = args.residual_mode
    default_dir = bags_root / "ekf_residual_analysis_good_bags" / f"diagnostics_{mode}"
    out_dir = args.output_dir or default_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    # Apply --imu-topic override BEFORE any bag-reading. Mutates era's module-level
    # constants so era._read_bag dispatches on the right topic.
    if args.imu_topic:
        old_imu = era.TOPIC_IMU
        era.TOPIC_IMU = args.imu_topic
        era.REQUIRED_TOPICS = tuple(
            args.imu_topic if t == old_imu else t for t in era.REQUIRED_TOPICS)
        print(f"IMU topic override: {old_imu} -> {era.TOPIC_IMU}")

    q_diag_ref: list[float] | None = None
    q_yaml_path: Path | None = None
    if args.q_yaml is not None:
        q_yaml_path = args.q_yaml.resolve()
        if not q_yaml_path.is_file():
            print(f"ERROR: --q-yaml not found: {q_yaml_path}", file=sys.stderr)
            return 1
        try:
            q_diag_ref = era.load_q_diagonal_from_yaml(q_yaml_path)
        except Exception as e:
            print(f"ERROR: failed to parse --q-yaml: {e}", file=sys.stderr)
            return 1
        print(f"Q_diagonal_ref loaded from {q_yaml_path}")
        print(f"  {q_diag_ref}")

    bags = era._find_bag_dirs(bags_root)
    include = _split_csv(args.include)
    exclude = _split_csv(args.exclude)
    n_pre = len(bags)
    bags = era._filter_bags(bags, include, exclude)
    if not bags:
        print(f"ERROR: filter excluded all {n_pre} bags", file=sys.stderr)
        return 1

    axes = DIAG_GROUPS[args.diag_sensors]
    print(f"Diagnostics: {len(bags)}/{n_pre} bags, axes={axes}, "
          f"mode={mode}, max_prior_age_sec={args.max_prior_age_sec}")
    print(f"Output: {out_dir}")

    per_bag: list[dict[str, Any]] = []
    for b in bags:
        entry = _process_bag(b, out_dir, axes, args.max_match_gap,
                             mode=mode, max_prior_age_s=args.max_prior_age_sec,
                             q_diag_ref=q_diag_ref)
        if entry is not None:
            per_bag.append(entry)

    aggregate = {
        "bags_root": str(bags_root),
        "n_bags_processed": len(per_bag),
        "residual_mode": mode,
        "max_match_gap_s": args.max_match_gap,
        "max_prior_age_s": args.max_prior_age_sec,
        "residual_mode_note": (
            "prior_approx uses the previous published EKF odometry sample "
            "before the sensor timestamp. This is not an exact EKF "
            "innovation, but avoids comparing against a posterior state "
            "that may already include the same measurement."
        ),
        "include_filter": include,
        "exclude_filter": exclude,
        "axes_requested": axes,
        "Q_diagonal_ref": list(q_diag_ref) if q_diag_ref else era.Q_DIAG_REF,
        "Q_diagonal_ref_source": (str(q_yaml_path) if q_yaml_path
                                   else "hardcoded HEAD constant era.Q_DIAG_REF"),
        "imu_topic": era.TOPIC_IMU,
        "regime_thresholds": {
            "pressure_step_dzdt_m_s": P_STEP_DZDT,
            "pressure_steady_dzdt_m_s": P_STEADY_DZDT,
            "pressure_steady_dev_m": P_STEADY_DEV,
            "manual_control_neutral": list(MC_NEUTRAL),
            "manual_control_thresh": MC_THRESH,
            "nis_chi2_95": NIS_CHI2_95,
        },
        "bags": {e["bag"]: {
            "bag_type": e["bag_type"],
            "manual_control_msgs": e.get("manual_control_msgs", 0),
            "axes": {ax: data.get("regimes", {})
                     for ax, data in e["axes"].items()
                     if isinstance(data, dict) and not data.get("skipped")},
        } for e in per_bag},
    }
    summary_path = out_dir / "diagnostics_summary.json"
    summary_path.write_text(json.dumps(aggregate, indent=2))
    print(f"\nSaved aggregate: {summary_path}")

    synthesis = _build_synthesis(per_bag)
    # Annotate the synthesis with the residual mode used for this run.
    synthesis = synthesis.replace(
        "EKF residual diagnostics — regime-split",
        f"EKF residual diagnostics — regime-split (mode={mode})",
    )
    print("\n" + "=" * 72)
    print(synthesis)

    compare_block = ""
    if args.compare_with is not None:
        prev_path = args.compare_with if args.compare_with.is_absolute() \
            else repo_root / args.compare_with
        if not prev_path.is_file():
            print(f"\nWARN: --compare-with target not found: {prev_path}",
                  file=sys.stderr)
        else:
            compare_block = _build_compare_block(summary_path, prev_path)
            print(compare_block)

    report_md = args.report_md if args.report_md.is_absolute() \
        else repo_root / args.report_md
    if report_md.exists():
        with open(report_md, "a", encoding="utf-8") as f:
            f.write("\n" + synthesis + "\n")
            if compare_block:
                f.write("\n" + compare_block + "\n")
        print(f"\nAppended synthesis to: {report_md}")
    else:
        print(f"\nSynthesis NOT appended (file missing): {report_md}",
              file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
