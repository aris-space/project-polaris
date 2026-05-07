#!/usr/bin/env python3
"""Multi-bag EKF mission diagnosis.

Runs ekf_sbl_overlay.py per bag, classifies GOOD / DEGRADED / FAILED based on
mean and max EKF↔SBL error, and diagnoses the failure cause for non-GOOD bags
(heading rotation, DVL dropout, init transient, SBL quality, EKF covariance
blow-up). When a constant heading rotation explains the error, applies the
correction to the local DR track and re-classifies (SALVAGED).

Outputs (under <output_dir>):
  <bag>/ekf_sbl_<bag>_overlay.png         (ekf_sbl_overlay.py)
  <bag>/ekf_sbl_<bag>_drift.png           (ekf_sbl_overlay.py)
  <bag>/ekf_sbl_<bag>_timeseries.png      (ekf_sbl_overlay.py)
  <bag>/ekf_sbl_<bag>_metadata.json       (ekf_sbl_overlay.py)
  <bag>/ekf_sbl_<bag>_corrected.png       (this script, only if SALVAGED)
  <bag>/diagnosis.json                    (this script)
  summary.csv
  diagnosis_report.md
  aggregate_stats.json

Usage:
  python scripts/diagnose_ekf_missions.py recordings/rosbags --output-dir diagnosis
"""
from __future__ import annotations

import argparse
import csv
import fnmatch
import json
import math
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mcap_ros2.reader import read_ros2_messages

# On Windows, default console encoding is cp1252 and chokes on '°', '→', etc.
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

# Sibling-script imports — add scripts/ to sys.path
_SCRIPTS_DIR = Path(__file__).resolve().parent
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

from ekf_sbl_overlay import (  # noqa: E402
    read_bag as overlay_read_bag,
    gate_sbl,
    navsatfix_to_track,
    align_to_anchor,
    pair_errors,
)
from analyze_mcap_dvl_health import (  # noqa: E402
    LOCKED_TWIST_LINEAR_VAR_MAX,
    _time_weighted_lock_fraction,
    _invert_lock_intervals,
    _merge_intervals,
)
from report_overlay import (  # noqa: E402
    draw_track,
    add_direction_arrows,
    add_start_marker,
    add_scale_bar,
    add_satellite_basemap,
    _utm_epsg,
)


# ── Constants ────────────────────────────────────────────────────────────────

_T_DVL_VEL = "/sensors/dvl/velocity"
_T_DVL_ODOM_COV = "/sensors/dvl/odometry_cov"
_T_ODOM_LOCAL = "/odometry/filtered/local"
_T_PRESSURE_POSE = "/sensors/pressure/pose_enu"
_T_ROSOUT = "/rosout"

# Health-profile topics in a single set for one-pass reading
_HEALTH_TOPICS = {
    _T_DVL_VEL, _T_DVL_ODOM_COV, _T_ODOM_LOCAL,
    _T_PRESSURE_POSE, _T_ROSOUT,
}

# rosout severity levels (rcl_interfaces/Log)
_ROSOUT_LEVEL_WARN = 30
_ROSOUT_LEVEL_ERROR = 40
_ROSOUT_LEVEL_FATAL = 50

# Substrings in EKF / navsat warning text that we count specifically
_EKF_WARN_PATTERNS = (
    "covariance",
    "pos_def", "positive-definite", "positive definite", "not positive",
    "innovation",
    "filter divergence",
    "nan",
    "rejected",
    "transform",
    "stale",
)

# Classification (mean_error_m / max_error_m, in metres)
GOOD_MEAN_MAX = 3.0
GOOD_MAX_MAX = 8.0
FAILED_MEAN_MIN = 10.0
FAILED_MAX_MIN = 20.0
MIN_PAIRS = 50

# Cause-A heading
HEADING_MIN_SEGMENTS = 3
HEADING_MIN_ABS_DEG = 5.0
HEADING_MAX_STD_DEG = 8.0
SBL_SEG_MIN_CHORD_M = 3.0
SBL_SEG_RATIO_MAX = 1.10
SBL_SEG_DR_MIN_CHORD_M = 1.0

# Cause-B DVL
DVL_LOCK_MIN_FRAC = 0.85
DVL_DROPOUT_MIN_S = 5.0

# Cause-C init transient
INIT_WINDOW_S = 30.0
INIT_RATIO_MIN = 2.0
INIT_STEADY_MAX_M = 4.0

# Cause-D SBL quality
SBL_STD_MEDIAN_MAX = 1.5
SBL_OVERLAP_MIN = 0.6
SBL_HIGH_STD_M = 1.5

# Cause-E covariance growth
COV_GROWTH_RATIO_MIN = 3.0
COV_WINDOW_S = 30.0


# ── Utilities ────────────────────────────────────────────────────────────────

def wrap_signed_deg(deg: float) -> float:
    x = ((deg + 180.0) % 360.0) - 180.0
    return x + 360.0 if x <= -180.0 else x


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


# ── Data structures ──────────────────────────────────────────────────────────

@dataclass
class CauseResult:
    name: str
    triggered: bool
    evidence: dict = field(default_factory=dict)


@dataclass
class HealthProfile:
    """Single-pass sensor / filter health, computed for every bag."""
    duration_s: float | None = None
    # DVL
    dvl_source_topic: str | None = None
    dvl_lock_fraction: float | None = None
    dvl_longest_unlock_s: float | None = None
    dvl_n_unlock_intervals: int = 0
    dvl_unlock_intervals_s: list[list[float]] = field(default_factory=list)
    # Pressure / depth
    depth_z_min_m: float | None = None
    depth_z_max_m: float | None = None
    depth_z_range_m: float | None = None
    # /odometry/filtered/local
    odom_n: int = 0
    cov_xy_initial: float | None = None      # mean over first 10 s
    cov_xy_max: float | None = None
    cov_xy_final: float | None = None        # mean over last 10 s
    cov_xy_growth_ratio: float | None = None # final / initial
    cov_xy_blowup_t_s: float | None = None   # first time cov_xy > 5x initial
    cov_yaw_max: float | None = None
    cov_nan_or_neg: bool = False
    # speed
    speed_mean_mps: float | None = None
    speed_p95_mps: float | None = None
    # rosout
    rosout_n_warn: int = 0
    rosout_n_error: int = 0
    rosout_n_ekf_pattern: int = 0
    rosout_top_messages: list[str] = field(default_factory=list)


@dataclass
class BagDiagnosis:
    bag_name: str
    bag_dir: str
    duration_s: float | None = None
    classification: str = "UNKNOWN"
    metrics: dict = field(default_factory=dict)
    causes: list[dict] = field(default_factory=list)
    correction: dict | None = None
    health: HealthProfile | None = None


# ── Bag discovery ────────────────────────────────────────────────────────────

def find_bag_dirs(root: Path, pattern: str,
                  include: list[str], exclude: list[str]) -> list[Path]:
    out: set[Path] = set()
    for meta in root.rglob("metadata.yaml"):
        d = meta.parent.resolve()
        # Skip companion bags created by post-processing / replay scripts
        if (d.name.endswith("__bodyframe")
                or d.name.endswith("__rltwist")
                or d.name.endswith("_ekf")
                or "/ekf_replay/" in str(d).replace("\\", "/")):
            continue
        if not any(d.glob("*.mcap")):
            continue
        name = d.name
        if pattern and not fnmatch.fnmatch(name, pattern):
            continue
        if include and not any(fnmatch.fnmatch(name, p) for p in include):
            continue
        if exclude and any(fnmatch.fnmatch(name, p) for p in exclude):
            continue
        out.add(d)
    return sorted(out)


# ── Health profile (single-pass over the bag, runs for every classification) ─

def _odom_lock_from_cov(cov) -> bool:
    if len(cov) < 15:
        return False
    return max(float(cov[0]), float(cov[7]), float(cov[14])) < LOCKED_TWIST_LINEAR_VAR_MAX


def compute_health_profile(bag_dir: Path,
                           t_axis_out: list | None = None) -> HealthProfile:
    """Single pass through the MCAP, computing every sensor / filter signal we
    care about. If t_axis_out is provided, append (t_ns, cov_xy, cov_yaw,
    dvl_lock, depth_z) tuples for the per-bag timeline plot."""
    hp = HealthProfile()
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        return hp

    dvl_ts_lock_vel: list[tuple[int, bool]] = []
    dvl_ts_lock_cov: list[tuple[int, bool]] = []
    depth_ts: list[int] = []
    depth_z: list[float] = []
    odom_ts: list[int] = []
    odom_cov_xy: list[float] = []
    odom_cov_yaw: list[float] = []
    odom_speed: list[float] = []
    rosout_messages: list[tuple[int, int, str, str]] = []  # (level, count, name, msg)
    rosout_msg_counts: dict[str, int] = {}

    for msg in read_ros2_messages(str(mcap)):
        topic = msg.channel.topic
        if topic not in _HEALTH_TOPICS:
            continue
        ros = msg.ros_msg
        try:
            if topic == _T_DVL_VEL:
                t = _stamp_ns(ros.header.stamp)
                if t > 0:
                    dvl_ts_lock_vel.append((t, bool(ros.beam_velocities_valid)))
            elif topic == _T_DVL_ODOM_COV:
                t = _stamp_ns(ros.header.stamp)
                if t > 0:
                    dvl_ts_lock_cov.append((t, _odom_lock_from_cov(ros.twist.covariance)))
            elif topic == _T_PRESSURE_POSE:
                t = _stamp_ns(ros.header.stamp)
                if t > 0:
                    depth_ts.append(t)
                    depth_z.append(float(ros.pose.pose.position.z))
            elif topic == _T_ODOM_LOCAL:
                t = _stamp_ns(ros.header.stamp)
                if t == 0:
                    continue
                c = ros.pose.covariance
                xx = float(c[0]); yy = float(c[7]); yaw = float(c[35])
                # Detect NaN / negative on diagonal (any of 6)
                for i in range(6):
                    v = float(c[i * 6 + i])
                    if not math.isfinite(v) or v < 0:
                        hp.cov_nan_or_neg = True
                        break
                odom_ts.append(t)
                odom_cov_xy.append(xx + yy)
                odom_cov_yaw.append(yaw)
                t_lin = ros.twist.twist.linear
                odom_speed.append(math.sqrt(
                    float(t_lin.x) ** 2 + float(t_lin.y) ** 2 + float(t_lin.z) ** 2
                ))
            elif topic == _T_ROSOUT:
                level = int(getattr(ros, "level", 0))
                if level < _ROSOUT_LEVEL_WARN:
                    continue
                name = str(getattr(ros, "name", ""))
                txt = str(getattr(ros, "msg", ""))
                rosout_messages.append((level, 0, name, txt))
                rosout_msg_counts[txt] = rosout_msg_counts.get(txt, 0) + 1
        except (AttributeError, IndexError, ValueError):
            continue

    # ── DVL aggregates (prefer /sensors/dvl/velocity)
    ts_lock = dvl_ts_lock_vel if dvl_ts_lock_vel else dvl_ts_lock_cov
    if ts_lock:
        hp.dvl_source_topic = _T_DVL_VEL if dvl_ts_lock_vel else _T_DVL_ODOM_COV
        hp.dvl_lock_fraction = _time_weighted_lock_fraction(ts_lock)
        unlock = _invert_lock_intervals(ts_lock, ts_lock[0][0], ts_lock[-1][0])
        unlock = _merge_intervals(unlock, gap_merge_ns=int(0.5e9))
        hp.dvl_n_unlock_intervals = len(unlock)
        if unlock:
            durations = [(b - a) / 1e9 for a, b in unlock]
            hp.dvl_longest_unlock_s = float(max(durations))
            t0 = ts_lock[0][0]
            # Keep only intervals ≥ 1 s and store relative seconds
            hp.dvl_unlock_intervals_s = [
                [round((a - t0) / 1e9, 2), round((b - t0) / 1e9, 2)]
                for a, b in unlock if (b - a) / 1e9 >= 1.0
            ][:30]  # cap

    # ── Depth aggregates
    if depth_z:
        zs = np.array(depth_z)
        hp.depth_z_min_m = float(zs.min())
        hp.depth_z_max_m = float(zs.max())
        hp.depth_z_range_m = float(zs.max() - zs.min())

    # ── Local EKF cov / speed
    if odom_ts:
        hp.odom_n = len(odom_ts)
        ts_arr = np.array(odom_ts, dtype=np.int64)
        cov_xy_arr = np.array(odom_cov_xy)
        cov_yaw_arr = np.array(odom_cov_yaw)
        spd = np.array(odom_speed)
        t_rel = (ts_arr - ts_arr[0]) / 1e9
        hp.duration_s = float(t_rel[-1] - t_rel[0])
        # First / last 10 s windows (pad if short)
        early = cov_xy_arr[t_rel < 10.0]
        late = cov_xy_arr[t_rel >= max(t_rel[-1] - 10.0, 0.0)]
        if len(early) >= 1:
            hp.cov_xy_initial = float(np.mean(early))
        if len(late) >= 1:
            hp.cov_xy_final = float(np.mean(late))
        hp.cov_xy_max = float(np.max(cov_xy_arr))
        hp.cov_yaw_max = float(np.max(cov_yaw_arr))
        if hp.cov_xy_initial and hp.cov_xy_initial > 0 and hp.cov_xy_final is not None:
            hp.cov_xy_growth_ratio = hp.cov_xy_final / hp.cov_xy_initial
        if hp.cov_xy_initial and hp.cov_xy_initial > 0:
            threshold = 5.0 * hp.cov_xy_initial
            hits = np.where(cov_xy_arr > threshold)[0]
            if len(hits) > 0:
                hp.cov_xy_blowup_t_s = float(t_rel[hits[0]])
        hp.speed_mean_mps = float(np.mean(spd))
        hp.speed_p95_mps = float(np.percentile(spd, 95))
        if t_axis_out is not None:
            # Down-sample to ~1000 points for plotting
            n = len(ts_arr)
            stride = max(1, n // 1000)
            for i in range(0, n, stride):
                t_axis_out.append((int(ts_arr[i]), float(cov_xy_arr[i]),
                                   float(cov_yaw_arr[i]), None, None))

    # ── rosout aggregates
    if rosout_messages:
        for level, _, name, txt in rosout_messages:
            if level >= _ROSOUT_LEVEL_ERROR:
                hp.rosout_n_error += 1
            else:
                hp.rosout_n_warn += 1
            tl = txt.lower()
            if any(p in tl for p in _EKF_WARN_PATTERNS):
                hp.rosout_n_ekf_pattern += 1
        # Top 5 most frequent messages
        top = sorted(rosout_msg_counts.items(), key=lambda x: -x[1])[:5]
        hp.rosout_top_messages = [f"({n}x) {m[:140]}" for m, n in top]

    return hp


def _print_health_summary(hp: HealthProfile) -> None:
    parts = []
    if hp.dvl_lock_fraction is not None:
        parts.append(f"dvl_lock={hp.dvl_lock_fraction:.3f}")
    if hp.dvl_longest_unlock_s is not None:
        parts.append(f"longest_unlock={hp.dvl_longest_unlock_s:.1f}s")
    if hp.cov_xy_initial is not None and hp.cov_xy_final is not None:
        parts.append(f"cov_xy {hp.cov_xy_initial:.3g}->{hp.cov_xy_final:.3g}")
    if hp.cov_xy_growth_ratio is not None:
        parts.append(f"growth={hp.cov_xy_growth_ratio:.1f}x")
    if hp.cov_xy_blowup_t_s is not None:
        parts.append(f"5x@t={hp.cov_xy_blowup_t_s:.1f}s")
    if hp.depth_z_range_m is not None:
        parts.append(f"depth_range={hp.depth_z_range_m:.2f}m")
    if hp.rosout_n_warn or hp.rosout_n_error:
        parts.append(f"rosout w/e={hp.rosout_n_warn}/{hp.rosout_n_error}")
    if parts:
        print(f"    health: {' | '.join(parts)}")


def plot_health_timeline(timeline_pts: list, hp: HealthProfile,
                         bag_name: str, classification: str, out_path: Path) -> None:
    """Two-panel plot: cov_xy and cov_yaw over time, with DVL unlock bands and depth."""
    if not timeline_pts:
        return
    ts = np.array([p[0] for p in timeline_pts], dtype=np.int64)
    cov_xy = np.array([p[1] for p in timeline_pts])
    cov_yaw = np.array([p[2] for p in timeline_pts])
    t0 = int(ts[0])
    t_rel = (ts - t0) / 1e9

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                                   gridspec_kw={"hspace": 0.18,
                                                "height_ratios": [3, 1]})

    # Top: cov_xy and cov_yaw (log scale, dual y-axis)
    ax1.set_yscale("log")
    ax1.plot(t_rel, cov_xy, color="royalblue", lw=1.4, label="cov_xy (xx + yy)")
    ax1b = ax1.twinx()
    ax1b.set_yscale("log")
    ax1b.plot(t_rel, cov_yaw, color="darkorange", lw=1.0, alpha=0.8,
              label="cov_yaw")
    if hp.cov_xy_initial and hp.cov_xy_initial > 0:
        ax1.axhline(hp.cov_xy_initial, color="navy", lw=0.6, ls=":", alpha=0.5)
        ax1.axhline(5.0 * hp.cov_xy_initial, color="firebrick", lw=0.6, ls="--",
                    alpha=0.6, label="5× initial cov_xy")
    if hp.cov_xy_blowup_t_s is not None:
        ax1.axvline(hp.cov_xy_blowup_t_s, color="firebrick", lw=0.8,
                    ls="--", alpha=0.6,
                    label=f"5× blow-up @ {hp.cov_xy_blowup_t_s:.0f}s")
    # DVL unlock bands (red shading)
    for a, b in hp.dvl_unlock_intervals_s:
        ax1.axvspan(a, b, color="red", alpha=0.10, zorder=0)
        ax2.axvspan(a, b, color="red", alpha=0.10, zorder=0)
    ax1.set_ylabel("cov_xy (m²)", color="royalblue")
    ax1b.set_ylabel("cov_yaw (rad²)", color="darkorange")
    ax1.tick_params(axis="y", labelcolor="royalblue")
    ax1b.tick_params(axis="y", labelcolor="darkorange")
    ax1.grid(True, lw=0.3, alpha=0.5, which="both")
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax1b.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper left")

    # Bottom: DVL lock fraction (text only) + depth range
    if hp.dvl_lock_fraction is not None:
        ax2.set_title(
            f"DVL lock fraction: {hp.dvl_lock_fraction:.3f}  |  "
            f"longest unlock: {hp.dvl_longest_unlock_s or 0:.1f}s  |  "
            f"depth range: {hp.depth_z_range_m or 0:.2f}m",
            fontsize=9, loc="left",
        )
    ax2.set_xlabel("Time since first /odometry/filtered/local sample (s)")
    ax2.set_ylabel("(red = DVL unlock)")
    ax2.set_yticks([])
    ax2.set_ylim(0, 1)
    ax2.grid(True, lw=0.3, alpha=0.5)

    fig.suptitle(f"{bag_name}  —  health timeline  ({classification})",
                 fontsize=10)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"    saved: {out_path.name}")


# ── Step 1: subprocess overlay ───────────────────────────────────────────────

def run_overlay(bag_dir: Path, out_dir: Path, sbl_overlay_script: Path,
                track_mode: str, force: bool,
                timeout_s: int) -> tuple[bool, dict | None]:
    metadata_path = out_dir / f"ekf_sbl_{bag_dir.name}_metadata.json"
    if metadata_path.exists() and not force:
        try:
            print(f"  cached metadata: {metadata_path.name}")
            return True, json.loads(metadata_path.read_text())
        except json.JSONDecodeError:
            pass

    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable, str(sbl_overlay_script),
        str(bag_dir),
        "--output-dir", str(out_dir),
        "--track", track_mode,
    ]
    print(f"  running ekf_sbl_overlay.py --track {track_mode} (timeout {timeout_s}s)")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        print(f"  ERROR: overlay subprocess timed out after {timeout_s}s", file=sys.stderr)
        return False, None
    if result.returncode != 0:
        tail = (result.stderr or "")[-400:]
        print(f"  ERROR: overlay rc={result.returncode}\n{tail}", file=sys.stderr)
        return False, None
    if not metadata_path.exists():
        print(f"  ERROR: no metadata at {metadata_path}", file=sys.stderr)
        return False, None
    return True, json.loads(metadata_path.read_text())


# ── Step 2: classification ───────────────────────────────────────────────────

def classify(meta: dict) -> str:
    n_sbl = meta.get("n_sbl_gated") or 0
    n_ekf = meta.get("n_ekf_anchored") or 0
    mean = meta.get("mean_error_m")
    mx = meta.get("max_error_m")
    if n_sbl < MIN_PAIRS or n_ekf < MIN_PAIRS or mean is None or mx is None:
        return "INSUFFICIENT_DATA"
    if mean > FAILED_MEAN_MIN or mx > FAILED_MAX_MIN:
        return "FAILED"
    if mean < GOOD_MEAN_MAX and mx < GOOD_MAX_MAX:
        return "GOOD"
    return "DEGRADED"


# ── Step 3: cause diagnostics ────────────────────────────────────────────────

def find_sbl_straight_segments(track,
                               min_chord: float = SBL_SEG_MIN_CHORD_M,
                               ratio_max: float = SBL_SEG_RATIO_MAX
                               ) -> list[tuple[int, int]]:
    """Greedy non-overlapping straight-segment finder over (E, N_utm)."""
    E = track.E
    N = track.N_utm
    n = len(E)
    if n < 2:
        return []
    segs: list[tuple[int, int]] = []
    cursor = 0
    while cursor < n - 1:
        e0, n0 = float(E[cursor]), float(N[cursor])
        last_e, last_n = e0, n0
        path = 0.0
        found: int | None = None
        for j in range(cursor + 1, n):
            e, ny = float(E[j]), float(N[j])
            path += math.hypot(e - last_e, ny - last_n)
            last_e, last_n = e, ny
            chord = math.hypot(e - e0, ny - n0)
            if chord <= 0.0:
                continue
            ratio = path / chord
            if chord >= min_chord and ratio <= ratio_max:
                found = j
            elif found is not None and ratio > ratio_max:
                break
        if found is not None:
            segs.append((cursor, found))
            cursor = found + 1
        else:
            break
    return segs


def diag_heading(sbl_track, dr_track, dr_E_aligned, dr_N_aligned) -> CauseResult:
    """Cause A: constant heading rotation between SBL and DR/EKF track."""
    if dr_track is None or dr_E_aligned is None:
        return CauseResult("heading_error", False, {"reason": "no DR/EKF track"})

    segs = find_sbl_straight_segments(sbl_track)
    deltas: list[float] = []
    seg_records: list[dict] = []
    for (i0, i1) in segs:
        dE_sbl = float(sbl_track.E[i1] - sbl_track.E[i0])
        dN_sbl = float(sbl_track.N_utm[i1] - sbl_track.N_utm[i0])
        chord_sbl = math.hypot(dE_sbl, dN_sbl)
        if chord_sbl <= 0:
            continue
        b_sbl = math.degrees(math.atan2(dE_sbl, dN_sbl))

        t0 = int(sbl_track.t_ns[i0])
        t1 = int(sbl_track.t_ns[i1])
        mask = (dr_track.t_ns >= t0) & (dr_track.t_ns <= t1)
        idx = np.where(mask)[0]
        if len(idx) < 2:
            continue
        dE_dr = float(dr_E_aligned[idx[-1]] - dr_E_aligned[idx[0]])
        dN_dr = float(dr_N_aligned[idx[-1]] - dr_N_aligned[idx[0]])
        chord_dr = math.hypot(dE_dr, dN_dr)
        if chord_dr < SBL_SEG_DR_MIN_CHORD_M:
            continue
        b_dr = math.degrees(math.atan2(dE_dr, dN_dr))
        delta = wrap_signed_deg(b_dr - b_sbl)
        deltas.append(delta)
        seg_records.append({
            "t0_ns": t0, "t1_ns": t1,
            "sbl_bearing_deg": b_sbl,
            "dr_bearing_deg": b_dr,
            "delta_deg": delta,
            "sbl_chord_m": chord_sbl,
            "dr_chord_m": chord_dr,
        })

    if len(deltas) < HEADING_MIN_SEGMENTS:
        return CauseResult("heading_error", False, {
            "n_segments": len(deltas),
            "reason": "too few straight segments",
        })

    arr = np.array(deltas)
    mean_signed = float(np.mean(arr))
    mean_abs = float(np.mean(np.abs(arr)))
    std = float(np.std(arr))
    triggered = mean_abs >= HEADING_MIN_ABS_DEG and std <= HEADING_MAX_STD_DEG
    return CauseResult("heading_error", triggered, {
        "heading_error_deg": mean_signed,
        "mean_abs_deg": mean_abs,
        "std_deg": std,
        "n_segments": len(deltas),
        "segments": seg_records,
    })


def _read_dvl_lock_series(bag_dir: Path) -> tuple[list[tuple[int, bool]], str]:
    """Return time-stamped lock series, preferring /sensors/dvl/velocity."""
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        return [], "(no mcap)"

    ts_vel: list[tuple[int, bool]] = []
    ts_cov: list[tuple[int, bool]] = []
    for msg in read_ros2_messages(str(mcap)):
        topic = msg.channel.topic
        if topic == _T_DVL_VEL:
            try:
                t = _stamp_ns(msg.ros_msg.header.stamp)
                if t > 0:
                    ts_vel.append((t, bool(msg.ros_msg.beam_velocities_valid)))
            except AttributeError:
                pass
        elif topic == _T_DVL_ODOM_COV:
            try:
                t = _stamp_ns(msg.ros_msg.header.stamp)
                if t == 0:
                    continue
                c = msg.ros_msg.twist.covariance
                mx = max(float(c[0]), float(c[7]), float(c[14]))
                ts_cov.append((t, mx < LOCKED_TWIST_LINEAR_VAR_MAX))
            except (AttributeError, IndexError):
                pass
    if ts_vel:
        return ts_vel, _T_DVL_VEL
    return ts_cov, _T_DVL_ODOM_COV


def diag_dvl_dropout(bag_dir: Path, high_err_iv_s: list,
                     sbl_t0_ns: int) -> CauseResult:
    """Cause B: DVL lock loss correlated with EKF-error spikes."""
    ts_lock, src = _read_dvl_lock_series(bag_dir)
    if not ts_lock:
        return CauseResult("dvl_dropout", False, {"reason": "no DVL lock data"})

    lock_frac = _time_weighted_lock_fraction(ts_lock)
    t_start = ts_lock[0][0]
    t_end = ts_lock[-1][0]
    unlock_iv = _invert_lock_intervals(ts_lock, t_start, t_end)
    unlock_iv = _merge_intervals(unlock_iv, gap_merge_ns=int(0.5e9))

    longest_unlock_s = 0.0
    if unlock_iv:
        longest_unlock_s = max((b - a) / 1e9 for a, b in unlock_iv)

    overlaps: list[dict] = []
    for a_ns, b_ns in unlock_iv:
        a_rel = (a_ns - sbl_t0_ns) / 1e9
        b_rel = (b_ns - sbl_t0_ns) / 1e9
        if (b_rel - a_rel) < DVL_DROPOUT_MIN_S:
            continue
        for h in high_err_iv_s:
            ht0, ht1, peak = float(h[0]), float(h[1]), float(h[2])
            ov_a = max(a_rel, ht0)
            ov_b = min(b_rel, ht1)
            if ov_b > ov_a:
                overlaps.append({
                    "unlock_s": [a_rel, b_rel],
                    "high_err_s": [ht0, ht1, peak],
                    "overlap_s": ov_b - ov_a,
                })

    triggered = (
        (lock_frac is not None and lock_frac < DVL_LOCK_MIN_FRAC)
        or bool(overlaps)
    )
    return CauseResult("dvl_dropout", triggered, {
        "source_topic": src,
        "lock_fraction": lock_frac,
        "longest_unlock_s": longest_unlock_s,
        "n_unlock_intervals": len(unlock_iv),
        "n_high_err_overlaps": len(overlaps),
        "high_err_overlaps": overlaps[:20],   # cap for JSON readability
    })


def diag_init_transient(es_anchored) -> CauseResult:
    """Cause C: large error in first 30 s, settles to small steady state."""
    if es_anchored is None or es_anchored.n_pairs < 10:
        return CauseResult("init_transient", False, {"reason": "insufficient pairs"})
    t = es_anchored.t_rel_s
    e = es_anchored.err_m
    early = e[t < INIT_WINDOW_S]
    late = e[t >= INIT_WINDOW_S]
    if len(early) < 5 or len(late) < 5:
        return CauseResult("init_transient", False, {"reason": "too few samples"})
    mean_early = float(np.mean(early))
    mean_late = float(np.mean(late))
    ratio = mean_early / max(mean_late, 1e-9)
    triggered = ratio >= INIT_RATIO_MIN and mean_late < INIT_STEADY_MAX_M
    return CauseResult("init_transient", triggered, {
        "mean_first_30s": mean_early,
        "mean_after_30s": mean_late,
        "ratio": ratio,
    })


def diag_sbl_quality(bag_data, high_err_iv_s, meta) -> CauseResult:
    """Cause D: SBL acoustic quality degrades during error spikes."""
    std_p50 = meta.get("sbl_acoustic_std_p50")
    std_p95 = meta.get("sbl_acoustic_std_p95")

    if not bag_data.acoustic:
        return CauseResult("sbl_quality", False, {
            "reason": "no acoustic topic",
            "std_p50": std_p50, "std_p95": std_p95,
        })

    a_t = np.array([m.t_ns for m in bag_data.acoustic], dtype=np.int64)
    a_v = np.array([m.std_m for m in bag_data.acoustic])
    if len(a_t) == 0:
        return CauseResult("sbl_quality", False, {"reason": "no acoustic samples"})

    a_t0 = int(a_t[0])
    a_t_rel = (a_t - a_t0) / 1e9

    n_high_err_with_high_acoustic = 0
    for h in high_err_iv_s or []:
        ht0, ht1 = float(h[0]), float(h[1])
        mask = (a_t_rel >= ht0) & (a_t_rel <= ht1)
        if mask.any() and np.mean(a_v[mask]) > SBL_HIGH_STD_M:
            n_high_err_with_high_acoustic += 1
    overlap_ratio = (
        n_high_err_with_high_acoustic / len(high_err_iv_s)
        if high_err_iv_s else 0.0
    )

    triggered = (
        (std_p50 is not None and std_p50 > SBL_STD_MEDIAN_MAX)
        or overlap_ratio >= SBL_OVERLAP_MIN
    )
    return CauseResult("sbl_quality", triggered, {
        "std_p50": std_p50,
        "std_p95": std_p95,
        "overlap_ratio": overlap_ratio,
        "n_high_err": len(high_err_iv_s or []),
        "n_with_high_acoustic": n_high_err_with_high_acoustic,
    })


def diag_covariance_blowup(bag_dir: Path) -> CauseResult:
    """Cause E: /odometry/filtered/local pose covariance grows unboundedly."""
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        return CauseResult("ekf_divergence", False, {"reason": "no mcap"})

    ts: list[int] = []
    cov_xy: list[float] = []
    cov_max_diag: list[float] = []
    nan_or_neg = False
    for msg in read_ros2_messages(str(mcap)):
        if msg.channel.topic != _T_ODOM_LOCAL:
            continue
        try:
            t = _stamp_ns(msg.ros_msg.header.stamp)
            if t == 0:
                continue
            c = msg.ros_msg.pose.covariance
            diag = [float(c[i * 6 + i]) for i in range(6)]
            if any(not math.isfinite(d) or d < 0 for d in diag):
                nan_or_neg = True
            ts.append(t)
            cov_xy.append(diag[0] + diag[1])
            cov_max_diag.append(max(diag))
        except (AttributeError, IndexError, ValueError):
            continue

    if not ts:
        return CauseResult("ekf_divergence", False, {
            "reason": "no /odometry/filtered/local",
        })
    if nan_or_neg:
        return CauseResult("ekf_divergence", True, {
            "reason": "NaN/negative on covariance diagonal",
            "n_pose_msgs": len(ts),
        })

    ts_arr = np.array(ts, dtype=np.int64)
    cov_xy_arr = np.array(cov_xy)
    t_rel = (ts_arr - ts_arr[0]) / 1e9
    early_mask = t_rel < COV_WINDOW_S
    late_mask = t_rel >= (t_rel.max() - COV_WINDOW_S)
    if early_mask.sum() < 3 or late_mask.sum() < 3:
        return CauseResult("ekf_divergence", False, {
            "reason": "track too short",
            "n_pose_msgs": len(ts),
        })

    early_mean = float(np.mean(cov_xy_arr[early_mask]))
    late_mean = float(np.mean(cov_xy_arr[late_mask]))
    ratio = late_mean / max(early_mean, 1e-9)
    triggered = ratio >= COV_GROWTH_RATIO_MIN
    return CauseResult("ekf_divergence", triggered, {
        "n_pose_msgs": len(ts),
        "early_mean_xy": early_mean,
        "late_mean_xy": late_mean,
        "growth_ratio": ratio,
        "max_diag_value": float(max(cov_max_diag)),
    })


# ── Step 4: heading correction ───────────────────────────────────────────────

def rotate_around(E: np.ndarray, N: np.ndarray,
                  anchor_E: float, anchor_N: float,
                  angle_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Standard 2-D math CCW rotation by angle_deg around (anchor_E, anchor_N)."""
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    dE = E - anchor_E
    dN = N - anchor_N
    return anchor_E + c * dE - s * dN, anchor_N + s * dE + c * dN


def plot_corrected_overlay(sbl_track, anchored_aligned,
                           dr_corrected: tuple[np.ndarray, np.ndarray],
                           bag_name: str, info_lines: list[str],
                           out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(9, 9))
    epsg = _utm_epsg(sbl_track.zone_num, sbl_track.zone_letter)
    if not add_satellite_basemap(ax, epsg, alpha=1.0, zoom=18):
        ax.set_facecolor("#dddddd")

    draw_track(ax, sbl_track.E, sbl_track.N_utm, "crimson",
               "SBL (reference)", lw=3.0, zorder=5)
    add_direction_arrows(ax, sbl_track.E, sbl_track.N_utm, "crimson", zorder=6, n=12)

    if anchored_aligned is not None:
        _, E_orig, N_orig = anchored_aligned
        ax.plot(E_orig, N_orig, color="royalblue", lw=2.0, alpha=0.55,
                zorder=3, label="EKF anchored (original)")

    cE, cN = dr_corrected
    draw_track(ax, cE, cN, "darkorange",
               "DR corrected (heading rotation)", lw=2.6, zorder=4)
    add_direction_arrows(ax, cE, cN, "darkorange", zorder=5, n=12)

    add_start_marker(ax, float(sbl_track.E[0]), float(sbl_track.N_utm[0]), zorder=9)

    xs = [sbl_track.E, cE]
    ys = [sbl_track.N_utm, cN]
    if anchored_aligned is not None:
        xs.append(anchored_aligned[1])
        ys.append(anchored_aligned[2])
    x_all = np.concatenate(xs); y_all = np.concatenate(ys)
    span = max(x_all.max() - x_all.min(), y_all.max() - y_all.min())
    pad = max(5.0, 0.05 * span)
    cx = 0.5 * (x_all.min() + x_all.max())
    cy = 0.5 * (y_all.min() + y_all.max())
    half = 0.5 * span + pad
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color("black"); spine.set_linewidth(0.8)

    ax.text(0.02, 0.02, "\n".join(info_lines), transform=ax.transAxes,
            fontsize=8, va="bottom", ha="left", zorder=11,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="white",
                      edgecolor="lightgray", alpha=0.85, linewidth=0.5))
    add_scale_bar(ax, length_m=10.0)
    leg = ax.legend(loc="upper right", frameon=True, fontsize=9)
    if leg is not None:
        leg.get_frame().set_alpha(0.85)
        leg.get_frame().set_edgecolor("lightgray")

    fig.suptitle(f"{bag_name} — heading-corrected DR vs SBL", fontsize=10)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved: {out_path.name}")


# ── Per-bag orchestration ────────────────────────────────────────────────────

def diagnose_bag(bag_dir: Path, out_dir: Path, sbl_overlay_script: Path,
                 track_mode: str, force: bool, do_correct: bool,
                 overlay_timeout_s: int) -> BagDiagnosis:
    name = bag_dir.name
    print(f"\n[{name}]")

    diag = BagDiagnosis(bag_name=name, bag_dir=str(bag_dir))

    # Step 1
    ok, meta = run_overlay(bag_dir, out_dir, sbl_overlay_script, track_mode,
                           force, overlay_timeout_s)
    if not ok or meta is None:
        diag.classification = "OVERLAY_FAILED"
        return diag

    diag.metrics = {
        k: meta.get(k) for k in (
            "n_sbl_raw", "n_sbl_gated",
            "n_ekf_anchored", "n_ekf_local", "n_gnss_surfaced",
            "drift_rate_m_per_100m_anchored", "drift_rate_m_per_100m_local",
            "mean_error_m", "max_error_m",
            "sbl_acoustic_std_p50", "sbl_acoustic_std_p95",
        )
    }

    # Step 2
    diag.classification = classify(meta)
    print(f"  class: {diag.classification}  "
          f"mean={meta.get('mean_error_m')}  max={meta.get('max_error_m')}")

    # Health profile — runs for every classification (we need it to compare
    # GOOD vs FAILED bags side-by-side).
    print("  computing health profile...")
    timeline_pts: list = []
    diag.health = compute_health_profile(bag_dir, t_axis_out=timeline_pts)
    if diag.health.duration_s is not None and diag.duration_s is None:
        diag.duration_s = diag.health.duration_s
    _print_health_summary(diag.health)
    if timeline_pts:
        plot_health_timeline(
            timeline_pts, diag.health,
            bag_name=name,
            classification=diag.classification,
            out_path=out_dir / f"ekf_sbl_{name}_health.png",
        )

    if diag.classification in ("GOOD", "INSUFFICIENT_DATA"):
        return diag

    # Step 3
    print("  diagnosing causes...")
    try:
        bag = overlay_read_bag(bag_dir, want_local=True)
    except SystemExit:
        return diag

    sbl_msgs = gate_sbl(bag.sbl, max_sbl_std=5.0)
    sbl_track = navsatfix_to_track(sbl_msgs, "SBL") if sbl_msgs else None
    if sbl_track is None or len(sbl_track.t_ns) == 0:
        return diag
    diag.duration_s = float((sbl_track.t_ns[-1] - sbl_track.t_ns[0]) / 1e9)

    anchored_track = navsatfix_to_track(bag.anchored, "anchored") if bag.anchored else None
    local_track = navsatfix_to_track(bag.local, "local") if bag.local else None

    anchored_aligned = local_aligned = None
    es_anchored = None
    if anchored_track is not None:
        E, N, _, _ = align_to_anchor(anchored_track, sbl_track)
        anchored_aligned = (anchored_track, E, N)
        es_anchored = pair_errors(anchored_track, sbl_track, E, N)
    if local_track is not None:
        E, N, _, _ = align_to_anchor(local_track, sbl_track)
        local_aligned = (local_track, E, N)

    high_err_iv = meta.get("high_error_intervals_s") or []
    sbl_t0_ns = int(sbl_track.t_ns[0])

    causes: list[dict] = []

    # A: heading — prefer pure DR; fall back to anchored (same shape, different origin)
    target_aligned = local_aligned or anchored_aligned
    if target_aligned is not None:
        c_a = diag_heading(sbl_track, target_aligned[0], target_aligned[1], target_aligned[2])
    else:
        c_a = CauseResult("heading_error", False, {"reason": "no DR/EKF track"})
    if c_a.triggered:
        causes.append({"name": c_a.name, "evidence": c_a.evidence})
        print(f"    heading_error: TRIGGERED  Δ={c_a.evidence.get('heading_error_deg'):+.1f}° "
              f"(std {c_a.evidence.get('std_deg'):.1f}°, "
              f"{c_a.evidence.get('n_segments')} segments)")

    # B: DVL dropout
    c_b = diag_dvl_dropout(bag_dir, high_err_iv, sbl_t0_ns)
    if c_b.triggered:
        causes.append({"name": c_b.name, "evidence": c_b.evidence})
        lf = c_b.evidence.get("lock_fraction")
        lf_str = f"{lf:.3f}" if isinstance(lf, float) else "?"
        print(f"    dvl_dropout:   TRIGGERED  lock_frac={lf_str}, "
              f"longest_unlock={c_b.evidence.get('longest_unlock_s'):.1f}s")

    # C: init transient
    c_c = diag_init_transient(es_anchored)
    if c_c.triggered:
        causes.append({"name": c_c.name, "evidence": c_c.evidence})
        print(f"    init_transient: TRIGGERED  early/late = "
              f"{c_c.evidence['mean_first_30s']:.2f} m / "
              f"{c_c.evidence['mean_after_30s']:.2f} m")

    # D: SBL quality
    c_d = diag_sbl_quality(bag, high_err_iv, meta)
    if c_d.triggered:
        causes.append({"name": c_d.name, "evidence": c_d.evidence})
        print(f"    sbl_quality:    TRIGGERED  std_p50={c_d.evidence.get('std_p50')}, "
              f"overlap={c_d.evidence.get('overlap_ratio'):.2f}")

    # E: covariance blow-up
    c_e = diag_covariance_blowup(bag_dir)
    if c_e.triggered:
        causes.append({"name": c_e.name, "evidence": c_e.evidence})
        print(f"    ekf_divergence: TRIGGERED  ratio={c_e.evidence.get('growth_ratio')}")

    diag.causes = causes

    # Demote classification when SBL itself is the suspect signal
    if c_d.triggered and diag.classification in ("DEGRADED", "FAILED"):
        diag.classification = f"{diag.classification}-SBL-UNTRUSTED"

    # Step 4: heading correction (only if cause A confidently triggered)
    if (do_correct and c_a.triggered and target_aligned is not None):
        h_deg = float(c_a.evidence["heading_error_deg"])
        anchor_E = float(sbl_track.E[0])
        anchor_N = float(sbl_track.N_utm[0])
        # DR has bearing offset of +h_deg vs SBL → rotate CCW by +h_deg corrects.
        cE, cN = rotate_around(target_aligned[1], target_aligned[2],
                               anchor_E, anchor_N, +h_deg)
        es_corr = pair_errors(target_aligned[0], sbl_track, cE, cN)
        if es_corr.n_pairs:
            corr_mean = float(np.mean(es_corr.err_m))
            corr_max = float(np.max(es_corr.err_m))
            diag.correction = {
                "applied_heading_offset_deg": h_deg,
                "corrected_mean_error_m": corr_mean,
                "corrected_max_error_m": corr_max,
                "corrected_drift_rate_m_per_100m": float(es_corr.drift_rate_m_per_100m),
                "corrected_n_pairs": int(es_corr.n_pairs),
            }
            print(f"  heading correction {h_deg:+.1f}° → "
                  f"mean {corr_mean:.2f} m  max {corr_max:.2f} m  "
                  f"drift {es_corr.drift_rate_m_per_100m:.2f} m/100m")

            if corr_mean < GOOD_MEAN_MAX and corr_max < GOOD_MAX_MAX:
                diag.classification = "SALVAGED"

            info_lines = [
                f"Heading correction: {h_deg:+.2f}°",
                f"Original: mean {meta.get('mean_error_m'):.2f} m  max {meta.get('max_error_m'):.2f} m",
                f"Corrected: mean {corr_mean:.2f} m  max {corr_max:.2f} m",
                f"Drift (corrected): {es_corr.drift_rate_m_per_100m:.2f} m/100 m",
            ]
            plot_corrected_overlay(
                sbl_track, anchored_aligned, (cE, cN),
                bag_name=name, info_lines=info_lines,
                out_path=out_dir / f"ekf_sbl_{name}_corrected.png",
            )

    return diag


# ── Aggregate output ─────────────────────────────────────────────────────────

def _health_to_dict(hp: HealthProfile | None) -> dict | None:
    if hp is None:
        return None
    return {
        "duration_s": hp.duration_s,
        "dvl_source_topic": hp.dvl_source_topic,
        "dvl_lock_fraction": hp.dvl_lock_fraction,
        "dvl_longest_unlock_s": hp.dvl_longest_unlock_s,
        "dvl_n_unlock_intervals": hp.dvl_n_unlock_intervals,
        "dvl_unlock_intervals_s": hp.dvl_unlock_intervals_s,
        "depth_z_min_m": hp.depth_z_min_m,
        "depth_z_max_m": hp.depth_z_max_m,
        "depth_z_range_m": hp.depth_z_range_m,
        "odom_n": hp.odom_n,
        "cov_xy_initial": hp.cov_xy_initial,
        "cov_xy_max": hp.cov_xy_max,
        "cov_xy_final": hp.cov_xy_final,
        "cov_xy_growth_ratio": hp.cov_xy_growth_ratio,
        "cov_xy_blowup_t_s": hp.cov_xy_blowup_t_s,
        "cov_yaw_max": hp.cov_yaw_max,
        "cov_nan_or_neg": hp.cov_nan_or_neg,
        "speed_mean_mps": hp.speed_mean_mps,
        "speed_p95_mps": hp.speed_p95_mps,
        "rosout_n_warn": hp.rosout_n_warn,
        "rosout_n_error": hp.rosout_n_error,
        "rosout_n_ekf_pattern": hp.rosout_n_ekf_pattern,
        "rosout_top_messages": hp.rosout_top_messages,
    }


def write_per_bag_json(diag: BagDiagnosis, path: Path) -> None:
    payload = {
        "bag": diag.bag_name,
        "bag_dir": diag.bag_dir,
        "duration_s": diag.duration_s,
        "classification": diag.classification,
        "metrics": diag.metrics,
        "causes": diag.causes,
        "correction": diag.correction,
        "health": _health_to_dict(diag.health),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")


def write_summary_csv(diags: list[BagDiagnosis], path: Path) -> None:
    fields = [
        "bag", "duration_s", "classification",
        "mean_error_m", "max_error_m",
        "drift_rate_m_per_100m_anchored", "drift_rate_m_per_100m_local",
        "primary_cause",
        "heading_error_deg",
        "applied_heading_offset_deg",
        "corrected_mean_error_m", "corrected_max_error_m",
        "corrected_drift_rate_m_per_100m",
        # Health profile (always populated)
        "dvl_lock_fraction", "dvl_longest_unlock_s",
        "cov_xy_initial", "cov_xy_max", "cov_xy_final",
        "cov_xy_growth_ratio", "cov_xy_blowup_t_s",
        "depth_z_range_m",
        "speed_mean_mps", "speed_p95_mps",
        "rosout_n_warn", "rosout_n_ekf_pattern",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for d in diags:
            primary = d.causes[0]["name"] if d.causes else ""
            heading_err = ""
            for c in d.causes:
                if c["name"] == "heading_error":
                    heading_err = c["evidence"].get("heading_error_deg", "")
                    break
            row = {
                "bag": d.bag_name,
                "duration_s": d.duration_s if d.duration_s is not None else "",
                "classification": d.classification,
                "mean_error_m": d.metrics.get("mean_error_m", "") or "",
                "max_error_m": d.metrics.get("max_error_m", "") or "",
                "drift_rate_m_per_100m_anchored":
                    d.metrics.get("drift_rate_m_per_100m_anchored", "") or "",
                "drift_rate_m_per_100m_local":
                    d.metrics.get("drift_rate_m_per_100m_local", "") or "",
                "primary_cause": primary,
                "heading_error_deg": heading_err,
                "applied_heading_offset_deg":
                    (d.correction or {}).get("applied_heading_offset_deg", ""),
                "corrected_mean_error_m":
                    (d.correction or {}).get("corrected_mean_error_m", ""),
                "corrected_max_error_m":
                    (d.correction or {}).get("corrected_max_error_m", ""),
                "corrected_drift_rate_m_per_100m":
                    (d.correction or {}).get("corrected_drift_rate_m_per_100m", ""),
            }
            # Health columns
            h = d.health
            if h is not None:
                row.update({
                    "dvl_lock_fraction": h.dvl_lock_fraction if h.dvl_lock_fraction is not None else "",
                    "dvl_longest_unlock_s": h.dvl_longest_unlock_s if h.dvl_longest_unlock_s is not None else "",
                    "cov_xy_initial": h.cov_xy_initial if h.cov_xy_initial is not None else "",
                    "cov_xy_max": h.cov_xy_max if h.cov_xy_max is not None else "",
                    "cov_xy_final": h.cov_xy_final if h.cov_xy_final is not None else "",
                    "cov_xy_growth_ratio": h.cov_xy_growth_ratio if h.cov_xy_growth_ratio is not None else "",
                    "cov_xy_blowup_t_s": h.cov_xy_blowup_t_s if h.cov_xy_blowup_t_s is not None else "",
                    "depth_z_range_m": h.depth_z_range_m if h.depth_z_range_m is not None else "",
                    "speed_mean_mps": h.speed_mean_mps if h.speed_mean_mps is not None else "",
                    "speed_p95_mps": h.speed_p95_mps if h.speed_p95_mps is not None else "",
                    "rosout_n_warn": h.rosout_n_warn,
                    "rosout_n_ekf_pattern": h.rosout_n_ekf_pattern,
                })
            w.writerow(row)


def aggregate_stats(diags: list[BagDiagnosis]) -> dict:
    eligible = [d for d in diags if d.classification in ("GOOD", "SALVAGED")]
    means: list[float] = []
    maxs: list[float] = []
    drifts: list[float] = []
    for d in eligible:
        if d.classification == "SALVAGED" and d.correction:
            means.append(d.correction["corrected_mean_error_m"])
            maxs.append(d.correction["corrected_max_error_m"])
            drifts.append(d.correction["corrected_drift_rate_m_per_100m"])
        else:
            mean = d.metrics.get("mean_error_m")
            mx = d.metrics.get("max_error_m")
            dr = d.metrics.get("drift_rate_m_per_100m_anchored")
            if mean is not None:
                means.append(mean)
                maxs.append(mx if mx is not None else mean)
                drifts.append(dr if dr is not None else 0.0)

    out: dict = {
        "n_total": len(diags),
        "n_eligible": len(eligible),
        "n_good": sum(1 for d in eligible if d.classification == "GOOD"),
        "n_salvaged": sum(1 for d in eligible if d.classification == "SALVAGED"),
        "n_failed": sum(1 for d in diags if d.classification.startswith("FAILED")),
        "n_degraded": sum(1 for d in diags if d.classification.startswith("DEGRADED")),
    }
    if means:
        out["mean_error_m_avg"] = float(np.mean(means))
        out["mean_error_m_worst_bag"] = float(np.max(means))
        out["max_error_m_avg"] = float(np.mean(maxs))
        out["max_error_m_worst_bag"] = float(np.max(maxs))
        out["drift_rate_m_per_100m_avg"] = float(np.mean(drifts))
        out["drift_rate_m_per_100m_worst"] = float(np.max(drifts))
    return out


def write_diagnosis_report(diags: list[BagDiagnosis], agg: dict, path: Path) -> None:
    L: list[str] = ["# EKF Mission Diagnosis Report", ""]

    # ── Summary table
    L += ["## Summary", ""]
    L.append("| Bag | Class | Duration | Mean | Max | Cause | Heading | Corrected mean |")
    L.append("|---|---|---|---|---|---|---|---|")
    for d in diags:
        primary = d.causes[0]["name"] if d.causes else "-"
        h = "-"
        for c in d.causes:
            if c["name"] == "heading_error":
                v = c["evidence"].get("heading_error_deg")
                if v is not None:
                    h = f"{v:+.1f}°"
                break
        cm = "-"
        if d.correction:
            cm = f"{d.correction['corrected_mean_error_m']:.2f} m"
        mn = (f"{d.metrics['mean_error_m']:.2f} m"
              if d.metrics.get("mean_error_m") is not None else "-")
        mx = (f"{d.metrics['max_error_m']:.2f} m"
              if d.metrics.get("max_error_m") is not None else "-")
        dur = f"{d.duration_s:.0f} s" if d.duration_s else "-"
        L.append(f"| {d.bag_name} | {d.classification} | {dur} | {mn} | {mx} | "
                 f"{primary} | {h} | {cm} |")
    L.append("")

    # ── Per-bag detail
    L += ["## Per-bag diagnosis", ""]
    for d in diags:
        L.append(f"### {d.bag_name}")
        L.append(f"- **Class**: `{d.classification}`")
        if d.duration_s is not None:
            L.append(f"- **Duration**: {d.duration_s:.1f} s")
        if d.metrics.get("mean_error_m") is not None:
            L.append(f"- **Mean / Max error**: "
                     f"{d.metrics['mean_error_m']:.2f} m / "
                     f"{d.metrics['max_error_m']:.2f} m")
            if d.metrics.get("drift_rate_m_per_100m_anchored") is not None:
                L.append(f"- **Drift rate (anchored EKF)**: "
                         f"{d.metrics['drift_rate_m_per_100m_anchored']:.2f} m / 100 m")
            if d.metrics.get("drift_rate_m_per_100m_local") is not None:
                L.append(f"- **Drift rate (local DR)**: "
                         f"{d.metrics['drift_rate_m_per_100m_local']:.2f} m / 100 m")
        n_sbl = d.metrics.get("n_sbl_gated") or 0
        n_ekf = d.metrics.get("n_ekf_anchored") or 0
        L.append(f"- **SBL gated / EKF anchored samples**: {n_sbl} / {n_ekf}")
        L.append(f"- **Overlay PNG**: `{d.bag_name}/ekf_sbl_{d.bag_name}_overlay.png`")
        if d.causes:
            L.append("- **Causes** (in trigger order):")
            for c in d.causes:
                ev = c["evidence"]
                summary = ", ".join(
                    f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}"
                    for k, v in ev.items()
                    if not isinstance(v, (list, dict))
                )
                L.append(f"  - **{c['name']}**: {summary}")
        elif d.classification.startswith(("DEGRADED", "FAILED")):
            L.append("- **Causes**: none triggered (no obvious single cause)")
        if d.correction:
            corr = d.correction
            L.append(f"- **Heading correction applied**: "
                     f"{corr['applied_heading_offset_deg']:+.2f}° → "
                     f"mean {corr['corrected_mean_error_m']:.2f} m, "
                     f"max {corr['corrected_max_error_m']:.2f} m, "
                     f"drift {corr['corrected_drift_rate_m_per_100m']:.2f} m/100m  "
                     f"(see `{d.bag_name}/ekf_sbl_{d.bag_name}_corrected.png`)")
        L.append("")

    # ── Aggregate
    L += ["## Aggregate (GOOD ∪ SALVAGED)", ""]
    if agg.get("n_eligible", 0) == 0:
        L.append("No eligible bags (all FAILED, INSUFFICIENT_DATA, or unsalvageable).")
    else:
        L.append(f"- **Eligible bags**: {agg['n_eligible']} of {agg['n_total']} "
                 f"({agg['n_good']} GOOD + {agg['n_salvaged']} SALVAGED)")
        if agg.get("mean_error_m_avg") is not None:
            L.append(f"- **Mean error**: avg {agg['mean_error_m_avg']:.2f} m, "
                     f"worst-bag mean {agg['mean_error_m_worst_bag']:.2f} m")
            L.append(f"- **Max error**: avg {agg['max_error_m_avg']:.2f} m, "
                     f"worst-bag max {agg['max_error_m_worst_bag']:.2f} m")
            L.append(f"- **Drift rate**: avg {agg['drift_rate_m_per_100m_avg']:.2f} m/100m, "
                     f"worst {agg['drift_rate_m_per_100m_worst']:.2f} m/100m")
    L.append("")

    # ── Cause distribution
    cause_counts: dict[str, int] = {}
    for d in diags:
        for c in d.causes:
            cause_counts[c["name"]] = cause_counts.get(c["name"], 0) + 1
    if cause_counts:
        L += ["## Cause distribution", ""]
        for name, n in sorted(cause_counts.items(), key=lambda x: -x[1]):
            L.append(f"- **{name}**: {n} bag(s)")
        L.append("")

    # ── Health-profile side-by-side table (the actual comparison)
    L += ["## Health profile — side-by-side", ""]
    L.append("Each bag's single-pass sensor / filter health, regardless of "
             "classification. Look across rows to see which signals separate "
             "GOOD from FAILED.")
    L.append("")
    L.append("| Bag | Class | DVL lock | Longest unlock | cov_xy initial → max → final | growth | 5× blow-up @ | Depth range | Speed (avg, p95) | rosout warn / EKF-pattern |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    for d in diags:
        h = d.health
        if h is None:
            L.append(f"| {d.bag_name} | {d.classification} | – | – | – | – | – | – | – | – |")
            continue
        lock = f"{h.dvl_lock_fraction:.3f}" if h.dvl_lock_fraction is not None else "–"
        unl = f"{h.dvl_longest_unlock_s:.1f} s" if h.dvl_longest_unlock_s is not None else "–"
        cov_str = "–"
        if h.cov_xy_initial is not None and h.cov_xy_max is not None and h.cov_xy_final is not None:
            cov_str = (f"{h.cov_xy_initial:.2e} → {h.cov_xy_max:.2e} → "
                       f"{h.cov_xy_final:.2e}")
        gr = f"{h.cov_xy_growth_ratio:.1f}×" if h.cov_xy_growth_ratio is not None else "–"
        blow = (f"{h.cov_xy_blowup_t_s:.0f} s"
                if h.cov_xy_blowup_t_s is not None else "no")
        depth = f"{h.depth_z_range_m:.2f} m" if h.depth_z_range_m is not None else "–"
        spd = "–"
        if h.speed_mean_mps is not None and h.speed_p95_mps is not None:
            spd = f"{h.speed_mean_mps:.2f} / {h.speed_p95_mps:.2f} m/s"
        ros = f"{h.rosout_n_warn} / {h.rosout_n_ekf_pattern}"
        L.append(f"| {d.bag_name.replace('zermatt_', '')} | {d.classification} | "
                 f"{lock} | {unl} | {cov_str} | {gr} | {blow} | {depth} | "
                 f"{spd} | {ros} |")
    L.append("")

    # ── Group-mean comparison
    by_class: dict[str, list[BagDiagnosis]] = {}
    for d in diags:
        # Bucket SBL-UNTRUSTED into FAILED for grouping
        cls = d.classification.split("-")[0]
        by_class.setdefault(cls, []).append(d)

    def _avg(seq):
        seq = [x for x in seq if x is not None]
        return float(np.mean(seq)) if seq else None

    L += ["", "## Group means (helps spot what differentiates GOOD vs FAILED)", ""]
    L.append("| Class | n | DVL lock avg | Longest unlock avg | cov_xy_initial avg | cov_xy_max avg | growth avg | rosout warn avg |")
    L.append("|---|---|---|---|---|---|---|---|")
    for cls in ("GOOD", "DEGRADED", "FAILED", "INSUFFICIENT_DATA"):
        if cls not in by_class:
            continue
        ds = by_class[cls]
        hs = [d.health for d in ds if d.health is not None]
        n = len(ds)
        avg_lock = _avg([h.dvl_lock_fraction for h in hs])
        avg_unl = _avg([h.dvl_longest_unlock_s for h in hs])
        avg_cov_i = _avg([h.cov_xy_initial for h in hs])
        avg_cov_m = _avg([h.cov_xy_max for h in hs])
        avg_growth = _avg([h.cov_xy_growth_ratio for h in hs])
        avg_warn = _avg([float(h.rosout_n_warn) for h in hs])
        def _f(v, fmt=".3f"):
            return format(v, fmt) if v is not None else "–"
        L.append(f"| {cls} | {n} | {_f(avg_lock)} | "
                 f"{_f(avg_unl, '.1f')} s | "
                 f"{_f(avg_cov_i, '.2e')} | {_f(avg_cov_m, '.2e')} | "
                 f"{_f(avg_growth, '.1f')}× | {_f(avg_warn, '.1f')} |")
    L.append("")

    # ── Top rosout messages across non-GOOD bags (often very revealing)
    bad_msgs: dict[str, int] = {}
    for d in diags:
        if d.classification.startswith(("DEGRADED", "FAILED")) and d.health:
            for top in d.health.rosout_top_messages:
                bad_msgs[top] = bad_msgs.get(top, 0) + 1
    if bad_msgs:
        L += ["## Frequent rosout messages on DEGRADED / FAILED bags", ""]
        for m, _n in sorted(bad_msgs.items(), key=lambda x: -x[1])[:15]:
            L.append(f"- {m}")
        L.append("")

    path.write_text("\n".join(L), encoding="utf-8")


# ── CLI / main ───────────────────────────────────────────────────────────────

def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bags_root", type=Path, help="Root directory containing bag dirs.")
    ap.add_argument("--output-dir", type=Path, required=True,
                    help="Output directory for diagnosis results.")
    ap.add_argument("--pattern", type=str, default="zermatt_*",
                    help="Glob on bag-dir basenames (default: zermatt_*)")
    ap.add_argument("--include", type=str, default="",
                    help="Comma-separated additional include patterns")
    ap.add_argument("--exclude", type=str, default="",
                    help="Comma-separated exclude patterns")
    ap.add_argument("--sbl-overlay-script", type=Path,
                    default=_SCRIPTS_DIR / "ekf_sbl_overlay.py",
                    help="Path to ekf_sbl_overlay.py")
    ap.add_argument("--force", action="store_true",
                    help="Re-run overlay subprocess even if metadata.json exists")
    ap.add_argument("--no-correct", action="store_true",
                    help="Skip heading correction step")
    ap.add_argument("--track-mode", choices=("anchored", "local", "both"),
                    default="both",
                    help="Passed through to ekf_sbl_overlay.py (default: both)")
    ap.add_argument("--overlay-timeout", type=int, default=3600,
                    help="Per-bag overlay subprocess timeout in seconds "
                         "(default: 3600 = 60 min; bump higher for >1 GB bags)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    bags_root = args.bags_root.resolve()
    if not bags_root.is_dir():
        print(f"ERROR: not a directory: {bags_root}", file=sys.stderr)
        return 1

    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    include = [s.strip() for s in args.include.split(",") if s.strip()]
    exclude = [s.strip() for s in args.exclude.split(",") if s.strip()]

    bags = find_bag_dirs(bags_root, args.pattern, include, exclude)
    if not bags:
        print(f"ERROR: no bags found matching pattern={args.pattern!r} under {bags_root}",
              file=sys.stderr)
        return 1

    print(f"Found {len(bags)} bag(s):")
    for b in bags:
        print(f"  - {b.name}")

    diags: list[BagDiagnosis] = []
    for bag_dir in bags:
        per_bag_out = out_dir / bag_dir.name
        diag = diagnose_bag(
            bag_dir, per_bag_out, args.sbl_overlay_script,
            args.track_mode, args.force,
            do_correct=not args.no_correct,
            overlay_timeout_s=args.overlay_timeout,
        )
        write_per_bag_json(diag, per_bag_out / "diagnosis.json")
        diags.append(diag)

    write_summary_csv(diags, out_dir / "summary.csv")
    agg = aggregate_stats(diags)
    (out_dir / "aggregate_stats.json").write_text(
        json.dumps(agg, indent=2), encoding="utf-8"
    )
    write_diagnosis_report(diags, agg, out_dir / "diagnosis_report.md")

    print(f"\n{'=' * 60}")
    print(f"Done — {len(diags)} bag(s) processed")
    print(f"  summary.csv          → {out_dir / 'summary.csv'}")
    print(f"  diagnosis_report.md  → {out_dir / 'diagnosis_report.md'}")
    print(f"  aggregate_stats.json → {out_dir / 'aggregate_stats.json'}")
    print(f"{'=' * 60}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
