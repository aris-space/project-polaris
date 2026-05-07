"""
EKF vs SBL drift analysis: compares the GNSS-anchored EKF track against the
SBL (Water Linked UGPS G2) acoustic position as an independent underwater
reference.

Pipeline assumed (from ekf_anchored.launch.py):
    imu_yaw_correction → ekf_local_node → odometry_validator → gnss_anchored_pose
The anchored track lives on /gps/filtered/global (NavSatFix in lat/lon).
There is no global EKF — /odometry/filtered/global is local DR shifted by a
fixed GNSS anchor offset.

Usage:
    python scripts/ekf_sbl_overlay.py /path/to/bag_dir \
        [--track anchored|local|both] [--max-sbl-std 5.0] [--max-gnss-hacc 2.0] \
        [--smooth-sbl 3] [--output-dir ./output]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import utm
from mcap_ros2.reader import read_ros2_messages

from odom_to_gnss_overlay import (
    FixMsg, UbxHpMsg, GpsFilteredMsg, DRTrack,
    _stamp_ns, _h_acc_from_navsatfix, _h_acc_from_ubx,
    merge_h_acc, gate_fixes,
)
from report_overlay import (
    draw_track, add_direction_arrows, add_start_marker,
    add_scale_bar, add_satellite_basemap, _utm_epsg,
)
from sbl_accuracy_analysis import nsf_h_std, nearest_idx


# ── topics ────────────────────────────────────────────────────────────────────

_T_GLOBAL = "/gps/filtered/global"
_T_LOCAL  = "/gps/filtered"
_T_SBL    = "/waterlinked_ugps/navsatfix"
_T_ACOUST = "/waterlinked_ugps/locator_acoustic_quality"
_T_FIX    = "/fix"
_T_UBX    = "/ubx_nav_hp_pos_llh"


# ── data classes ──────────────────────────────────────────────────────────────

@dataclass
class SblMsg:
    t_ns: int
    lat:  float
    lon:  float
    h_std: float          # max horizontal 1-σ from covariance (m)


@dataclass
class AcousticMsg:
    t_ns: int
    std_m: float


@dataclass
class BagData:
    anchored: list = field(default_factory=list)   # list[GpsFilteredMsg]
    local:    list = field(default_factory=list)   # list[GpsFilteredMsg]
    sbl:      list = field(default_factory=list)   # list[SblMsg]
    acoustic: list = field(default_factory=list)   # list[AcousticMsg]
    fix:      list = field(default_factory=list)   # list[FixMsg]
    ubx_hp:   list = field(default_factory=list)   # list[UbxHpMsg]


# ── bag reader ────────────────────────────────────────────────────────────────

def read_bag(bag_dir: Path, want_local: bool) -> BagData:
    mcap_path = bag_dir / f"{bag_dir.name}_0.mcap"
    if not mcap_path.exists():
        candidates = list(bag_dir.glob("*.mcap"))
        if not candidates:
            print(f"ERROR: no MCAP found in {bag_dir}", file=sys.stderr)
            sys.exit(1)
        mcap_path = candidates[0]

    wanted = {_T_GLOBAL, _T_SBL, _T_ACOUST, _T_FIX, _T_UBX}
    if want_local:
        wanted.add(_T_LOCAL)

    data = BagData()
    for msg in read_ros2_messages(str(mcap_path)):
        topic = msg.channel.topic
        if topic not in wanted:
            continue
        ros = msg.ros_msg
        try:
            t = _stamp_ns(ros.header.stamp)
            if t == 0:
                continue
            if topic == _T_GLOBAL:
                data.anchored.append(GpsFilteredMsg(t_ns=t,
                                                    lat=float(ros.latitude),
                                                    lon=float(ros.longitude)))
            elif topic == _T_LOCAL:
                data.local.append(GpsFilteredMsg(t_ns=t,
                                                 lat=float(ros.latitude),
                                                 lon=float(ros.longitude)))
            elif topic == _T_SBL:
                data.sbl.append(SblMsg(
                    t_ns=t,
                    lat=float(ros.latitude),
                    lon=float(ros.longitude),
                    h_std=nsf_h_std(list(ros.position_covariance)),
                ))
            elif topic == _T_ACOUST:
                data.acoustic.append(AcousticMsg(t_ns=t, std_m=float(ros.vector.x)))
            elif topic == _T_FIX:
                data.fix.append(FixMsg(
                    t_ns=t,
                    lat=float(ros.latitude),
                    lon=float(ros.longitude),
                    status=int(ros.status.status),
                    cov=list(ros.position_covariance),
                    cov_type=int(ros.position_covariance_type),
                ))
            elif topic == _T_UBX:
                data.ubx_hp.append(UbxHpMsg(t_ns=t, h_acc_raw=int(ros.h_acc)))
        except AttributeError:
            continue
    return data


# ── tracks ────────────────────────────────────────────────────────────────────

def navsatfix_to_track(msgs: list, name: str) -> DRTrack | None:
    """Convert any list of (t_ns, lat, lon) messages to a UTM DRTrack."""
    if not msgs:
        print(f"WARNING: no messages for {name}")
        return None
    t_ns = np.array([m.t_ns for m in msgs], dtype=np.int64)
    lat = np.array([m.lat for m in msgs])
    lon = np.array([m.lon for m in msgs])
    E = np.empty(len(msgs))
    N = np.empty(len(msgs))
    zone_num, zone_letter = None, None
    for i, m in enumerate(msgs):
        e, n, zn, zl = utm.from_latlon(m.lat, m.lon)
        E[i], N[i] = e, n
        if zone_num is None:
            zone_num, zone_letter = zn, zl
    dE = np.diff(E, prepend=E[0])
    dN = np.diff(N, prepend=N[0])
    dist = np.cumsum(np.hypot(dE, dN))
    return DRTrack(t_ns=t_ns, lat=lat, lon=lon,
                   E=E, N_utm=N, dist=dist,
                   zone_num=zone_num, zone_letter=zone_letter)


def gate_sbl(msgs: list, max_sbl_std: float) -> list:
    """Drop null-island, drop h_std > threshold, drop frozen-position runs."""
    if not msgs:
        return []
    out = []
    last_lat, last_lon = None, None
    frozen_run = 0
    for m in msgs:
        if abs(m.lat) < 0.1 and abs(m.lon) < 0.1:
            continue
        if m.h_std > max_sbl_std or m.h_std <= 0.0:
            continue
        if last_lat is not None and m.lat == last_lat and m.lon == last_lon:
            frozen_run += 1
            if frozen_run >= 3:        # tolerate up to 2 stale repeats; drop deeper runs
                continue
        else:
            frozen_run = 0
        last_lat, last_lon = m.lat, m.lon
        out.append(m)
    return out


def smooth_track(track: DRTrack, window: int) -> DRTrack:
    """Centred median filter on E and N (window odd, ≥ 3). Keeps t_ns and dist."""
    if window < 3 or len(track.E) < window:
        return track
    half = window // 2
    n = len(track.E)
    E_s = track.E.copy()
    N_s = track.N_utm.copy()
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        E_s[i] = np.median(track.E[lo:hi])
        N_s[i] = np.median(track.N_utm[lo:hi])
    return DRTrack(t_ns=track.t_ns, lat=track.lat, lon=track.lon,
                   E=E_s, N_utm=N_s, dist=track.dist,
                   zone_num=track.zone_num, zone_letter=track.zone_letter)


# ── pairing & errors ──────────────────────────────────────────────────────────

@dataclass
class ErrorSeries:
    t_rel_s:    np.ndarray   # seconds since first SBL sample
    err_m:      np.ndarray   # Euclidean EKF↔SBL in UTM
    dist_m:     np.ndarray   # cumulative EKF distance at the matched index
    drift_rate_m_per_100m: float
    n_pairs:    int


def align_to_anchor(ekf: DRTrack, sbl: DRTrack) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Shift EKF track so its first sample sits on the first SBL sample."""
    first_sbl_t = int(sbl.t_ns[0])
    idx = int(np.argmin(np.abs(ekf.t_ns - first_sbl_t)))
    shift_E = sbl.E[0] - ekf.E[idx]
    shift_N = sbl.N_utm[0] - ekf.N_utm[idx]
    return ekf.E + shift_E, ekf.N_utm + shift_N, shift_E, shift_N


def pair_errors(ekf: DRTrack, sbl: DRTrack,
                ekf_E_aligned: np.ndarray, ekf_N_aligned: np.ndarray,
                max_gap_s: float = 1.0) -> ErrorSeries:
    """For each SBL sample, find the nearest EKF sample within max_gap_s."""
    if len(sbl.t_ns) == 0 or len(ekf.t_ns) == 0:
        return ErrorSeries(np.array([]), np.array([]), np.array([]), 0.0, 0)
    max_gap_ns = int(max_gap_s * 1e9)
    t_rel, err, dist = [], [], []
    t0 = int(sbl.t_ns[0])
    for i in range(len(sbl.t_ns)):
        t = int(sbl.t_ns[i])
        j = nearest_idx(ekf.t_ns, t)
        if abs(int(ekf.t_ns[j]) - t) > max_gap_ns:
            continue
        e = math.hypot(ekf_E_aligned[j] - sbl.E[i],
                       ekf_N_aligned[j] - sbl.N_utm[i])
        err.append(e)
        dist.append(float(ekf.dist[j]))
        t_rel.append((t - t0) / 1e9)
    err = np.array(err);  dist = np.array(dist);  t_rel = np.array(t_rel)
    if len(dist) >= 2 and dist.max() > dist.min():
        slope = float(np.polyfit(dist, err, 1)[0])
    else:
        slope = 0.0
    return ErrorSeries(t_rel_s=t_rel, err_m=err, dist_m=dist,
                       drift_rate_m_per_100m=slope * 100.0, n_pairs=len(err))


# ── plots ─────────────────────────────────────────────────────────────────────

def plot_overlay(
    sbl: DRTrack,
    ekf_anchored: tuple[DRTrack, np.ndarray, np.ndarray] | None,
    ekf_local: tuple[DRTrack, np.ndarray, np.ndarray] | None,
    gnss_E: np.ndarray, gnss_N: np.ndarray,
    info_lines: list[str],
    bag_name: str,
    out_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(9, 9))

    epsg = _utm_epsg(sbl.zone_num, sbl.zone_letter)
    on_satellite = add_satellite_basemap(ax, epsg, alpha=1.0, zoom=18)
    if not on_satellite:
        ax.set_facecolor("#dddddd")

    draw_track(ax, sbl.E, sbl.N_utm, "crimson", "SBL (reference)", lw=3.0, zorder=5)
    add_direction_arrows(ax, sbl.E, sbl.N_utm, "crimson", zorder=6, n=12)

    if ekf_anchored is not None:
        _, E, N = ekf_anchored
        draw_track(ax, E, N, "royalblue", "EKF anchored (/gps/filtered/global)",
                   lw=2.6, zorder=4)
        add_direction_arrows(ax, E, N, "royalblue", zorder=5, n=12)

    if ekf_local is not None:
        _, E, N = ekf_local
        ax.plot(E, N, color="deepskyblue", lw=2.0, zorder=3, alpha=0.95,
                linestyle=(0, (5, 3)), label="EKF pure DR (/gps/filtered)",
                solid_capstyle="round")

    if len(gnss_E) > 0:
        ax.scatter(gnss_E, gnss_N, marker="o", s=14, c="limegreen",
                   edgecolors="black", linewidths=0.4, zorder=7,
                   label="GNSS /fix (surfaced, gated)")

    if len(sbl.E) > 0:
        add_start_marker(ax, sbl.E[0], sbl.N_utm[0], zorder=9)

    # Frame around plot extent + small margin
    xs = [sbl.E]
    ys = [sbl.N_utm]
    for trk in (ekf_anchored, ekf_local):
        if trk is not None:
            xs.append(trk[1]); ys.append(trk[2])
    if len(gnss_E) > 0:
        xs.append(gnss_E); ys.append(gnss_N)
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

    fig.suptitle(bag_name, fontsize=10)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path.name}")


def plot_drift(es_anchored: ErrorSeries | None,
               es_local: ErrorSeries | None,
               smoothed: bool,
               bag_name: str,
               out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(9, 5))
    plotted = False
    if es_anchored is not None and es_anchored.n_pairs:
        ax.scatter(es_anchored.dist_m, es_anchored.err_m, s=18, c="royalblue",
                   alpha=0.75, label=f"Anchored EKF — {es_anchored.drift_rate_m_per_100m:.2f} m/100 m")
        if es_anchored.dist_m.max() > es_anchored.dist_m.min():
            x = np.array([es_anchored.dist_m.min(), es_anchored.dist_m.max()])
            slope = es_anchored.drift_rate_m_per_100m / 100.0
            intercept = float(np.mean(es_anchored.err_m) - slope * np.mean(es_anchored.dist_m))
            ax.plot(x, slope * x + intercept, "--", color="navy", lw=1.2)
        plotted = True
    if es_local is not None and es_local.n_pairs:
        ax.scatter(es_local.dist_m, es_local.err_m, s=18, c="deepskyblue",
                   alpha=0.6, label=f"Pure DR — {es_local.drift_rate_m_per_100m:.2f} m/100 m")
        plotted = True

    if not plotted:
        ax.text(0.5, 0.5, "No paired SBL↔EKF samples", ha="center", va="center",
                transform=ax.transAxes)

    ax.set_xlabel("Distance traveled (m)")
    ax.set_ylabel("EKF↔SBL Euclidean error (m)")
    caption = f"{bag_name} — drift vs SBL"
    if smoothed:
        caption += "  (SBL ~1 m noise; median-smoothed)"
    ax.set_title(caption, fontsize=10)
    ax.grid(True, lw=0.3, alpha=0.5)
    ax.set_ylim(bottom=0)
    if plotted:
        ax.legend(fontsize=9, loc="best")

    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path.name}")


def plot_timeseries(
    sbl: DRTrack,
    ekf_anchored: tuple[DRTrack, np.ndarray, np.ndarray] | None,
    es_anchored: ErrorSeries | None,
    acoustic: list,
    bag_name: str,
    out_path: Path,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True,
                             gridspec_kw={"hspace": 0.18})

    t0 = int(sbl.t_ns[0])
    sbl_t = (sbl.t_ns - t0) / 1e9
    sbl_E_rel = sbl.E - sbl.E[0]
    sbl_N_rel = sbl.N_utm - sbl.N_utm[0]

    ax = axes[0]
    ax.plot(sbl_t, sbl_E_rel, color="crimson", lw=1.5, label="SBL E")
    ax.plot(sbl_t, sbl_N_rel, color="darkred",  lw=1.5, ls="--", label="SBL N")
    if ekf_anchored is not None:
        ekf, E_al, N_al = ekf_anchored
        t = (ekf.t_ns - t0) / 1e9
        # Re-zero the aligned EKF on the SBL anchor too:
        ax.plot(t, E_al - sbl.E[0], color="royalblue", lw=1.5, label="EKF E")
        ax.plot(t, N_al - sbl.N_utm[0], color="navy", lw=1.5, ls="--", label="EKF N")
    ax.set_ylabel("Position (m, rel. to first SBL)")
    ax.legend(fontsize=8, loc="upper left", ncol=2)
    ax.grid(True, lw=0.3, alpha=0.5)

    ax = axes[1]
    if es_anchored is not None and es_anchored.n_pairs:
        ax.plot(es_anchored.t_rel_s, es_anchored.err_m,
                color="royalblue", lw=1.4, label="EKF↔SBL error")
        ax.axhline(3.0, color="orange", lw=0.8, ls=":", label="3 m threshold")
        ax.legend(fontsize=8, loc="upper left")
    else:
        ax.text(0.5, 0.5, "No paired samples", ha="center", va="center",
                transform=ax.transAxes)
    ax.set_ylabel("Euclidean error (m)")
    ax.set_ylim(bottom=0)
    ax.grid(True, lw=0.3, alpha=0.5)

    ax = axes[2]
    if acoustic:
        a_t = (np.array([a.t_ns for a in acoustic], dtype=np.int64) - t0) / 1e9
        a_v = np.array([a.std_m for a in acoustic])
        ax.plot(a_t, a_v, color="darkviolet", lw=1.2)
        ax.set_ylabel("SBL acoustic std (m)")
    else:
        ax.text(0.5, 0.5, "/waterlinked_ugps/locator_acoustic_quality not in bag",
                ha="center", va="center", transform=ax.transAxes)
        ax.set_ylabel("SBL acoustic std (m)")
    ax.set_xlabel("Time (s, rel. to first SBL sample)")
    ax.grid(True, lw=0.3, alpha=0.5)

    fig.suptitle(f"{bag_name} — EKF vs SBL time series", fontsize=10)
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path.name}")


# ── metadata ──────────────────────────────────────────────────────────────────

def high_error_intervals(es: ErrorSeries, threshold: float = 3.0) -> list[list[float]]:
    """Return [[t_start, t_end, peak_err], ...] for runs where error > threshold."""
    if es.n_pairs == 0:
        return []
    intervals = []
    in_run = False
    t_start = 0.0
    peak = 0.0
    for t, e in zip(es.t_rel_s, es.err_m):
        if e > threshold:
            if not in_run:
                in_run = True
                t_start = float(t)
                peak = float(e)
            else:
                peak = max(peak, float(e))
        else:
            if in_run:
                intervals.append([t_start, float(t), peak])
                in_run = False
    if in_run:
        intervals.append([t_start, float(es.t_rel_s[-1]), peak])
    return intervals


def save_metadata(out_path: Path,
                  bag_name: str, track_mode: str,
                  n_sbl_raw: int, n_sbl_gated: int,
                  ekf_anchored: DRTrack | None, ekf_local: DRTrack | None,
                  n_gnss: int,
                  sbl: DRTrack | None,
                  acoustic: list,
                  es_anchored: ErrorSeries | None,
                  es_local: ErrorSeries | None) -> None:
    h_std = np.array([m.h_std for m in []])  # filled below from the gated SBL track
    if sbl is not None and len(sbl.lat) > 0:
        # We saved h_std on the SblMsg objects pre-track; recompute from SBL DRTrack proxy
        # by treating it as not-available here. Pull from raw if needed.
        pass

    meta = {
        "bag": bag_name,
        "track_mode": track_mode,
        "n_sbl_raw": int(n_sbl_raw),
        "n_sbl_gated": int(n_sbl_gated),
        "n_ekf_anchored": int(0 if ekf_anchored is None else len(ekf_anchored.t_ns)),
        "n_ekf_local":    int(0 if ekf_local    is None else len(ekf_local.t_ns)),
        "n_gnss_surfaced": int(n_gnss),
        "drift_rate_m_per_100m_anchored":
            float(es_anchored.drift_rate_m_per_100m) if es_anchored is not None else None,
        "drift_rate_m_per_100m_local":
            float(es_local.drift_rate_m_per_100m) if es_local is not None else None,
        "mean_error_m": (
            float(np.mean(es_anchored.err_m)) if es_anchored is not None and es_anchored.n_pairs else None
        ),
        "max_error_m": (
            float(np.max(es_anchored.err_m)) if es_anchored is not None and es_anchored.n_pairs else None
        ),
        "high_error_intervals_s": (
            high_error_intervals(es_anchored) if es_anchored is not None else []
        ),
    }
    if acoustic:
        a_v = np.array([a.std_m for a in acoustic])
        meta["sbl_acoustic_std_p50"] = float(np.percentile(a_v, 50))
        meta["sbl_acoustic_std_p95"] = float(np.percentile(a_v, 95))

    out_path.write_text(json.dumps(meta, indent=2))
    print(f"Saved: {out_path.name}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def _parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path, help="Bag directory containing *.mcap")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output directory (default: bag_dir/ekf_sbl_analysis)")
    ap.add_argument("--track", choices=("anchored", "local", "both"),
                    default="anchored",
                    help="Which EKF track to overlay against SBL (default: anchored)")
    ap.add_argument("--max-sbl-std", type=float, default=5.0,
                    help="Reject SBL fixes with reported h_std > this (m). Default 5.0")
    ap.add_argument("--max-gnss-hacc", type=float, default=2.0,
                    help="Quality gate for the optional surfaced /fix overlay (m)")
    ap.add_argument("--smooth-sbl", type=int, default=3,
                    help="Centred median-filter window on SBL E/N (0 = off, default 3)")
    return ap.parse_args(argv)


def main() -> int:
    args = _parse_args()
    bag_dir: Path = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        return 1

    out_dir: Path = args.output_dir or (bag_dir / "ekf_sbl_analysis")
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix = f"ekf_sbl_{bag_dir.name}"

    want_local = args.track in ("local", "both")
    want_anchored = args.track in ("anchored", "both")

    print(f"{'=' * 60}")
    print(f"EKF vs SBL drift analysis")
    print(f"Bag        : {bag_dir.name}")
    print(f"Track mode : {args.track}")
    print(f"max_sbl_std: {args.max_sbl_std} m")
    print(f"smooth_sbl : {args.smooth_sbl}")
    print(f"Output     : {out_dir}")
    print(f"{'=' * 60}")

    bag = read_bag(bag_dir, want_local=want_local)
    print(f"\nMessages:")
    print(f"  /gps/filtered/global       : {len(bag.anchored)}")
    if want_local:
        print(f"  /gps/filtered              : {len(bag.local)}")
    print(f"  /waterlinked_ugps/navsatfix: {len(bag.sbl)}")
    print(f"  /waterlinked_ugps/locator_acoustic_quality: {len(bag.acoustic)}")
    print(f"  /fix                       : {len(bag.fix)}")
    print(f"  /ubx_nav_hp_pos_llh        : {len(bag.ubx_hp)}")

    # ─ SBL track
    n_sbl_raw = len(bag.sbl)
    sbl_gated_msgs = gate_sbl(bag.sbl, args.max_sbl_std)
    if not sbl_gated_msgs:
        print(f"ERROR: no SBL fixes pass quality gate (max h_std={args.max_sbl_std})",
              file=sys.stderr)
        return 1
    sbl_track = navsatfix_to_track(sbl_gated_msgs, "SBL")
    if sbl_track is None:
        return 1
    if args.smooth_sbl >= 3:
        sbl_track = smooth_track(sbl_track, args.smooth_sbl)
    print(f"\nSBL: gated {len(sbl_gated_msgs)}/{n_sbl_raw}  "
          f"(rejected {n_sbl_raw - len(sbl_gated_msgs)})")

    # ─ EKF tracks
    ekf_anchored_track = ekf_local_track = None
    if want_anchored:
        ekf_anchored_track = navsatfix_to_track(bag.anchored, "/gps/filtered/global")
        if ekf_anchored_track is None and not want_local:
            print("ERROR: no /gps/filtered/global messages and no fallback track requested",
                  file=sys.stderr)
            return 1
    if want_local:
        ekf_local_track = navsatfix_to_track(bag.local, "/gps/filtered")

    # ─ Align (anchor each EKF to first SBL)
    anchored_aligned = local_aligned = None
    es_anchored = es_local = None
    if ekf_anchored_track is not None:
        E, N, _, _ = align_to_anchor(ekf_anchored_track, sbl_track)
        anchored_aligned = (ekf_anchored_track, E, N)
        es_anchored = pair_errors(ekf_anchored_track, sbl_track, E, N)
        print(f"\nAnchored EKF: {es_anchored.n_pairs} paired samples, "
              f"drift {es_anchored.drift_rate_m_per_100m:.2f} m/100 m, "
              f"mean err {np.mean(es_anchored.err_m) if es_anchored.n_pairs else 0.0:.2f} m, "
              f"max err {np.max(es_anchored.err_m) if es_anchored.n_pairs else 0.0:.2f} m")
    if ekf_local_track is not None:
        E, N, _, _ = align_to_anchor(ekf_local_track, sbl_track)
        local_aligned = (ekf_local_track, E, N)
        es_local = pair_errors(ekf_local_track, sbl_track, E, N)
        print(f"Pure DR EKF : {es_local.n_pairs} paired samples, "
              f"drift {es_local.drift_rate_m_per_100m:.2f} m/100 m")

    # ─ Optional surfaced GNSS overlay
    gnss_E = gnss_N = np.array([])
    if bag.fix:
        h_accs = merge_h_acc(bag.fix, bag.ubx_hp)
        gated = gate_fixes(bag.fix, h_accs, args.max_gnss_hacc)
        if gated:
            ge, gn = [], []
            for f, _ in gated:
                e, n, _, _ = utm.from_latlon(f.lat, f.lon)
                ge.append(e); gn.append(n)
            gnss_E = np.array(ge); gnss_N = np.array(gn)
            print(f"GNSS surfaced overlay: {len(gated)}/{len(bag.fix)} fixes pass "
                  f"(max h_acc={args.max_gnss_hacc} m)")

    # ─ Plots
    info_lines = [f"Track mode: {args.track}",
                  f"SBL: {len(sbl_gated_msgs)}/{n_sbl_raw} fixes"]
    if es_anchored is not None and es_anchored.n_pairs:
        info_lines.append(
            f"Anchored drift: {es_anchored.drift_rate_m_per_100m:.2f} m/100m")
        info_lines.append(
            f"Mean err: {np.mean(es_anchored.err_m):.2f} m  "
            f"Max err: {np.max(es_anchored.err_m):.2f} m")
    if es_local is not None and es_local.n_pairs:
        info_lines.append(
            f"DR drift: {es_local.drift_rate_m_per_100m:.2f} m/100m")

    plot_overlay(
        sbl_track,
        anchored_aligned if want_anchored else None,
        local_aligned if want_local else None,
        gnss_E, gnss_N, info_lines,
        bag_dir.name, out_dir / f"{prefix}_overlay.png",
    )
    plot_drift(es_anchored, es_local,
               smoothed=(args.smooth_sbl >= 3),
               bag_name=bag_dir.name,
               out_path=out_dir / f"{prefix}_drift.png")
    plot_timeseries(sbl_track,
                    anchored_aligned if want_anchored else None,
                    es_anchored,
                    bag.acoustic,
                    bag_name=bag_dir.name,
                    out_path=out_dir / f"{prefix}_timeseries.png")
    save_metadata(
        out_dir / f"{prefix}_metadata.json",
        bag_dir.name, args.track,
        n_sbl_raw, len(sbl_gated_msgs),
        ekf_anchored_track, ekf_local_track,
        len(gnss_E), sbl_track, bag.acoustic,
        es_anchored, es_local,
    )

    print(f"\n{'=' * 60}")
    print(f"Done. Output: {out_dir}")
    print(f"{'=' * 60}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
