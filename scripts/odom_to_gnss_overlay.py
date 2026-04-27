"""
Dead-reckoning ground truth evaluation: /gps/filtered vs raw GNSS /fix.

navsat_transform publishes /gps/filtered = local EKF odom position expressed as
GPS lat/lon.  This script compares that dead-reckoning track against quality-gated
raw /fix messages to measure positional drift.

Origin note: /gps/filtered is anchored to navsat_transform's datum (which may be a
fixed YAML datum that differs from the robot's actual GPS position at bag start).
The script corrects this constant offset by aligning both tracks at the first gated
fix, so the drift curve shows only accumulated dead-reckoning error, not datum bias.

Usage:
    python scripts/odom_to_gnss_overlay.py /path/to/bag_dir [--max-h-acc 2.0] [--output-dir ./output]
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
from mcap_ros2.reader import read_ros2_messages
import utm

try:
    import contextily as ctx
    _HAS_CONTEXTILY = True
except ImportError:
    _HAS_CONTEXTILY = False

_COV_UNKNOWN  = 0
_COV_APPROX   = 1
_COV_DIAGONAL = 2
_COV_KNOWN    = 3


# ── dataclasses ───────────────────────────────────────────────────────────────

@dataclass
class FixMsg:
    t_ns:     int
    lat:      float
    lon:      float
    status:   int
    cov:      list
    cov_type: int

@dataclass
class UbxHpMsg:
    t_ns:      int
    h_acc_raw: int

@dataclass
class GpsFilteredMsg:
    t_ns: int
    lat:  float
    lon:  float

@dataclass
class BagData:
    fix_msgs:          list = field(default_factory=list)
    ubx_hp_msgs:       list = field(default_factory=list)
    gps_filtered_msgs: list = field(default_factory=list)


# ── math helpers ──────────────────────────────────────────────────────────────

def _h_acc_from_navsatfix(cov: list, cov_type: int) -> float | None:
    """Largest horizontal 1-sigma from NavSatFix position_covariance (ENU, row-major 3×3)."""
    if cov_type == _COV_UNKNOWN:
        return None
    if cov_type == _COV_DIAGONAL:
        if len(cov) < 5:
            return None
        return math.sqrt(max(0.0, float(cov[0]), float(cov[4])))
    if len(cov) < 9:
        return None
    a, b, d = float(cov[0]), float(cov[1]), float(cov[4])
    tr   = a + d
    det  = a * d - b * b
    disc = max(0.0, tr * tr - 4.0 * det)
    return math.sqrt(max(0.0, 0.5 * (tr + math.sqrt(disc))))


def _h_acc_from_ubx(h_acc_raw: int) -> float | None:
    """Convert raw ublox h_acc (0.1 mm units) to meters."""
    if h_acc_raw == 0xFFFFFFFF:
        return None
    return float(h_acc_raw) * 1e-4


# ── bag reader ────────────────────────────────────────────────────────────────

_TOPIC_FIX      = "/fix"
_TOPIC_UBX      = "/ubx_nav_hp_pos_llh"
_TOPIC_GPS_FILT = "/gps/filtered"


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def read_bag(bag_dir: Path, source_topic: str = _TOPIC_GPS_FILT) -> BagData:
    """Read /fix, /ubx_nav_hp_pos_llh, and the chosen filtered-GPS source from bag.

    ``source_topic`` selects which NavSatFix track is treated as the
    dead-reckoning / filter output to compare against raw /fix:
      - "/gps/filtered"        (default) — local EKF position, via navsat_transform
      - "/gps/filtered/global" — global EKF position, via global_ekf_to_navsatfix
    """
    mcap_path = bag_dir / f"{bag_dir.name}_0.mcap"
    if not mcap_path.exists():
        candidates = list(bag_dir.glob("*_0.mcap"))
        if not candidates:
            print(f"ERROR: no *_0.mcap found in {bag_dir}", file=sys.stderr)
            sys.exit(1)
        mcap_path = candidates[0]

    wanted = {_TOPIC_FIX, _TOPIC_UBX, source_topic}
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
            if topic == _TOPIC_FIX:
                data.fix_msgs.append(FixMsg(
                    t_ns=t,
                    lat=float(ros.latitude),
                    lon=float(ros.longitude),
                    status=int(ros.status.status),
                    cov=list(ros.position_covariance),
                    cov_type=int(ros.position_covariance_type),
                ))
            elif topic == _TOPIC_UBX:
                data.ubx_hp_msgs.append(UbxHpMsg(
                    t_ns=t,
                    h_acc_raw=int(ros.h_acc),
                ))
            elif topic == source_topic:
                data.gps_filtered_msgs.append(GpsFilteredMsg(
                    t_ns=t,
                    lat=float(ros.latitude),
                    lon=float(ros.longitude),
                ))
        except AttributeError:
            continue

    return data


# ── GNSS quality gate ─────────────────────────────────────────────────────────

_UBX_MATCH_NS = 50_000_000


def merge_h_acc(fix_msgs: list, ubx_msgs: list) -> list:
    """For each /fix, return best h_acc: UBX HP (if within 50 ms) else NavSatFix covariance."""
    ubx_t = np.array([m.t_ns for m in ubx_msgs], dtype=np.int64) if ubx_msgs else None
    result = []
    for fix in fix_msgs:
        if ubx_t is not None and len(ubx_t):
            idx = int(np.argmin(np.abs(ubx_t - fix.t_ns)))
            if abs(int(ubx_t[idx]) - fix.t_ns) <= _UBX_MATCH_NS:
                val = _h_acc_from_ubx(ubx_msgs[idx].h_acc_raw)
                if val is not None:
                    result.append(val)
                    continue
        result.append(_h_acc_from_navsatfix(fix.cov, fix.cov_type))
    return result


def gate_fixes(fix_msgs: list, h_accs: list, max_h_acc_m: float) -> list:
    """Return (fix, h_acc) pairs that pass the quality gate."""
    accepted = []
    for fix, h in zip(fix_msgs, h_accs):
        if fix.status < 0:
            continue
        if h is None:
            continue
        if h > max_h_acc_m:
            continue
        accepted.append((fix, h))
    return accepted


# ── dead-reckoning track from /gps/filtered ───────────────────────────────────

@dataclass
class DRTrack:
    """Dead-reckoning track from /gps/filtered, expressed in UTM."""
    t_ns:        np.ndarray
    lat:         np.ndarray
    lon:         np.ndarray
    E:           np.ndarray
    N_utm:       np.ndarray
    dist:        np.ndarray
    zone_num:    int
    zone_letter: str


@dataclass
class DriftResult:
    drift_rate_m_per_m: float
    drift_rate_pct:     float
    total_distance_m:   float
    total_time_s:       float
    n_pairs:            int
    errors_m:           np.ndarray
    distances_m:        np.ndarray


def build_dr_track(bag_data: BagData, source_topic: str = _TOPIC_GPS_FILT) -> DRTrack:
    """Convert filtered-GPS NavSatFix messages to a UTM track."""
    msgs = bag_data.gps_filtered_msgs
    if not msgs:
        print(f"ERROR: no {source_topic} messages in bag.", file=sys.stderr)
        sys.exit(1)

    t_ns = np.array([m.t_ns for m in msgs], dtype=np.int64)
    lats = np.array([m.lat  for m in msgs])
    lons = np.array([m.lon  for m in msgs])

    E_arr = np.empty(len(msgs))
    N_arr = np.empty(len(msgs))
    zone_num, zone_letter = None, None
    for i, m in enumerate(msgs):
        e, n, zn, zl = utm.from_latlon(m.lat, m.lon)
        E_arr[i], N_arr[i] = e, n
        if zone_num is None:
            zone_num, zone_letter = zn, zl

    dE   = np.diff(E_arr, prepend=E_arr[0])
    dN   = np.diff(N_arr, prepend=N_arr[0])
    dist = np.cumsum(np.hypot(dE, dN))

    print(f"\n/gps/filtered track: {len(msgs)} messages, "
          f"total distance {dist[-1]:.2f} m, "
          f"duration {(t_ns[-1] - t_ns[0]) / 1e9:.1f} s")
    print(f"  Start: lat={lats[0]:.7f}  lon={lons[0]:.7f}")
    print(f"  UTM:   E={E_arr[0]:.3f}  N={N_arr[0]:.3f}  zone={zone_num}{zone_letter}")

    return DRTrack(t_ns=t_ns, lat=lats, lon=lons,
                   E=E_arr, N_utm=N_arr, dist=dist,
                   zone_num=zone_num, zone_letter=zone_letter)


# ── heading verification ──────────────────────────────────────────────────────

def verify_heading(dr_track: DRTrack, gated_fixes: list) -> None:
    """Compare /gps/filtered bearing vs /fix bearing over the first 10 s of the track."""
    t0_ns  = int(dr_track.t_ns[0])
    t10_ns = t0_ns + 10_000_000_000

    mask = (dr_track.t_ns >= t0_ns) & (dr_track.t_ns <= t10_ns)
    if mask.sum() < 2:
        print("\nHeading verification: skipped (< 2 /gps/filtered messages in first 10 s)")
        return

    dE_dr = float(dr_track.E[mask][-1]     - dr_track.E[mask][0])
    dN_dr = float(dr_track.N_utm[mask][-1] - dr_track.N_utm[mask][0])
    if math.hypot(dE_dr, dN_dr) < 0.5:
        print("\nHeading verification: skipped (vehicle moved < 0.5 m in first 10 s)")
        return
    bearing_dr = math.degrees(math.atan2(dE_dr, dN_dr))

    gnss_in_window = [(f, h) for f, h in gated_fixes if t0_ns <= f.t_ns <= t10_ns]
    if len(gnss_in_window) < 2:
        print("\nHeading verification: skipped (< 2 GNSS fixes in first 10 s)")
        return

    first_g, last_g = gnss_in_window[0][0], gnss_in_window[-1][0]
    e1, n1, _, _ = utm.from_latlon(first_g.lat, first_g.lon)
    e2, n2, _, _ = utm.from_latlon(last_g.lat,  last_g.lon)
    if math.hypot(e2 - e1, n2 - n1) < 0.5:
        print("\nHeading verification: skipped (GNSS moved < 0.5 m in first 10 s)")
        return
    bearing_gnss = math.degrees(math.atan2(e2 - e1, n2 - n1))

    diff_deg = abs(math.degrees(
        ((math.radians(bearing_dr - bearing_gnss) + math.pi) % (2 * math.pi)) - math.pi
    ))
    print(f"\nHeading verification (first 10 s):")
    print(f"  /gps/filtered bearing: {bearing_dr:.1f}°")
    print(f"  GNSS /fix bearing    : {bearing_gnss:.1f}°")
    print(f"  Difference           : {diff_deg:.1f}°", end="")
    if diff_deg > 5.0:
        print(f"  *** WARNING: bearing mismatch > 5° — check yaw_offset / datum ***")
    else:
        print("  (OK)")


# ── satellite overlay plot ────────────────────────────────────────────────────

def plot_overlay(
    dr_track: DRTrack,
    gated_fixes: list,
    bag_name: str,
    out_dir: Path,
) -> tuple:
    """Satellite overlay: GNSS ground truth (blue) + /gps/filtered dead-reckoning (red).

    The DR track is shifted to align with the first gated fix, removing any constant
    offset between the navsat_transform datum and the robot's actual GPS position.
    """
    fig, ax = plt.subplots(figsize=(10, 10))

    t0 = gated_fixes[0][0].t_ns if gated_fixes else int(dr_track.t_ns[0])
    gnss_E, gnss_N, gnss_h, gnss_t = [], [], [], []
    for fix, h in gated_fixes:
        e, n, _, _ = utm.from_latlon(fix.lat, fix.lon)
        gnss_E.append(e);  gnss_N.append(n)
        gnss_h.append(h);  gnss_t.append((fix.t_ns - t0) / 1e9)
    gnss_E = np.array(gnss_E);  gnss_N = np.array(gnss_N)
    gnss_t = np.array(gnss_t);  gnss_h = np.array(gnss_h)

    dr_t_rel = (dr_track.t_ns - t0) / 1e9
    t_max = max(float(gnss_t.max()) if len(gnss_t) else 0.0, float(dr_t_rel.max()))

    # Shift DR track so it aligns with the first gated fix (removes fixed datum offset).
    if len(gnss_E) > 0:
        first_idx = int(np.argmin(np.abs(dr_track.t_ns - gated_fixes[0][0].t_ns)))
        shift_E = gnss_E[0] - dr_track.E[first_idx]
        shift_N = gnss_N[0] - dr_track.N_utm[first_idx]
    else:
        shift_E, shift_N = 0.0, 0.0
    dr_E_plot = dr_track.E     + shift_E
    dr_N_plot = dr_track.N_utm + shift_N

    sc_dr = ax.scatter(
        dr_E_plot, dr_N_plot,
        c=dr_t_rel, cmap="Reds", s=6, alpha=0.7, zorder=3,
        vmin=0, vmax=t_max, label="/gps/filtered (dead-reckoning, aligned)",
    )
    step = max(1, len(dr_E_plot) // 10)
    for i in range(step, len(dr_E_plot), step):
        dE = dr_E_plot[i] - dr_E_plot[i - 1]
        dN = dr_N_plot[i] - dr_N_plot[i - 1]
        if math.hypot(dE, dN) > 0.05:
            ax.annotate("", xy=(dr_E_plot[i], dr_N_plot[i]),
                        xytext=(dr_E_plot[i - 1], dr_N_plot[i - 1]),
                        arrowprops=dict(arrowstyle="->", color="darkred", lw=1.0))

    if len(gnss_E) > 0:
        ax.scatter(
            gnss_E, gnss_N, c=gnss_t, cmap="Blues", s=20, alpha=0.9,
            zorder=4, vmin=0, vmax=t_max, label="GNSS /fix (gated)",
        )
        for i in range(0, len(gnss_E), max(1, len(gnss_E) // 10)):
            circle = plt.Circle(
                (gnss_E[i], gnss_N[i]), gnss_h[i],
                fill=False, color="steelblue", linewidth=0.8, alpha=0.5, zorder=3,
            )
            ax.add_patch(circle)
        ax.scatter([gnss_E[0]], [gnss_N[0]], marker="*", s=200, c="gold",
                   zorder=6, label="Start (first gated fix)")

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("UTM Easting (m)")
    ax.set_ylabel("UTM Northing (m)")
    ax.set_title(f"{bag_name}\nGNSS fixes used: {len(gated_fixes)}", fontsize=9)
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, lw=0.3, alpha=0.5)
    plt.colorbar(sc_dr, ax=ax, label="Time (s)", fraction=0.03)

    if _HAS_CONTEXTILY:
        try:
            ctx.add_basemap(ax, crs=f"EPSG:326{dr_track.zone_num:02d}", zoom="auto",
                            source=ctx.providers.Esri.WorldImagery, alpha=0.6)
        except Exception as e:
            print(f"WARNING: contextily satellite tiles failed: {e}")

    out_path = out_dir / f"{bag_name}_overlay.png"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"Saved: {out_path.name}")
    return fig, ax, out_path


# ── drift curve ───────────────────────────────────────────────────────────────

def plot_drift(
    dr_track: DRTrack,
    gated_fixes: list,
    bag_name: str,
    out_dir: Path,
) -> tuple:
    """Error-vs-distance drift curve.

    Error = Euclidean distance between each /fix and the nearest /gps/filtered
    point in time.  The initial offset (fixed datum mismatch) is subtracted so
    the curve starts at zero and shows only accumulated dead-reckoning drift.
    """
    if len(gated_fixes) < 5:
        print(f"WARNING: only {len(gated_fixes)} GNSS fixes — skipping drift curve.")
        result = DriftResult(0.0, 0.0,
                             float(dr_track.dist[-1]),
                             float((dr_track.t_ns[-1] - dr_track.t_ns[0]) / 1e9),
                             0, np.array([]), np.array([]))
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.text(0.5, 0.5, "< 5 GNSS fixes\n(drift curve unavailable)",
                ha="center", va="center", transform=ax.transAxes)
        out_path = out_dir / f"{bag_name}_drift.png"
        fig.savefig(out_path, dpi=300, bbox_inches="tight")
        plt.close(fig)
        return fig, ax, result, out_path

    # Same datum-mismatch correction as the overlay: shift DR track so it aligns
    # with the first gated fix in UTM space.  Errors are then true Euclidean
    # distances (always ≥ 0) rather than a subtracted scalar that can go negative.
    first_idx = int(np.argmin(np.abs(dr_track.t_ns - gated_fixes[0][0].t_ns)))
    e0_gnss, n0_gnss, _, _ = utm.from_latlon(gated_fixes[0][0].lat, gated_fixes[0][0].lon)
    shift_E = e0_gnss - dr_track.E[first_idx]
    shift_N = n0_gnss - dr_track.N_utm[first_idx]

    dr_t = dr_track.t_ns
    errors, distances = [], []
    for fix, _ in gated_fixes:
        idx = int(np.argmin(np.abs(dr_t - fix.t_ns)))
        gap_s = abs(int(dr_t[idx]) - fix.t_ns) / 1e9
        if gap_s > 0.5:
            continue
        e_gnss, n_gnss, _, _ = utm.from_latlon(fix.lat, fix.lon)
        err = math.hypot(
            (dr_track.E[idx]     + shift_E) - e_gnss,
            (dr_track.N_utm[idx] + shift_N) - n_gnss,
        )
        errors.append(err)
        distances.append(float(dr_track.dist[idx]))

    errors    = np.array(errors)
    distances = np.array(distances)

    if len(distances) == 0:
        print("WARNING: all GNSS fixes exceeded 0.5 s gap to /gps/filtered — no pairs found.")

    if len(distances) >= 2 and distances.max() > distances.min():
        coeffs = np.polyfit(distances, errors, 1)
        slope, intercept = float(coeffs[0]), float(coeffs[1])
    else:
        slope, intercept = 0.0, 0.0

    result = DriftResult(
        drift_rate_m_per_m=slope,
        drift_rate_pct=slope * 100.0,
        total_distance_m=float(dr_track.dist[-1]),
        total_time_s=float((dr_track.t_ns[-1] - dr_track.t_ns[0]) / 1e9),
        n_pairs=len(errors),
        errors_m=errors,
        distances_m=distances,
    )

    print(f"\nDrift rate: {slope * 100:.2f} m per 100 m traveled  ({slope * 1000:.2f} m per km)")

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.scatter(distances, errors, s=20, c="steelblue", zorder=4, alpha=0.8,
               label="Dead-reckoning error (m)")
    if len(distances) > 0:
        sort_idx = np.argsort(distances)
        ax.plot(distances[sort_idx],
                np.polyval([slope, intercept], distances[sort_idx]),
                "--", color="crimson", lw=1.5,
                label=f"Drift fit: {slope * 100:.2f} m / 100 m")
    ax.set_xlabel("Distance traveled (m)")
    ax.set_ylabel("Position error vs GNSS (m)")
    ax.set_title(f"{bag_name} — Dead-reckoning drift", fontsize=9)
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.3, alpha=0.5)
    ax.axhline(0, color="gray", lw=0.6, ls=":")
    ax.set_ylim(bottom=0)

    out_path = out_dir / f"{bag_name}_drift.png"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"Saved: {out_path.name}")
    return fig, ax, result, out_path


# ── dual-panel figure ─────────────────────────────────────────────────────────

def save_dual_panel(
    overlay_png: Path,
    drift_png: Path,
    bag_name: str,
    out_dir: Path,
) -> Path:
    """Combine overlay (left) and drift curve (right) into one figure using PIL."""
    try:
        from PIL import Image as PilImage
        if not overlay_png.exists() or not drift_png.exists():
            raise FileNotFoundError("one or both panel images missing")
        with PilImage.open(overlay_png) as im:
            img_o = np.array(im)
        with PilImage.open(drift_png) as im:
            img_d = np.array(im)
        fig, axes = plt.subplots(1, 2, figsize=(20, 10),
                                 gridspec_kw={"wspace": 0.05})
        axes[0].imshow(img_o);  axes[0].axis("off")
        axes[1].imshow(img_d);  axes[1].axis("off")
        fig.suptitle(f"{bag_name} — Dead-reckoning evaluation", fontsize=11)
    except (ImportError, FileNotFoundError, OSError):
        fig = plt.figure(figsize=(10, 4))
        fig.text(0.5, 0.5,
                 "Install Pillow for dual-panel figure (pip install pillow)",
                 ha="center", va="center", fontsize=12)
        fig.suptitle(f"{bag_name} — Dead-reckoning evaluation", fontsize=11)

    out_path = out_dir / f"{bag_name}_combined.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path.name}")
    return out_path


# ── JSON metadata ─────────────────────────────────────────────────────────────

def save_metadata(
    dr_track: DRTrack,
    gated_fixes: list,
    total_fixes: int,
    drift: DriftResult,
    bag_name: str,
    out_dir: Path,
) -> Path:
    accepted_h = [h for _, h in gated_fixes]
    meta = {
        "bag": bag_name,
        "dr_track": {
            "start_lat":    float(dr_track.lat[0]),
            "start_lon":    float(dr_track.lon[0]),
            "start_E":      float(dr_track.E[0]),
            "start_N":      float(dr_track.N_utm[0]),
            "zone_num":     dr_track.zone_num,
            "zone_letter":  dr_track.zone_letter,
            "start_t_s":    float(dr_track.t_ns[0]) / 1e9,
            "n_messages":   len(dr_track.t_ns),
        },
        "gnss": {
            "total_fixes":    total_fixes,
            "accepted_fixes": len(gated_fixes),
            "rejected_fixes": total_fixes - len(gated_fixes),
            "h_acc_min_m":  float(min(accepted_h))                    if accepted_h else None,
            "h_acc_mean_m": float(sum(accepted_h) / len(accepted_h))  if accepted_h else None,
            "h_acc_max_m":  float(max(accepted_h))                    if accepted_h else None,
        },
        "drift": {
            "rate_m_per_m":    drift.drift_rate_m_per_m,
            "rate_pct":        drift.drift_rate_pct,
            "rate_m_per_100m": drift.drift_rate_m_per_m * 100.0,
            "n_pairs":         drift.n_pairs,
        },
        "track": {
            "total_distance_m": drift.total_distance_m,
            "total_time_s":     drift.total_time_s,
        },
    }
    out_path = out_dir / f"{bag_name}_odom_gnss_metadata.json"
    out_path.write_text(json.dumps(meta, indent=2))
    print(f"Saved: {out_path.name}")
    return out_path


# ── CLI ───────────────────────────────────────────────────────────────────────

def _parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path, help="Bag directory containing <name>_0.mcap")
    ap.add_argument("--max-h-acc", type=float, default=2.0,
                    help="Max horizontal accuracy (m) for GNSS ground truth gate (default: 2.0)")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output directory for PNGs and JSON (default: bag_dir/odom_gnss_analysis)")
    ap.add_argument("--source-topic", default=_TOPIC_GPS_FILT,
                    choices=[_TOPIC_GPS_FILT, "/gps/filtered/global"],
                    help=("NavSatFix topic to compare against /fix. "
                          "/gps/filtered = local EKF dead-reckoning (default); "
                          "/gps/filtered/global = global EKF (GPS-fused) position."))
    return ap.parse_args(argv)


def main():
    args = _parse_args()
    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)
    out_dir = args.output_dir or (bag_dir / "odom_gnss_analysis")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"{'=' * 60}")
    print(f"Dead-reckoning evaluation  (source: {args.source_topic})")
    print(f"Bag      : {bag_dir.name}")
    print(f"Max h_acc: {args.max_h_acc} m")
    print(f"Output   : {out_dir}")
    print(f"{'=' * 60}")

    bag_data = read_bag(bag_dir, source_topic=args.source_topic)
    print(f"\nMessages: /fix={len(bag_data.fix_msgs)}  "
          f"/ubx_hp={len(bag_data.ubx_hp_msgs)}  "
          f"{args.source_topic}={len(bag_data.gps_filtered_msgs)}")

    if not bag_data.ubx_hp_msgs:
        print("WARNING: /ubx_nav_hp_pos_llh not in bag — using NavSatFix covariance for h_acc.")

    dr_track = build_dr_track(bag_data, source_topic=args.source_topic)

    h_accs = merge_h_acc(bag_data.fix_msgs, bag_data.ubx_hp_msgs)
    gated  = gate_fixes(bag_data.fix_msgs, h_accs, args.max_h_acc)

    total = len(bag_data.fix_msgs)
    if not gated:
        print(f"ERROR: no /fix passes quality gate (max_h_acc={args.max_h_acc} m, "
              f"tried {total} fixes). Try --max-h-acc with a higher value.", file=sys.stderr)
        sys.exit(1)
    accepted_h = [h for _, h in gated]
    print(f"\nGNSS quality gate:")
    print(f"  Total /fix    : {total}")
    print(f"  Accepted      : {len(gated)}  ({100 * len(gated) / total:.0f}%)")
    print(f"  Rejected      : {total - len(gated)}")
    print(f"  h_acc accepted: min={min(accepted_h):.3f} m  "
          f"mean={sum(accepted_h)/len(accepted_h):.3f} m  max={max(accepted_h):.3f} m")

    verify_heading(dr_track, gated)

    _, _, overlay_png = plot_overlay(dr_track, gated, bag_dir.name, out_dir)
    plt.close("all")

    _, _, drift_result, drift_png = plot_drift(dr_track, gated, bag_dir.name, out_dir)
    plt.close("all")

    save_dual_panel(overlay_png, drift_png, bag_dir.name, out_dir)

    save_metadata(dr_track, gated, total, drift_result, bag_dir.name, out_dir)

    print(f"\n{'=' * 60}")
    print(f"SUMMARY")
    print(f"  Total distance    : {drift_result.total_distance_m:.2f} m")
    print(f"  Total time        : {drift_result.total_time_s:.1f} s")
    print(f"  Drift rate        : {drift_result.drift_rate_pct:.2f} m per 100 m")
    print(f"  GNSS fixes used   : {len(gated)} / {len(bag_data.fix_msgs)}")
    print(f"  DR pairs matched  : {drift_result.n_pairs} / {len(gated)}")
    print(f"  Output dir        : {out_dir}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
