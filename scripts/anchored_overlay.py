"""
Anchored dead-reckoning overlay: /odometry/filtered/global (from gnss_anchored_pose)
vs raw /fix.

Differences from odom_to_gnss_overlay.py:
- Reads /odometry/filtered/global (nav_msgs/Odometry, map frame) from the new
  gnss_anchored_pose node, instead of /gps/filtered (NavSatFix from
  navsat_transform).
- Converts map-frame XY → UTM via datum_UTM + (x, y), where datum is the first
  valid /fix in the bag (the /fix that the anchored-pose node would have
  selected).
- Does NOT apply a "shift to align at first gated fix" correction. The whole
  point of this evaluation is whether one /fix anchor is enough — if it is, the
  two tracks should already align naturally.
- Drift curve uses raw Euclidean error vs the natural anchor.

Usage:
    python scripts/anchored_overlay.py /path/to/bag_dir [--max-h-acc 2.0] [--output-dir ./output]
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

try:
    import contextily as ctx
    _HAS_CONTEXTILY = True
except ImportError:
    _HAS_CONTEXTILY = False

_COV_UNKNOWN  = 0
_COV_DIAGONAL = 2

_TOPIC_FIX        = "/fix"
_TOPIC_UBX        = "/ubx_nav_hp_pos_llh"
_TOPIC_GLOBAL_ODOM = "/odometry/filtered/global"


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
class GlobalOdomMsg:
    t_ns: int
    x:    float   # in map frame (ENU offset from datum)
    y:    float
    z:    float

@dataclass
class BagData:
    fix_msgs:    list = field(default_factory=list)
    ubx_msgs:    list = field(default_factory=list)
    global_odom: list = field(default_factory=list)


# ── math helpers ──────────────────────────────────────────────────────────────

def _h_acc_from_navsatfix(cov: list, cov_type: int) -> float | None:
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
    if h_acc_raw == 0xFFFFFFFF:
        return None
    return float(h_acc_raw) * 1e-4


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


# ── bag reader ────────────────────────────────────────────────────────────────

def read_bag(bag_dir: Path) -> BagData:
    mcap_path = bag_dir / f"{bag_dir.name}_0.mcap"
    if not mcap_path.exists():
        candidates = list(bag_dir.glob("*_0.mcap"))
        if not candidates:
            print(f"ERROR: no *_0.mcap found in {bag_dir}", file=sys.stderr)
            sys.exit(1)
        mcap_path = candidates[0]

    wanted = {_TOPIC_FIX, _TOPIC_UBX, _TOPIC_GLOBAL_ODOM}
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
                data.ubx_msgs.append(UbxHpMsg(
                    t_ns=t,
                    h_acc_raw=int(ros.h_acc),
                ))
            elif topic == _TOPIC_GLOBAL_ODOM:
                data.global_odom.append(GlobalOdomMsg(
                    t_ns=t,
                    x=float(ros.pose.pose.position.x),
                    y=float(ros.pose.pose.position.y),
                    z=float(ros.pose.pose.position.z),
                ))
        except AttributeError:
            continue

    return data


# ── GNSS quality gate ─────────────────────────────────────────────────────────

_UBX_MATCH_NS = 50_000_000


def merge_h_acc(fix_msgs: list, ubx_msgs: list) -> list:
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


# ── datum recovery ────────────────────────────────────────────────────────────

def find_anchor_fix(fix_msgs: list) -> FixMsg | None:
    """First /fix that gnss_anchored_pose would have accepted as the datum:
    status >= 0, |lat| > 0.1, |lon| > 0.1. Mirrors the node's logic."""
    for f in fix_msgs:
        if f.status < 0:
            continue
        if abs(f.lat) < 0.1 and abs(f.lon) < 0.1:
            continue
        return f
    return None


# ── dead-reckoning track ──────────────────────────────────────────────────────

@dataclass
class DRTrack:
    """Dead-reckoning from /odometry/filtered/global, projected to UTM via the
    datum (first valid /fix). NO alignment shift applied."""
    t_ns:        np.ndarray
    E:           np.ndarray
    N_utm:       np.ndarray
    dist:        np.ndarray
    zone_num:    int
    zone_letter: str
    datum_lat:   float
    datum_lon:   float
    datum_E:     float
    datum_N:     float


def build_dr_track(bag_data: BagData, anchor: FixMsg) -> DRTrack:
    msgs = bag_data.global_odom
    if not msgs:
        print(f"ERROR: no {_TOPIC_GLOBAL_ODOM} messages in bag.", file=sys.stderr)
        sys.exit(1)

    datum_E, datum_N, zone_num, zone_letter = utm.from_latlon(anchor.lat, anchor.lon)

    t_ns = np.array([m.t_ns for m in msgs], dtype=np.int64)
    E = np.array([datum_E + m.x for m in msgs])
    N = np.array([datum_N + m.y for m in msgs])

    dE   = np.diff(E, prepend=E[0])
    dN   = np.diff(N, prepend=N[0])
    dist = np.cumsum(np.hypot(dE, dN))

    print(f"\nAnchored DR track: {len(msgs)} messages, "
          f"distance {dist[-1]:.2f} m, "
          f"duration {(t_ns[-1] - t_ns[0]) / 1e9:.1f} s")
    print(f"  Datum (first valid /fix): lat={anchor.lat:.7f}  lon={anchor.lon:.7f}")
    print(f"  UTM datum:    E={datum_E:.3f}  N={datum_N:.3f}  zone={zone_num}{zone_letter}")
    print(f"  Track start:  E={E[0]:.3f}  N={N[0]:.3f}  "
          f"(map_xy=({msgs[0].x:.3f}, {msgs[0].y:.3f}))")

    return DRTrack(
        t_ns=t_ns, E=E, N_utm=N, dist=dist,
        zone_num=zone_num, zone_letter=zone_letter,
        datum_lat=anchor.lat, datum_lon=anchor.lon,
        datum_E=datum_E, datum_N=datum_N,
    )


# ── satellite overlay (no alignment shift) ────────────────────────────────────

def plot_overlay(
    dr_track: DRTrack,
    gated_fixes: list,
    bag_name: str,
    out_dir: Path,
) -> Path:
    fig, ax = plt.subplots(figsize=(10, 10))

    t0 = gated_fixes[0][0].t_ns if gated_fixes else int(dr_track.t_ns[0])

    gnss_E, gnss_N, gnss_h, gnss_t = [], [], [], []
    for fix, h in gated_fixes:
        e, n, _, _ = utm.from_latlon(fix.lat, fix.lon)
        gnss_E.append(e); gnss_N.append(n)
        gnss_h.append(h); gnss_t.append((fix.t_ns - t0) / 1e9)
    gnss_E = np.array(gnss_E); gnss_N = np.array(gnss_N)
    gnss_t = np.array(gnss_t); gnss_h = np.array(gnss_h)

    dr_t_rel = (dr_track.t_ns - t0) / 1e9
    t_max = max(float(gnss_t.max()) if len(gnss_t) else 0.0, float(dr_t_rel.max()))

    sc_dr = ax.scatter(
        dr_track.E, dr_track.N_utm,
        c=dr_t_rel, cmap="Reds", s=6, alpha=0.7, zorder=3,
        vmin=0, vmax=t_max, label="/odometry/filtered/global (anchored DR)",
    )
    step = max(1, len(dr_track.E) // 10)
    for i in range(step, len(dr_track.E), step):
        dE = dr_track.E[i]    - dr_track.E[i - 1]
        dN = dr_track.N_utm[i] - dr_track.N_utm[i - 1]
        if math.hypot(dE, dN) > 0.05:
            ax.annotate("", xy=(dr_track.E[i], dr_track.N_utm[i]),
                        xytext=(dr_track.E[i - 1], dr_track.N_utm[i - 1]),
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

    # Mark the anchor explicitly so the test is visually obvious.
    ax.scatter([dr_track.datum_E], [dr_track.datum_N], marker="X", s=180,
               c="lime", edgecolors="black", linewidths=1.0, zorder=7,
               label="Anchor /fix (datum origin)")

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("UTM Easting (m)")
    ax.set_ylabel("UTM Northing (m)")
    ax.set_title(f"{bag_name}\nGNSS fixes used: {len(gated_fixes)}  (no alignment shift)",
                 fontsize=9)
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, lw=0.3, alpha=0.5)
    plt.colorbar(sc_dr, ax=ax, label="Time (s)", fraction=0.03)

    if _HAS_CONTEXTILY:
        try:
            ctx.add_basemap(ax, crs=f"EPSG:326{dr_track.zone_num:02d}", zoom="auto",
                            source=ctx.providers.Esri.WorldImagery, alpha=0.6)
        except Exception as e:
            print(f"WARNING: contextily satellite tiles failed: {e}")

    out_path = out_dir / "overlay.png"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path.name}")
    return out_path


# ── drift curve (raw error, no shift) ─────────────────────────────────────────

@dataclass
class DriftResult:
    drift_rate_m_per_m: float
    drift_rate_pct:     float
    total_distance_m:   float
    total_time_s:       float
    n_pairs:            int
    initial_offset_m:   float   # error at t=0 — quality of the anchor itself
    errors_m:           np.ndarray
    distances_m:        np.ndarray


def plot_drift(
    dr_track: DRTrack,
    gated_fixes: list,
    bag_name: str,
    out_dir: Path,
) -> tuple[DriftResult, Path]:
    if len(gated_fixes) < 5:
        print(f"WARNING: only {len(gated_fixes)} GNSS fixes — drift curve trivial.")
        result = DriftResult(0.0, 0.0,
                             float(dr_track.dist[-1]),
                             float((dr_track.t_ns[-1] - dr_track.t_ns[0]) / 1e9),
                             0, 0.0, np.array([]), np.array([]))
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.text(0.5, 0.5, "< 5 GNSS fixes\n(drift curve unavailable)",
                ha="center", va="center", transform=ax.transAxes)
        out_path = out_dir / "drift.png"
        fig.savefig(out_path, dpi=300, bbox_inches="tight")
        plt.close(fig)
        return result, out_path

    dr_t = dr_track.t_ns
    errors, distances = [], []
    for fix, _ in gated_fixes:
        idx = int(np.argmin(np.abs(dr_t - fix.t_ns)))
        gap_s = abs(int(dr_t[idx]) - fix.t_ns) / 1e9
        if gap_s > 0.5:
            continue
        e_gnss, n_gnss, _, _ = utm.from_latlon(fix.lat, fix.lon)
        err = math.hypot(dr_track.E[idx] - e_gnss, dr_track.N_utm[idx] - n_gnss)
        errors.append(err)
        distances.append(float(dr_track.dist[idx]))

    errors    = np.array(errors)
    distances = np.array(distances)

    initial_offset = float(errors[0]) if len(errors) else 0.0

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
        initial_offset_m=initial_offset,
        errors_m=errors,
        distances_m=distances,
    )

    print(f"\nAnchor quality (error at t=0): {initial_offset:.3f} m")
    print(f"Drift rate: {slope * 100:.2f} m per 100 m  ({slope * 1000:.2f} m / km)")

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.scatter(distances, errors, s=20, c="steelblue", zorder=4, alpha=0.8,
               label="Position error vs GNSS (m)")
    if len(distances) > 0:
        sort_idx = np.argsort(distances)
        ax.plot(distances[sort_idx],
                np.polyval([slope, intercept], distances[sort_idx]),
                "--", color="crimson", lw=1.5,
                label=f"Drift fit: {slope * 100:.2f} m / 100 m  "
                      f"(intercept {intercept:.2f} m)")
    ax.axhline(initial_offset, color="green", lw=0.8, ls=":",
               label=f"Anchor offset at t=0: {initial_offset:.2f} m")
    ax.set_xlabel("Distance traveled (m)")
    ax.set_ylabel("Position error vs GNSS (m)")
    ax.set_title(f"{bag_name} — Anchored DR drift  (no alignment shift)", fontsize=9)
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.3, alpha=0.5)
    ax.set_ylim(bottom=0)

    out_path = out_dir / "drift.png"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path.name}")
    return result, out_path


# ── dual-panel ────────────────────────────────────────────────────────────────

def save_dual_panel(overlay_png: Path, drift_png: Path, bag_name: str, out_dir: Path) -> Path:
    try:
        from PIL import Image as PilImage
        with PilImage.open(overlay_png) as im:
            img_o = np.array(im)
        with PilImage.open(drift_png) as im:
            img_d = np.array(im)
        fig, axes = plt.subplots(1, 2, figsize=(20, 10),
                                 gridspec_kw={"wspace": 0.05})
        axes[0].imshow(img_o); axes[0].axis("off")
        axes[1].imshow(img_d); axes[1].axis("off")
        fig.suptitle(f"{bag_name} — Anchored dead-reckoning evaluation", fontsize=11)
    except (ImportError, FileNotFoundError, OSError):
        fig = plt.figure(figsize=(10, 4))
        fig.text(0.5, 0.5, "Install Pillow for dual-panel figure",
                 ha="center", va="center", fontsize=12)

    out_path = out_dir / "combined.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path.name}")
    return out_path


# ── metadata ──────────────────────────────────────────────────────────────────

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
        "datum": {
            "lat":  dr_track.datum_lat,
            "lon":  dr_track.datum_lon,
            "E":    dr_track.datum_E,
            "N":    dr_track.datum_N,
            "zone": f"{dr_track.zone_num}{dr_track.zone_letter}",
        },
        "anchored_dr_track": {
            "n_messages":       len(dr_track.t_ns),
            "total_distance_m": drift.total_distance_m,
            "total_time_s":     drift.total_time_s,
        },
        "gnss": {
            "total_fixes":    total_fixes,
            "accepted_fixes": len(gated_fixes),
            "h_acc_min_m":  float(min(accepted_h))                    if accepted_h else None,
            "h_acc_mean_m": float(sum(accepted_h) / len(accepted_h))  if accepted_h else None,
            "h_acc_max_m":  float(max(accepted_h))                    if accepted_h else None,
        },
        "drift": {
            "rate_m_per_m":     drift.drift_rate_m_per_m,
            "rate_m_per_100m":  drift.drift_rate_m_per_m * 100.0,
            "n_pairs":          drift.n_pairs,
            "initial_offset_m": drift.initial_offset_m,
        },
    }
    out_path = out_dir / "metadata.json"
    out_path.write_text(json.dumps(meta, indent=2))
    print(f"Saved: {out_path.name}")
    return out_path


# ── CLI ───────────────────────────────────────────────────────────────────────

def _parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path, help="Bag directory (anchored_replay output)")
    ap.add_argument("--max-h-acc", type=float, default=2.0,
                    help="Max h_acc (m) for GNSS quality gate (default 2.0)")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output dir (default: bag_dir/anchored_analysis)")
    return ap.parse_args(argv)


def main():
    args = _parse_args()
    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)
    out_dir = args.output_dir or (bag_dir / "anchored_analysis")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"{'=' * 60}")
    print(f"Anchored DR evaluation  (source: {_TOPIC_GLOBAL_ODOM})")
    print(f"Bag      : {bag_dir.name}")
    print(f"Max h_acc: {args.max_h_acc} m")
    print(f"Output   : {out_dir}")
    print(f"{'=' * 60}")

    bag_data = read_bag(bag_dir)
    print(f"\nMessages: /fix={len(bag_data.fix_msgs)}  "
          f"/ubx_hp={len(bag_data.ubx_msgs)}  "
          f"{_TOPIC_GLOBAL_ODOM}={len(bag_data.global_odom)}")

    if not bag_data.fix_msgs:
        print("ERROR: no /fix messages in bag — cannot recover datum.", file=sys.stderr)
        sys.exit(1)
    if not bag_data.global_odom:
        print(f"ERROR: no {_TOPIC_GLOBAL_ODOM} messages — anchored-pose node didn't run "
              "or didn't receive a valid fix.", file=sys.stderr)
        sys.exit(1)

    anchor = find_anchor_fix(bag_data.fix_msgs)
    if anchor is None:
        print("ERROR: no /fix passes the anchor gate (status>=0, |lat|>0.1, |lon|>0.1).",
              file=sys.stderr)
        sys.exit(1)

    dr_track = build_dr_track(bag_data, anchor)

    h_accs = merge_h_acc(bag_data.fix_msgs, bag_data.ubx_msgs)
    gated  = gate_fixes(bag_data.fix_msgs, h_accs, args.max_h_acc)

    total = len(bag_data.fix_msgs)
    if not gated:
        print(f"ERROR: no /fix passes quality gate (max_h_acc={args.max_h_acc} m).",
              file=sys.stderr)
        sys.exit(1)
    accepted_h = [h for _, h in gated]
    print(f"\nGNSS quality gate:")
    print(f"  Total /fix    : {total}")
    print(f"  Accepted      : {len(gated)}  ({100 * len(gated) / total:.0f}%)")
    print(f"  h_acc accepted: min={min(accepted_h):.3f} m  "
          f"mean={sum(accepted_h)/len(accepted_h):.3f} m  max={max(accepted_h):.3f} m")

    overlay_png = plot_overlay(dr_track, gated, bag_dir.name, out_dir)
    drift_result, drift_png = plot_drift(dr_track, gated, bag_dir.name, out_dir)
    save_dual_panel(overlay_png, drift_png, bag_dir.name, out_dir)
    save_metadata(dr_track, gated, total, drift_result, bag_dir.name, out_dir)

    print(f"\n{'=' * 60}")
    print(f"SUMMARY")
    print(f"  Anchor offset (t=0)  : {drift_result.initial_offset_m:.2f} m")
    print(f"  Total distance       : {drift_result.total_distance_m:.2f} m")
    print(f"  Total time           : {drift_result.total_time_s:.1f} s")
    print(f"  Drift rate           : {drift_result.drift_rate_pct:.2f} m / 100 m")
    print(f"  GNSS fixes used      : {len(gated)} / {total}")
    print(f"  DR pairs matched     : {drift_result.n_pairs} / {len(gated)}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
