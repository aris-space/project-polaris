"""
Dead-reckoning ground truth evaluation: odom frame → lat/lon vs raw GNSS /fix.

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
from scipy.spatial.transform import Rotation
from mcap_ros2.reader import read_ros2_messages
import utm

try:
    import contextily as ctx
    _HAS_CONTEXTILY = True
except ImportError:
    _HAS_CONTEXTILY = False

# navsat_transform.yaml constants
_YAW_OFFSET = math.pi / 2.0      # 1.5708 rad
_MAG_DECL   = 0.0623             # rad
_IMU_TF_YAW = math.pi            # base_link ← imu_link static TF yaw

# NavSatFix covariance type constants
_COV_UNKNOWN  = 0
_COV_APPROX   = 1
_COV_DIAGONAL = 2
_COV_KNOWN    = 3


# ── dataclasses ──────────────────────────────────────────────────────────────

@dataclass
class FixMsg:
    t_ns: int
    lat: float
    lon: float
    status: int      # status.status field; ≥0 means fix
    cov: list        # 9-element row-major position_covariance
    cov_type: int    # COVARIANCE_TYPE_* constant

@dataclass
class UbxHpMsg:
    t_ns: int
    h_acc_raw: int   # raw 0.1 mm units from ublox

@dataclass
class ImuMsg:
    t_ns: int
    qx: float
    qy: float
    qz: float
    qw: float

@dataclass
class OdomMsg:
    t_ns: int
    x: float
    y: float
    cov_xx: float    # pose.covariance[0]  (x variance)
    cov_yy: float    # pose.covariance[7]  (y variance)

@dataclass
class BagData:
    fix_msgs:    list = field(default_factory=list)
    ubx_hp_msgs: list = field(default_factory=list)
    imu_msgs:    list = field(default_factory=list)
    odom_msgs:   list = field(default_factory=list)


# ── pure math helpers ────────────────────────────────────────────────────────

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
    tr = a + d
    det = a * d - b * b
    disc = max(0.0, tr * tr - 4.0 * det)
    return math.sqrt(max(0.0, 0.5 * (tr + math.sqrt(disc))))


def _h_acc_from_ubx(h_acc_raw: int) -> float | None:
    """Convert raw ublox h_acc (0.1 mm units) to meters. Returns None for sentinel 0xFFFFFFFF."""
    if h_acc_raw == 0xFFFFFFFF:
        return None
    return float(h_acc_raw) * 1e-4


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rad) from unit quaternion using scipy ZYX Euler convention."""
    return float(Rotation.from_quat([qx, qy, qz, qw]).as_euler("ZYX")[0])


def _wrap_pi(angle: float) -> float:
    """Wrap angle to [-π, π)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _compute_psi(theta_imu_rad: float) -> float:
    """Compute fixed rotation angle ψ (rad) from raw IMU yaw.

    Chain: θ_imu → +π (base_link←imu_link TF) → +yaw_offset → +mag_decl
    """
    theta_base = _wrap_pi(theta_imu_rad + _IMU_TF_YAW)
    return _wrap_pi(theta_base + _YAW_OFFSET + _MAG_DECL)


def _odom_to_utm(x: float, y: float, E0: float, N0: float, psi: float) -> tuple[float, float]:
    """Apply fixed rotation matrix T: odom (x, y) → UTM (E, N)."""
    E = E0 + math.cos(psi) * x - math.sin(psi) * y
    N = N0 + math.sin(psi) * x + math.cos(psi) * y
    return E, N


# ── bag reader ───────────────────────────────────────────────────────────────

_TOPIC_FIX  = "/fix"
_TOPIC_UBX  = "/ubx_nav_hp_pos_llh"
_TOPIC_IMU  = "/imu/data"
_TOPIC_ODOM = "/odometry/filtered/local"


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def read_bag(bag_dir: Path) -> BagData:
    """Read all relevant topics from the bag in a single pass."""
    mcap_path = bag_dir / f"{bag_dir.name}_0.mcap"
    if not mcap_path.exists():
        candidates = list(bag_dir.glob("*_0.mcap"))
        if not candidates:
            print(f"ERROR: no *_0.mcap found in {bag_dir}", file=sys.stderr)
            sys.exit(1)
        mcap_path = candidates[0]

    wanted = {_TOPIC_FIX, _TOPIC_UBX, _TOPIC_IMU, _TOPIC_ODOM}
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

            elif topic == _TOPIC_IMU:
                q = ros.orientation
                data.imu_msgs.append(ImuMsg(
                    t_ns=t,
                    qx=float(q.x), qy=float(q.y), qz=float(q.z), qw=float(q.w),
                ))

            elif topic == _TOPIC_ODOM:
                p = ros.pose.pose.position
                cov = ros.pose.covariance  # 36-element flat array
                data.odom_msgs.append(OdomMsg(
                    t_ns=t,
                    x=float(p.x),
                    y=float(p.y),
                    cov_xx=float(cov[0]),
                    cov_yy=float(cov[7]),
                ))
        except AttributeError:
            continue

    return data


# ── GNSS quality gate ────────────────────────────────────────────────────────

_UBX_MATCH_NS = 50_000_000  # 50 ms


def merge_h_acc(
    fix_msgs: list,
    ubx_msgs: list,
) -> list:
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


def gate_fixes(
    fix_msgs: list,
    h_accs: list,
    max_h_acc_m: float,
) -> list:
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


# ── datum and ψ ──────────────────────────────────────────────────────────────

@dataclass
class Datum:
    E0: float
    N0: float
    zone_num: int
    zone_letter: str
    t_ns: int
    psi: float
    theta_imu: float
    theta_base: float


def compute_datum(
    bag_data: BagData,
    max_h_acc_m: float,
) -> Datum:
    """Find first quality-gated fix, compute UTM datum and ψ."""
    if not bag_data.fix_msgs:
        print("ERROR: no /fix messages in bag.", file=sys.stderr)
        sys.exit(1)
    if not bag_data.odom_msgs:
        print("ERROR: no /odometry/filtered/local messages in bag.", file=sys.stderr)
        sys.exit(1)

    h_accs = merge_h_acc(bag_data.fix_msgs, bag_data.ubx_hp_msgs)
    if not bag_data.ubx_hp_msgs:
        print("WARNING: /ubx_nav_hp_pos_llh not in bag — using NavSatFix covariance for h_acc.")

    gated = gate_fixes(bag_data.fix_msgs, h_accs, max_h_acc_m)
    if not gated:
        total = len(bag_data.fix_msgs)
        print(f"ERROR: no /fix passes quality gate (max_h_acc={max_h_acc_m} m, "
              f"tried {total} fixes). Try --max-h-acc with a higher value.", file=sys.stderr)
        sys.exit(1)

    total = len(bag_data.fix_msgs)
    accepted_h = [h for _, h in gated]
    print(f"\nGNSS quality gate:")
    print(f"  Total /fix        : {total}")
    print(f"  Accepted          : {len(gated)}  ({100*len(gated)/total:.0f}%)")
    print(f"  Rejected          : {total - len(gated)}")
    print(f"  h_acc accepted    : min={min(accepted_h):.3f} m  "
          f"mean={sum(accepted_h)/len(accepted_h):.3f} m  max={max(accepted_h):.3f} m")

    first_fix, _ = gated[0]
    e0, n0, zone_num, zone_letter = utm.from_latlon(first_fix.lat, first_fix.lon)

    if not bag_data.imu_msgs:
        print("ERROR: no /imu/data messages in bag.", file=sys.stderr)
        sys.exit(1)
    imu_t = np.array([m.t_ns for m in bag_data.imu_msgs], dtype=np.int64)
    idx = int(np.argmin(np.abs(imu_t - first_fix.t_ns)))
    imu_gap_s = abs(imu_t[idx] - first_fix.t_ns) / 1e9
    if imu_gap_s > 0.5:
        print(f"ERROR: nearest /imu/data is {imu_gap_s:.2f} s away from first fix "
              f"(limit: 0.5 s).", file=sys.stderr)
        sys.exit(1)

    imu = bag_data.imu_msgs[idx]
    theta_imu = _quat_to_yaw(imu.qx, imu.qy, imu.qz, imu.qw)
    theta_base = _wrap_pi(theta_imu + _IMU_TF_YAW)
    psi = _compute_psi(theta_imu)

    # Sanity check: was heading still changing at datum time?
    t_lo = first_fix.t_ns - 1_000_000_000
    t_hi = first_fix.t_ns + 1_000_000_000
    yaws_before = [_quat_to_yaw(m.qx, m.qy, m.qz, m.qw)
                   for m in bag_data.imu_msgs if t_lo <= m.t_ns <= first_fix.t_ns]
    yaws_after  = [_quat_to_yaw(m.qx, m.qy, m.qz, m.qw)
                   for m in bag_data.imu_msgs if first_fix.t_ns <= m.t_ns <= t_hi]
    if yaws_before and yaws_after:
        all_yaws = yaws_before + yaws_after
        unwrapped = np.unwrap(all_yaws)
        delta_deg = abs(math.degrees(unwrapped[-1] - unwrapped[0]))
        if delta_deg > 2.0:
            print(f"WARNING: IMU heading changed {delta_deg:.1f}° in ±1 s around datum fix. "
                  "Xsens NorthReference filter may not have converged.")

    print(f"\nDatum:")
    print(f"  First qualified fix  : {first_fix.t_ns / 1e9:.3f} s  "
          f"lat={first_fix.lat:.7f}  lon={first_fix.lon:.7f}")
    print(f"  UTM E0={e0:.3f}  N0={n0:.3f}  zone={zone_num}{zone_letter}")
    print(f"  theta_imu  = {math.degrees(theta_imu):.2f} deg  "
          f"theta_base = {math.degrees(theta_base):.2f} deg  psi = {math.degrees(psi):.2f} deg")

    return Datum(E0=e0, N0=n0, zone_num=zone_num, zone_letter=zone_letter,
                 t_ns=first_fix.t_ns, psi=psi,
                 theta_imu=theta_imu, theta_base=theta_base)


# ── odom → lat/lon conversion ─────────────────────────────────────────────

@dataclass
class OdomTrack:
    t_ns:    np.ndarray   # (N,) int64
    lat:     np.ndarray   # (N,)
    lon:     np.ndarray   # (N,)
    E:       np.ndarray   # (N,) UTM easting
    N_utm:   np.ndarray   # (N,) UTM northing
    dist:    np.ndarray   # (N,) cumulative distance traveled (m)
    sigma_p: np.ndarray   # (N,) sqrt(P_xx + P_yy) EKF self-reported uncertainty


def convert_odom(bag_data: BagData, datum: Datum) -> OdomTrack:
    """Convert /odometry/filtered/local messages to lat/lon using fixed T matrix."""
    msgs = bag_data.odom_msgs
    if not msgs:
        print("ERROR: no /odometry/filtered/local in bag.", file=sys.stderr)
        sys.exit(1)

    t_ns = np.array([m.t_ns for m in msgs], dtype=np.int64)
    xs   = np.array([m.x    for m in msgs])
    ys   = np.array([m.y    for m in msgs])
    cxx  = np.array([m.cov_xx for m in msgs])
    cyy  = np.array([m.cov_yy for m in msgs])

    E_arr = datum.E0 + np.cos(datum.psi) * xs - np.sin(datum.psi) * ys
    N_arr = datum.N0 + np.sin(datum.psi) * xs + np.cos(datum.psi) * ys

    lat_arr = np.empty(len(msgs))
    lon_arr = np.empty(len(msgs))
    for i in range(len(msgs)):
        lat_arr[i], lon_arr[i] = utm.to_latlon(
            E_arr[i], N_arr[i], datum.zone_num, datum.zone_letter
        )

    dE = np.diff(E_arr, prepend=E_arr[0])
    dN = np.diff(N_arr, prepend=N_arr[0])
    dist = np.cumsum(np.hypot(dE, dN))
    sigma_p = np.sqrt(np.maximum(0.0, cxx + cyy))

    print(f"\nOdom track: {len(msgs)} messages, "
          f"total distance {dist[-1]:.2f} m, "
          f"duration {(t_ns[-1] - t_ns[0]) / 1e9:.1f} s")
    return OdomTrack(t_ns=t_ns, lat=lat_arr, lon=lon_arr,
                     E=E_arr, N_utm=N_arr, dist=dist, sigma_p=sigma_p)


# ── heading verification ───────────────────────────────────────────────────

def verify_heading(
    odom_track: OdomTrack,
    gated_fixes: list,
    datum: Datum,
) -> None:
    """Compare odom displacement bearing vs GNSS bearing over first 10 s."""
    t0_ns = datum.t_ns
    t10_ns = t0_ns + 10_000_000_000

    mask_o = (odom_track.t_ns >= t0_ns) & (odom_track.t_ns <= t10_ns)
    if mask_o.sum() < 2:
        print("\nHeading verification: skipped (< 2 odom messages in first 10 s)")
        return

    dE_o = odom_track.E[mask_o][-1] - odom_track.E[mask_o][0]
    dN_o = odom_track.N_utm[mask_o][-1] - odom_track.N_utm[mask_o][0]
    if math.hypot(dE_o, dN_o) < 0.5:
        print("\nHeading verification: skipped (vehicle moved < 0.5 m in first 10 s)")
        return
    bearing_odom = math.degrees(math.atan2(dE_o, dN_o))

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

    diff_deg = abs(math.degrees(_wrap_pi(math.radians(bearing_odom - bearing_gnss))))
    print(f"\nHeading verification (first 10 s):")
    print(f"  Odom bearing : {bearing_odom:.1f}°")
    print(f"  GNSS bearing : {bearing_gnss:.1f}°")
    print(f"  Difference   : {diff_deg:.1f}°", end="")
    if diff_deg > 5.0:
        print(f"  *** WARNING: bearing mismatch > 5° — ψ may be wrong ***")
    else:
        print("  (OK)")


# ── satellite overlay plot ─────────────────────────────────────────────────

def plot_overlay(
    odom_track: OdomTrack,
    gated_fixes: list,
    datum: Datum,
    bag_name: str,
    out_dir: Path,
) -> tuple:
    """Satellite overlay: GNSS ground truth (blue) + odom converted (red)."""
    fig, ax = plt.subplots(figsize=(10, 10))

    t0 = gated_fixes[0][0].t_ns if gated_fixes else datum.t_ns
    gnss_E, gnss_N, gnss_h, gnss_t = [], [], [], []
    for fix, h in gated_fixes:
        e, n, _, _ = utm.from_latlon(fix.lat, fix.lon)
        gnss_E.append(e)
        gnss_N.append(n)
        gnss_h.append(h)
        gnss_t.append((fix.t_ns - t0) / 1e9)
    gnss_E = np.array(gnss_E)
    gnss_N = np.array(gnss_N)
    gnss_t = np.array(gnss_t)
    gnss_h = np.array(gnss_h)

    odom_t_rel = (odom_track.t_ns - t0) / 1e9
    t_max = max(float(gnss_t.max()) if len(gnss_t) else 0.0, float(odom_t_rel.max()))

    sc_odom = ax.scatter(
        odom_track.E, odom_track.N_utm,
        c=odom_t_rel, cmap="Reds", s=6, alpha=0.7, zorder=3,
        vmin=0, vmax=t_max, label="Odom (converted)",
    )
    step = max(1, len(odom_track.E) // 10)
    for i in range(step, len(odom_track.E), step):
        dE = odom_track.E[i] - odom_track.E[i - 1]
        dN = odom_track.N_utm[i] - odom_track.N_utm[i - 1]
        if math.hypot(dE, dN) > 0.05:
            ax.annotate("", xy=(odom_track.E[i], odom_track.N_utm[i]),
                        xytext=(odom_track.E[i - 1], odom_track.N_utm[i - 1]),
                        arrowprops=dict(arrowstyle="->", color="darkred", lw=1.0))

    ax.scatter(
        gnss_E, gnss_N, c=gnss_t, cmap="Blues", s=20, alpha=0.9,
        zorder=4, vmin=0, vmax=t_max, label="GNSS /fix (gated)",
    )
    for i in range(0, len(gnss_E), max(1, len(gnss_E) // 20)):
        circle = plt.Circle(
            (gnss_E[i], gnss_N[i]), gnss_h[i],
            fill=False, color="steelblue", linewidth=0.8, alpha=0.5, zorder=3,
        )
        ax.add_patch(circle)

    ax.scatter([datum.E0], [datum.N0], marker="*", s=200, c="gold",
               zorder=6, label=f"Datum (psi={math.degrees(datum.psi):.1f} deg)")

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("UTM Easting (m)")
    ax.set_ylabel("UTM Northing (m)")
    ax.set_title(
        f"{bag_name}\npsi={math.degrees(datum.psi):.1f} deg  "
        f"GNSS fixes used: {len(gated_fixes)}",
        fontsize=9,
    )
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, lw=0.3, alpha=0.5)
    plt.colorbar(sc_odom, ax=ax, label="Time (s)", fraction=0.03)

    if _HAS_CONTEXTILY:
        try:
            ctx.add_basemap(ax, crs=f"EPSG:326{datum.zone_num:02d}", zoom="auto",
                            source=ctx.providers.OpenStreetMap.Mapnik, alpha=0.6)
        except Exception as e:
            print(f"WARNING: contextily satellite tiles failed: {e}")

    out_path = out_dir / f"{bag_name}_overlay.png"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"Saved: {out_path.name}")
    return fig, ax, out_path


def _parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path, help="Bag directory containing <name>_0.mcap")
    ap.add_argument("--max-h-acc", type=float, default=2.0,
                    help="Max horizontal accuracy (m) for GNSS ground truth gate (default: 2.0)")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output directory for PNGs and JSON (default: bag_dir/odom_gnss_analysis)")
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
    print(f"Bag      : {bag_dir.name}")
    print(f"Max h_acc: {args.max_h_acc} m")
    print(f"Output   : {out_dir}")
    print(f"{'=' * 60}")

    bag_data = read_bag(bag_dir)
    print(f"\nMessages: /fix={len(bag_data.fix_msgs)}  "
          f"/ubx_hp={len(bag_data.ubx_hp_msgs)}  "
          f"/imu={len(bag_data.imu_msgs)}  "
          f"/odom={len(bag_data.odom_msgs)}")
    if not bag_data.odom_msgs:
        print("ERROR: /odometry/filtered/local not found in bag.", file=sys.stderr)
        sys.exit(1)

    datum     = compute_datum(bag_data, args.max_h_acc)
    odom      = convert_odom(bag_data, datum)
    h_accs    = merge_h_acc(bag_data.fix_msgs, bag_data.ubx_hp_msgs)
    gated     = gate_fixes(bag_data.fix_msgs, h_accs, args.max_h_acc)
    verify_heading(odom, gated, datum)
    overlay_fig, _, _ = plot_overlay(odom, gated, datum, bag_dir.name, out_dir)


if __name__ == "__main__":
    main()
