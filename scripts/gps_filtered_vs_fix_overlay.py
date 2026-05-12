"""
Anchored EKF vs GNSS overlay with sensor-error decomposition.

Compares /gps/filtered/global (the EKF's map-frame state inverse-projected to
NavSatFix) against /fix (raw u-blox GNSS NavSatFix as ground truth) on a bag.
The anchored EKF (gnss_anchored_pose) is open-loop after the first /fix lock,
so any disagreement after the anchor is genuine dead-reckoning drift.

Outputs four panels per bag and physical-parameter estimates:

  1. Trajectory on satellite basemap (Swisstopo SwissImage / Esri).
  2. |error| vs cumulative path length, with a naive linear fit. Useful to
     see the scale and noisiness of the comparison; do NOT read its slope as
     drift rate (the linear fit on a magnitude is dominated by the constant
     bias vector ≈ 0.5 m from anchor RTK noise + antenna lever-arm).
  3. **Along-track error vs path length**, slope = DVL scale-factor error.
     Forward distance always projects positively, so this works even on
     closed loops with multiple heading changes.
  4. **Cross-track error per heading-leg**, slope per leg = yaw bias.
     Aggregated across legs to give one yaw-bias estimate per bag.

Quality gate
------------
/fix samples are filtered (status >= 0 + covariance known + horizontal sigma <=
--max-h-acc-m) so that the comparison is against a clean RTK ground truth.
Default 1.0 m (RTK Float) — tighten to 0.5 m for RTK-Fixed-only comparisons
on bags where it's available.

Usage
-----
    python scripts/gps_filtered_vs_fix_overlay.py /path/to/bag_dir
        [--max-h-acc-m 0.5] [--no-satellite] [...]

Multiple bag dirs may be passed; each gets its own report block.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Force UTF-8 stdout/stderr so degree signs etc. print cleanly on Windows cp1252.
try:
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    sys.stderr.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
except (AttributeError, ValueError):
    pass

import matplotlib
matplotlib.use("Agg")
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
import numpy as np
import utm
from mcap_ros2.reader import read_ros2_messages


_TOPIC_GLOBAL = "/gps/filtered/global"
_TOPIC_REF = "/fix"
_TOPIC_ODOM_GLOBAL = "/odometry/filtered/global"
_PAIR_MAX_GAP_NS = 200_000_000  # 200 ms — generous gating for stamp pairing
_ODOM_MATCH_MAX_NS = 200_000_000  # 200 ms — yaw lookup tolerance

# NavSatFix.position_covariance_type values (from sensor_msgs/msg/NavSatFix).
_COV_UNKNOWN = 0
_COV_APPROXIMATED = 1
_COV_DIAGONAL_KNOWN = 2
_COV_KNOWN = 3

# Leg detection — group consecutive samples whose heading is within this
# tolerance of the running leg-mean heading. ~5° matches typical AUV waypoint
# tracking precision.
_LEG_HEADING_TOL_RAD = math.radians(8.0)
_LEG_MIN_PAIRS = 6  # minimum paired samples per leg for a yaw-bias slope fit
_LEG_MIN_LENGTH_M = 3.0  # minimum along-leg path length for a meaningful fit
_LEG_MIN_SPEED_MPS = 0.05  # below this, heading is undefined; pause the leg


@dataclass
class FixSample:
    t_ns: int
    lat: float
    lon: float
    status: int
    cov: tuple[float, ...]  # 9 values, row-major ENU
    cov_type: int


@dataclass
class OdomSample:
    """Map-frame Odometry state from /odometry/filtered/global."""
    t_ns: int
    yaw: float  # rad, derived from quaternion (only z-rotation matters for boat)


@dataclass
class GlobalEkfTrack:
    """Global-EKF state samples loaded from an `ekf_offline_diagnostic` CSV.

    The CSV's `x` and `y` columns are the global EKF's map-frame state in
    metres relative to the GPS datum. To plot alongside the bag's NavSatFix-
    based tracks (which we project to UTM), we add the datum's UTM origin
    elsewhere in the pipeline.
    """
    t_ns: np.ndarray
    x_map: np.ndarray  # metres, datum-relative (map frame)
    y_map: np.ndarray  # metres, datum-relative (map frame)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (z-rotation) from a quaternion. ZYX intrinsic Euler convention."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _angular_diff(a: float, b: float) -> float:
    """Smallest signed difference a - b in (-pi, pi]."""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d


def _circular_mean(angles: list[float]) -> float:
    """Mean angle (rad) on the unit circle."""
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


def _horizontal_sigma_m(sample: FixSample) -> float | None:
    """Largest horizontal 1-sigma (m) from NavSatFix ENU position_covariance.

    Returns None if covariance is UNKNOWN or malformed. Uses the largest
    eigenvalue of the East-North 2x2 block (matches selector.py's gate so
    we filter /fix by the SAME quality criterion the live system would
    have used to admit it as a "good" fix).
    """
    t = sample.cov_type
    cov = sample.cov
    if t == _COV_UNKNOWN or len(cov) < 5:
        return None
    if t == _COV_DIAGONAL_KNOWN:
        return math.sqrt(max(0.0, float(cov[0]), float(cov[4])))
    if t in (_COV_KNOWN, _COV_APPROXIMATED):
        a = float(cov[0])
        b = float(cov[1])
        d = float(cov[4])
        trace = a + d
        det = a * d - b * b
        disc = trace * trace - 4.0 * det
        if disc < 0.0:
            disc = 0.0
        lam_max = 0.5 * (trace + math.sqrt(disc))
        return math.sqrt(max(0.0, lam_max))
    return None


def gate_ref_track(track: list[FixSample], max_h_acc_m: float | None
                   ) -> tuple[list[FixSample], dict[str, int]]:
    """Filter /fix samples for ground-truth quality.

    Drops:
      - status < 0 (no fix)
      - cov_type == UNKNOWN (can't quality-check)
      - horizontal sigma > max_h_acc_m (too uncertain to trust as truth)

    If max_h_acc_m is None, only the first two filters apply.
    Returns (kept_samples, counters).
    """
    counters = {"in": len(track), "skip_no_fix": 0,
                "skip_cov_unknown": 0, "skip_h_acc": 0, "out": 0}
    kept: list[FixSample] = []
    for s in track:
        if s.status < 0:
            counters["skip_no_fix"] += 1
            continue
        sigma = _horizontal_sigma_m(s)
        if sigma is None:
            counters["skip_cov_unknown"] += 1
            continue
        if max_h_acc_m is not None and sigma > max_h_acc_m:
            counters["skip_h_acc"] += 1
            continue
        kept.append(s)
    counters["out"] = len(kept)
    return kept, counters


def read_diag_csv(path: Path) -> GlobalEkfTrack:
    """Read state.x, state.y from an ekf_offline_diagnostic CSV.

    The CSV header is:
      t_sim, dt_since_prev_global, x, y, z, yaw, vx_world, vy_world, vz_world, ...

    State.x, state.y are the global EKF's map-frame state in metres
    relative to the GPS datum. We pull just those three columns (t_sim, x, y)
    and skip rows where x/y are missing or non-finite.
    """
    if not path.is_file():
        print(f"WARNING: diag CSV not found at {path} — global-EKF overlay skipped",
              file=sys.stderr)
        return GlobalEkfTrack(t_ns=np.array([], dtype=np.int64),
                              x_map=np.array([]), y_map=np.array([]))
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        header = f.readline().strip().split(",")
        try:
            i_t = header.index("t_sim")
            i_x = header.index("x")
            i_y = header.index("y")
        except ValueError as e:
            print(f"WARNING: diag CSV {path} missing expected columns: {e}",
                  file=sys.stderr)
            return GlobalEkfTrack(t_ns=np.array([], dtype=np.int64),
                                  x_map=np.array([]), y_map=np.array([]))
        for line in f:
            parts = line.rstrip("\n").split(",")
            try:
                t_s = float(parts[i_t])
                x = float(parts[i_x])
                y = float(parts[i_y])
            except (ValueError, IndexError):
                continue
            if not (math.isfinite(t_s) and math.isfinite(x) and math.isfinite(y)):
                continue
            rows.append((t_s, x, y))
    if not rows:
        return GlobalEkfTrack(t_ns=np.array([], dtype=np.int64),
                              x_map=np.array([]), y_map=np.array([]))
    arr = np.asarray(rows, dtype=float)
    return GlobalEkfTrack(
        t_ns=(arr[:, 0] * 1e9).astype(np.int64),
        x_map=arr[:, 1],
        y_map=arr[:, 2],
    )


def read_bag(bag_dir: Path
             ) -> tuple[list[FixSample], list[FixSample], list[OdomSample]]:
    mcap_path = bag_dir / f"{bag_dir.name}_0.mcap"
    if not mcap_path.exists():
        candidates = list(bag_dir.glob("*_0.mcap"))
        if not candidates:
            print(f"ERROR: no *_0.mcap found in {bag_dir}", file=sys.stderr)
            sys.exit(1)
        mcap_path = candidates[0]

    wanted = {_TOPIC_GLOBAL, _TOPIC_REF, _TOPIC_ODOM_GLOBAL}
    global_track: list[FixSample] = []
    ref_track: list[FixSample] = []
    odom_track: list[OdomSample] = []

    for msg in read_ros2_messages(str(mcap_path)):
        topic = msg.channel.topic
        if topic not in wanted:
            continue
        ros = msg.ros_msg
        try:
            t = _stamp_ns(ros.header.stamp)
            if t == 0:
                continue
            if topic == _TOPIC_ODOM_GLOBAL:
                q = ros.pose.pose.orientation
                yaw = _quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
                odom_track.append(OdomSample(t_ns=t, yaw=yaw))
                continue
            sample = FixSample(
                t_ns=t,
                lat=float(ros.latitude),
                lon=float(ros.longitude),
                status=int(ros.status.status),
                cov=tuple(float(x) for x in ros.position_covariance),
                cov_type=int(ros.position_covariance_type),
            )
        except AttributeError:
            continue
        if topic == _TOPIC_GLOBAL:
            global_track.append(sample)
        else:
            ref_track.append(sample)

    global_track.sort(key=lambda s: s.t_ns)
    ref_track.sort(key=lambda s: s.t_ns)
    odom_track.sort(key=lambda s: s.t_ns)
    return global_track, ref_track, odom_track


def odom_arrays(odom: list[OdomSample]) -> tuple[np.ndarray, np.ndarray]:
    if not odom:
        return np.array([], dtype=np.int64), np.array([], dtype=float)
    t = np.array([o.t_ns for o in odom], dtype=np.int64)
    y = np.array([o.yaw for o in odom], dtype=float)
    return t, y


def project_track(samples: list[FixSample],
                  zone_number: int, zone_letter: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project each sample to UTM in the *given* zone, return E, N, t_ns arrays.
    Forcing the zone keeps both tracks in the same projected frame even at
    zone-boundary edge cases."""
    es = np.empty(len(samples), dtype=float)
    ns = np.empty(len(samples), dtype=float)
    ts = np.empty(len(samples), dtype=np.int64)
    for i, s in enumerate(samples):
        e, n, _, _ = utm.from_latlon(s.lat, s.lon,
                                     force_zone_number=zone_number,
                                     force_zone_letter=zone_letter)
        es[i] = e
        ns[i] = n
        ts[i] = s.t_ns
    return es, ns, ts


def cumulative_distance(e: np.ndarray, n: np.ndarray) -> np.ndarray:
    """Cumulative path length along (E, N) points starting at 0."""
    if e.size < 2:
        return np.zeros(e.size)
    seg = np.hypot(np.diff(e), np.diff(n))
    return np.concatenate(([0.0], np.cumsum(seg)))


@dataclass
class PairData:
    """Per-paired-sample arrays. Each index i corresponds to the i-th /fix
    sample that paired successfully against a /gps/filtered/global sample
    within _PAIR_MAX_GAP_NS."""
    t_rel: np.ndarray   # seconds since first paired sample
    dist: np.ndarray    # cumulative /fix path length [m]
    err: np.ndarray     # |global - fix| [m]
    e_along: np.ndarray  # signed along-track error in EKF heading frame [m]
    e_cross: np.ndarray  # signed cross-track error (port-positive) [m]
    yaw: np.ndarray     # EKF yaw at each pair time [rad]; nan when unavailable
    g_e: np.ndarray     # paired /gps/filtered/global UTM E [m]
    g_n: np.ndarray     # paired /gps/filtered/global UTM N [m]
    r_e: np.ndarray     # paired /fix UTM E [m]
    r_n: np.ndarray     # paired /fix UTM N [m]
    aligned: bool = False  # whether first-point alignment has been applied
    bias_e: float = 0.0   # bias vector subtracted out (East), 0 if not aligned
    bias_n: float = 0.0   # bias vector subtracted out (North)


def align_first_point(pd: PairData) -> PairData:
    """Subtract the error vector at the first paired sample from all subsequent
    samples. After alignment:
      - err[0] == 0 by construction.
      - The slope of |err| vs cumulative path length is a clean drift rate
        (no more bias_vec confound, no more closed-loop sign ambiguity).
      - The intercept of the fit is ~0 by construction.
    The trajectory plot still uses original g_e/g_n so the satellite-backdrop
    map is geographically correct; only the per-sample error metrics use the
    aligned values.
    """
    if pd.t_rel.size == 0 or pd.aligned:
        return pd
    bias_e = float(pd.g_e[0] - pd.r_e[0])
    bias_n = float(pd.g_n[0] - pd.r_n[0])

    de_aligned = (pd.g_e - pd.r_e) - bias_e
    dn_aligned = (pd.g_n - pd.r_n) - bias_n
    err = np.hypot(de_aligned, dn_aligned)

    if np.any(np.isfinite(pd.yaw)):
        cy = np.cos(pd.yaw)
        sy = np.sin(pd.yaw)
        e_along = de_aligned * cy + dn_aligned * sy
        e_cross = -de_aligned * sy + dn_aligned * cy
        # Preserve NaN where yaw was unavailable.
        bad = ~np.isfinite(pd.yaw)
        e_along[bad] = np.nan
        e_cross[bad] = np.nan
    else:
        e_along = np.full_like(err, np.nan)
        e_cross = np.full_like(err, np.nan)

    return PairData(
        t_rel=pd.t_rel,
        dist=pd.dist,
        err=err,
        e_along=e_along,
        e_cross=e_cross,
        yaw=pd.yaw,
        g_e=pd.g_e,
        g_n=pd.g_n,
        r_e=pd.r_e,
        r_n=pd.r_n,
        aligned=True,
        bias_e=bias_e,
        bias_n=bias_n,
    )


def pair_errors(global_e: np.ndarray, global_n: np.ndarray, global_t: np.ndarray,
                ref_e: np.ndarray, ref_n: np.ndarray, ref_t: np.ndarray,
                odom_t: np.ndarray, odom_yaw: np.ndarray
                ) -> PairData:
    """For each /fix sample, find the nearest /gps/filtered/global sample
    within _PAIR_MAX_GAP_NS, plus the nearest /odometry/filtered/global yaw
    within _ODOM_MATCH_MAX_NS. Decompose the position error into along- and
    cross-track components in the EKF's heading frame."""
    if global_t.size == 0 or ref_t.size == 0:
        empty = np.array([])
        return PairData(empty, empty, empty, empty, empty, empty,
                        empty, empty, empty, empty)
    cum = cumulative_distance(ref_e, ref_n)
    has_odom = odom_t.size > 0
    out: dict[str, list] = {k: [] for k in
                             ("t_rel", "dist", "err", "e_along", "e_cross",
                              "yaw", "g_e", "g_n", "r_e", "r_n")}
    t0 = int(min(int(global_t[0]), int(ref_t[0])))
    for i in range(ref_t.size):
        ts = int(ref_t[i])
        j = int(np.argmin(np.abs(global_t - ts)))
        if abs(int(global_t[j]) - ts) > _PAIR_MAX_GAP_NS:
            continue
        de = float(global_e[j] - ref_e[i])
        dn = float(global_n[j] - ref_n[i])
        e_mag = math.hypot(de, dn)

        if has_odom:
            k = int(np.argmin(np.abs(odom_t - ts)))
            if abs(int(odom_t[k]) - ts) <= _ODOM_MATCH_MAX_NS:
                yaw_i = float(odom_yaw[k])
                # Convert yaw (z-axis rotation, NED-style for navsat) into a
                # planar unit vector in the same East/North frame as our
                # error vector. robot_localization publishes yaw in ENU, so
                # heading = (cos yaw, sin yaw) (East = 0, North = +pi/2).
                cy = math.cos(yaw_i)
                sy = math.sin(yaw_i)
                e_along = de * cy + dn * sy
                # Cross-track: rotate error by -yaw, take y component.
                # Positive = error to the port side of the boat.
                e_cross = -de * sy + dn * cy
            else:
                yaw_i = float("nan")
                e_along = float("nan")
                e_cross = float("nan")
        else:
            yaw_i = float("nan")
            e_along = float("nan")
            e_cross = float("nan")

        out["t_rel"].append((ts - t0) * 1e-9)
        out["dist"].append(float(cum[i]))
        out["err"].append(e_mag)
        out["e_along"].append(e_along)
        out["e_cross"].append(e_cross)
        out["yaw"].append(yaw_i)
        out["g_e"].append(float(global_e[j]))
        out["g_n"].append(float(global_n[j]))
        out["r_e"].append(float(ref_e[i]))
        out["r_n"].append(float(ref_n[i]))
    return PairData(**{k: np.asarray(v) for k, v in out.items()})


def drift_rate_m_per_100m(dist: np.ndarray, err: np.ndarray
                          ) -> tuple[float, float, float] | None:
    """Linear fit err = slope * dist + intercept. Returns
    (slope_m_per_100m, intercept_m, sigma_residual_m), or None if degenerate."""
    if dist.size < 2 or float(dist.max() - dist.min()) <= 0.0:
        return None
    slope, intercept = np.polyfit(dist, err, 1)
    residuals = err - (slope * dist + intercept)
    sigma_residual = float(np.std(residuals, ddof=2)) if residuals.size > 2 else float("nan")
    return float(slope) * 100.0, float(intercept), sigma_residual


def slope_se_m_per_100m(dist: np.ndarray, sigma_residual: float) -> float | None:
    """Standard error of the slope (m / 100 m units) from linear regression."""
    if dist.size < 3 or not math.isfinite(sigma_residual):
        return None
    var_x = float(np.var(dist, ddof=0))
    if var_x <= 0.0:
        return None
    se_slope_per_m = sigma_residual / math.sqrt(dist.size * var_x)
    return se_slope_per_m * 100.0


# ── Per-leg analysis: split closed-loop trajectory into approximately straight
#    legs to extract yaw bias from cross-track-vs-along-track-distance slopes.

@dataclass
class Leg:
    indices: np.ndarray       # indices into PairData
    mean_yaw: float           # rad
    along_dist: np.ndarray    # signed along-leg distance [m] within this leg
    e_cross_local: np.ndarray  # cross-track error in this leg's heading frame [m]
    slope_yaw_rad_per_m: float | None
    slope_yaw_se_rad_per_m: float | None
    intercept_cross_m: float | None
    sigma_residual_m: float | None
    length_m: float


def _ekf_speed(t_ns: np.ndarray, e: np.ndarray, n: np.ndarray) -> np.ndarray:
    """Instantaneous speed (m/s) from the /gps/filtered/global UTM track,
    same length as inputs. Used only for the leg-detection 'is moving' gate."""
    if t_ns.size < 2:
        return np.zeros(t_ns.size)
    dt = np.diff(t_ns).astype(float) * 1e-9
    de = np.diff(e)
    dn = np.diff(n)
    speed_mid = np.hypot(de, dn) / np.maximum(dt, 1e-9)
    speed = np.empty(t_ns.size)
    speed[0] = speed_mid[0] if speed_mid.size else 0.0
    speed[-1] = speed_mid[-1] if speed_mid.size else 0.0
    if speed_mid.size > 1:
        speed[1:-1] = 0.5 * (speed_mid[:-1] + speed_mid[1:])
    return speed


def identify_legs(pd: PairData,
                  heading_tol_rad: float = _LEG_HEADING_TOL_RAD,
                  min_pairs: int = _LEG_MIN_PAIRS,
                  min_length_m: float = _LEG_MIN_LENGTH_M,
                  min_speed_mps: float = _LEG_MIN_SPEED_MPS) -> list[Leg]:
    """Group consecutive pair samples whose yaw stays within heading_tol_rad
    of the running leg-mean. Skip samples below min_speed_mps (heading is
    undefined when the boat barely moves). Return only legs with at least
    min_pairs samples and min_length_m of along-leg path."""
    if pd.t_rel.size == 0 or not np.any(np.isfinite(pd.yaw)):
        return []
    speed = _ekf_speed(
        np.asarray(pd.t_rel * 1e9, dtype=np.int64),  # back to ns
        pd.g_e, pd.g_n,
    )

    legs_indices: list[list[int]] = []
    current: list[int] = []
    current_yaws: list[float] = []
    for i, yaw_i in enumerate(pd.yaw):
        if not math.isfinite(yaw_i) or speed[i] < min_speed_mps:
            if len(current) >= min_pairs:
                legs_indices.append(current)
            current, current_yaws = [], []
            continue
        if not current:
            current = [i]
            current_yaws = [yaw_i]
            continue
        leg_mean = _circular_mean(current_yaws)
        if abs(_angular_diff(yaw_i, leg_mean)) <= heading_tol_rad:
            current.append(i)
            current_yaws.append(yaw_i)
        else:
            if len(current) >= min_pairs:
                legs_indices.append(current)
            current = [i]
            current_yaws = [yaw_i]
    if len(current) >= min_pairs:
        legs_indices.append(current)

    legs: list[Leg] = []
    for idxs in legs_indices:
        idxs_arr = np.asarray(idxs, dtype=int)
        yaws = pd.yaw[idxs_arr]
        mean_yaw = _circular_mean(list(yaws))
        cy, sy = math.cos(mean_yaw), math.sin(mean_yaw)
        # Project gated /fix positions of this leg onto the leg's heading.
        e_seg = pd.r_e[idxs_arr]
        n_seg = pd.r_n[idxs_arr]
        # Along-leg distance, anchored at first sample of leg.
        e0, n0 = e_seg[0], n_seg[0]
        along = (e_seg - e0) * cy + (n_seg - n0) * sy
        # Cross-track error in this leg's heading frame.
        de_seg = pd.g_e[idxs_arr] - pd.r_e[idxs_arr]
        dn_seg = pd.g_n[idxs_arr] - pd.r_n[idxs_arr]
        e_cross_local = -de_seg * sy + dn_seg * cy
        leg_length = float(along.max() - along.min())

        slope = se = intercept = sigma_res = None
        if (idxs_arr.size >= 3
                and leg_length >= min_length_m
                and float(along.max() - along.min()) > 0.0):
            s, b = np.polyfit(along, e_cross_local, 1)
            r = e_cross_local - (s * along + b)
            sigma_res = float(np.std(r, ddof=2))
            var_x = float(np.var(along, ddof=0))
            slope = float(s)
            intercept = float(b)
            se = sigma_res / math.sqrt(idxs_arr.size * var_x) if var_x > 0 else None

        if leg_length < min_length_m:
            continue  # too short — drop entirely

        legs.append(Leg(
            indices=idxs_arr,
            mean_yaw=mean_yaw,
            along_dist=along,
            e_cross_local=e_cross_local,
            slope_yaw_rad_per_m=slope,
            slope_yaw_se_rad_per_m=se,
            intercept_cross_m=intercept,
            sigma_residual_m=sigma_res,
            length_m=leg_length,
        ))
    return legs


def aggregate_yaw_bias(legs: list[Leg]) -> tuple[float, float, int] | None:
    """Path-length-weighted mean and std of per-leg slope (= yaw bias rad).
    Returns (mean_rad, std_rad, n_legs_with_fit) or None if no usable legs."""
    fits = [(L.slope_yaw_rad_per_m, L.length_m)
            for L in legs if L.slope_yaw_rad_per_m is not None]
    if not fits:
        return None
    slopes = np.array([s for s, _ in fits], dtype=float)
    weights = np.array([w for _, w in fits], dtype=float)
    weights = np.maximum(weights, 1e-6)
    w_sum = float(weights.sum())
    mean = float(np.sum(slopes * weights) / w_sum)
    if slopes.size > 1:
        var = float(np.sum(weights * (slopes - mean) ** 2) / w_sum)
        std = math.sqrt(max(0.0, var))
    else:
        std = float("nan")
    return mean, std, slopes.size


def _utm_epsg(zone_num: int, zone_letter: str) -> str:
    """EPSG code for a UTM zone (32600 + zone for North, 32700 + zone for South)."""
    north = zone_letter.upper() >= "N"
    return f"EPSG:{(32600 if north else 32700) + zone_num}"


_SWISSTOPO_URL = (
    "https://wmts.geo.admin.ch/1.0.0/ch.swisstopo.swissimage-product/default/"
    "current/3857/{z}/{x}/{y}.jpeg"
)


def add_satellite_basemap(ax, epsg: str, alpha: float = 1.0,
                          zoom: int = 19) -> bool:
    """Try Swisstopo SwissImage (best for Swiss bags) then Esri WorldImagery.
    Returns True if a basemap was successfully added."""
    try:
        import contextily as ctx
    except ImportError as exc:
        print(f"  WARN: contextily not available ({exc}); plotting on plain background",
              file=sys.stderr)
        return False

    for label, source in (("Swisstopo SwissImage", _SWISSTOPO_URL),
                          ("Esri WorldImagery",   ctx.providers.Esri.WorldImagery)):
        try:
            ctx.add_basemap(ax, crs=epsg, source=source, alpha=alpha,
                            zoom=zoom, attribution_size=4)
            return True
        except Exception as exc:
            print(f"  WARN: {label} basemap failed ({exc})", file=sys.stderr)
    print("  WARN: all basemap providers failed; plotting on plain background",
          file=sys.stderr)
    return False


def _nice_scale_length(span_m: float) -> float:
    """Round-number scale-bar length, ~1/5 of the visible span."""
    target = span_m / 5.0
    for cand in (0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0):
        if cand >= target:
            return cand
    return 200.0


def add_scale_bar(ax, length_m: float | None = None) -> None:
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    span_x = x1 - x0
    span_y = y1 - y0
    if length_m is None:
        length_m = _nice_scale_length(max(span_x, span_y))
    bx = x1 - 0.06 * span_x - length_m
    by = y0 + 0.06 * span_y
    line, = ax.plot([bx, bx + length_m], [by, by], color="black", lw=2.5, zorder=10,
                    solid_capstyle="butt")
    line.set_path_effects([pe.Stroke(linewidth=4.5, foreground="white"), pe.Normal()])
    txt = ax.text(bx + length_m / 2, by + 0.015 * span_y, f"{length_m:g} m",
                  ha="center", va="bottom", fontsize=9, fontweight="bold",
                  color="black", zorder=10)
    txt.set_path_effects([pe.Stroke(linewidth=2.5, foreground="white"), pe.Normal()])


@dataclass
class GlobalEkfStats:
    """Pair-error stats of the global EKF track vs gated /fix."""
    n_pairs: int
    mean: float
    median: float
    p95: float
    max: float
    err_t_rel: np.ndarray   # seconds since first paired sample
    err: np.ndarray         # |global_ekf − fix| [m]
    e_utm: np.ndarray       # global EKF UTM E [m] (datum_E + state.x)
    n_utm: np.ndarray       # global EKF UTM N [m]


@dataclass
class Analysis:
    """Everything the plot needs to draw and the report has already printed."""
    zone_number: int
    zone_letter: str
    datum_e: float
    datum_n: float
    g_e: np.ndarray
    g_n: np.ndarray
    r_e: np.ndarray
    r_n: np.ndarray
    pd: PairData
    legs: list[Leg]
    fit_total: tuple[float, float, float] | None  # (slope_per_100m, intercept_m, sigma_residual)
    fit_along: tuple[float, float, float] | None  # along-track DVL-scale fit
    yaw_bias_rad: tuple[float, float, int] | None  # (mean_rad, std_rad, n_legs)
    global_ekf: GlobalEkfStats | None  # populated only if --diag-csv was given
    global_ekf_label: str  # display label, e.g. "global EKF"
    # Label hooks consumed by _draw_trajectory_panel and _draw_total_error_panel
    # so that sister scripts (global_ekf_vs_fix_overlay.py) can relabel the
    # primary track without forking the drawing helpers. Defaults preserve the
    # current "anchored" wording on this script.
    primary_source_label: str = "/gps/filtered/global (anchored)"
    primary_source_short: str = "anchored"


def _global_ekf_vs_fix(track: GlobalEkfTrack,
                       datum_e: float, datum_n: float,
                       ref_e: np.ndarray, ref_n: np.ndarray, ref_t: np.ndarray
                       ) -> GlobalEkfStats | None:
    """Project global EKF map-frame XY into the SAME UTM zone as /fix
    (by adding the datum's UTM origin), pair each /fix sample with the
    nearest global-EKF sample within _PAIR_MAX_GAP_NS, return stats."""
    if track.t_ns.size == 0 or ref_t.size == 0:
        return None
    e_utm = datum_e + track.x_map
    n_utm = datum_n + track.y_map
    err = []
    t_rel = []
    t0 = int(min(int(track.t_ns[0]), int(ref_t[0])))
    for i in range(ref_t.size):
        ts = int(ref_t[i])
        j = int(np.argmin(np.abs(track.t_ns - ts)))
        if abs(int(track.t_ns[j]) - ts) > _PAIR_MAX_GAP_NS:
            continue
        d = math.hypot(e_utm[j] - ref_e[i], n_utm[j] - ref_n[i])
        err.append(d)
        t_rel.append((ts - t0) * 1e-9)
    if not err:
        return None
    err_arr = np.asarray(err)
    err_sorted = np.sort(err_arr)
    n = err_sorted.size
    return GlobalEkfStats(
        n_pairs=n,
        mean=float(np.mean(err_arr)),
        median=float(np.median(err_arr)),
        p95=float(err_sorted[int(0.95 * (n - 1))]),
        max=float(err_sorted[-1]),
        err_t_rel=np.asarray(t_rel),
        err=err_arr,
        e_utm=e_utm,
        n_utm=n_utm,
    )


def analyse(global_track: list[FixSample], ref_track: list[FixSample],
            odom_track: list[OdomSample],
            global_ekf_csv: GlobalEkfTrack | None,
            global_ekf_label: str,
            first_point_align: bool = True,
            primary_source_label: str = "/gps/filtered/global (anchored)",
            primary_source_short: str = "anchored") -> Analysis | None:
    if not global_track or not ref_track:
        return None
    datum = next(
        (s for s in global_track
         if s.status >= 0 and not (abs(s.lat) < 0.1 and abs(s.lon) < 0.1)),
        global_track[0],
    )
    _, _, zone_number, zone_letter = utm.from_latlon(datum.lat, datum.lon)
    datum_e, datum_n, _, _ = utm.from_latlon(
        datum.lat, datum.lon,
        force_zone_number=zone_number, force_zone_letter=zone_letter,
    )
    g_e, g_n, g_t = project_track(global_track, zone_number, zone_letter)
    r_e, r_n, r_t = project_track(ref_track, zone_number, zone_letter)
    odom_t, odom_yaw = odom_arrays(odom_track)

    pd = pair_errors(g_e, g_n, g_t, r_e, r_n, r_t, odom_t, odom_yaw)
    if first_point_align and pd.t_rel.size > 0:
        pd = align_first_point(pd)
    legs = identify_legs(pd) if pd.t_rel.size else []
    fit_total = drift_rate_m_per_100m(pd.dist, pd.err) if pd.t_rel.size else None
    # Along-track slope = DVL scale factor (fractional).
    if pd.t_rel.size and np.any(np.isfinite(pd.e_along)):
        ok = np.isfinite(pd.e_along)
        fit_along = drift_rate_m_per_100m(pd.dist[ok], pd.e_along[ok])
    else:
        fit_along = None
    yaw_bias_rad = aggregate_yaw_bias(legs) if legs else None

    global_ekf_stats = (
        _global_ekf_vs_fix(global_ekf_csv, datum_e, datum_n, r_e, r_n, r_t)
        if global_ekf_csv is not None and global_ekf_csv.t_ns.size > 0
        else None
    )

    return Analysis(zone_number=zone_number, zone_letter=zone_letter,
                    datum_e=datum_e, datum_n=datum_n,
                    g_e=g_e, g_n=g_n, r_e=r_e, r_n=r_n,
                    pd=pd, legs=legs,
                    fit_total=fit_total, fit_along=fit_along,
                    yaw_bias_rad=yaw_bias_rad,
                    global_ekf=global_ekf_stats,
                    global_ekf_label=global_ekf_label,
                    primary_source_label=primary_source_label,
                    primary_source_short=primary_source_short)


def report(label: str, global_track: list[FixSample],
           ref_track_raw: list[FixSample], odom_track: list[OdomSample],
           max_h_acc_m: float | None,
           global_ekf_csv: GlobalEkfTrack | None,
           global_ekf_label: str,
           first_point_align: bool = True,
           primary_source_label: str = "/gps/filtered/global (anchored)",
           primary_source_short: str = "anchored"
           ) -> tuple[list[FixSample], Analysis | None]:
    """Print the report and return (gated /fix track, Analysis bundle)."""
    print()
    print("=" * 92)
    print(f"file: {label}")
    print("=" * 92)
    print(f"  /gps/filtered/global samples: {len(global_track)}")
    print(f"  /fix samples (raw):           {len(ref_track_raw)}")
    print(f"  /odometry/filtered/global samples: {len(odom_track)}")
    if global_ekf_csv is not None:
        print(f"  global-EKF CSV samples:       {global_ekf_csv.t_ns.size}")

    ref_track, gate_counts = gate_ref_track(ref_track_raw, max_h_acc_m)
    if max_h_acc_m is not None:
        print(f"  /fix gate: status>=0 AND horizontal sigma <= {max_h_acc_m:.2f} m")
    else:
        print("  /fix gate: status>=0 only (no h_acc threshold)")
    print(f"    in={gate_counts['in']}  out={gate_counts['out']}  "
          f"dropped: no_fix={gate_counts['skip_no_fix']}  "
          f"cov_unknown={gate_counts['skip_cov_unknown']}  "
          f"h_acc_too_loose={gate_counts['skip_h_acc']}")

    if not global_track or not ref_track:
        print("  (one of the tracks is empty after gating — cannot compare)")
        return ref_track, None

    A = analyse(global_track, ref_track, odom_track,
                global_ekf_csv, global_ekf_label,
                first_point_align=first_point_align,
                primary_source_label=primary_source_label,
                primary_source_short=primary_source_short)
    if A is None or A.pd.t_rel.size == 0:
        print("  (no paired samples within the 200 ms gap)")
        return ref_track, A

    pd = A.pd
    print(f"  UTM zone {A.zone_number}{A.zone_letter}  ({pd.t_rel.size} paired samples)")
    if pd.aligned:
        bias_mag = math.hypot(pd.bias_e, pd.bias_n)
        print(f"  FIRST-POINT ALIGNED: subtracted initial error vector "
              f"({pd.bias_e:+.3f}, {pd.bias_n:+.3f}) m, |bias|={bias_mag:.3f} m")
        print("  (slope of |error| vs path is now a defensible drift rate; "
              "intercept ~ 0 by construction)")
    else:
        print("  NOT first-point aligned — slope is sign-uninformative (see notes)")

    err = pd.err
    err_sorted = np.sort(err)
    print()
    align_tag = "first-point aligned" if pd.aligned else "raw"
    print(f"  |error|  /gps/filtered/global  vs  /fix (gated, {align_tag})")
    print(f"    mean   = {float(np.mean(err)):.3f} m")
    print(f"    median = {float(np.median(err)):.3f} m")
    print(f"    p95    = {float(err_sorted[int(0.95 * (err_sorted.size - 1))]):.3f} m")
    print(f"    max    = {float(err_sorted[-1]):.3f} m")

    if A.global_ekf is not None:
        ge = A.global_ekf
        print()
        print(f"  |error|  {A.global_ekf_label}  vs  /fix (gated)  — NO alignment")
        print(f"    n_pairs= {ge.n_pairs}")
        print(f"    mean   = {ge.mean:.3f} m")
        print(f"    median = {ge.median:.3f} m")
        print(f"    p95    = {ge.p95:.3f} m")
        print(f"    max    = {ge.max:.3f} m")
        # Direct comparison line for easy readout.
        print()
        print(f"  /gps/filtered/global vs {A.global_ekf_label}: "
              f"median {float(np.median(err)):.3f} vs {ge.median:.3f} m  "
              f"(Δ = {ge.median - float(np.median(err)):+.3f} m)")
    elif global_ekf_csv is not None:
        # Track was supplied but no pairs landed (empty CSV, mismatched timestamps, etc.).
        print()
        print(f"  {A.global_ekf_label}: no paired samples vs /fix (CSV size {global_ekf_csv.t_ns.size}).")

    # Linear fit of |error| vs cumulative path length.
    # - With first-point alignment: slope IS the drift rate (intercept ~ 0 by
    #   construction, signs are physically meaningful).
    # - Without alignment: slope is the rate of change of |bias_vec + drift_vec|,
    #   which is sign-confounded by closed-loop geometry. Reported only as a
    #   diagnostic in that mode.
    print()
    if A.fit_total is not None:
        slope, intercept, sigma_res = A.fit_total
        se = slope_se_m_per_100m(pd.dist, sigma_res)
        path_len = float(pd.dist.max() - pd.dist.min())
        if pd.aligned:
            print("  Drift rate (slope of |error| vs cumulative /fix path, "
                  "first-point aligned):")
            se_str = f"  (SE {se:.3f})" if se is not None else ""
            print(f"    slope         = {slope:+.3f} m / 100 m{se_str}")
            print(f"    intercept     = {intercept:+.3f} m  "
                  f"(should be ~ 0 — alignment by construction)")
            print(f"    σ_residual    = {sigma_res:.3f} m   path span = {path_len:.2f} m")
            ci_lo = slope - (1.96 * se if se is not None else 0)
            ci_hi = slope + (1.96 * se if se is not None else 0)
            print(f"    95% CI on drift rate: [{ci_lo:+.3f}, {ci_hi:+.3f}] m / 100 m")
        else:
            print("  Naive slope of |error| vs cumulative /fix path  (NOT drift rate):")
            print(f"    slope         = {slope:+.3f} m / 100 m   "
                  f"(SE {se:.3f})" if se is not None else f"    slope         = {slope:+.3f} m / 100 m")
            print(f"    intercept     = {intercept:+.3f} m  "
                  f"(~ static offset: anchor RTK noise + lever-arm)")
            print(f"    σ_residual    = {sigma_res:.3f} m   path span = {path_len:.2f} m")
            print("    (sign-uninformative on closed loops — see along/cross decomposition below)")
    else:
        print("  Linear fit: degenerate (insufficient path).")

    # ── Along-track fit → DVL scale factor.
    print()
    if A.fit_along is not None:
        slope_per_100m, intercept_along, sigma_res_along = A.fit_along
        # slope_per_100m = scale_fraction × 100; convert to %.
        scale_pct = slope_per_100m  # m/100m == % of forward distance
        ok = np.isfinite(pd.e_along)
        se = slope_se_m_per_100m(pd.dist[ok], sigma_res_along)
        print("  Along-track decomposition  →  DVL scale factor:")
        print(f"    slope (e_along vs path)      = {slope_per_100m:+.3f} m / 100 m")
        print(f"    DVL scale-factor estimate    = {scale_pct:+.3f} %  "
              f"(positive ≡ DVL over-reads forward velocity)")
        if se is not None:
            print(f"    slope SE                     = {se:.3f} m / 100 m   "
                  f"(±{se:.2f} % on the scale factor)")
        print(f"    intercept                    = {intercept_along:+.3f} m  "
              f"(static along-track offset; lever-arm + anchor noise)")
        print(f"    σ_residual                   = {sigma_res_along:.3f} m")
    else:
        print("  Along-track decomposition: /odometry/filtered/global yaw not available (no fit).")

    # ── Per-leg cross-track fits → yaw bias.
    print()
    if A.legs:
        print(f"  Cross-track / yaw-bias decomposition  ({len(A.legs)} legs identified):")
        for i, L in enumerate(A.legs):
            heading_deg = math.degrees(L.mean_yaw)
            if L.slope_yaw_rad_per_m is not None:
                deg_per_unit = math.degrees(L.slope_yaw_rad_per_m)
                line = (f"    leg {i+1:2d}: heading={heading_deg:+7.2f}°  "
                        f"len={L.length_m:6.2f} m  n={L.indices.size:3d}  "
                        f"yaw_bias={deg_per_unit:+7.3f}°  (slope={L.slope_yaw_rad_per_m:+.4f} rad/m)")
                if L.slope_yaw_se_rad_per_m is not None:
                    line += f"  ±{math.degrees(L.slope_yaw_se_rad_per_m):.3f}°"
                print(line)
            else:
                print(f"    leg {i+1:2d}: heading={heading_deg:+7.2f}°  "
                      f"len={L.length_m:6.2f} m  n={L.indices.size:3d}  (fit skipped)")
        if A.yaw_bias_rad is not None:
            mean_rad, std_rad, n = A.yaw_bias_rad
            print(f"    aggregated yaw bias (length-weighted, {n} legs): "
                  f"{math.degrees(mean_rad):+.3f}° ± {math.degrees(std_rad):.3f}° (1σ)")
    else:
        print("  Cross-track / yaw-bias decomposition: no legs of sufficient length identified.")

    # /fix horizontal-sigma distribution (so the gate threshold can be tuned).
    sigmas = [_horizontal_sigma_m(s) for s in ref_track]
    sigmas = [s for s in sigmas if s is not None]
    if sigmas:
        sigma_arr = np.asarray(sigmas)
        print()
        print("  /fix horizontal sigma (gated samples only):")
        print(f"    median  = {float(np.median(sigma_arr)):.3f} m")
        print(f"    p95     = {float(np.quantile(sigma_arr, 0.95)):.3f} m")
        print(f"    max     = {float(sigma_arr.max()):.3f} m")

    return ref_track, A


def _draw_trajectory_panel(ax, A: Analysis, ref_track_count: int,
                           satellite: bool = True) -> None:
    """Trajectory on satellite basemap, in absolute UTM. When pd.aligned, the
    anchored /gps/filtered/global track is visually shifted by -bias_vec so it
    starts at the same point as /fix; the satellite basemap stays at the real
    UTM coordinates of /fix (the truth) so map features remain geographically
    correct. The global EKF track (when present via --diag-csv) is NOT shifted
    — alignment for that source is a separate experiment (see plan)."""
    pd = A.pd
    g_e, g_n, r_e, r_n = A.g_e, A.g_n, A.r_e, A.r_n

    # First-point-aligned display shift for the anchored EKF track.
    if pd.aligned:
        g_e_disp = g_e - pd.bias_e
        g_n_disp = g_n - pd.bias_n
        anchored_label = (f"{A.primary_source_label}, "
                          f"first-point aligned: |bias|={math.hypot(pd.bias_e, pd.bias_n):.2f} m removed")
    else:
        g_e_disp = g_e
        g_n_disp = g_n
        anchored_label = A.primary_source_label

    ax.plot(g_e_disp, g_n_disp, color="tab:red", lw=2.4, alpha=0.92,
            label=anchored_label, zorder=5,
            solid_capstyle="round", solid_joinstyle="round")
    ax.plot(r_e, r_n, color="black", lw=1.6, alpha=0.7,
            label=f"/fix (gated, n={ref_track_count})", zorder=6)
    ax.scatter(r_e, r_n, color="black", s=14, alpha=0.85, zorder=7,
               edgecolors="white", linewidths=0.5)
    if A.global_ekf is not None:
        ge = A.global_ekf
        ax.plot(ge.e_utm, ge.n_utm, color="tab:blue", lw=2.0, alpha=0.92,
                label=f"{A.global_ekf_label} (raw, not aligned)", zorder=4,
                solid_capstyle="round", solid_joinstyle="round")
    if g_e_disp.size:
        ax.scatter([g_e_disp[0]], [g_n_disp[0]], marker="*", s=180, c="gold",
                   edgecolors="black", linewidths=0.6, zorder=9,
                   label=f"start ({A.primary_source_short})")

    pad = 5.0
    if A.global_ekf is not None:
        ge = A.global_ekf
        e_lo = float(min(r_e.min(), g_e_disp.min(), ge.e_utm.min())) - pad
        e_hi = float(max(r_e.max(), g_e_disp.max(), ge.e_utm.max())) + pad
        n_lo = float(min(r_n.min(), g_n_disp.min(), ge.n_utm.min())) - pad
        n_hi = float(max(r_n.max(), g_n_disp.max(), ge.n_utm.max())) + pad
    else:
        e_lo = float(min(r_e.min(), g_e_disp.min())) - pad
        e_hi = float(max(r_e.max(), g_e_disp.max())) + pad
        n_lo = float(min(r_n.min(), g_n_disp.min())) - pad
        n_hi = float(max(r_n.max(), g_n_disp.max())) + pad
    span = max(e_hi - e_lo, n_hi - n_lo)
    cx = 0.5 * (e_lo + e_hi)
    cy = 0.5 * (n_lo + n_hi)
    half = 0.5 * span
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal")

    on_satellite = False
    if satellite:
        on_satellite = add_satellite_basemap(
            ax, _utm_epsg(A.zone_number, A.zone_letter)
        )
    if not on_satellite:
        ax.set_facecolor("#dddddd")
        ax.grid(alpha=0.3)

    ax.set_xticks([])
    ax.set_yticks([])
    add_scale_bar(ax)
    align_tag = f"first-point aligned for {A.primary_source_short}" if pd.aligned else "no alignment"
    ax.set_title(f"Trajectories  (UTM zone {A.zone_number}{A.zone_letter}, "
                 f"{align_tag})")
    legobj = ax.legend(loc="upper right", frameon=True, fontsize=9)
    if legobj is not None:
        legobj.get_frame().set_alpha(0.85)


def _draw_total_error_panel(ax, A: Analysis) -> None:
    """|error| vs cumulative path length. With first-point alignment (default),
    the slope is a drift rate; otherwise it's a sign-uninformative diagnostic."""
    pd = A.pd
    ax.scatter(pd.dist, pd.err, color="tab:red", s=14, alpha=0.55,
               edgecolors="white", linewidths=0.3,
               label=f"|error|  (n={pd.err.size})")
    if A.fit_total is not None:
        slope_per_100m, intercept, sigma_res = A.fit_total
        slope_per_m = slope_per_100m / 100.0
        xs = np.linspace(float(pd.dist.min()), float(pd.dist.max()), 64)
        if pd.aligned:
            se = slope_se_m_per_100m(pd.dist, sigma_res)
            se_str = f", SE {se:.3f}" if se is not None else ""
            ci_str = ""
            if se is not None:
                ci_lo = slope_per_100m - 1.96 * se
                ci_hi = slope_per_100m + 1.96 * se
                ci_str = f"\n95% CI [{ci_lo:+.2f}, {ci_hi:+.2f}] m/100m"
            label_text = (f"drift rate (first-point aligned)\n"
                          f"slope = {slope_per_100m:+.3f} m / 100 m{se_str}\n"
                          f"intercept = {intercept:+.2f} m\n"
                          f"σ_residual = {sigma_res:.2f} m"
                          f"{ci_str}")
            line_color = "tab:blue"
        else:
            label_text = (f"linear fit (NOT drift rate)\n"
                          f"slope = {slope_per_100m:+.2f} m / 100 m\n"
                          f"intercept = {intercept:+.2f} m\n"
                          f"σ_residual = {sigma_res:.2f} m")
            line_color = "tab:gray"
        ax.plot(xs, slope_per_m * xs + intercept, color=line_color, lw=2.0,
                label=label_text)
    ax.axhline(float(np.median(pd.err)), color="tab:green", linestyle="--",
               linewidth=0.9, alpha=0.7,
               label=f"median {A.primary_source_short} = {float(np.median(pd.err)):.2f} m")
    if A.global_ekf is not None:
        ge = A.global_ekf
        ax.axhline(ge.median, color="tab:blue", linestyle="--",
                   linewidth=0.9, alpha=0.7,
                   label=f"median {A.global_ekf_label} = {ge.median:.2f} m")
    ax.set_xlabel("Cumulative path length along gated /fix [m]")
    ax.set_ylabel("|estimate − /fix| [m]")
    title = f"{A.primary_source_short.capitalize()} error magnitude vs path"
    if pd.aligned:
        title += "  (first-point aligned → slope IS drift rate)"
    else:
        title += "  (raw, slope is geometry-confused)"
    if A.global_ekf is not None:
        title += "  + global-EKF median"
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.set_ylim(bottom=0.0)
    ax.legend(fontsize=8, loc="best")


def _draw_along_track_panel(ax, A: Analysis) -> None:
    """Along-track error (signed) vs cumulative path length → DVL scale factor."""
    pd = A.pd
    ok = np.isfinite(pd.e_along)
    if ok.any():
        ax.scatter(pd.dist[ok], pd.e_along[ok], color="tab:orange", s=14,
                   alpha=0.6, edgecolors="white", linewidths=0.3,
                   label=f"e_along (n={int(ok.sum())})")
        if A.fit_along is not None:
            slope_per_100m, intercept_a, sigma_res_a = A.fit_along
            xs = np.linspace(float(pd.dist[ok].min()), float(pd.dist[ok].max()), 64)
            ax.plot(xs, (slope_per_100m / 100.0) * xs + intercept_a,
                    color="tab:blue", lw=2.0,
                    label=(f"DVL scale fit\n"
                           f"scale = {slope_per_100m:+.3f} %\n"
                           f"intercept = {intercept_a:+.2f} m\n"
                           f"σ_residual = {sigma_res_a:.2f} m"))
        ax.axhline(0.0, color="black", linewidth=0.6, alpha=0.5)
        ax.set_xlabel("Cumulative path length along gated /fix [m]")
        ax.set_ylabel("e_along  (forward error in EKF heading frame) [m]")
        ax.set_title("Along-track error  →  DVL scale factor")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="best")
    else:
        ax.text(0.5, 0.5,
                "/odometry/filtered/global yaw not available\n"
                "(can't decompose into along/cross)",
                ha="center", va="center", transform=ax.transAxes)
        ax.set_title("Along-track error  →  DVL scale factor")


def _draw_cross_track_panel(ax, A: Analysis) -> None:
    """Cross-track error (signed, per-leg) vs along-leg distance → yaw bias."""
    if A.legs:
        cmap = plt.colormaps.get_cmap("tab10")
        for i, L in enumerate(A.legs):
            color = cmap(i % 10)
            ax.scatter(L.along_dist, L.e_cross_local, s=14,
                       color=color, alpha=0.6, edgecolors="white",
                       linewidths=0.3,
                       label=f"leg {i+1}  hdg {math.degrees(L.mean_yaw):+.0f}°")
            if L.slope_yaw_rad_per_m is not None:
                xs = np.linspace(float(L.along_dist.min()),
                                 float(L.along_dist.max()), 32)
                ys = L.slope_yaw_rad_per_m * xs + (L.intercept_cross_m or 0.0)
                ax.plot(xs, ys, color=color, lw=1.8, alpha=0.9)
        ax.axhline(0.0, color="black", linewidth=0.6, alpha=0.5)
        title = "Cross-track per leg  →  yaw bias"
        if A.yaw_bias_rad is not None:
            mean_rad, std_rad, n = A.yaw_bias_rad
            title += (f"  (length-weighted: "
                      f"{math.degrees(mean_rad):+.2f}° ± {math.degrees(std_rad):.2f}° / "
                      f"{n} legs)")
        ax.set_xlabel("Along-leg distance [m]")
        ax.set_ylabel("e_cross_local  (port-positive in leg frame) [m]")
        ax.set_title(title)
        ax.grid(alpha=0.3)
        handles, _ = ax.get_legend_handles_labels()
        if len(handles) > 6:
            handles = handles[:6] + [plt.Line2D([0], [0], lw=0,
                                                label=f"... +{len(handles)-6} more")]
        ax.legend(handles=handles, fontsize=7, loc="best", ncol=2)
    else:
        ax.text(0.5, 0.5,
                "no constant-heading legs of sufficient length",
                ha="center", va="center", transform=ax.transAxes)
        ax.set_title("Cross-track per leg  →  yaw bias")


# Panel name → (draw function, figsize, kwargs key for ref_track_count).
_PANELS = (
    ("trajectory",  _draw_trajectory_panel,  (8.5, 8.5), True),
    ("total_error", _draw_total_error_panel, (10.0, 6.5), False),
    ("along_track", _draw_along_track_panel, (10.0, 6.5), False),
    ("cross_track", _draw_cross_track_panel, (10.0, 6.5), False),
)


def plot(label: str, ref_track: list[FixSample], A: Analysis,
         output_dir: Path, output_basename: str,
         satellite: bool = True) -> None:
    """Write four separate PNGs (one per panel) into output_dir.

    Files written:
      - {output_basename}_trajectory.png   (satellite + tracks, first-point-aligned anchored if applicable)
      - {output_basename}_total_error.png  (|error| vs path + drift-rate fit)
      - {output_basename}_along_track.png  (e_along + DVL scale fit)
      - {output_basename}_cross_track.png  (per-leg e_cross + yaw bias)

    Each panel includes a suptitle with the bag label. Split panels are easier
    to embed in reports / presentations than a 4-up composite.
    """
    if A is None or A.pd.t_rel.size == 0:
        return

    output_dir.mkdir(parents=True, exist_ok=True)
    for name, draw_fn, figsize, needs_ref_count in _PANELS:
        fig, ax = plt.subplots(figsize=figsize)
        if needs_ref_count:
            draw_fn(ax, A, len(ref_track), satellite=satellite)
        else:
            draw_fn(ax, A)
        fig.suptitle(label, fontsize=10)
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        out_path = output_dir / f"{output_basename}_{name}.png"
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"  wrote plot: {out_path}")


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag_dirs", nargs="+", type=Path,
                   help="One or more rosbag2 directories (each contains a *_0.mcap).")
    p.add_argument("--output-dir", type=Path, default=None,
                   help=("Where to write per-bag PNGs. Default: <bag_dir>/. "
                         "Four PNGs are written per bag with the pattern "
                         "<basename>_{trajectory,total_error,along_track,cross_track}.png. "
                         "With --output-dir, the basename is <bag_name>_gps_vs_fix; "
                         "without, the basename is gps_vs_fix and the files land "
                         "directly in the bag directory."))
    p.add_argument("--no-plot", action="store_true",
                   help="Skip plotting; only print the report.")
    p.add_argument("--max-h-acc-m", type=float, default=0.5,
                   help=("Maximum /fix horizontal sigma (m) admitted as ground "
                         "truth. /fix samples with worse covariance are dropped "
                         "before comparison. Pass --max-h-acc-m -1 to disable "
                         "the threshold (status>=0 + cov-known still apply). "
                         "Default 0.5 m (RTK Float gate); tighten to 0.05 for "
                         "RTK-Fixed-only ground truth on bags where it's "
                         "available. Loosening to 1.0 m admits more samples "
                         "at the cost of a noisier truth reference."))
    p.add_argument("--no-satellite", action="store_true",
                   help=("Skip the contextily satellite basemap (faster; useful "
                         "offline or when tile providers are unreachable)."))
    p.add_argument("--diag-csv", type=Path, default=None,
                   help=("Optional path to an ekf_offline_diagnostic CSV "
                         "(e.g. <bag>/ekf_replay/<run>_diag/diag.csv). When "
                         "given, the global EKF's state.x/state.y are loaded, "
                         "projected to the same UTM frame as the bag's tracks "
                         "(by adding the datum's UTM origin), and overlaid in "
                         "the trajectory panel. Stats vs gated /fix are also "
                         "reported. The script runs anchored-only when this "
                         "is omitted."))
    p.add_argument("--global-ekf-label", type=str, default="global EKF",
                   help="Display label for the --diag-csv overlay. Default 'global EKF'.")
    p.add_argument("--no-first-point-align", dest="first_point_align",
                   action="store_false",
                   help=("Disable first-point alignment. Without alignment, "
                         "the |error| slope is sign-uninformative on closed "
                         "loops (dominated by the constant anchor-noise + "
                         "lever-arm bias vector). With alignment (default), "
                         "the slope is a defensible drift rate and the "
                         "intercept is ~0 by construction."))
    p.set_defaults(first_point_align=True)
    args = p.parse_args(argv)

    max_h_acc = None if args.max_h_acc_m is not None and args.max_h_acc_m < 0 else args.max_h_acc_m

    # Load the diag CSV once (if given) — same data is reused per bag dir.
    global_ekf_csv: GlobalEkfTrack | None = None
    if args.diag_csv is not None:
        global_ekf_csv = read_diag_csv(args.diag_csv)
        print(f"[diag-csv] loaded {global_ekf_csv.t_ns.size} samples from {args.diag_csv}")

    rc = 0
    for bag_dir in args.bag_dirs:
        if not bag_dir.is_dir():
            print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
            rc = 1
            continue
        global_track, ref_track_raw, odom_track = read_bag(bag_dir)
        ref_track, A = report(str(bag_dir), global_track, ref_track_raw,
                              odom_track, max_h_acc,
                              global_ekf_csv, args.global_ekf_label,
                              first_point_align=args.first_point_align)
        if not args.no_plot and A is not None:
            if args.output_dir:
                out_dir = args.output_dir
                out_basename = f"{bag_dir.name}_gps_vs_fix"
            else:
                out_dir = bag_dir
                out_basename = "gps_vs_fix"
            plot(str(bag_dir), ref_track, A, out_dir, out_basename,
                 satellite=not args.no_satellite)
    return rc


if __name__ == "__main__":
    sys.exit(main())
