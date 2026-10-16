"""
Offline global-EKF vs /fix overlay.

Sister script to gps_filtered_vs_fix_overlay.py. The existing script compares
the **anchored** EKF (gnss_anchored_pose) output against /fix; this one
compares the **offline global EKF** output against /fix. Same gating,
first-point alignment, per-axis decomposition, and 4-panel plot — only the
data source for the primary track changes.

Inputs
------
Reads from TWO mcap files:

  1. <source_bag_dir>/<source_bag_name>_0.mcap   — the original recording
     bag. Provides:
       - /fix (sensor_msgs/NavSatFix)  → RTK ground truth.

  2. <replay_output_dir>/replay_outputs_mcap_0.mcap  — the offline-replay
     output mcap. Provides:
       - /gps/filtered/global   (sensor_msgs/NavSatFix, global EKF
         back-projected via global_ekf_to_navsatfix_node) — preferred.
       - /odometry/filtered/global  (nav_msgs/Odometry, map-frame XY +
         orientation) — used when the NavSatFix topic is missing from the
         replay mcap, projected to UTM via the recorded datum.
       - /gps/validated  (NavSatFix) — datum source, the gated GNSS fix
         that gnss_datum_watchdog used to anchor navsat_transform.
         Falls back to /gps/filtered if /gps/validated is empty.

The script never reads /gps/filtered/global_anchored or
/odometry/filtered/global_anchored — those belong to the anchored shadow
and are the existing script's job.

Outputs
-------
Four PNGs per bag (mirroring gps_filtered_vs_fix_overlay.py's split):

  - global_ekf_vs_fix_trajectory.png
  - global_ekf_vs_fix_total_error.png
  - global_ekf_vs_fix_along_track.png
  - global_ekf_vs_fix_cross_track.png

Default location: <replay_output_dir>/<basename>_<panel>.png.

First-point alignment is OFF by default for this script (unlike the
anchored script, where it's on). The global EKF's initial offset is real
signal — the cold-start convergence transient, not just RTK noise +
lever-arm — and removing it would hide the very thing we want to measure.
Use --first-point-align to opt into the same alignment behaviour as the
anchored script when you want a like-for-like comparison.

Usage
-----
    python scripts/global_ekf_vs_fix_overlay.py \
        <source_bag_dir> <replay_output_dir> \
        [--max-h-acc-m 0.5] [--no-first-point-align] [...]

The two positional arguments are explicit — no auto-discovery — so the
script works for any layout, not just the canonical
<source_bag>/ekf_replay/<run>/replay_outputs_mcap/ pattern.
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import replace
from pathlib import Path
from typing import Optional

import numpy as np
import utm

# --- Monkey-patch mcap_ros2 to support the legacy ROS1-compat 'time' and
# 'duration' field types. The replay-output mcaps written by some
# rosbag2-mcap versions encode std_msgs/Header's stamp field as the
# lowercase 'time' rather than 'builtin_interfaces/Time'. Wire format is the
# same: int32 sec + uint32 nanosec. Without this patch, read_ros2_messages
# raises NotImplementedError on every Odometry / NavSatFix in those mcaps.
from mcap_ros2 import _dynamic as _mcap_dyn  # noqa: E402


class _LegacyTime:
    __slots__ = ("sec", "nanosec")

    def __init__(self, sec: int, nanosec: int) -> None:
        self.sec = sec
        self.nanosec = nanosec


def _read_legacy_time(reader) -> _LegacyTime:
    sec = reader.int32()
    nanosec = reader.uint32()
    return _LegacyTime(sec, nanosec)


_mcap_dyn.FIELD_PARSERS.setdefault("time", _read_legacy_time)
_mcap_dyn.FIELD_PARSERS.setdefault("duration", _read_legacy_time)

from mcap_ros2.reader import read_ros2_messages  # noqa: E402

# Reuse everything we can from the sibling script. Both scripts live in
# scripts/ so this import resolves when running from the repo root.
from gps_filtered_vs_fix_overlay import (
    # Dataclasses
    FixSample, OdomSample, PairData, Analysis,
    # Constants
    _COV_DIAGONAL_KNOWN,
    # Helpers we call directly
    _stamp_ns, _quat_to_yaw, _horizontal_sigma_m,
    gate_ref_track,
    # The analysis + plot pipeline
    analyse, plot,
)

# Force UTF-8 stdout/stderr so degree signs etc. print cleanly on Windows cp1252.
try:
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    sys.stderr.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
except (AttributeError, ValueError):
    pass


_TOPIC_GLOBAL_NAVSATFIX = "/gps/filtered/global"
_TOPIC_GLOBAL_ODOM      = "/odometry/filtered/global"
_TOPIC_DATUM_PRIMARY    = "/gps/validated"
_TOPIC_DATUM_FALLBACK   = "/gps/filtered"
_TOPIC_FIX              = "/fix"

# Tight synthetic covariance for the FixSample we synthesise when projecting
# /odometry/filtered/global through the datum. This bypasses the /fix RTK
# gate for the global-EKF track, which is what we want — the gate is for
# /fix (ground truth) only.
_SYNTH_COV = (0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0)
_SYNTH_COV_TYPE = _COV_DIAGONAL_KNOWN
_SYNTH_STATUS = 2  # NavSatStatus.STATUS_GBAS_FIX (RTK Fixed equivalent)


def _find_mcap(dir_or_file: Path) -> Path:
    """Resolve a directory or .mcap path to a concrete .mcap file."""
    p = Path(dir_or_file)
    if p.is_file() and p.suffix == ".mcap":
        return p
    if p.is_dir():
        candidates = sorted(p.glob("*.mcap"))
        if not candidates:
            print(f"ERROR: no .mcap files under {p}", file=sys.stderr)
            sys.exit(1)
        return candidates[0]
    print(f"ERROR: not a file or directory: {p}", file=sys.stderr)
    sys.exit(1)


def read_global_ekf_data(replay_dir_or_mcap: Path
                         ) -> tuple[list[FixSample], list[OdomSample], str,
                                    tuple[float, float] | None]:
    """Read the global EKF data from the replay-output mcap.

    Returns (global_track, odom_track, source_path_tag, datum_lat_lon).

    `global_track` is a list of FixSample for the EKF's position estimate.
    Preferentially built from /gps/filtered/global if present (NavSatFix path,
    directly comparable). Otherwise built from /odometry/filtered/global
    projected to UTM via the recorded datum and back to lat/lon (so the rest
    of the pipeline can treat it uniformly).

    `odom_track` is a list of OdomSample with yaw from
    /odometry/filtered/global's pose.orientation.

    `source_path_tag` is one of "navsatfix", "odometry-via-datum".

    `datum_lat_lon` is the (lat, lon) used to project the Odometry path, or
    None if the NavSatFix path was used (datum extraction wasn't needed).
    """
    mcap_path = _find_mcap(replay_dir_or_mcap)

    wanted = [
        _TOPIC_GLOBAL_NAVSATFIX,
        _TOPIC_GLOBAL_ODOM,
        _TOPIC_DATUM_PRIMARY,
        _TOPIC_DATUM_FALLBACK,
    ]
    global_navsatfix: list[FixSample] = []
    global_odom_xy:   list[tuple[int, float, float]] = []
    odom_track:       list[OdomSample] = []
    datum_primary:    list[FixSample] = []
    datum_fallback:   list[FixSample] = []

    # Use topics= filter to avoid decoding /tf and /tf_static which contain
    # a `time` field that mcap_ros2's dynamic decoder doesn't support yet.
    for msg in read_ros2_messages(str(mcap_path), topics=wanted):
        topic = msg.channel.topic
        if topic not in wanted:
            continue
        ros = msg.ros_msg
        try:
            t = _stamp_ns(ros.header.stamp)
            if t == 0:
                continue
            if topic == _TOPIC_GLOBAL_ODOM:
                # Odometry message
                p = ros.pose.pose.position
                q = ros.pose.pose.orientation
                global_odom_xy.append((t, float(p.x), float(p.y)))
                odom_track.append(OdomSample(
                    t_ns=t,
                    yaw=_quat_to_yaw(float(q.x), float(q.y),
                                     float(q.z), float(q.w)),
                ))
            else:
                # NavSatFix
                sample = FixSample(
                    t_ns=t,
                    lat=float(ros.latitude),
                    lon=float(ros.longitude),
                    status=int(ros.status.status),
                    cov=tuple(float(x) for x in ros.position_covariance),
                    cov_type=int(ros.position_covariance_type),
                )
                if topic == _TOPIC_GLOBAL_NAVSATFIX:
                    global_navsatfix.append(sample)
                elif topic == _TOPIC_DATUM_PRIMARY:
                    datum_primary.append(sample)
                elif topic == _TOPIC_DATUM_FALLBACK:
                    datum_fallback.append(sample)
        except AttributeError:
            continue

    global_navsatfix.sort(key=lambda s: s.t_ns)
    global_odom_xy.sort(key=lambda x: x[0])
    odom_track.sort(key=lambda s: s.t_ns)
    datum_primary.sort(key=lambda s: s.t_ns)
    datum_fallback.sort(key=lambda s: s.t_ns)

    # Preferred path: NavSatFix already exists.
    if global_navsatfix:
        return global_navsatfix, odom_track, "navsatfix", None

    # Fallback path: project Odometry x,y through the datum.
    if not global_odom_xy:
        print(f"ERROR: neither /gps/filtered/global nor /odometry/filtered/global "
              f"found in {mcap_path}", file=sys.stderr)
        sys.exit(1)

    datum = _pick_datum(datum_primary, datum_fallback)
    if datum is None:
        print(f"ERROR: no usable datum found in {mcap_path}. Looked at "
              f"{_TOPIC_DATUM_PRIMARY} and {_TOPIC_DATUM_FALLBACK}.",
              file=sys.stderr)
        sys.exit(1)

    datum_lat, datum_lon = datum.lat, datum.lon
    datum_e, datum_n, zone_n, zone_l = utm.from_latlon(datum_lat, datum_lon)

    synth_track: list[FixSample] = []
    for t_ns, x_map, y_map in global_odom_xy:
        utm_e = datum_e + x_map
        utm_n = datum_n + y_map
        try:
            lat, lon = utm.to_latlon(utm_e, utm_n, zone_n, zone_l)
        except utm.OutOfRangeError:
            # Global EKF diverged off the planet; skip the sample.
            continue
        synth_track.append(FixSample(
            t_ns=t_ns,
            lat=float(lat),
            lon=float(lon),
            status=_SYNTH_STATUS,
            cov=_SYNTH_COV,
            cov_type=_SYNTH_COV_TYPE,
        ))
    synth_track.sort(key=lambda s: s.t_ns)
    return synth_track, odom_track, "odometry-via-datum", (datum_lat, datum_lon)


def _pick_datum(primary: list[FixSample],
                fallback: list[FixSample]) -> FixSample | None:
    """First valid (status>=0, not null-island) sample from primary, else fallback."""
    def first_valid(track: list[FixSample]) -> FixSample | None:
        for s in track:
            if s.status >= 0 and not (abs(s.lat) < 0.1 and abs(s.lon) < 0.1):
                return s
        return None
    return first_valid(primary) or first_valid(fallback)


def read_fix_from_source_bag(source_bag_dir: Path) -> list[FixSample]:
    """Read raw /fix samples from the source recording bag. No gating."""
    mcap_path = source_bag_dir / f"{source_bag_dir.name}_0.mcap"
    if not mcap_path.exists():
        candidates = list(source_bag_dir.glob("*_0.mcap"))
        if not candidates:
            print(f"ERROR: no *_0.mcap found in {source_bag_dir}", file=sys.stderr)
            sys.exit(1)
        mcap_path = candidates[0]

    fix_track: list[FixSample] = []
    for msg in read_ros2_messages(str(mcap_path), topics=[_TOPIC_FIX]):
        if msg.channel.topic != _TOPIC_FIX:
            continue
        ros = msg.ros_msg
        try:
            t = _stamp_ns(ros.header.stamp)
            if t == 0:
                continue
            fix_track.append(FixSample(
                t_ns=t,
                lat=float(ros.latitude),
                lon=float(ros.longitude),
                status=int(ros.status.status),
                cov=tuple(float(x) for x in ros.position_covariance),
                cov_type=int(ros.position_covariance_type),
            ))
        except AttributeError:
            continue
    fix_track.sort(key=lambda s: s.t_ns)
    return fix_track


def report(label: str,
           global_track: list[FixSample],
           ref_track_raw: list[FixSample],
           odom_track: list[OdomSample],
           source_path_tag: str,
           datum_lat_lon: tuple[float, float] | None,
           max_h_acc_m: float | None,
           first_point_align: bool,
           primary_source_label: str,
           primary_source_short: str,
           ) -> tuple[list[FixSample], Analysis | None]:
    """Print the report and return (gated /fix track, Analysis)."""
    print()
    print("=" * 92)
    print(f"file: {label}")
    print("=" * 92)
    src_topic = (_TOPIC_GLOBAL_NAVSATFIX if source_path_tag == "navsatfix"
                 else _TOPIC_GLOBAL_ODOM + " (projected via datum)")
    print(f"  global EKF source: {src_topic}")
    print(f"    samples:          {len(global_track)}")
    if datum_lat_lon is not None:
        lat, lon = datum_lat_lon
        print(f"    datum lat/lon:    {lat:+.7f}, {lon:+.7f}  "
              f"(from {_TOPIC_DATUM_PRIMARY} or fallback {_TOPIC_DATUM_FALLBACK})")
    print(f"  /fix samples (raw):                   {len(ref_track_raw)}")
    print(f"  /odometry/filtered/global samples (yaw): {len(odom_track)}")

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
                None, "global EKF",   # no --diag-csv overlay in this script
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
        if bias_mag > 5.0:
            print(f"    WARNING: |bias|={bias_mag:.1f} m is large — likely a cold-start")
            print(f"    convergence transient, not just RTK + lever-arm. Consider")
            print(f"    rerunning with --no-first-point-align to see the raw comparison.")
        else:
            print("    (slope of |error| vs path is now a defensible drift rate;")
            print("     intercept ~ 0 by construction)")
    else:
        print("  NOT first-point aligned — slope is sign-uninformative (see notes)")

    err = pd.err
    err_sorted = np.sort(err)
    print()
    align_tag = "first-point aligned" if pd.aligned else "raw"
    print(f"  |error|  global EKF vs /fix (gated, {align_tag})")
    print(f"    mean   = {float(np.mean(err)):.3f} m")
    print(f"    median = {float(np.median(err)):.3f} m")
    print(f"    p95    = {float(err_sorted[int(0.95 * (err_sorted.size - 1))]):.3f} m")
    print(f"    max    = {float(err_sorted[-1]):.3f} m")

    # Linear fit of |error| vs cumulative path length.
    print()
    if A.fit_total is not None:
        slope, intercept, sigma_res = A.fit_total
        from gps_filtered_vs_fix_overlay import slope_se_m_per_100m
        se = slope_se_m_per_100m(pd.dist, sigma_res)
        path_len = float(pd.dist.max() - pd.dist.min())
        if pd.aligned:
            print("  Drift rate (slope of |error| vs cumulative /fix path, "
                  "first-point aligned):")
            se_str = f"  (SE {se:.3f})" if se is not None else ""
            print(f"    slope         = {slope:+.3f} m / 100 m{se_str}")
            print(f"    intercept     = {intercept:+.3f} m  "
                  f"(noise floor — see EKF_RESEARCH_NOTES 2026-05-12 addendum)")
            print(f"    σ_residual    = {sigma_res:.3f} m   path span = {path_len:.2f} m")
            if se is not None:
                ci_lo = slope - 1.96 * se
                ci_hi = slope + 1.96 * se
                print(f"    95% CI on drift rate: [{ci_lo:+.3f}, {ci_hi:+.3f}] m / 100 m")
        else:
            print("  Naive slope of |error| vs cumulative /fix path  (NOT drift rate):")
            print(f"    slope         = {slope:+.3f} m / 100 m")
            print(f"    intercept     = {intercept:+.3f} m  "
                  f"(static offset: anchor + lever-arm + cold-start transient)")
            print(f"    σ_residual    = {sigma_res:.3f} m   path span = {path_len:.2f} m")
            print("    (sign-uninformative on closed loops — see along/cross decomposition below)")
    else:
        print("  Linear fit: degenerate (insufficient path).")

    # Along-track decomposition → DVL scale factor.
    print()
    if A.fit_along is not None:
        slope_per_100m, intercept_along, sigma_res_along = A.fit_along
        from gps_filtered_vs_fix_overlay import slope_se_m_per_100m
        ok = np.isfinite(pd.e_along)
        se = slope_se_m_per_100m(pd.dist[ok], sigma_res_along)
        print("  Along-track decomposition  →  DVL scale factor (as seen by global EKF):")
        print(f"    slope (e_along vs path)      = {slope_per_100m:+.3f} m / 100 m")
        print(f"    DVL scale-factor estimate    = {slope_per_100m:+.3f} %")
        if se is not None:
            print(f"    slope SE                     = {se:.3f} m / 100 m   "
                  f"(±{se:.2f} % on the scale factor)")
        print(f"    intercept                    = {intercept_along:+.3f} m")
        print(f"    σ_residual                   = {sigma_res_along:.3f} m")
    else:
        print("  Along-track decomposition: no usable yaw (no fit).")

    # Per-leg cross-track fits → yaw bias.
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

    # /fix horizontal-sigma distribution.
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


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source_bag_dir", type=Path,
                   help="Source recording bag directory (contains /fix).")
    p.add_argument("replay_output_dir", type=Path,
                   help=("Directory containing replay_outputs_mcap_0.mcap (or "
                         "a direct path to the .mcap file). Provides the global "
                         "EKF data."))
    p.add_argument("--output-dir", type=Path, default=None,
                   help=("Where to write per-bag PNGs. Default: "
                         "<replay_output_dir>/. Four PNGs are written with "
                         "the pattern <basename>_{trajectory,total_error,"
                         "along_track,cross_track}.png. With --output-dir, "
                         "the basename is <source_bag>_<run>_global_ekf_vs_fix; "
                         "without, the basename is global_ekf_vs_fix."))
    p.add_argument("--no-plot", action="store_true",
                   help="Skip plotting; only print the report.")
    p.add_argument("--max-h-acc-m", type=float, default=0.5,
                   help=("Maximum /fix horizontal sigma (m) admitted as ground "
                         "truth. Default 0.5 m matches the anchored script."))
    p.add_argument("--no-satellite", action="store_true",
                   help="Skip the contextily satellite basemap.")
    p.add_argument("--first-point-align", dest="first_point_align",
                   action="store_true",
                   help=("Subtract the first paired sample's error vector from "
                         "all subsequent samples. OFF by default for the global "
                         "EKF — its cold-start transient is real signal we want "
                         "to see, not anchor-noise + lever-arm to remove. "
                         "Use this flag only for like-for-like comparison "
                         "against the anchored script's defaults."))
    p.set_defaults(first_point_align=False)
    args = p.parse_args(argv)

    max_h_acc = None if args.max_h_acc_m < 0 else args.max_h_acc_m

    if not args.source_bag_dir.is_dir():
        print(f"ERROR: source_bag_dir is not a directory: {args.source_bag_dir}",
              file=sys.stderr)
        return 1

    # Read both mcaps.
    global_track, odom_track, source_path_tag, datum_lat_lon = (
        read_global_ekf_data(args.replay_output_dir)
    )
    fix_track_raw = read_fix_from_source_bag(args.source_bag_dir)

    # Label for the plot / report header.
    label = f"{args.source_bag_dir.name}  ←  {args.replay_output_dir.name}"
    primary_label = (f"/gps/filtered/global (offline global EKF)"
                     if source_path_tag == "navsatfix"
                     else "/odometry/filtered/global (offline global EKF)")
    primary_short = "global EKF"

    ref_track, A = report(label, global_track, fix_track_raw, odom_track,
                          source_path_tag, datum_lat_lon, max_h_acc,
                          args.first_point_align,
                          primary_label, primary_short)
    if args.no_plot or A is None:
        return 0

    # Output destination.
    replay_path = Path(args.replay_output_dir)
    replay_dir = replay_path if replay_path.is_dir() else replay_path.parent
    if args.output_dir:
        out_dir = args.output_dir
        # When --output-dir is given, prefix the basename with both bag and
        # run names so several replays don't collide in the same dir.
        out_basename = f"{args.source_bag_dir.name}_{replay_dir.name}_global_ekf_vs_fix"
    else:
        out_dir = replay_dir
        out_basename = "global_ekf_vs_fix"

    plot(label, ref_track, A, out_dir, out_basename,
         satellite=not args.no_satellite)
    return 0


if __name__ == "__main__":
    sys.exit(main())
