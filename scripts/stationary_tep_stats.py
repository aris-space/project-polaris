#!/usr/bin/env python3
"""
TEP Section 6 stationary offline statistics: mean, std, sample variance, IMU/DVL Δt distribution,
and bias-drift summaries (windowed means vs time, linear slope) on header.stamp.

- stationary_01: full-bag stats (comparison baseline; field conditions often rougher).
- stationary_02: stats restricted to DVL bottom-lock-true time unions (no-lock segments excluded).

Timing for inclusion, lock windows, and Δt: **message header.stamp** (sensor / ROS time), not MCAP
log_time. That matches how filters order measurements and is appropriate for stationary measurement
noise estimation (no base_link transform required — stats are on raw message fields in sensor frames).

Lock segments use the same boolean as analyze_mcap_dvl_health.py (Dvl.beam_velocities_valid), but
interval endpoints are consecutive **DVL header.stamp** values after sorting by stamp.

Dependency: pip install rosbags numpy

Outputs (default paths under repo recordings/):
  - stationary_tep_stats.json
  - stationary_tep_stats_report.html
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags numpy", file=sys.stderr)
    raise

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import analyze_mcap_dvl_health as dvl_mod  # noqa: E402

TOPIC_IMU = "/imu/data"
TOPIC_DVL_COV = dvl_mod.TOPIC_COV
TOPIC_DVL_VEL = dvl_mod.TOPIC_VEL
_odom_lock_from_cov = dvl_mod._odom_lock_from_cov
_merge_intervals = dvl_mod._merge_intervals


def _stamp_ns(msg: Any) -> int:
    return int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)


def _include_lock_intervals_from_stamp_lock(stamp_lock: list[tuple[int, bool]]) -> list[tuple[int, int]]:
    """Merged [a,b) spans where lock True between consecutive DVL samples in stamp order."""
    if len(stamp_lock) < 2:
        return []
    stamp_lock = sorted(stamp_lock, key=lambda x: x[0])
    inc: list[tuple[int, int]] = []
    for i in range(len(stamp_lock) - 1):
        ta, lk = stamp_lock[i]
        tb = stamp_lock[i + 1][0]
        if lk:
            inc.append((ta, tb))
    return _merge_intervals(inc)


def dvl_lock_include_intervals_header_stamp_ns(bag_dir: Path) -> list[tuple[int, int]]:
    """Bottom-lock-true unions in header.stamp time from /sensors/dvl/velocity."""
    stamp_lock: list[tuple[int, bool]] = []
    with AnyReader([bag_dir]) as reader:
        for connection, _t_log, raw in reader.messages():
            if connection.topic != TOPIC_DVL_VEL:
                continue
            msg = reader.deserialize(raw, connection.msgtype)
            stamp_lock.append((_stamp_ns(msg), bool(msg.beam_velocities_valid)))
    if len(stamp_lock) >= 2:
        return _include_lock_intervals_from_stamp_lock(stamp_lock)
    # Fallback: no Dvl msgs — build from odometry_cov stamp + cov lock (e.g. calibration bags)
    cov_sl: list[tuple[int, bool]] = []
    with AnyReader([bag_dir]) as reader:
        for connection, _t_log, raw in reader.messages():
            if connection.topic != TOPIC_DVL_COV:
                continue
            msg = reader.deserialize(raw, connection.msgtype)
            cov_sl.append((_stamp_ns(msg), _odom_lock_from_cov(msg)))
    return _include_lock_intervals_from_stamp_lock(cov_sl) if len(cov_sl) >= 2 else []


def _in_lock_union(t_ns: int, intervals: list[tuple[int, int]]) -> bool:
    """True if t_ns lies in [a, b) for any merged lock-true interval."""
    for a, b in intervals:
        if a <= t_ns < b:
            return True
    return False


def _dt_stats_sec(times_ns: np.ndarray) -> dict[str, Any]:
    if times_ns.size < 2:
        return {
            "n_messages": int(times_ns.size),
            "n_dt": 0,
            "dt_mean_s": None,
            "dt_std_s": None,
            "dt_min_s": None,
            "dt_max_s": None,
            "dt_p50_s": None,
            "dt_p95_s": None,
            "dt_p99_s": None,
            "approx_mean_hz": None,
            "histogram_dt_ms": {},
        }
    dt = np.diff(times_ns.astype(np.float64)) / 1e9
    sdt = np.sort(dt)
    n = len(dt)

    def pct(q: float) -> float:
        return float(np.percentile(sdt, q))

    # Histogram bins (ms)
    edges_ms = [0, 2, 5, 10, 15, 20, 30, 50, 100, float("inf")]
    labels = ["0–2", "2–5", "5–10", "10–15", "15–20", "20–30", "30–50", "50–100", "100+"]
    dt_ms = dt * 1000.0
    hist = {lab: int(np.sum((dt_ms >= lo) & (dt_ms < hi))) for lab, lo, hi in zip(labels, edges_ms[:-1], edges_ms[1:])}

    return {
        "n_messages": int(times_ns.size),
        "n_dt": int(n),
        "n_zero_dt": int(np.sum(dt == 0.0)),
        "n_negative_dt": int(np.sum(dt < 0.0)),
        "dt_mean_s": float(dt.mean()),
        "dt_std_s": float(dt.std(ddof=1)) if n > 1 else 0.0,
        "dt_min_s": float(dt.min()),
        "dt_max_s": float(dt.max()),
        "dt_p50_s": pct(50),
        "dt_p95_s": pct(95),
        "dt_p99_s": pct(99),
        "approx_mean_hz": float(1.0 / dt.mean()) if dt.mean() > 0 else None,
        "histogram_dt_ms": hist,
    }


def _resolve_bags_root(repo_root: Path, bags_root_arg: Path | None) -> Path:
    return (bags_root_arg if bags_root_arg is not None else (repo_root / "recordings" / "rosbags")).resolve()


def _latest_date_dir(bags_root: Path) -> Path | None:
    dated = sorted(
        d for d in bags_root.iterdir() if d.is_dir() and d.name[:4].isdigit() and d.name.count("-") == 2
    )
    return dated[-1] if dated else None


def _series_stats(values: np.ndarray) -> dict[str, float | None]:
    v = values[np.isfinite(values)]
    n = v.size
    if n == 0:
        return {"n": 0, "mean": None, "std_sample": None, "variance_sample": None}
    mean = float(v.mean())
    if n == 1:
        return {"n": 1, "mean": mean, "std_sample": 0.0, "variance_sample": 0.0}
    var = float(v.var(ddof=1))
    return {"n": n, "mean": mean, "std_sample": math.sqrt(var), "variance_sample": var}


def _bias_drift_analysis(
    times_ns: np.ndarray,
    vals: np.ndarray,
    axis_names: tuple[str, ...],
    window_sec: float,
    quantity: str,
    unit_note: str,
) -> dict[str, Any]:
    """
    Non-overlapping bins on relative time (first sample at 0). Per-bin mean; least-squares slope
    of bin mean vs bin mid-time (drift rate in [quantity unit] per second).
    """
    empty = {
        "quantity": quantity,
        "unit_slope_note": unit_note,
        "window_sec": window_sec,
        "recording_span_rel_s": None,
        "n_segments": 0,
        "segments": [],
        "slope_per_s": {k: None for k in axis_names},
        "mean_last_minus_first": {k: None for k in axis_names},
    }
    if times_ns.size == 0 or vals.size == 0 or vals.shape[0] != times_ns.size:
        return empty
    t0 = float(times_ns[0])
    t_rel = (times_ns.astype(np.float64) - t0) / 1e9
    t_end = float(t_rel[-1])
    segments: list[dict[str, Any]] = []
    w = 0
    while w * window_sec <= t_end + 1e-12:
        w_lo = w * window_sec
        w_hi = (w + 1) * window_sec
        mask = (t_rel >= w_lo) & (t_rel < w_hi)
        w += 1
        if not np.any(mask):
            continue
        chunk = vals[mask]
        tm = float(np.mean(t_rel[mask]))
        means = {axis_names[i]: float(chunk[:, i].mean()) for i in range(len(axis_names))}
        segments.append(
            {
                "t_mid_rel_s": round(tm, 3),
                "t_bin_start_rel_s": round(w_lo, 3),
                "t_bin_end_rel_s": round(w_hi, 3),
                "n": int(np.sum(mask)),
                "mean": {k: round(v, 9) for k, v in means.items()},
            }
        )

    out = {
        **empty,
        "recording_span_rel_s": round(t_end, 3),
        "n_segments": len(segments),
        "segments": segments,
    }
    if len(segments) >= 2:
        t_m = np.array([s["t_mid_rel_s"] for s in segments], dtype=np.float64)
        for i, name in enumerate(axis_names):
            m = np.array([s["mean"][name] for s in segments], dtype=np.float64)
            slope, _intercept = np.polyfit(t_m, m, 1)
            out["slope_per_s"][name] = float(slope)
            out["mean_last_minus_first"][name] = float(segments[-1]["mean"][name] - segments[0]["mean"][name])
    return out


def collect_imu_dvl(
    bag_dir: Path,
    lock_intervals: list[tuple[int, int]] | None,
    dvl_require_lock: bool,
    drift_window_sec: float,
) -> dict[str, Any]:
    """
    lock_intervals=None -> use all messages (filter by stamp only for membership: always true).
    Else -> keep IMU / DVL samples whose **header.stamp** lies in union(lock_intervals) [a,b).
    dvl_require_lock -> odometry_cov samples must also pass twist covariance lock heuristic.

    Δt stats use **sorted** header.stamp per topic so inter-sample spacing matches sensor timeline
    (fusion ordering), not MCAP write order.
    """
    imu_ts: list[int] = []
    imu_gx: list[float] = []
    imu_gy: list[float] = []
    imu_gz: list[float] = []
    imu_ax: list[float] = []
    imu_ay: list[float] = []
    imu_az: list[float] = []

    dvl_ts: list[int] = []
    dvl_vx: list[float] = []
    dvl_vy: list[float] = []
    dvl_vz: list[float] = []

    with AnyReader([bag_dir]) as reader:
        for connection, _t_log, raw in reader.messages():
            if connection.topic == TOPIC_IMU:
                msg = reader.deserialize(raw, connection.msgtype)
                st = _stamp_ns(msg)
                if lock_intervals is not None and not _in_lock_union(st, lock_intervals):
                    continue
                imu_ts.append(st)
                imu_gx.append(float(msg.angular_velocity.x))
                imu_gy.append(float(msg.angular_velocity.y))
                imu_gz.append(float(msg.angular_velocity.z))
                imu_ax.append(float(msg.linear_acceleration.x))
                imu_ay.append(float(msg.linear_acceleration.y))
                imu_az.append(float(msg.linear_acceleration.z))
            elif connection.topic == TOPIC_DVL_COV:
                msg = reader.deserialize(raw, connection.msgtype)
                st = _stamp_ns(msg)
                if lock_intervals is not None and not _in_lock_union(st, lock_intervals):
                    continue
                if dvl_require_lock and not _odom_lock_from_cov(msg):
                    continue
                dvl_ts.append(st)
                dvl_vx.append(float(msg.twist.twist.linear.x))
                dvl_vy.append(float(msg.twist.twist.linear.y))
                dvl_vz.append(float(msg.twist.twist.linear.z))

    if imu_ts:
        imu_ts_a = np.array(imu_ts, dtype=np.int64)
        o = np.argsort(imu_ts_a)
        imu_ts_s = imu_ts_a[o]
        gx = np.array(imu_gx, dtype=np.float64)[o]
        gy = np.array(imu_gy, dtype=np.float64)[o]
        gz = np.array(imu_gz, dtype=np.float64)[o]
        ax = np.array(imu_ax, dtype=np.float64)[o]
        ay = np.array(imu_ay, dtype=np.float64)[o]
        az = np.array(imu_az, dtype=np.float64)[o]
    else:
        imu_ts_s = np.array([], dtype=np.int64)
        gx = gy = gz = ax = ay = az = np.array([], dtype=np.float64)

    if dvl_ts:
        dvl_ts_a = np.array(dvl_ts, dtype=np.int64)
        od = np.argsort(dvl_ts_a)
        dvl_ts_s = dvl_ts_a[od]
        vx = np.array(dvl_vx, dtype=np.float64)[od]
        vy = np.array(dvl_vy, dtype=np.float64)[od]
        vz = np.array(dvl_vz, dtype=np.float64)[od]
    else:
        dvl_ts_s = np.array([], dtype=np.int64)
        vx = vy = vz = np.array([], dtype=np.float64)

    imu_block = {
        "angular_velocity_rad_s": {
            "x": _series_stats(gx),
            "y": _series_stats(gy),
            "z": _series_stats(gz),
        },
        "linear_acceleration_m_s2": {
            "x": _series_stats(ax),
            "y": _series_stats(ay),
            "z": _series_stats(az),
        },
        "header_stamp_dt": _dt_stats_sec(imu_ts_s),
    }

    speed = np.sqrt(vx**2 + vy**2 + vz**2) if vx.size else np.array([], dtype=np.float64)
    dvl_block = {
        "twist_linear_m_s": {
            "x": _series_stats(vx),
            "y": _series_stats(vy),
            "z": _series_stats(vz),
        },
        "speed_norm_m_s": _series_stats(speed),
        "header_stamp_dt": _dt_stats_sec(dvl_ts_s),
    }

    gstack = np.column_stack([gx, gy, gz]) if gx.size else np.zeros((0, 3))
    astack = np.column_stack([ax, ay, az]) if ax.size else np.zeros((0, 3))
    vstack = np.column_stack([vx, vy, vz]) if vx.size else np.zeros((0, 3))

    bias_drift = {
        "method": (
            f"Non-overlapping {drift_window_sec:g}s bins on header.stamp (t_rel=0 at first kept sample). "
            "Per-bin sample mean; slope = linear fit of bin mean vs bin mid-time."
        ),
        "window_sec": drift_window_sec,
        "imu": {
            "angular_velocity_rad_s": _bias_drift_analysis(
                imu_ts_s,
                gstack,
                ("x", "y", "z"),
                drift_window_sec,
                "imu_angular_velocity",
                "rad/s per second (= rad/s² scale for linear trend of mean rate)",
            ),
            "linear_acceleration_m_s2": _bias_drift_analysis(
                imu_ts_s,
                astack,
                ("x", "y", "z"),
                drift_window_sec,
                "imu_linear_acceleration",
                "m/s² per second",
            ),
        },
        "dvl_odometry_cov_twist_linear_m_s": _bias_drift_analysis(
            dvl_ts_s,
            vstack,
            ("x", "y", "z"),
            drift_window_sec,
            "dvl_twist_linear",
            "m/s per second (= m/s² scale for linear trend of mean velocity)",
        ),
        "interpretation": [
            "Each slope is a single straight line fit to the sequence of bin means vs bin mid-time. "
            "If the true drift bends (e.g. temperature), you still see one average slope — use the "
            "per-bin table to spot curvature or steps.",
            "IMU gyro: slopes are tiny in rad/s per second but integrate over ~20 min into visible "
            "mean-offset changes; compare to local EKF gyro fusion expectations.",
            "DVL: slope on mean twist is not a measured physical acceleration; it summarizes how "
            "the filter/driver output creeps at rest. Large slopes under lock warrant Foxglove review; "
            "modest slopes pair with covariance inflation in odometry_covariance_node.",
        ],
    }

    return {"imu": imu_block, "dvl_odometry_cov": dvl_block, "bias_drift": bias_drift}


def interpretation_notes_for_bag(mask_lock: bool, bag_name: str) -> list[str]:
    """Short human-readable context for the HTML report (not statistical truth)."""
    bn = bag_name.lower()
    notes: list[str] = []
    if mask_lock:
        notes.append(
            "Lock-masked cohort: only samples whose header.stamp lies inside bottom-lock-true "
            "intervals (from /sensors/dvl/velocity). DVL twist statistics exclude no-lock gaps."
        )
        notes.append(
            "DVL bias drift under lock: bottom track can stay valid while the reported twist mean creeps "
            "(sensor bias, water motion, temperature). A small non-zero drift slope is normal; the stack "
            "adds a separate inflation factor on those stationary-derived variances (see "
            "odometry_covariance_node) instead of discarding measurements."
        )
        notes.append(
            "IMU samples here use the same stamp mask, so gyro/accel drift in this card is aligned "
            "in time with the DVL cohort (not necessarily the full wall-clock span if lock was intermittent)."
        )
    else:
        notes.append(
            "Full bag: every IMU and every odometry_cov message. DVL stats mix long locked segments "
            "with brief no-lock bursts — apparent variance and drift can be inflated by those bursts."
        )
        notes.append(
            "stationary_01 field notes: rougher weather / mooring at the steg — compare IMU noise "
            "and DVL stability to stationary_02 for environmental contrast, not as the only DVL truth."
        )
    if "stationary_02" in bn and mask_lock:
        notes.append(
            "Primary reference for DVL + IMU stationary priors under sustained lock (calmer conditions in the field log)."
        )
    return notes


def analyze_pair(
    bag_dir: Path,
    label: str,
    field_note: str,
    mask_lock: bool,
    drift_window_sec: float,
) -> dict[str, Any]:
    dvl_res = dvl_mod.analyze_bag(bag_dir)
    # Masking uses **header.stamp** intervals from DVL velocity (fallback: odometry_cov stamps).
    intervals_stamp = dvl_lock_include_intervals_header_stamp_ns(bag_dir) if mask_lock else None
    locked_span_s = sum((b - a) / 1e9 for a, b in intervals_stamp) if intervals_stamp else None
    total_span_s = dvl_res.duration_ns / 1e9 if dvl_res.duration_ns else None

    stats = collect_imu_dvl(
        bag_dir,
        lock_intervals=intervals_stamp,
        dvl_require_lock=bool(mask_lock),
        drift_window_sec=drift_window_sec,
    )

    return {
        "bag_name": bag_dir.name,
        "bag_path": str(bag_dir.resolve()),
        "label": label,
        "field_note": field_note,
        "masking": {
            "exclude_no_lock_segments": mask_lock,
            "lock_true_intervals_header_stamp_ns": (intervals_stamp or [])[:50],
            "n_lock_intervals": len(intervals_stamp) if intervals_stamp else 0,
            "approx_locked_span_s": locked_span_s,
            "dvl_health_script_duration_s": total_span_s,
            "lock_fraction_time_weighted_dvl_log_time": dvl_res.lock_fraction_time_dvl,
            "note": (
                "Lock fraction and dvl_health flags come from analyze_mcap_dvl_health.py (MCAP log_time). "
                "Statistics and lock mask use header.stamp."
            ),
        },
        "dvl_health_flags": dvl_res.flags,
        "statistics": {
            "imu": stats["imu"],
            "dvl_odometry_cov": stats["dvl_odometry_cov"],
        },
        "bias_drift": stats["bias_drift"],
        "interpretation": interpretation_notes_for_bag(mask_lock, bag_dir.name),
    }


def build_html_report(payload: dict[str, Any]) -> str:
    """Self-contained HTML: hydrographic / instrument aesthetic."""
    data_json = json.dumps(payload, indent=2)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Stationary TEP §6 — IMU / DVL statistics</title>
  <link rel="preconnect" href="https://fonts.googleapis.com" />
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin />
  <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;600&family=Instrument+Serif:ital@0;1&display=swap" rel="stylesheet" />
  <style>
    :root {{
      --bg: #0c0f12;
      --panel: #141a20;
      --ink: #e6edf3;
      --muted: #8b9caa;
      --accent: #3dd6c6;
      --accent-dim: #2a8f85;
      --warn: #e8b86d;
      --grid: #1e2832;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--ink);
      font-family: "IBM Plex Mono", ui-monospace, monospace;
      font-size: 14px;
      line-height: 1.55;
      min-height: 100vh;
      background-image:
        linear-gradient(160deg, rgba(61, 214, 198, 0.04) 0%, transparent 45%),
        repeating-linear-gradient(90deg, transparent, transparent 80px, var(--grid) 80px, var(--grid) 81px);
    }}
    header {{
      padding: 2.5rem 6vw 1.5rem;
      border-bottom: 1px solid var(--grid);
    }}
    h1 {{
      font-family: "Instrument Serif", Georgia, serif;
      font-weight: 400;
      font-size: clamp(1.75rem, 4vw, 2.35rem);
      letter-spacing: 0.02em;
      margin: 0 0 0.5rem;
    }}
    h1 span {{ color: var(--accent); font-style: italic; }}
    .subtitle {{ color: var(--muted); max-width: 52rem; }}
    main {{ padding: 2rem 6vw 4rem; }}
    .cards {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 1.25rem;
      margin-bottom: 2.5rem;
    }}
    .card {{
      background: var(--panel);
      border: 1px solid var(--grid);
      padding: 1.25rem 1.35rem;
      position: relative;
      overflow: hidden;
    }}
    .card::before {{
      content: "";
      position: absolute;
      top: 0; left: 0; right: 0;
      height: 3px;
      background: linear-gradient(90deg, var(--accent), var(--accent-dim));
    }}
    .card h2 {{
      font-family: "Instrument Serif", Georgia, serif;
      font-size: 1.35rem;
      margin: 0 0 0.75rem;
      font-weight: 400;
    }}
    .badge {{
      display: inline-block;
      font-size: 0.7rem;
      text-transform: uppercase;
      letter-spacing: 0.12em;
      padding: 0.2rem 0.5rem;
      background: rgba(61, 214, 198, 0.12);
      color: var(--accent);
      margin-bottom: 0.6rem;
    }}
    .note {{ color: var(--warn); font-size: 0.85rem; margin-top: 0.5rem; }}
    .interp-box {{
      font-size: 0.78rem;
      color: var(--muted);
      border-left: 3px solid var(--accent-dim);
      padding: 0.45rem 0 0.45rem 0.85rem;
      margin: 0.85rem 0;
      line-height: 1.48;
      background: rgba(61, 214, 198, 0.04);
    }}
    .interp-box strong {{ color: var(--accent); font-weight: 600; font-size: 0.72rem; letter-spacing: 0.06em; }}
    .interp-box ul {{ margin: 0.35rem 0 0 1.1rem; padding: 0; }}
    .interp-box li {{ margin-bottom: 0.35rem; }}
    .interp-global {{ max-width: 52rem; margin: 0 6vw 1.5rem; font-size: 0.8rem; color: var(--muted); }}
    table {{
      width: 100%;
      border-collapse: collapse;
      margin: 1rem 0;
      font-size: 0.82rem;
    }}
    th, td {{
      text-align: left;
      padding: 0.45rem 0.6rem;
      border-bottom: 1px solid var(--grid);
    }}
    th {{ color: var(--muted); font-weight: 600; text-transform: uppercase; font-size: 0.68rem; letter-spacing: 0.08em; }}
    tr:nth-child(even) td {{ background: rgba(255,255,255,0.02); }}
    .num {{ text-align: right; font-variant-numeric: tabular-nums; }}
    section h3 {{
      font-family: "Instrument Serif", Georgia, serif;
      font-size: 1.15rem;
      margin: 2rem 0 0.75rem;
      color: var(--accent);
    }}
    .hist {{
      display: flex;
      align-items: flex-end;
      gap: 4px;
      height: 100px;
      margin-top: 0.75rem;
    }}
    .hist div {{
      flex: 1;
      background: linear-gradient(180deg, var(--accent), var(--accent-dim));
      min-width: 4px;
      border-radius: 2px 2px 0 0;
      opacity: 0.85;
      position: relative;
    }}
    .hist div span {{
      position: absolute;
      bottom: -1.35rem;
      left: 50%;
      transform: translateX(-50%) rotate(-35deg);
      transform-origin: center;
      font-size: 0.6rem;
      color: var(--muted);
      white-space: nowrap;
    }}
    .ekf-note {{
      background: var(--panel);
      border: 1px dashed var(--accent-dim);
      padding: 1rem 1.2rem;
      margin-top: 2rem;
      font-size: 0.85rem;
      color: var(--muted);
    }}
    .ekf-note code {{ color: var(--accent); }}
  </style>
</head>
<body>
  <header>
    <h1>POLARIS <span>Stationary</span> telemetry — TEP §6</h1>
    <p class="subtitle">Mean, sample standard deviation, sample variance, and <strong>header.stamp</strong> Δt for IMU and DVL (<code>/sensors/dvl/odometry_cov</code>). Inclusion and Δt use <strong>sensor timestamps</strong> (sorted per topic), not MCAP log_time. DVL lock mask uses <code>/sensors/dvl/velocity</code> <code>beam_velocities_valid</code> on stamp intervals. No <code>base_link</code> transform — raw message fields in sensor frames.</p>
  </header>
  <div id="global-interpretation" class="interp-global"></div>
  <main id="main"></main>
  <script type="application/json" id="payload">{data_json}</script>
  <script>
    const payload = JSON.parse(document.getElementById("payload").textContent);
    const main = document.getElementById("main");

    function interpList(title, items) {{
      if (!items || !items.length) return "";
      let h = "<div class='interp-box'><strong>" + title + "</strong><ul>";
      for (const t of items) h += "<li>" + t + "</li>";
      h += "</ul></div>";
      return h;
    }}

    function fmt(x, d=6) {{
      if (x === null || x === undefined || (typeof x === "number" && !isFinite(x))) return "—";
      if (typeof x === "number") return x.toPrecision(d);
      return String(x);
    }}

    function rowStat(name, o) {{
      if (!o || o.n === 0) return `<tr><td>${{name}}</td><td class="num">0</td><td class="num">—</td><td class="num">—</td><td class="num">—</td></tr>`;
      return `<tr><td>${{name}}</td><td class="num">${{o.n}}</td><td class="num">${{fmt(o.mean)}}</td><td class="num">${{fmt(o.std_sample)}}</td><td class="num">${{fmt(o.variance_sample)}}</td></tr>`;
    }}

    function tableAxis(title, block) {{
      let h = `<h3>${{title}}</h3><table><thead><tr><th>Axis</th><th class="num">n</th><th class="num">mean</th><th class="num">std (sample)</th><th class="num">variance (sample)</th></tr></thead><tbody>`;
      for (const k of ["x","y","z"]) h += rowStat(k, block[k]);
      h += "</tbody></table>";
      return h;
    }}

    function driftBlock(title, block) {{
      if (!block || !block.n_segments) return "";
      let h = `<h3>${{title}}</h3><p style="font-size:0.72rem;color:var(--muted);">Window ${{block.window_sec}}s · span ${{block.recording_span_rel_s}}s · ${{block.n_segments}} segments · slope = d(mean)/dt along bin centers</p>`;
      h += "<table><thead><tr><th>Axis</th><th class='num'>slope / s</th><th class='num'>Δ last−1st bin mean</th></tr></thead><tbody>";
      for (const ax of ["x","y","z"]) {{
        const s = block.slope_per_s && block.slope_per_s[ax];
        const d = block.mean_last_minus_first && block.mean_last_minus_first[ax];
        h += `<tr><td>${{ax}}</td><td class="num">${{s != null ? s.toExponential(4) : "—"}}</td><td class="num">${{d != null ? d.toExponential(4) : "—"}}</td></tr>`;
      }}
      h += "</tbody></table>";
      h += "<p style='font-size:0.7rem;color:var(--muted);'>Bin means (sample)</p><table><thead><tr><th>t_mid s</th><th class='num'>n</th><th class='num'>mx</th><th class='num'>my</th><th class='num'>mz</th></tr></thead><tbody>";
      for (const seg of (block.segments || [])) {{
        const m = seg.mean || {{}};
        h += `<tr><td>${{seg.t_mid_rel_s}}</td><td class="num">${{seg.n}}</td><td class="num">${{fmt(m.x,5)}}</td><td class="num">${{fmt(m.y,5)}}</td><td class="num">${{fmt(m.z,5)}}</td></tr>`;
      }}
      h += "</tbody></table>";
      return h;
    }}

    function dtTable(dt) {{
      if (!dt || dt.n_dt === 0) return "<p class='note'>Not enough messages for Δt.</p>";
      let h = `<table><thead><tr><th>Δt metric</th><th class="num">value</th></tr></thead><tbody>`;
      h += `<tr><td>n intervals</td><td class="num">${{dt.n_dt}}</td></tr>`;
      h += `<tr><td>mean Δt (s)</td><td class="num">${{fmt(dt.dt_mean_s)}}</td></tr>`;
      h += `<tr><td>std Δt (s)</td><td class="num">${{fmt(dt.dt_std_s)}}</td></tr>`;
      h += `<tr><td>min / max Δt (s)</td><td class="num">${{fmt(dt.dt_min_s)}} / ${{fmt(dt.dt_max_s)}}</td></tr>`;
      h += `<tr><td>p50 / p95 / p99 Δt (s)</td><td class="num">${{fmt(dt.dt_p50_s)}} / ${{fmt(dt.dt_p95_s)}} / ${{fmt(dt.dt_p99_s)}}</td></tr>`;
      h += `<tr><td>≈ mean rate (Hz)</td><td class="num">${{dt.approx_mean_hz != null ? dt.approx_mean_hz.toFixed(2) : "—"}}</td></tr>`;
      h += "</tbody></table>";
      const hist = dt.histogram_dt_ms || {{}};
      const keys = Object.keys(hist);
      const maxC = Math.max(1, ...keys.map(k => hist[k]));
      h += "<p style='margin-top:1rem;color:var(--muted);font-size:0.75rem;'>Δt histogram (ms, consecutive sorted header.stamp)</p><div class='hist'>";
      for (const k of keys) {{
        const pct = (hist[k] / maxC) * 100;
        h += `<div style="height:${{pct}}%" title="${{k}} ms: ${{hist[k]}}"><span>${{k}}</span></div>`;
      }}
      h += "</div>";
      return h;
    }}

    function renderBag(b) {{
      const m = b.masking;
      const st = b.statistics;
      const bd = b.bias_drift || {{}};
      let html = `<div class="card"><span class="badge">${{b.label}}</span><h2>${{b.bag_name}}</h2>`;
      html += `<p style="color:var(--muted);font-size:0.8rem;">${{b.field_note}}</p>`;
      html += interpList("How to read this card", b.interpretation || []);
      if (m.exclude_no_lock_segments) {{
        html += `<p style="font-size:0.78rem;">Lock-true span (header.stamp) ≈ <strong style="color:var(--accent)">${{m.approx_locked_span_s != null ? m.approx_locked_span_s.toFixed(1) : "—"}}</strong> s · intervals: <strong>${{m.n_lock_intervals}}</strong> · ref. frac locked (log_time, dvl_health): <strong>${{m.lock_fraction_time_weighted_dvl_log_time != null ? (100*m.lock_fraction_time_weighted_dvl_log_time).toFixed(2)+"%" : "—"}}</strong></p>`;
      }} else {{
        html += `<p style="font-size:0.78rem;">Full bag (no lock mask). Ref. DVL lock fraction (log_time): <strong>${{m.lock_fraction_time_weighted_dvl_log_time != null ? (100*m.lock_fraction_time_weighted_dvl_log_time).toFixed(2)+"%" : "—"}}</strong></p>`;
      }}
      if (m.note) html += `<p style="font-size:0.72rem;color:var(--muted);">${{m.note}}</p>`;
      if (b.dvl_health_flags && b.dvl_health_flags.length)
        html += `<p class="note">DVL flags: ${{b.dvl_health_flags.join(", ")}}</p>`;
      html += tableAxis("IMU angular velocity (rad/s)", st.imu.angular_velocity_rad_s);
      html += tableAxis("IMU linear acceleration (m/s²)", st.imu.linear_acceleration_m_s2);
      html += "<h3>IMU header.stamp Δt</h3>" + dtTable(st.imu.header_stamp_dt);
      html += tableAxis("DVL twist linear (m/s) — odometry_cov", st.dvl_odometry_cov.twist_linear_m_s);
      html += "<table><thead><tr><th>Speed ||v||</th><th class='num'>n</th><th class='num'>mean</th><th class='num'>std</th><th class='num'>var</th></tr></thead><tbody>";
      const sn = st.dvl_odometry_cov.speed_norm_m_s;
      html += rowStat("‖v‖", sn) + "</tbody></table>";
      html += "<h3>DVL header.stamp Δt</h3>" + dtTable(st.dvl_odometry_cov.header_stamp_dt);
      if (bd.imu) {{
        html += "<h3 style='margin-top:1.5rem;color:var(--warn);'>Bias drift (windowed mean vs time)</h3>";
        html += "<p style='font-size:0.75rem;color:var(--muted);'>" + (bd.method || "") + "</p>";
        html += interpList("Drift plots — interpretation", bd.interpretation || []);
        html += driftBlock("IMU gyro (rad/s bin means)", bd.imu.angular_velocity_rad_s);
        html += driftBlock("IMU accel (m/s² bin means)", bd.imu.linear_acceleration_m_s2);
        html += driftBlock("DVL twist linear (m/s bin means)", bd.dvl_odometry_cov_twist_linear_m_s);
      }}
      html += "</div>";
      return html;
    }}

    const ginterp = document.getElementById("global-interpretation");
    if (ginterp && payload.report_interpretation && payload.report_interpretation.length)
      ginterp.innerHTML = interpList("Whole-report context", payload.report_interpretation);

    main.innerHTML = `
      <div class="cards">
        ${{payload.bags.map(renderBag).join("")}}
      </div>
      <div class="ekf-note">
        <strong style="color:var(--ink);">Where to apply these numbers</strong><br/>
        Local EKF: <code>${{payload.ekf_config_path}}</code> — <code>process_noise_covariance</code> (15×15) models evolution; per-sensor measurement noise often comes from message covariances on <code>/imu/data</code> and <code>/sensors/dvl/odometry_cov</code> (robot_localization). Sample variances above use raw sensor-frame quantities; Δt uses <code>header.stamp</code> (same clock family filters use for ordering). If IMU covariances in the bag are zero or nominal, you can scale them using the sample variances as a <em>starting point</em> for angular-rate and acceleration diagonals (after removing mean bias). DVL: ‖v‖ variance on lock-masked data is a sanity check only.
      </div>
    `;
  </script>
</body>
</html>
"""


def main() -> None:
    ap = argparse.ArgumentParser(description="TEP §6 stationary IMU/DVL statistics.")
    ap.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Repository root (default: parent of scripts/).",
    )
    ap.add_argument(
        "--out-json",
        type=Path,
        default=None,
        help="Output JSON path (default: recordings/stationary_tep_stats.json)",
    )
    ap.add_argument(
        "--out-html",
        type=Path,
        default=None,
        help="Output HTML path (default: recordings/stationary_tep_stats_report.html)",
    )
    ap.add_argument(
        "--drift-window-sec",
        type=float,
        default=120.0,
        help="Duration of non-overlapping bins for bias-drift analysis (default: 120).",
    )
    ap.add_argument(
        "--bags-root",
        type=Path,
        default=None,
        help="Root containing dated rosbag folders (default: <repo>/recordings/rosbags).",
    )
    ap.add_argument(
        "--bags-date",
        type=str,
        default=None,
        help="Date folder under bags root (e.g. 2026-04-19). Default: latest available date folder.",
    )
    args = ap.parse_args()
    root = args.repo_root.resolve()
    out_json = args.out_json or (root / "recordings" / "stationary_tep_stats.json")
    out_html = args.out_html or (root / "recordings" / "stationary_tep_stats_report.html")

    rosbags_root = _resolve_bags_root(root, args.bags_root)
    if not rosbags_root.is_dir():
        print(f"Missing rosbags root: {rosbags_root}", file=sys.stderr)
        sys.exit(1)

    date_dir = (rosbags_root / args.bags_date) if args.bags_date else _latest_date_dir(rosbags_root)
    if date_dir is None or not date_dir.is_dir():
        print(
            f"No dated bag folder found under {rosbags_root}. Provide --bags-date explicitly.",
            file=sys.stderr,
        )
        sys.exit(1)

    b1 = next(date_dir.glob("stationary_01_*"), None)
    b2 = next(date_dir.glob("stationary_02_*"), None)
    if not b1 or not b1.is_dir() or not b2 or not b2.is_dir():
        stationary_dirs = sorted(d for d in date_dir.iterdir() if d.is_dir() and d.name.startswith("stationary_"))
        if len(stationary_dirs) < 2:
            print(
                f"Need at least two stationary_* bags under {date_dir} (or provide a date with stationary_01 and stationary_02).",
                file=sys.stderr,
            )
            sys.exit(1)
        b1, b2 = stationary_dirs[0], stationary_dirs[1]

    ekf_rel = "src/navigation/ekf_localization_pkg/config/ekf_local.yaml"

    payload: dict[str, Any] = {
        "tep_reference": "TEP Section 6 offline — mean, std, variance, dt distribution; stationary noise -> measurement covariance first estimate",
        "time_base": "message header.stamp for inclusion, lock windows (stamp), and delta-t (sorted stamps per topic); not MCAP log_time",
        "report_interpretation": [
            "Variance columns are sample variance around the full-run mean in each cohort — they mix "
            "white noise with any slow bias movement inside the recording.",
            "stationary_01 vs stationary_02: compare cards for weather/mooring stress (01) vs calmer "
            "lock-masked priors (02). Do not expect identical DVL drift; 02 is the intended source for "
            "lock-state measurement noise and drift-aware covariance tuning.",
            "IMU Δt bimodal histograms come from recorder batching (MCAP log order was also bursty); "
            "bias-drift bins use header.stamp time, so segment means follow the sensor clock.",
            "Foxglove still wins for validating that drift is smooth bias vs an event (bump, thruster, lock glitch).",
        ],
        "bias_drift_window_sec": args.drift_window_sec,
        "imu_topic": TOPIC_IMU,
        "dvl_topic": TOPIC_DVL_COV,
        "ekf_config_path": ekf_rel,
        "bags": [
            analyze_pair(
                b1,
                label=f"{b1.name} — full bag",
                field_note="Baseline / comparison. Field log: stormier conditions at the steg.",
                mask_lock=False,
                drift_window_sec=args.drift_window_sec,
            ),
            analyze_pair(
                b2,
                label=f"{b2.name} — lock-masked",
                field_note="Preferred for DVL-inclusive stationary stats: IMU and DVL samples only during bottom-lock-true unions; odometry_cov also covariance-gated.",
                mask_lock=True,
                drift_window_sec=args.drift_window_sec,
            ),
        ],
    }

    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    out_html.write_text(build_html_report(payload), encoding="utf-8")
    print(f"Wrote {out_json}")
    print(f"Wrote {out_html}")


if __name__ == "__main__":
    main()
