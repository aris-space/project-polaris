#!/usr/bin/env python3
"""
DVL lock, beams, altitude, and odometry usability for raw ROS 2 MCAP bags under recordings/rosbags.

Uses rosbags deserialization (no ROS install). Skips __bodyframe / rosbags_bodyframe paths.

Lock (bottom tracking):
  - Primary: marine_acoustic_msgs/Dvl.beam_velocities_valid on /sensors/dvl/velocity (driver maps JSON velocity_valid).
  - Secondary: /sensors/dvl/odometry_cov linear twist variance (vx, vy, vz diagonals): in these pool bags, locked epochs are ~1e-6 m²/s² (performance model); no-lock inflation is ~1.0 m²/s² (robot param may differ from the node default 1e6).

Dependency: pip install rosbags
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

# odometry_covariance_node may use a large no_lock_variance (default 1e6), but recordings show
# ~1.0 m²/s² on vx/vy/vz when lock is lost; locked epochs are ~1e-6 (performance variant floor).
LOCKED_TWIST_LINEAR_VAR_MAX = 1e-3

TOPIC_VEL = "/sensors/dvl/velocity"
TOPIC_ODO = "/sensors/dvl/odometry"
TOPIC_COV = "/sensors/dvl/odometry_cov"
TOPIC_DR = "/sensors/dvl/dead_reckoning"


def _under_raw_rosbags(path: Path) -> bool:
    lower = [p.lower() for p in path.parts]
    if "rosbags" not in lower:
        return False
    if any("bodyframe" in p.lower() for p in path.parts):
        return False
    return True


def find_raw_bag_dirs(root: Path) -> list[Path]:
    out: list[Path] = []
    for meta in root.rglob("metadata.yaml"):
        d = meta.parent.resolve()
        if not _under_raw_rosbags(d):
            continue
        if d.name.endswith("__bodyframe"):
            continue
        if any(d.glob("*.mcap")):
            out.append(d)
    return sorted(set(out))


def _odom_speed(msg: Any) -> float:
    t = msg.twist.twist.linear
    return math.sqrt(float(t.x) ** 2 + float(t.y) ** 2 + float(t.z) ** 2)


def _odom_lock_from_cov(msg: Any) -> bool:
    c = msg.twist.covariance
    if len(c) < 15:
        return False
    mx = max(float(c[0]), float(c[7]), float(c[14]))
    return mx < LOCKED_TWIST_LINEAR_VAR_MAX


def _dvl_speed(msg: Any) -> float:
    v = msg.velocity
    return math.sqrt(float(v.x) ** 2 + float(v.y) ** 2 + float(v.z) ** 2)


@dataclass
class BagDvlResult:
    bag_name: str
    bag_path: str
    topics_dvl_related: dict[str, str] = field(default_factory=dict)
    duration_ns: int = 0
    # Dvl topic
    n_dvl: int = 0
    lock_fraction_time_dvl: float | None = None
    lock_false_intervals_ns: list[tuple[int, int]] = field(default_factory=list)
    num_good_beams_hist: dict[int, int] = field(default_factory=dict)
    altitude_valid_lock: list[float] = field(default_factory=list)
    range_min_per_msg: list[float] = field(default_factory=list)
    speed_no_lock_dvl: list[float] = field(default_factory=list)
    speed_lock_dvl: list[float] = field(default_factory=list)
    # odometry_cov
    n_cov: int = 0
    n_cov_at_dvl_stamp: int = 0
    lock_fraction_time_cov: float | None = None
    speed_no_lock_cov: list[float] = field(default_factory=list)
    speed_lock_cov: list[float] = field(default_factory=list)
    cov_lock_disagree_with_dvl: int = 0
    # flags
    flags: list[str] = field(default_factory=list)
    include_intervals_ns: list[tuple[int, int]] = field(default_factory=list)
    exclude_intervals_ns: list[tuple[int, int]] = field(default_factory=list)
    verdict_stationary_dvl: str = ""
    verdict_maneuver_dvl: str = ""


def _time_weighted_lock_fraction(ts_lock: list[tuple[int, bool]]) -> float | None:
    if len(ts_lock) < 2:
        return None
    ts_lock = sorted(ts_lock, key=lambda x: x[0])
    t0, t1 = ts_lock[0][0], ts_lock[-1][0]
    span = t1 - t0
    if span <= 0:
        return None
    locked = 0
    for i in range(len(ts_lock) - 1):
        t_a, lk = ts_lock[i]
        t_b = ts_lock[i + 1][0]
        if lk:
            locked += t_b - t_a
    return locked / span


def _invert_lock_intervals(ts_lock: list[tuple[int, bool]], t_start: int, t_end: int) -> list[tuple[int, int]]:
    """Intervals where lock is False (time-weighted segments)."""
    if not ts_lock or t_end <= t_start:
        return []
    ts_lock = sorted(ts_lock, key=lambda x: x[0])
    out: list[tuple[int, int]] = []
    for i in range(len(ts_lock) - 1):
        t_a, lk = ts_lock[i]
        t_b = ts_lock[i + 1][0]
        if not lk:
            out.append((t_a, t_b))
    return out


def _merge_intervals(iv: list[tuple[int, int]], gap_merge_ns: int = 0) -> list[tuple[int, int]]:
    if not iv:
        return []
    iv = sorted(iv)
    merged = [iv[0]]
    for a, b in iv[1:]:
        la, lb = merged[-1]
        if a <= lb + gap_merge_ns:
            merged[-1] = (la, max(lb, b))
        else:
            merged.append((a, b))
    return merged


def _percentile(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    n = len(sorted_vals)
    if n == 1:
        return sorted_vals[0]
    k = (n - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_vals[int(k)]
    return sorted_vals[f] * (c - k) + sorted_vals[c] * (k - f)


def analyze_bag(bag_dir: Path) -> BagDvlResult:
    name = bag_dir.name
    res = BagDvlResult(bag_name=name, bag_path=str(bag_dir.resolve()))

    dvl_ts_lock: list[tuple[int, bool]] = []
    cov_ts_lock: list[tuple[int, bool]] = []

    with AnyReader([bag_dir]) as reader:
        topics = {c.topic: c.msgtype for c in reader.connections}
        for t, mt in topics.items():
            if t.startswith("/sensors/dvl/"):
                res.topics_dvl_related[t] = mt

        dvl_by_stamp: dict[int, tuple[bool, float, int, float, list[float]]] = {}
        dvl_header_stamps: set[int] = set()

        # Pass 1: Dvl messages (defines velocity-report header stamps).
        for connection, t_ns, raw in reader.messages():
            if connection.topic != TOPIC_VEL:
                continue
            msg = reader.deserialize(raw, connection.msgtype)
            res.n_dvl += 1
            lock = bool(msg.beam_velocities_valid)
            dvl_ts_lock.append((t_ns, lock))
            spd = _dvl_speed(msg)
            ngb = int(msg.num_good_beams)
            res.num_good_beams_hist[ngb] = res.num_good_beams_hist.get(ngb, 0) + 1
            alt = float(msg.altitude)
            ranges = [float(msg.range[i]) for i in range(4)] if hasattr(msg, "range") else []
            if lock:
                res.speed_lock_dvl.append(spd)
                if math.isfinite(alt) and alt > 0:
                    res.altitude_valid_lock.append(alt)
                if ranges:
                    finite = [x for x in ranges if math.isfinite(x) and x > 0]
                    if finite:
                        res.range_min_per_msg.append(min(finite))
            else:
                res.speed_no_lock_dvl.append(spd)
            st = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
            dvl_by_stamp[st] = (lock, spd, ngb, alt, ranges)
            dvl_header_stamps.add(st)

        # Pass 2: odometry_cov (filter out dead-reckoning epochs when Dvl was recorded).
        for connection, t_ns, raw in reader.messages():
            if connection.topic != TOPIC_COV:
                continue
            msg = reader.deserialize(raw, connection.msgtype)
            res.n_cov += 1
            st = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
            if res.n_dvl > 0 and st not in dvl_header_stamps:
                continue
            res.n_cov_at_dvl_stamp += 1
            lk = _odom_lock_from_cov(msg)
            cov_ts_lock.append((t_ns, lk))
            spd = _odom_speed(msg)
            if lk:
                res.speed_lock_cov.append(spd)
            else:
                res.speed_no_lock_cov.append(spd)
            if st in dvl_by_stamp:
                d_lock, _, _, _, _ = dvl_by_stamp[st]
                if d_lock != lk:
                    res.cov_lock_disagree_with_dvl += 1

    if res.n_dvl == 0 and res.n_cov > 0:
        res.flags.append(
            "no_dvl_velocity_messages_in_bag_cov_lock_uses_all_odometry_cov_may_mix_dr_epochs"
        )

    # Duration from union of dvl and cov times
    all_ts: list[int] = [t for t, _ in dvl_ts_lock] + [t for t, _ in cov_ts_lock]
    if len(all_ts) >= 2:
        all_ts.sort()
        res.duration_ns = all_ts[-1] - all_ts[0]

    if dvl_ts_lock:
        res.lock_fraction_time_dvl = _time_weighted_lock_fraction(dvl_ts_lock)
        t0 = min(t for t, _ in dvl_ts_lock)
        t1 = max(t for t, _ in dvl_ts_lock)
        res.lock_false_intervals_ns = _merge_intervals(
            _invert_lock_intervals(dvl_ts_lock, t0, t1)
        )
    if cov_ts_lock:
        res.lock_fraction_time_cov = _time_weighted_lock_fraction(cov_ts_lock)
        if not res.lock_false_intervals_ns:
            t0 = min(t for t, _ in cov_ts_lock)
            t1 = max(t for t, _ in cov_ts_lock)
            res.lock_false_intervals_ns = _merge_intervals(
                _invert_lock_intervals(cov_ts_lock, t0, t1)
            )

    # Flags
    lf = res.lock_fraction_time_dvl
    if lf is None:
        lf = res.lock_fraction_time_cov
    if lf is not None:
        if lf < 0.1:
            res.flags.append("no_lock_most_of_bag")
        elif lf < 0.85:
            res.flags.append("intermittent_lock")

    thr_stationary = 0.05  # m/s — pool noise / bias; tune for reporting
    min_nol_samples = 12  # avoid p95 noise when almost always locked
    for label, speeds in (
        ("dvl_topic", res.speed_no_lock_dvl),
        ("odom_cov", res.speed_no_lock_cov),
    ):
        if len(speeds) < min_nol_samples:
            continue
        s = sorted(speeds)
        p95 = _percentile(s, 95)
        if p95 > thr_stationary:
            res.flags.append(f"suspicious_nonzero_velocity_while_no_lock_{label}_p95_{p95:.4f}_m_s")

    # Covariance sanity when "locked" from cov
    for v in res.speed_lock_cov[:5000]:  # sample cap
        pass
    # Check for negative variances on cov messages — re-scan would be heavy; spot check via separate loop if needed

    # Include / exclude intervals (use Dvl lock if available else cov)
    primary_ts_lock = dvl_ts_lock if dvl_ts_lock else cov_ts_lock
    if primary_ts_lock:
        primary_ts_lock = sorted(primary_ts_lock, key=lambda x: x[0])
        t0 = primary_ts_lock[0][0]
        t1 = primary_ts_lock[-1][0]
        # include: merged True segments
        inc: list[tuple[int, int]] = []
        for i in range(len(primary_ts_lock) - 1):
            ta, lk = primary_ts_lock[i]
            tb = primary_ts_lock[i + 1][0]
            if lk:
                inc.append((ta, tb))
        res.include_intervals_ns = _merge_intervals(inc)
        res.exclude_intervals_ns = _merge_intervals(_invert_lock_intervals(primary_ts_lock, t0, t1))

    # Verdicts
    if lf is None:
        res.verdict_stationary_dvl = "unknown (no DVL lock signal in bag)"
        res.verdict_maneuver_dvl = "unknown"
    elif lf >= 0.85 and not any("suspicious_nonzero" in f for f in res.flags):
        res.verdict_stationary_dvl = "good_enough_for_stationary_noise_if_imu_is_primary_dvl_secondary"
        res.verdict_maneuver_dvl = "good_enough_for_simple_maneuver_checks"
    elif lf >= 0.5:
        res.verdict_stationary_dvl = "marginal_prefer_segments_with_lock"
        res.verdict_maneuver_dvl = "marginal_use_include_intervals_only"
    else:
        res.verdict_stationary_dvl = "poor_do_not_trust_dvl_for_stationary_analysis"
        res.verdict_maneuver_dvl = "poor_exclude_or_short_windows_only"

    if res.cov_lock_disagree_with_dvl > max(5, res.n_dvl // 50):
        res.flags.append("many_cov_vs_dvl_lock_disagreements")

    return res


def _fmt_iv_ns(iv: list[tuple[int, int]], max_show: int = 8) -> str:
    if not iv:
        return "(none)"
    parts = []
    for a, b in iv[:max_show]:
        parts.append(f"{(b-a)/1e9:.2f}s @ {a/1e9:.3f}..{b/1e9:.3f}s_rel")
    if len(iv) > max_show:
        parts.append(f"... +{len(iv)-max_show} more")
    return "; ".join(parts)


def result_to_dict(r: BagDvlResult) -> dict[str, Any]:
    def pct(xs: list[float], p: float) -> float:
        if not xs:
            return float("nan")
        return _percentile(sorted(xs), p)

    alt = r.altitude_valid_lock
    alt_s = sorted(alt) if alt else []
    beams_hist = dict(sorted(r.num_good_beams_hist.items()))

    return {
        "bag_name": r.bag_name,
        "bag_path": r.bag_path,
        "topics_dvl_related": r.topics_dvl_related,
        "duration_sec": r.duration_ns / 1e9 if r.duration_ns else 0.0,
        "dvl_velocity_messages": r.n_dvl,
        "odometry_cov_messages_total": r.n_cov,
        "odometry_cov_messages_at_dvl_velocity_stamp": r.n_cov_at_dvl_stamp,
        "lock_fraction_time_weighted": {
            "from_dvl_beam_velocities_valid": r.lock_fraction_time_dvl,
            "from_odometry_cov_covariance": r.lock_fraction_time_cov,
        },
        "periods_without_lock_sec_total": sum((b - a) / 1e9 for a, b in r.lock_false_intervals_ns),
        "num_good_beams_histogram": beams_hist,
        "altitude_m_when_lock_valid": {
            "n": len(alt),
            "min": min(alt) if alt else None,
            "max": max(alt) if alt else None,
            "p50": _percentile(alt_s, 50) if alt_s else None,
        },
        "speed_m_s_when_no_lock": {
            "dvl_topic_p50": pct(r.speed_no_lock_dvl, 50),
            "dvl_topic_p95": pct(r.speed_no_lock_dvl, 95),
            "odom_cov_p50": pct(r.speed_no_lock_cov, 50),
            "odom_cov_p95": pct(r.speed_no_lock_cov, 95),
        },
        "speed_m_s_when_lock": {
            "dvl_topic_p50": pct(r.speed_lock_dvl, 50),
            "dvl_topic_p95": pct(r.speed_lock_dvl, 95),
            "odom_cov_p50": pct(r.speed_lock_cov, 50),
            "odom_cov_p95": pct(r.speed_lock_cov, 95),
        },
        "cov_vs_dvl_lock_disagreements_at_shared_stamp": r.cov_lock_disagree_with_dvl,
        "flags": r.flags,
        "intervals_include_lock_true_ns": r.include_intervals_ns[:20],
        "intervals_exclude_lock_false_ns": r.exclude_intervals_ns[:20],
        "verdict": {
            "stationary_noise_analysis": r.verdict_stationary_dvl,
            "simple_maneuver_checks": r.verdict_maneuver_dvl,
        },
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="DVL lock and data health for raw MCAP rosbags.")
    ap.add_argument("root", type=Path, nargs="?", default=Path("recordings/rosbags"))
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    bags = find_raw_bag_dirs(args.root.resolve())
    if not bags:
        print("No raw MCAP bag directories found.", file=sys.stderr)
        sys.exit(1)

    results = []
    for b in bags:
        try:
            results.append(analyze_bag(b))
        except Exception as e:
            print(f"WARNING: skipping {b.name} — {e}", file=sys.stderr)
            r = BagDvlResult(bag_name=b.name, bag_path=str(b.resolve()))
            r.flags.append(f"unreadable: {e}")
            results.append(r)

    if args.json:
        print(json.dumps([result_to_dict(r) for r in results], indent=2))
        return

    for r in results:
        d = result_to_dict(r)
        print(f"\n=== {r.bag_name} ===")
        print(f"duration_s: {d['duration_sec']:.2f}")
        print("dvl_topics:", ", ".join(d["topics_dvl_related"]) or "(none)")
        print(
            f"Dvl msgs: {r.n_dvl}  odometry_cov: total={r.n_cov} "
            f"used_for_lock={r.n_cov_at_dvl_stamp}"
        )
        lf = d["lock_fraction_time_weighted"]
        print(f"lock fraction (time-weighted): dvl={lf['from_dvl_beam_velocities_valid']} cov={lf['from_odometry_cov_covariance']}")
        print(f"time without lock (sum of segments): {d['periods_without_lock_sec_total']:.2f} s")
        print(f"num_good_beams hist: {d['num_good_beams_histogram']}")
        am = d["altitude_m_when_lock_valid"]
        print(f"altitude (lock valid): n={am['n']} min/max/p50={am['min']}/{am['max']}/{am['p50']}")
        print(f"speed no-lock p95 dvl/odom: {d['speed_m_s_when_no_lock']['dvl_topic_p95']}/{d['speed_m_s_when_no_lock']['odom_cov_p95']}")
        print(f"speed lock p95 dvl/odom: {d['speed_m_s_when_lock']['dvl_topic_p95']}/{d['speed_m_s_when_lock']['odom_cov_p95']}")
        print(f"flags: {d['flags'] or '(none)'}")
        print(f"verdict stationary: {d['verdict']['stationary_noise_analysis']}")
        print(f"verdict maneuver: {d['verdict']['simple_maneuver_checks']}")


if __name__ == "__main__":
    main()
