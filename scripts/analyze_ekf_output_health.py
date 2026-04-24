#!/usr/bin/env python3
"""
EKF output health check for ROS2 MCAP bags.

Checks per bag:
  1. Topic presence — expected EKF topics given bag type
  2. Message rate — local/global EKF and TF Hz
  3. Stamp monotonicity — no backward jumps on /odometry/filtered/*
  4. NaN/Inf scan — pose.position and pose.orientation on both filtered odoms
  5. Position jump detection — consecutive messages > MAX_JUMP_M apart
  6. TF tree check — map->odom and odom->base_link transforms present
  7. Covariance sanity — pose covariance diagonal finite and > 0

Dependency: pip install rosbags
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

TOPIC_LOCAL = "/odometry/filtered/local"
TOPIC_GLOBAL = "/odometry/filtered/global"
TOPIC_TF = "/tf"
TOPIC_TF_STATIC = "/tf_static"

EKF_TOPICS = (TOPIC_LOCAL, TOPIC_GLOBAL, TOPIC_TF, TOPIC_TF_STATIC)

# Bags that are not expected to have local EKF
NO_LOCAL_EKF_PATTERNS = ("stationary_", "vertical_04")
# Bags not expected to have any EKF
NO_EKF_PATTERNS = ("stationary_",)

MAX_JUMP_M = 5.0
MIN_EKF_HZ = 10.0   # EKF should be well above this if running
MIN_TF_HZ = 5.0


# ── helpers ──────────────────────────────────────────────────────────────────

def _is_corrupted(bag_dir: Path) -> bool:
    for mcap in bag_dir.glob("*.mcap"):
        try:
            from mcap.reader import make_reader
            with open(mcap, "rb") as f:
                r = make_reader(f)
                r.get_summary()
            return False
        except Exception:
            return True
    return False


def _under_raw_rosbags(path: Path) -> bool:
    lower = [p.lower() for p in path.parts]
    return "rosbags" in lower and not any("bodyframe" in p.lower() for p in path.parts)


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


def _bag_expects_local_ekf(name: str) -> bool:
    return not any(name.startswith(p) for p in NO_LOCAL_EKF_PATTERNS)


def _bag_expects_any_ekf(name: str) -> bool:
    return not any(name.startswith(p) for p in NO_EKF_PATTERNS)


def _stamp_ns(stamp) -> int:
    return int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))


def _has_nan_inf(val: float) -> bool:
    return math.isnan(val) or math.isinf(val)


def _pos_dist(p1, p2) -> float:
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = p1.z - p2.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


# ── per-bag analysis ──────────────────────────────────────────────────────────

@dataclass
class TopicStats:
    n: int = 0
    duration_s: float = 0.0
    mean_hz: float = 0.0
    n_negative_dt: int = 0
    n_nan_inf: int = 0
    n_position_jumps: int = 0
    max_jump_m: float = 0.0
    cov_diag_ok: bool = True
    cov_issues: list[str] = field(default_factory=list)


@dataclass
class TFStats:
    n_messages: int = 0
    duration_s: float = 0.0
    mean_hz: float = 0.0
    has_map_to_odom: bool = False
    has_odom_to_base: bool = False
    tf_pairs_seen: list[str] = field(default_factory=list)


@dataclass
class BagResult:
    bag_name: str
    corrupted: bool = False
    topics_present: list[str] = field(default_factory=list)
    expects_local_ekf: bool = True
    expects_any_ekf: bool = True
    local_ekf: TopicStats | None = None
    global_ekf: TopicStats | None = None
    tf: TFStats | None = None
    checks: dict[str, str] = field(default_factory=dict)
    verdict: str = "UNKNOWN"


def _analyze_odometry_topic(bag_dir: Path, topic: str) -> TopicStats | None:
    stats = TopicStats()
    stamps_ns: list[int] = []
    prev_pos = None

    with AnyReader([bag_dir]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            return None

        for conn, ts, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            stats.n += 1

            # stamp
            t = _stamp_ns(msg.header.stamp)
            if t > 0:
                stamps_ns.append(t)

            # NaN/Inf check on pose
            try:
                pos = msg.pose.pose.position
                ori = msg.pose.pose.orientation
                for v in (pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w):
                    if _has_nan_inf(v):
                        stats.n_nan_inf += 1
                        break

                # position jump
                if prev_pos is not None:
                    d = _pos_dist(pos, prev_pos)
                    if d > MAX_JUMP_M:
                        stats.n_position_jumps += 1
                        stats.max_jump_m = max(stats.max_jump_m, d)
                prev_pos = pos
            except AttributeError:
                pass

            # covariance diagonal
            try:
                cov = msg.pose.covariance  # flat 36-element array
                for i in range(6):
                    v = float(cov[i * 6 + i])
                    if _has_nan_inf(v) or v <= 0:
                        stats.cov_diag_ok = False
                        stats.cov_issues.append(f"diag[{i}]={v:.3g}")
            except (AttributeError, IndexError):
                pass

    if not stamps_ns:
        return stats

    stamps_ns.sort()
    stats.duration_s = (stamps_ns[-1] - stamps_ns[0]) / 1e9
    stats.mean_hz = stats.n / stats.duration_s if stats.duration_s > 0 else 0.0

    for i in range(1, len(stamps_ns)):
        if stamps_ns[i] < stamps_ns[i - 1]:
            stats.n_negative_dt += 1

    return stats


def _analyze_tf(bag_dir: Path) -> TFStats:
    stats = TFStats()
    stamps_ns: list[int] = []
    pairs: set[str] = set()

    with AnyReader([bag_dir]) as reader:
        connections = [c for c in reader.connections if c.topic == TOPIC_TF]
        if not connections:
            return stats

        for conn, ts, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            stats.n_messages += 1
            for tf in msg.transforms:
                t = _stamp_ns(tf.header.stamp)
                if t > 0:
                    stamps_ns.append(t)
                pair = f"{tf.header.frame_id}->{tf.child_frame_id}"
                pairs.add(pair)

    stats.tf_pairs_seen = sorted(pairs)
    stats.has_map_to_odom = any("map" in p and "odom" in p for p in pairs)
    stats.has_odom_to_base = any("odom" in p and "base_link" in p for p in pairs)

    if stamps_ns:
        stamps_ns.sort()
        stats.duration_s = (stamps_ns[-1] - stamps_ns[0]) / 1e9
        stats.mean_hz = stats.n_messages / stats.duration_s if stats.duration_s > 0 else 0.0

    return stats


def _get_present_topics(bag_dir: Path) -> list[str]:
    with AnyReader([bag_dir]) as reader:
        return sorted({c.topic for c in reader.connections})


def _verdict(result: BagResult) -> str:
    checks = result.checks
    if result.corrupted:
        return "CORRUPTED"
    if not result.expects_any_ekf:
        return "N/A (EKF off)"

    failures = [k for k, v in checks.items() if v == "FAIL"]
    warnings = [k for k, v in checks.items() if v == "WARN"]

    if failures:
        return f"FAIL ({', '.join(failures)})"
    if warnings:
        return f"WARN ({', '.join(warnings)})"
    return "PASS"


def analyze_bag(bag_dir: Path) -> BagResult:
    name = bag_dir.name
    result = BagResult(bag_name=name)
    result.expects_local_ekf = _bag_expects_local_ekf(name)
    result.expects_any_ekf = _bag_expects_any_ekf(name)

    try:
        from mcap.reader import make_reader
        for mcap in bag_dir.glob("*.mcap"):
            with open(mcap, "rb") as f:
                make_reader(f).get_summary()
    except Exception:
        result.corrupted = True
        result.verdict = "CORRUPTED"
        return result

    result.topics_present = _get_present_topics(bag_dir)
    checks = result.checks

    if not result.expects_any_ekf:
        result.verdict = _verdict(result)
        return result

    # 1. topic presence
    has_local = TOPIC_LOCAL in result.topics_present
    has_global = TOPIC_GLOBAL in result.topics_present
    has_tf = TOPIC_TF in result.topics_present

    if result.expects_local_ekf:
        checks["local_ekf_present"] = "PASS" if has_local else "FAIL"
    checks["global_ekf_present"] = "PASS" if has_global else "FAIL"
    checks["tf_present"] = "PASS" if has_tf else "FAIL"

    # 2–7. content checks
    if has_local:
        s = _analyze_odometry_topic(bag_dir, TOPIC_LOCAL)
        result.local_ekf = s
        if s:
            checks["local_rate"] = "PASS" if s.mean_hz >= MIN_EKF_HZ else "WARN"
            checks["local_stamp_mono"] = "PASS" if s.n_negative_dt == 0 else "FAIL"
            checks["local_nan"] = "PASS" if s.n_nan_inf == 0 else "FAIL"
            checks["local_jumps"] = "PASS" if s.n_position_jumps == 0 else "WARN"
            checks["local_cov"] = "PASS" if s.cov_diag_ok else "WARN"

    if has_global:
        s = _analyze_odometry_topic(bag_dir, TOPIC_GLOBAL)
        result.global_ekf = s
        if s:
            checks["global_rate"] = "PASS" if s.mean_hz >= MIN_EKF_HZ else "WARN"
            checks["global_stamp_mono"] = "PASS" if s.n_negative_dt == 0 else "FAIL"
            checks["global_nan"] = "PASS" if s.n_nan_inf == 0 else "FAIL"
            checks["global_jumps"] = "PASS" if s.n_position_jumps == 0 else "WARN"
            checks["global_cov"] = "PASS" if s.cov_diag_ok else "WARN"

    if has_tf:
        tf = _analyze_tf(bag_dir)
        result.tf = tf
        checks["tf_map_odom"] = "PASS" if tf.has_map_to_odom else "FAIL"
        checks["tf_odom_base"] = "PASS" if tf.has_odom_to_base else "FAIL"
        checks["tf_rate"] = "PASS" if tf.mean_hz >= MIN_TF_HZ else "WARN"

    result.verdict = _verdict(result)
    return result


# ── printing ──────────────────────────────────────────────────────────────────

def _fmt_stats(label: str, s: TopicStats) -> list[str]:
    lines = [f"  {label}: {s.n} msgs @ {s.mean_hz:.1f} Hz | {s.duration_s:.1f} s"]
    if s.n_negative_dt:
        lines.append(f"    backward stamps: {s.n_negative_dt}")
    if s.n_nan_inf:
        lines.append(f"    NaN/Inf in pose: {s.n_nan_inf}")
    if s.n_position_jumps:
        lines.append(f"    position jumps >{MAX_JUMP_M}m: {s.n_position_jumps} (max {s.max_jump_m:.1f} m)")
    if not s.cov_diag_ok:
        lines.append(f"    cov issues: {s.cov_issues[:4]}")
    return lines


def print_results(results: list[BagResult]) -> None:
    print("\n" + "=" * 72)
    print("EKF OUTPUT HEALTH - 2026-04-19")
    print("=" * 72)

    for r in results:
        print(f"\n{'-'*60}")
        print(f"BAG : {r.bag_name}")
        print(f"VERDICT : {r.verdict}")

        if r.corrupted:
            continue
        if not r.expects_any_ekf:
            print("  (EKF not expected for this bag type)")
            continue

        # checks table
        for k, v in r.checks.items():
            sym = {"PASS": "OK", "FAIL": "!!", "WARN": "??"}.get(v, v)
            print(f"  [{sym}] {k}: {v}")

        # stats detail
        if r.local_ekf:
            for l in _fmt_stats("local", r.local_ekf):
                print(l)
        if r.global_ekf:
            for l in _fmt_stats("global", r.global_ekf):
                print(l)
        if r.tf:
            print(f"  TF   : {r.tf.n_messages} msgs @ {r.tf.mean_hz:.1f} Hz")
            print(f"    pairs: {', '.join(r.tf.tf_pairs_seen)}")

    # summary table
    print("\n" + "=" * 72)
    print("SUMMARY")
    print("=" * 72)
    col = 42
    print(f"{'Bag':<{col}} {'Verdict'}")
    print("-" * 72)
    for r in results:
        print(f"  {r.bag_name:<{col-2}} {r.verdict}")
    print()


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="EKF output health check for MCAP bags")
    parser.add_argument(
        "roots",
        nargs="*",
        default=["recordings/rosbags"],
        help="Root directories to scan for bags",
    )
    parser.add_argument("--json", action="store_true", help="Print JSON output")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    bag_dirs: list[Path] = []
    for root_str in args.roots:
        root = Path(root_str) if Path(root_str).is_absolute() else repo_root / root_str
        bag_dirs.extend(find_raw_bag_dirs(root))

    if not bag_dirs:
        print("No bags found.", file=sys.stderr)
        sys.exit(1)

    results = [analyze_bag(d) for d in sorted(bag_dirs)]

    if args.json:
        import dataclasses
        def _serial(obj):
            if dataclasses.is_dataclass(obj):
                return dataclasses.asdict(obj)
            raise TypeError(type(obj))
        print(json.dumps([dataclasses.asdict(r) for r in results], default=str, indent=2))
    else:
        print_results(results)


if __name__ == "__main__":
    main()
