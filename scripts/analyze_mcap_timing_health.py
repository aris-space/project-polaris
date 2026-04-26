#!/usr/bin/env python3
"""
Per-topic timing health for raw ROS 2 MCAP bags (recordings/rosbags).

Uses MCAP envelope log_time in file (recording) order (not message header.stamp); excludes __bodyframe paths.
Dependency: pip install mcap

Computes rates, inter-message dt, monotonicity, duplicates, gaps, bursts, startup vs rest.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    from mcap.reader import make_reader
except ImportError:
    print("Install: pip install mcap", file=sys.stderr)
    raise

# Ordered: report these first; other topics in bag are skipped unless --all-topics
PRIORITY_TOPICS: tuple[str, ...] = (
    "/imu/data",
    "/sensors/dvl/odometry_cov",
    "/sensors/dvl/odometry",
    "/pixhawk/scaled_pressure",
    "/joy",
    "/joy_controller",
    "/joy_keyboard",
    "/pixhawk/manual_control",
    "/tf",
    "/tf_static",
)

MANUAL_TOPICS = frozenset(
    {"/joy", "/joy_controller", "/joy_keyboard", "/pixhawk/manual_control"}
)

STARTUP_NS = int(5e9)  # first 5 s from first message on this topic (log_time)
DEFAULT_ROOTS: tuple[str, ...] = (
    "recordings/rosbags",
)


def _is_raw_rosbag_mcap(p: Path) -> bool:
    parts = {x.lower() for x in p.parts}
    if "__bodyframe__" in str(p) or any("__bodyframe" in part for part in p.parts):
        return False
    return p.name.endswith("_0.mcap") and "rosbags" in p.parts


def _percentile(sorted_vals: list[float], q: float) -> float | None:
    """q in [0, 100]. Linear interpolation between closest ranks."""
    if not sorted_vals:
        return None
    if len(sorted_vals) == 1:
        return float(sorted_vals[0])
    pos = (q / 100.0) * (len(sorted_vals) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return float(sorted_vals[lo])
    w = pos - lo
    return float(sorted_vals[lo] * (1 - w) + sorted_vals[hi] * w)


def _fmt_float(x: float | None, nd: int = 6) -> str:
    if x is None:
        return "n/a"
    if math.isnan(x) or math.isinf(x):
        return str(x)
    return f"{x:.{nd}g}"


@dataclass
class TopicTimingReport:
    topic: str
    n: int
    duration_s: float
    mean_hz: float | None
    median_hz: float | None
    dt_mean_s: float | None
    dt_median_s: float | None
    dt_std_s: float | None
    dt_min_s: float | None
    dt_max_s: float | None
    dt_p90_s: float | None
    dt_p95_s: float | None
    dt_p99_s: float | None
    monotonic_nondecreasing: bool
    n_negative_dt: int
    n_zero_dt: int
    n_gap: int
    n_burst: int
    max_gap_s: float | None
    min_positive_dt_s: float | None
    gap_threshold_s: float | None
    burst_threshold_s: float | None
    issues_in_startup: dict[str, int] = field(default_factory=dict)
    issues_after_startup: dict[str, int] = field(default_factory=dict)
    note: str = ""
    suggestion: str = "OK"
    flags: list[str] = field(default_factory=list)

    def to_json_dict(self) -> dict[str, Any]:
        d = self.__dict__.copy()
        return d


def _collect_log_times(path: Path, topics_filter: set[str] | None) -> dict[str, list[int]]:
    """topic -> log_time sequence in MCAP iteration (recording) order."""
    out: dict[str, list[int]] = {}
    with open(path, "rb") as f:
        reader = make_reader(f)
        for _sch, ch, msg in reader.iter_messages():
            if topics_filter is not None and ch.topic not in topics_filter:
                continue
            out.setdefault(ch.topic, []).append(msg.log_time)
    return out


def _median_positive(vals: list[float]) -> float | None:
    pos = [v for v in vals if v > 0]
    if not pos:
        return None
    return float(statistics.median(pos))


def analyze_topic_times(topic: str, times_ns: list[int]) -> TopicTimingReport:
    n = len(times_ns)
    flags: list[str] = []
    note_parts: list[str] = []

    if n == 0:
        return TopicTimingReport(
            topic=topic,
            n=0,
            duration_s=0.0,
            mean_hz=None,
            median_hz=None,
            dt_mean_s=None,
            dt_median_s=None,
            dt_std_s=None,
            dt_min_s=None,
            dt_max_s=None,
            dt_p90_s=None,
            dt_p95_s=None,
            dt_p99_s=None,
            monotonic_nondecreasing=True,
            n_negative_dt=0,
            n_zero_dt=0,
            n_gap=0,
            n_burst=0,
            max_gap_s=None,
            min_positive_dt_s=None,
            gap_threshold_s=None,
            burst_threshold_s=None,
            note="no messages",
            suggestion="OK",
            flags=["absent"],
        )

    t0 = times_ns[0]
    t_end = times_ns[-1]
    duration_s = (t_end - t0) / 1e9

    # Latched / very few samples (tf_static): dt stats mostly meaningless
    if n < 2:
        suggestion = "OK"
        if topic in ("/tf_static", "/tf") and n == 1:
            note = "single message (latched or absent stream)"
        elif n == 1:
            note = "single message"
            suggestion = "inspect manually"
        return TopicTimingReport(
            topic=topic,
            n=n,
            duration_s=max(duration_s, 0.0),
            mean_hz=None,
            median_hz=None,
            dt_mean_s=None,
            dt_median_s=None,
            dt_std_s=None,
            dt_min_s=None,
            dt_max_s=None,
            dt_p90_s=None,
            dt_p95_s=None,
            dt_p99_s=None,
            monotonic_nondecreasing=True,
            n_negative_dt=0,
            n_zero_dt=0,
            n_gap=0,
            n_burst=0,
            max_gap_s=None,
            min_positive_dt_s=None,
            gap_threshold_s=None,
            burst_threshold_s=None,
            note=note or "",
            suggestion=suggestion,
            flags=[] if n > 0 else ["absent"],
        )

    dts_s: list[float] = []
    for i in range(1, n):
        dts_s.append((times_ns[i] - times_ns[i - 1]) / 1e9)

    n_neg = sum(1 for d in dts_s if d < 0)
    n_zero = sum(1 for d in dts_s if d == 0)
    monotonic = n_neg == 0

    pos_dts = [d for d in dts_s if d > 0]
    med_pos = _median_positive(dts_s)
    # Gap: freeze if much longer than typical stream (sensor); static TF uses huge threshold
    if topic == "/tf_static" and n <= 4:
        gap_thr = max(1.0, duration_s * 2) if duration_s > 0 else 3600.0
        burst_thr = None
        note_parts.append("few /tf_static samples: dt thresholds are relaxed")
    elif topic == "/tf" and n < 30:
        gap_thr = max(2.0, 30.0 * (med_pos or 0.1))
        burst_thr = (med_pos * 0.05) if med_pos and med_pos > 1e-6 else 1e-4
    else:
        gap_thr = max(0.5, 40.0 * (med_pos or 0.01)) if med_pos else max(0.5, duration_s * 0.5)
        burst_thr = (med_pos * 0.08) if med_pos and med_pos > 1e-9 else None

    n_gap = 0
    max_gap: float | None = None
    if med_pos is not None:
        for d in dts_s:
            if d > gap_thr:
                n_gap += 1
                max_gap = d if max_gap is None else max(max_gap, d)
    elif topic not in ("/tf_static",):
        for d in dts_s:
            if d > 1.0:
                n_gap += 1
                max_gap = d if max_gap is None else max(max_gap, d)

    n_burst = 0
    min_pos_dt: float | None = min(pos_dts) if pos_dts else None
    if burst_thr is not None and burst_thr > 0:
        n_burst = sum(1 for d in pos_dts if d < burst_thr)

    sorted_pos = sorted(pos_dts)
    dt_mean = float(statistics.mean(dts_s)) if dts_s else None
    dt_med = float(statistics.median(dts_s)) if dts_s else None
    dt_std = float(statistics.pstdev(dts_s)) if len(dts_s) > 1 else 0.0

    mean_hz = (n - 1) / duration_s if duration_s > 1e-9 else None
    # When dt CV is high, many tiny MCAP log_time gaps make 1/median(dt) meaningless; align median_hz with mean.
    median_hz: float | None = None
    if pos_dts and dt_mean and dt_mean > 1e-12:
        cv_dt = dt_std / dt_mean if dt_std is not None else 0.0
        med_dt = float(statistics.median(pos_dts))
        if cv_dt < 1.0 and med_dt > 1e-12:
            median_hz = 1.0 / med_dt
        elif mean_hz is not None:
            median_hz = mean_hz
            if cv_dt >= 1.0:
                note_parts.append("bursty log_time spacing: median Hz set to mean Hz")

    # Startup vs rest: attribute each dt to the *later* message timestamp
    neg_su = neg_rest = zero_su = zero_rest = gap_su = gap_rest = burst_su = burst_rest = 0
    startup_end = t0 + STARTUP_NS
    for i, d in enumerate(dts_s):
        t_later = times_ns[i + 1]
        in_su = t_later <= startup_end
        if d < 0:
            if in_su:
                neg_su += 1
            else:
                neg_rest += 1
        elif d == 0:
            if in_su:
                zero_su += 1
            else:
                zero_rest += 1
        else:
            if med_pos is not None and d > gap_thr:
                if in_su:
                    gap_su += 1
                else:
                    gap_rest += 1
            if burst_thr is not None and d > 0 and d < burst_thr:
                if in_su:
                    burst_su += 1
                else:
                    burst_rest += 1

    issues_in_startup = {
        "negative_dt": neg_su,
        "zero_dt": zero_su,
        "gap": gap_su,
        "burst": burst_su,
    }
    issues_after_startup = {
        "negative_dt": neg_rest,
        "zero_dt": zero_rest,
        "gap": gap_rest,
        "burst": burst_rest,
    }

    if n_neg:
        flags.append("non_monotonic_or_out_of_order")
    if n_zero:
        flags.append("duplicate_log_time")
    if n_gap and topic not in ("/tf_static",):
        flags.append("long_gaps")
    if n_burst and topic not in ("/tf_static",) and (med_pos or 0) < 1.0:
        flags.append("micro_bursts")

    # Suggestions
    suggestion = "OK"
    severe_rest = neg_rest > 0 or (gap_rest > max(2, n * 0.05) and topic not in MANUAL_TOPICS)
    startup_skew = (neg_su + gap_su + zero_su) > 0 and (neg_rest + gap_rest + zero_rest) == 0

    if n_neg and neg_rest > 0:
        suggestion = "exclude"
        flags.append("throughout_time_regression")
    elif n_neg and neg_su > 0 and neg_rest == 0:
        suggestion = "ignore startup segment"
    elif severe_rest:
        suggestion = "use with caution"
    elif n_gap and gap_rest > 0 and gap_su == 0:
        # Joystick streams: brief pauses are normal; only flag large freezes.
        if topic in MANUAL_TOPICS:
            if (max_gap or 0) > 3.0 or gap_rest >= 5:
                suggestion = "use with caution"
        else:
            suggestion = "use with caution"
    elif startup_skew and (n_gap or n_neg or n_zero):
        suggestion = "ignore startup segment"
    elif n_zero and zero_rest > max(1, n * 0.01):
        suggestion = "use with caution"
        flags.append("duplicate_timestamps_after_startup")
    elif n_burst and burst_rest > n * 0.15 and topic in ("/imu/data", "/sensors/dvl/odometry_cov", "/sensors/dvl/odometry"):
        suggestion = "inspect manually"

    if topic in ("/tf_static",) and n <= 4 and not flags:
        suggestion = "OK"
        note_parts.append("static TF: rate metrics not applicable")

    if not monotonic and suggestion == "OK":
        suggestion = "inspect manually"

    return TopicTimingReport(
        topic=topic,
        n=n,
        duration_s=round(duration_s, 6),
        mean_hz=round(mean_hz, 4) if mean_hz is not None else None,
        median_hz=round(median_hz, 4) if median_hz is not None else None,
        dt_mean_s=round(dt_mean, 9) if dt_mean is not None else None,
        dt_median_s=round(dt_med, 9) if dt_med is not None else None,
        dt_std_s=round(dt_std, 9) if dt_std is not None else None,
        dt_min_s=round(min(dts_s), 9),
        dt_max_s=round(max(dts_s), 9),
        dt_p90_s=round(_percentile(sorted_pos, 90), 9) if sorted_pos else None,
        dt_p95_s=round(_percentile(sorted_pos, 95), 9) if sorted_pos else None,
        dt_p99_s=round(_percentile(sorted_pos, 99), 9) if sorted_pos else None,
        monotonic_nondecreasing=monotonic,
        n_negative_dt=n_neg,
        n_zero_dt=n_zero,
        n_gap=n_gap,
        n_burst=n_burst,
        max_gap_s=round(max_gap, 6) if max_gap is not None else None,
        min_positive_dt_s=round(min_pos_dt, 9) if min_pos_dt is not None else None,
        gap_threshold_s=round(gap_thr, 6),
        burst_threshold_s=round(burst_thr, 9) if burst_thr is not None else None,
        issues_in_startup=issues_in_startup,
        issues_after_startup=issues_after_startup,
        note="; ".join(note_parts),
        suggestion=suggestion,
        flags=flags,
    )


def analyze_bag(mcap_path: Path, all_topics: bool) -> dict[str, Any]:
    topics_filter: set[str] | None
    if all_topics:
        topics_filter = None
    else:
        topics_filter = set(PRIORITY_TOPICS)

    collected = _collect_log_times(mcap_path, topics_filter)

    topic_list = list(PRIORITY_TOPICS) if not all_topics else sorted(collected.keys())
    reports: dict[str, TopicTimingReport] = {}
    for t in topic_list:
        if not all_topics and t not in collected:
            reports[t] = analyze_topic_times(t, [])
            continue
        if t not in collected:
            continue
        reports[t] = analyze_topic_times(t, collected[t])

    bag_suggestion = "OK"
    flagged_topics: list[str] = []
    for t, r in reports.items():
        if r.n == 0:
            continue
        if r.suggestion != "OK":
            flagged_topics.append(t)
        if r.suggestion == "exclude":
            bag_suggestion = "exclude"
        elif r.suggestion == "inspect manually" and bag_suggestion not in ("exclude",):
            bag_suggestion = "inspect manually"
        elif r.suggestion == "use with caution" and bag_suggestion in ("OK", "ignore startup segment"):
            bag_suggestion = "use with caution"
        elif r.suggestion == "ignore startup segment" and bag_suggestion == "OK":
            bag_suggestion = "ignore startup segment"

    return {
        "bag_name": mcap_path.parent.name,
        "mcap_path": str(mcap_path),
        "topics": {k: v.to_json_dict() for k, v in reports.items()},
        "bag_suggestion": bag_suggestion,
        "flagged_topics": flagged_topics,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="MCAP log_time timing health (raw rosbags only).")
    ap.add_argument(
        "roots",
        nargs="*",
        default=list(DEFAULT_ROOTS),
        help="Roots to scan for *_0.mcap (default: recordings/rosbags)",
    )
    ap.add_argument("--all-topics", action="store_true", help="Analyze every topic (slow, large output)")
    ap.add_argument("--json", action="store_true", help="Print JSON only")
    args = ap.parse_args()

    mcaps: list[Path] = []
    for r in args.roots:
        mcaps.extend(p for p in Path(r).rglob("*_0.mcap") if _is_raw_rosbag_mcap(p))
    mcaps = sorted(set(mcaps))
    if not mcaps:
        print("No raw *_0.mcap files under rosbags (excluding __bodyframe).", file=sys.stderr)
        return 1

    results = []
    for p in mcaps:
        try:
            results.append(analyze_bag(p, args.all_topics))
        except Exception as e:
            print(f"WARNING: skipping {p.parent.name} — {e}", file=sys.stderr)
            results.append({"bag_name": p.parent.name, "mcap_path": str(p), "topics": {}, "bag_suggestion": "unreadable", "flagged_topics": [], "error": str(e)})

    if args.json:
        print(json.dumps(results, indent=2))
        return 0

    # Human report
    print("# MCAP timing health (MCAP log_time, recording order)\n")
    print("Source: raw bags under `recordings/rosbags` (not `__bodyframe`).\n")

    global_flagged_bags: list[str] = []
    for bag in results:
        name = bag["bag_name"]
        if bag["bag_suggestion"] != "OK":
            global_flagged_bags.append(name)

    print("## Flagged bags (non-OK suggestion)\n")
    if not global_flagged_bags:
        print("- None\n")
    else:
        for bn in global_flagged_bags:
            b = next(x for x in results if x["bag_name"] == bn)
            print(f"- **{bn}** -> *{b['bag_suggestion']}*; topics: {', '.join(b['flagged_topics']) or '-'}")
        print()

    print("## Per-bag / per-topic summary\n")
    for bag in results:
        print(f"### {bag['bag_name']}\n")
        print(f"- **Bag action:** {bag['bag_suggestion']}")
        if bag["flagged_topics"]:
            print(f"- **Flagged topics:** {', '.join(bag['flagged_topics'])}")
        print()
        for tname in PRIORITY_TOPICS:
            tr = bag["topics"].get(tname)
            if not tr:
                continue
            if tr["n"] == 0:
                print(f"- `{tname}`: *absent*")
                continue
            print(f"- `{tname}`: **n={tr['n']}**, duration **{tr['duration_s']:.3f} s**, "
                  f"mean **{_fmt_float(tr['mean_hz'], 4)} Hz**, median **{_fmt_float(tr['median_hz'], 4)} Hz**")
            if tr["n"] >= 2:
                print(
                    f"  - dt (s): mean {_fmt_float(tr['dt_mean_s'])}, median {_fmt_float(tr['dt_median_s'])}, "
                    f"std {_fmt_float(tr['dt_std_s'])}, min {_fmt_float(tr['dt_min_s'])}, "
                    f"max {_fmt_float(tr['dt_max_s'])}, "
                    f"p90/p95/p99 {_fmt_float(tr['dt_p90_s'])}/{_fmt_float(tr['dt_p95_s'])}/{_fmt_float(tr['dt_p99_s'])}"
                )
            print(
                f"  - monotonic: {tr['monotonic_nondecreasing']}, "
                f"neg_dt={tr['n_negative_dt']}, zero_dt={tr['n_zero_dt']}, "
                f"gaps>{_fmt_float(tr['gap_threshold_s'])}s: {tr['n_gap']}, "
                f"bursts<{_fmt_float(tr['burst_threshold_s'])}s: {tr['n_burst']}"
            )
            if tr["n"] >= 2:
                print(
                    f"  - issues startup / after: "
                    f"neg {tr['issues_in_startup']['negative_dt']}/{tr['issues_after_startup']['negative_dt']}, "
                    f"zero {tr['issues_in_startup']['zero_dt']}/{tr['issues_after_startup']['zero_dt']}, "
                    f"gap {tr['issues_in_startup']['gap']}/{tr['issues_after_startup']['gap']}, "
                    f"burst {tr['issues_in_startup']['burst']}/{tr['issues_after_startup']['burst']}"
                )
            if tr["flags"]:
                print(f"  - flags: {', '.join(tr['flags'])}")
            if tr["note"]:
                print(f"  - note: {tr['note']}")
            print(f"  - **action: {tr['suggestion']}**")
        print()

    print("## Suggested next actions (legend)\n")
    print("- **OK**: recording order timestamps look consistent for this topic.")
    print("- **ignore startup segment**: problems concentrated in the first ~5 s on that topic.")
    print("- **use with caution**: gaps, duplicates, or irregularities persist after startup.")
    print("- **inspect manually**: borderline or sparse topic (e.g. few TF samples).")
    print("- **exclude**: time goes backward in the main part of the bag (replay/order risk).\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
