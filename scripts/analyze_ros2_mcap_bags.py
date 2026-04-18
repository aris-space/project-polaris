#!/usr/bin/env python3
"""Summarize ROS 2 MCAP bags: topics, counts, duration. No ROS install required (pip install mcap)."""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

try:
    from mcap.reader import make_reader
except ImportError:
    print("Install: pip install mcap", file=sys.stderr)
    raise

# Inferred from repo: ekf_localization_pkg/config/*.yaml, launches, mavlink_bridge, mode_control.
REQUIRED_GROUPS = {
    "imu_primary": ["/imu/data"],
    "imu_aux": [
        "/imu/angular_velocity",
        "/imu/acceleration",
        "/imu/mag",
        "/filter/quaternion",
        "/filter/euler",
    ],
    "dvl_primary": ["/sensors/dvl/odometry_cov", "/sensors/dvl/odometry"],
    "dvl_aux": ["/sensors/dvl/velocity", "/sensors/dvl/dead_reckoning"],
    "pressure_raw": ["/pixhawk/scaled_pressure"],
    "pressure_fused": ["/sensors/pressure/pose_enu", "/sensors/pressure/p_surface_pa"],
    "manual_input": ["/joy", "/joy_controller", "/joy_keyboard", "/pixhawk/manual_control"],
    "tf": ["/tf"],
    "tf_static": ["/tf_static"],
    "ekf_outputs": [
        "/odometry/filtered/local",
        "/odometry/filtered/global",
        "/odometry/gps",
    ],
}

# Minimum messages for "healthy" over typical pool test duration (>30s); scaled by duration.
def imu_expected_min(duration_s: float) -> float:
    return max(50.0, duration_s * 40.0)  # allow dropouts; ~100 Hz ideal

def dvl_expected_min(duration_s: float) -> float:
    return max(30.0, duration_s * 5.0)  # ~30 Hz ideal

def joy_expected_min(duration_s: float) -> float:
    return max(5.0, duration_s * 0.5)  # human-driven

def tf_expected_min(duration_s: float) -> float:
    return max(30.0, duration_s * 10.0)  # EKF ~30 Hz


def analyze_mcap(path: Path) -> dict:
    with open(path, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
    if not summary or not summary.statistics:
        return {"error": "no_summary", "path": str(path)}

    st = summary.statistics
    topics: dict[str, int] = {}
    for cid, cnt in st.channel_message_counts.items():
        ch = summary.channels[cid]
        topics[ch.topic] = int(cnt)

    start_ns = st.message_start_time
    end_ns = st.message_end_time
    duration_s = (end_ns - start_ns) / 1e9 if start_ns and end_ns else 0.0
    total = int(st.message_count)

    start_iso = None
    if start_ns:
        start_iso = datetime.fromtimestamp(start_ns / 1e9, tz=timezone.utc).isoformat()

    return {
        "path": str(path),
        "bag_name": path.parent.name,
        "mcap_file": path.name,
        "start_time_utc": start_iso,
        "duration_sec": round(duration_s, 3),
        "total_messages": total,
        "topics": dict(sorted(topics.items(), key=lambda x: x[0])),
    }


def classify_topic(name: str, count: int, duration_s: float) -> str:
    if count == 0:
        return "absent"
    if name == "/tf":
        if count < tf_expected_min(duration_s):
            return "sparse"
        return "healthy"
    if name in ("/imu/data",):
        if count < imu_expected_min(duration_s):
            return "sparse"
        return "healthy"
    if name.startswith("/sensors/dvl/") and "odometry" in name:
        if count < dvl_expected_min(duration_s):
            return "sparse"
        return "healthy"
    if name in ("/joy", "/joy_controller", "/pixhawk/manual_control"):
        if count < joy_expected_min(duration_s):
            return "sparse"
        return "healthy"
    if name == "/tf_static":
        if count < 1:
            return "sparse"
        return "healthy"
    if name == "/pixhawk/scaled_pressure":
        # MAVLink bridge may forward at low rate; ~0.2+ Hz over the bag is usable for depth trends.
        if duration_s > 30 and count < max(10.0, duration_s * 0.08):
            return "sparse"
        return "healthy"
    if count == 1 and name not in ("/rosout", "/temperature_sensors"):
        return "brief"
    return "healthy"


def check_requirements(topics: dict[str, int], duration_s: float) -> dict:
    flags: list[str] = []
    status: list[tuple[str, str, str]] = []  # key, verdict, note

    def any_present(keys: list[str]) -> tuple[bool, str]:
        hits = [(k, topics.get(k, 0)) for k in keys]
        present = [k for k, c in hits if c > 0]
        if not present:
            return False, "absent"
        return True, present[0]

    # IMU
    ok, _ = any_present(REQUIRED_GROUPS["imu_primary"])
    if not ok:
        flags.append("missing_imu_data")
        status.append(("imu", "fail", "topic absent: /imu/data"))
    else:
        c = topics["/imu/data"]
        cl = classify_topic("/imu/data", c, duration_s)
        if cl == "sparse":
            flags.append("imu_sparse")
            status.append(("imu", "issue", f"/imu/data count={c} low for duration {duration_s:.1f}s"))
        else:
            status.append(("imu", "pass", f"/imu/data count={c}"))

    # DVL (at least one odometry stream)
    dvl_topics = REQUIRED_GROUPS["dvl_primary"]
    counts = [topics.get(t, 0) for t in dvl_topics]
    if sum(counts) == 0:
        flags.append("missing_dvl")
        status.append(("dvl", "fail", "no /sensors/dvl/odometry or odometry_cov"))
    else:
        best = max(zip(dvl_topics, counts), key=lambda x: x[1])
        cl = classify_topic(best[0], best[1], duration_s)
        if cl == "sparse":
            flags.append("dvl_sparse")
            status.append(("dvl", "issue", f"{best[0]} count={best[1]} low"))
        else:
            status.append(("dvl", "pass", f"{best[0]} count={best[1]}"))

    # Pressure (optional but expected for Polaris pool config)
    pr = topics.get("/pixhawk/scaled_pressure", 0)
    pose = topics.get("/sensors/pressure/pose_enu", 0)
    if pr == 0 and pose == 0:
        flags.append("no_pressure")
        status.append(("pressure", "issue", "no scaled_pressure or pose_enu (optional if not using depth)"))
    elif pose > 0:
        status.append(("pressure", "pass", f"pose_enu={pose}, scaled_pressure={pr}"))
    else:
        cl = classify_topic("/pixhawk/scaled_pressure", pr, duration_s)
        if cl == "sparse":
            flags.append("pressure_sparse")
            status.append(("pressure", "issue", f"scaled_pressure sparse count={pr}"))
        else:
            hz = pr / duration_s if duration_s > 0 else 0.0
            status.append(
                ("pressure", "pass", f"scaled_pressure count={pr} (~{hz:.2f} Hz effective)"),
            )

    # Manual
    mj = max(topics.get(t, 0) for t in REQUIRED_GROUPS["manual_input"])
    if mj == 0:
        flags.append("no_manual_input")
        status.append(("manual", "issue", "no /joy, /joy_controller, or /pixhawk/manual_control"))
    else:
        if mj < joy_expected_min(duration_s):
            status.append(("manual", "issue", f"manual topics sparse max_count={mj}"))
        else:
            status.append(("manual", "pass", f"manual activity max_count={mj}"))

    # TF
    tf_c = topics.get("/tf", 0)
    if tf_c == 0:
        flags.append("no_dynamic_tf")
        status.append(
            (
                "tf",
                "issue",
                "no /tf (expected if EKF not running; needed for full fusion replay)",
            )
        )
    elif tf_c < tf_expected_min(duration_s):
        flags.append("tf_sparse")
        status.append(("tf", "issue", f"/tf count={tf_c} low"))
    else:
        status.append(("tf", "pass", f"/tf count={tf_c}"))

    # tf_static
    ts = topics.get("/tf_static", 0)
    if ts == 0:
        flags.append("no_tf_static")
        status.append(("tf_static", "fail", "missing /tf_static"))
    else:
        status.append(("tf_static", "pass", f"count={ts}"))

    # Truncation heuristic
    if duration_s < 3.0:
        flags.append("very_short_bag")
    if total := sum(topics.values()) < 100 and duration_s > 10:
        flags.append("suspiciously_few_messages")

    overall = "pass"
    if any("fail" in s[1] for s in status):
        overall = "fail"
    elif flags:
        overall = "issues"

    return {
        "flags": flags,
        "checks": status,
        "overall": overall,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "roots",
        nargs="*",
        default=["recordings/rosbags"],
        help="Directories to search for *_0.mcap (default: recordings/rosbags)",
    )
    ap.add_argument("--json", action="store_true", help="Print JSON only")
    args = ap.parse_args()

    mcaps: list[Path] = []
    for r in args.roots:
        mcaps.extend(sorted(Path(r).rglob("*_0.mcap")))
    if not mcaps:
        print("No *_0.mcap files found.", file=sys.stderr)
        return 1

    reports = []
    for p in mcaps:
        info = analyze_mcap(p)
        if "error" in info:
            reports.append(info)
            continue
        topics = info["topics"]
        dur = info["duration_sec"]
        req = check_requirements(topics, dur)
        per_topic_class = {
            t: classify_topic(t, c, dur) for t, c in topics.items()
        }
        brief = [t for t, cl in per_topic_class.items() if cl == "brief"]
        sparse = [t for t, cl in per_topic_class.items() if cl == "sparse"]
        info["requirements"] = req
        info["sparse_topics"] = sparse
        info["brief_topics"] = brief
        reports.append(info)

    if args.json:
        print(json.dumps(reports, indent=2))
        return 0

    # Human table
    print("# ROS 2 MCAP bag report\n")
    for info in reports:
        if "error" in info:
            print(f"## ERROR: {info['path']}\n")
            continue
        print(f"## {info['bag_name']}\n")
        print(f"- **File:** `{info['mcap_file']}`")
        print(f"- **Start (UTC):** {info['start_time_utc']}")
        print(f"- **Duration:** {info['duration_sec']} s")
        print(f"- **Total messages:** {info['total_messages']}")
        print(f"- **Status:** **{info['requirements']['overall'].upper()}**")
        if info["requirements"]["flags"]:
            print(f"- **Flags:** {', '.join(info['requirements']['flags'])}")
        print("\n### Requirement checks\n")
        for key, verdict, note in info["requirements"]["checks"]:
            print(f"- **{key}** — *{verdict}*: {note}")
        if info["sparse_topics"]:
            print("\n### Sparse topics (heuristic)\n")
            for t in info["sparse_topics"]:
                print(f"- `{t}`: {info['topics'][t]} msgs")
        if info["brief_topics"]:
            print("\n### Single-message topics (likely latch / startup)\n")
            for t in sorted(info["brief_topics"])[:20]:
                print(f"- `{t}`")
            if len(info["brief_topics"]) > 20:
                print(f"- … and {len(info['brief_topics']) - 20} more")
        print("\n### All topics (count)\n")
        for t, c in info["topics"].items():
            cl = classify_topic(t, c, info["duration_sec"])
            print(f"- `{t}`: **{c}** ({cl})")
        print("\n---\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
