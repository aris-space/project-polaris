#!/usr/bin/env python3
"""
Localization-oriented timing health using deserialized ``header.stamp`` (rosbags).

Relationship to other repo scripts
------------------------------------

1. ``analyze_mcap_timing_health.py``
   - Uses MCAP **log_time** in **file / recording order**.
   - Fast (``mcap`` only, no deserialize). Answers “did the recorder write monotonic times?”

2. ``stationary_tep_stats.py`` (TEP / stationary mission)
   - Uses **header.stamp**; **Δt statistics are computed on stamps sorted per topic**
     (sensor / fusion timeline), not MCAP write order. Same philosophy as **stamp_sorted**
     in this script.

3. ``analyze_mcap_dvl_health.py``
   - **Bottom lock**, beams, altitude, ``odometry_cov`` vs ``Dvl`` agreement.
   - **Interpret DVL timing / EKF velocity updates only after checking lock + whether
     ``/sensors/dvl/velocity`` exists** (pinging off → no ``Dvl`` msgs → cov-only path).

This script
------------
- Collects **valid** ``header.stamp`` values (skips ``sec=0,nanosec=0``).
- Runs ``analyze_mcap_timing_health.analyze_topic_times`` **twice** per topic:
  - **message_order**: stamps in rosbag iteration order (detects recorder reorder vs stamp).
  - **stamp_sorted**: ``sorted(stamps)`` — matches **filter ordering** and **stationary_tep_stats**
    Δt logic; duplicate stamps still produce ``zero_dt``.
- Embeds a **compact** ``dvl_data_health`` block from ``analyze_mcap_dvl_health`` per bag.

``/tf``: duplicate ``header.stamp`` values are **downgraded** in the report when
``n_negative_dt == 0`` — multiple TF edges often share one filter stamp (see
``_relax_tf_header_stamp_report``).

``/imu/data``: on **stamp_sorted** sequences, IMU is usually ~fixed rate with no
``micro_bursts``; if ``analyze_mcap_timing_health`` (MCAP **log_time**) flags IMU bursts,
that is often **recorder batching**, not ``header.stamp`` pathology.

``/sensors/pressure/pose_enu``: low-rate pose updates can show rare **positive** dt values
much smaller than the median (fusion / paired publishes). When stamps stay monotonic and
bursts are a small fraction of intervals, ``micro_bursts`` is stripped (see
``_relax_pressure_pose_enu_header_stamp_report``).

Dependencies: ``pip install rosbags``
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import analyze_mcap_dvl_health as dvl_health  # noqa: E402
import analyze_mcap_timing_health as mth  # noqa: E402

LOCALIZATION_TOPICS: tuple[str, ...] = (
    "/imu/data",
    "/sensors/dvl/velocity",
    "/sensors/dvl/odometry_cov",
    "/pixhawk/scaled_pressure",
    "/sensors/pressure/pose_enu",
    "/fix",
    "/gps/selected",
    "/waterlinked_ugps/navsatfix",
    "/waterlinked_ugps/locator_position_global",
    "/waterlinked_ugps/locator_position_relative_wrt_topside",
    "/odometry/filtered/local",
    "/odometry/filtered/global",
    "/tf",
    "/tf_static",
)

find_raw_bag_dirs = dvl_health.find_raw_bag_dirs


def _stamp_ns_from_header(stamp) -> int:
    sec = int(getattr(stamp, "sec", 0))
    nsec = int(getattr(stamp, "nanosec", 0))
    return sec * 1_000_000_000 + nsec


def extract_header_stamp_ns(msg: Any, msgtype: str) -> int | None:
    if "TFMessage" in msgtype or (
        hasattr(msg, "transforms") and msg.transforms is not None
    ):
        if not msg.transforms:
            return None
        st = msg.transforms[0].header.stamp
        t = _stamp_ns_from_header(st)
        return None if t == 0 else t

    if hasattr(msg, "header"):
        t = _stamp_ns_from_header(msg.header.stamp)
        return None if t == 0 else t

    return None


def collect_header_stamps(
    bag_dir: Path, topics: tuple[str, ...]
) -> tuple[dict[str, list[int]], dict[str, int], dict[str, int]]:
    wanted = set(topics)
    stamps: dict[str, list[int]] = {t: [] for t in topics}
    raw_n: dict[str, int] = {t: 0 for t in topics}
    skipped: dict[str, int] = {t: 0 for t in topics}

    with AnyReader([bag_dir]) as reader:
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            return stamps, raw_n, skipped

        for conn, _log_ts, raw in reader.messages(connections=conns):
            topic = conn.topic
            raw_n[topic] = raw_n.get(topic, 0) + 1
            msg = reader.deserialize(raw, conn.msgtype)
            ns = extract_header_stamp_ns(msg, conn.msgtype)
            if ns is None:
                skipped[topic] = skipped.get(topic, 0) + 1
                continue
            stamps[topic].append(ns)

    return stamps, raw_n, skipped


def _relax_tf_header_stamp_report(out: dict[str, Any]) -> None:
    """
    /tf is a stream of tf2_msgs/TFMessage; robot_localization often emits one message
    per edge (map→odom, odom→base_link) with the **same** header.stamp. Scalar stamp
    sequences then show many consecutive zeros — unlike a single-sensor topic (DVL, IMU).
    Do not treat that as pathological duplicate-clock or burst pathology when stamps
    never go backward.
    """
    explain = (
        "tf2 /tf: equal header.stamp across consecutive messages usually means multiple "
        "edges published for one filter tick — not duplicate sensor time (contrast DVL odometry)."
    )
    for view in ("stamp_sorted", "message_order"):
        d = out.get(view)
        if not isinstance(d, dict):
            continue
        prev = (d.get("note") or "").strip()
        d["note"] = (prev + "; " if prev else "") + explain
        if int(d.get("n_negative_dt", 0) or 0) != 0:
            continue
        d["suggestion"] = "OK"
        strip = {
            "duplicate_log_time",
            "duplicate_timestamps_after_startup",
            "micro_bursts",
        }
        d["flags"] = [f for f in d.get("flags", []) if f not in strip]


def _relax_pressure_pose_enu_header_stamp_report(out: dict[str, Any]) -> None:
    """
    pose_enu is ~10–20 Hz with median dt ~50–100 ms. A few consecutive stamps can sit
    only milliseconds apart (e.g. multi-step pipeline) while the stream stays strictly
    increasing — unlike DVL duplicate-stamp bugs.     Strip ``micro_bursts`` when that
    pattern is rare (burst interval count well below 5% of all intervals) and stamps
    never regress.
    """
    explain = (
        "/sensors/pressure/pose_enu: rare sub-median positive dt values are common when "
        "stamps remain monotonic — not duplicate-clock pathology (contrast DVL odometry)."
    )
    for view in ("stamp_sorted", "message_order"):
        d = out.get(view)
        if not isinstance(d, dict):
            continue
        n = int(d.get("n") or 0)
        n_burst = int(d.get("n_burst") or 0)
        if int(d.get("n_negative_dt", 0) or 0) != 0 or n < 3:
            continue
        denom = max(n - 1, 1)
        if (n_burst / denom) >= 0.05:
            continue
        prev = (d.get("note") or "").strip()
        d["note"] = (prev + "; " if prev else "") + explain
        d["flags"] = [f for f in d.get("flags", []) if f != "micro_bursts"]


def _enrich_report(
    topic: str,
    seq: list[int],
    raw_n: int,
    skipped: int,
) -> dict[str, Any]:
    mo = mth.analyze_topic_times(topic, seq)
    d_mo = mo.to_json_dict()
    d_mo["ordering"] = "bag_message_iteration_order"

    ss = mth.analyze_topic_times(topic, sorted(seq))
    d_ss = ss.to_json_dict()
    d_ss["ordering"] = "header_stamp_sorted"

    out: dict[str, Any] = {
        "n_messages_deserialized": raw_n,
        "n_zero_or_missing_stamp_skipped": skipped,
        "message_order": d_mo,
        "stamp_sorted": d_ss,
    }
    if raw_n > 0 and len(seq) == 0:
        note = "all messages had stamp sec=0,nsec=0 or no header"
        for key in ("message_order", "stamp_sorted"):
            out[key]["note"] = (out[key].get("note") or "").strip()
            if out[key]["note"]:
                out[key]["note"] += "; "
            out[key]["note"] += note
            out[key]["flags"] = list(
                dict.fromkeys(out[key].get("flags", []) + ["no_valid_stamp"])
            )
    if topic == "/tf":
        _relax_tf_header_stamp_report(out)
    elif topic == "/sensors/pressure/pose_enu":
        _relax_pressure_pose_enu_header_stamp_report(out)
    return out


def analyze_bag_header_stamps(bag_dir: Path) -> dict[str, Any]:
    stamps, raw_n, skipped = collect_header_stamps(bag_dir, LOCALIZATION_TOPICS)
    reports: dict[str, Any] = {}
    for t in LOCALIZATION_TOPICS:
        seq = stamps.get(t, [])
        reports[t] = _enrich_report(t, seq, raw_n.get(t, 0), skipped.get(t, 0))

    # Bag-level verdict uses **stamp_sorted** (fusion-relevant), not message order.
    bag_suggestion = "OK"
    flagged_topics: list[str] = []
    for t, tr in reports.items():
        ss = tr["stamp_sorted"]
        if ss["n"] == 0:
            continue
        if ss["suggestion"] != "OK":
            flagged_topics.append(t)
        if ss["suggestion"] == "exclude":
            bag_suggestion = "exclude"
        elif ss["suggestion"] == "inspect manually" and bag_suggestion != "exclude":
            bag_suggestion = "inspect manually"
        elif ss["suggestion"] == "use with caution" and bag_suggestion in (
            "OK",
            "ignore startup segment",
        ):
            bag_suggestion = "use with caution"
        elif ss["suggestion"] == "ignore startup segment" and bag_suggestion == "OK":
            bag_suggestion = "ignore startup segment"

    dvl_res = dvl_health.analyze_bag(bag_dir)
    dvl_dict = dvl_health.result_to_dict(dvl_res)

    mcap = next(bag_dir.glob("*_0.mcap"), None)
    return {
        "bag_name": bag_dir.name,
        "bag_dir": str(bag_dir),
        "mcap_path": str(mcap) if mcap else "",
        "time_source": "header.stamp",
        "ordering_note": (
            "bag_suggestion uses stamp_sorted (like stationary_tep_stats Δt); "
            "message_order detects recorder vs stamp mismatch."
        ),
        "dvl_data_health": dvl_dict,
        "topics": reports,
        "bag_suggestion": bag_suggestion,
        "flagged_topics": flagged_topics,
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "ROS 2 bag header.stamp timing (message order + stamp_sorted). "
            "See module docstring vs analyze_mcap_timing_health / stationary_tep_stats."
        )
    )
    ap.add_argument(
        "roots",
        nargs="*",
        default=["recordings/rosbags"],
        help="Roots to scan for rosbag2 folders (default: recordings/rosbags)",
    )
    ap.add_argument("--json", action="store_true", help="Print JSON only")
    args = ap.parse_args()

    bag_dirs: list[Path] = []
    for r in args.roots:
        bag_dirs.extend(find_raw_bag_dirs(Path(r)))
    bag_dirs = sorted(set(bag_dirs))
    if not bag_dirs:
        print("No raw rosbag dirs with MCAP under given roots.", file=sys.stderr)
        return 1

    results = [analyze_bag_header_stamps(d) for d in bag_dirs]

    if args.json:
        print(json.dumps(results, indent=2))
        return 0

    print("# Header stamp timing (`header.stamp`)\n")
    print(
        "Each topic: **stamp_sorted** (primary, fusion timeline) and **message_order** "
        "(recorder order). See module docstring for comparison to other scripts.\n"
    )
    for bag in results:
        print(f"## {bag['bag_name']}\n")
        dh = bag["dvl_data_health"]
        print("### DVL data validity (analyze_mcap_dvl_health)\n")
        print(
            f"- `Dvl` msgs: **{dh['dvl_velocity_messages']}** · "
            f"`odometry_cov` total: **{dh['odometry_cov_messages_total']}** "
            f"(aligned to Dvl stamp: **{dh['odometry_cov_messages_at_dvl_velocity_stamp']}**)"
        )
        lf = dh["lock_fraction_time_weighted"]
        print(
            f"- Lock fraction (time-weighted): Dvl **{lf['from_dvl_beam_velocities_valid']}** · "
            f"cov **{lf['from_odometry_cov_covariance']}**"
        )
        print(f"- Flags: {dh['flags'] or '—'}")
        print(
            f"- Verdict: stationary — *{dh['verdict']['stationary_noise_analysis']}* · "
            f"maneuver — *{dh['verdict']['simple_maneuver_checks']}*\n"
        )

        print(f"### Timing (bag suggestion: **{bag['bag_suggestion']}**)\n")
        if bag["flagged_topics"]:
            print(f"- Flagged (stamp_sorted): {', '.join(bag['flagged_topics'])}\n")

        for t in LOCALIZATION_TOPICS:
            tr = bag["topics"][t]
            raw = tr["n_messages_deserialized"]
            sk = tr["n_zero_or_missing_stamp_skipped"]
            if raw == 0:
                print(f"- `{t}`: *absent*")
                continue
            ss = tr["stamp_sorted"]
            mo = tr["message_order"]
            print(
                f"- `{t}`: raw **{raw}**, valid stamps **{ss['n']}**, skipped **{sk}**"
            )
            if ss["n"] < 2:
                print(f"  - stamp_sorted: {ss.get('note', '')} **{ss['suggestion']}**")
                continue
            mhz = ss.get("mean_hz")
            mhz_s = f"{mhz:.4g}" if mhz is not None else "n/a"
            print(
                f"  - **stamp_sorted**: {mhz_s} Hz mean, "
                f"zero_dt={ss['n_zero_dt']}, neg_dt={ss['n_negative_dt']}, "
                f"gaps={ss['n_gap']}, bursts={ss['n_burst']} → **{ss['suggestion']}**"
                + (f"  | flags: {ss['flags']}" if ss.get("flags") else "")
            )
            if mo["n"] >= 2 and (
                mo["n_negative_dt"] != ss["n_negative_dt"]
                or mo["monotonic_nondecreasing"] != ss["monotonic_nondecreasing"]
            ):
                print(
                    f"  - *message_order differs*: mono={mo['monotonic_nondecreasing']}, "
                    f"neg_dt={mo['n_negative_dt']} (recorder order vs stamp clock)*"
                )
        print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
