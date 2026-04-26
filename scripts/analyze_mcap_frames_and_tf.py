#!/usr/bin/env python3
"""
Frame IDs and TF inventory for raw ROS 2 rosbag2 folders (MCAP under recordings/rosbags).

Uses deserialized message headers (not body-frame companion bags).
Dependency: pip install rosbags
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

# Topics to inspect for header.frame_id / child_frame_id (when present on type).
FOCUS_TOPICS: tuple[str, ...] = (
    "/imu/data",
    "/sensors/dvl/odometry_cov",
    "/sensors/dvl/odometry",
    "/pixhawk/scaled_pressure",
    "/sensors/pressure/pose_enu",
    "/sensors/pressure/p_surface_pa",
)

TF_TOPICS: tuple[str, ...] = ("/tf_static", "/tf")

# Polaris / ekf_localization_pkg + dvl_a50_pkg conventions (see repo docs).
EXPECTED_STATIC_PARENT_CHILD: tuple[tuple[str, str], ...] = (
    ("base_link", "imu_link"),
    ("base_link", "dvl_a50_link"),
)
EXPECTED_IMU_HEADER_FRAME = "imu_link"
EXPECTED_DVL_SENSOR_FRAME = "dvl_a50_link"
DEFAULT_ROOTS: tuple[str, ...] = (
    "recordings/rosbags",
)


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


def _extract_frames(msg: Any) -> tuple[str | None, str | None]:
    """(header.frame_id, child_frame_id or None)."""
    fid = None
    cfid = None
    if hasattr(msg, "header"):
        h = msg.header
        fid = getattr(h, "frame_id", None) or None
        if isinstance(fid, str) and not fid.strip():
            fid = ""
    if hasattr(msg, "child_frame_id"):
        cfid = msg.child_frame_id
        if isinstance(cfid, str) and not cfid.strip():
            cfid = ""
    return fid, cfid if cfid is not None else None


def _collect_topic_frames(reader: AnyReader, topic: str) -> dict[str, Any]:
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        return {
            "present": False,
            "msgtype": None,
            "header_frame_id_values": [],
            "child_frame_id_values": [],
            "header_frame_id_counts": {},
            "child_frame_id_counts": {},
            "stable_header_frame_id": True,
            "stable_child_frame_id": True,
            "empty_header_frame_id_msgs": 0,
            "empty_child_frame_id_msgs": 0,
            "messages_sampled": 0,
        }

    msgtype = conns[0].msgtype
    hf_counts: Counter[str] = Counter()
    cf_counts: Counter[str] = Counter()
    empty_h = 0
    empty_c = 0
    n = 0
    for c, _ts, raw in reader.messages(connections=conns):
        n += 1
        msg = reader.deserialize(raw, c.msgtype)
        hf, cf = _extract_frames(msg)
        if hf is None or hf == "":
            empty_h += 1
            hf_key = "(empty)"
        else:
            hf_key = hf
        hf_counts[hf_key] += 1
        if cf is not None:
            if cf == "":
                empty_c += 1
                cf_counts["(empty)"] += 1
            else:
                cf_counts[cf] += 1

    hf_vals = [k for k in hf_counts if k != "(empty)"]
    cf_vals = list(cf_counts.keys())
    stable_h = len(hf_vals) <= 1
    stable_c = len([k for k in cf_vals if k != "(empty)"]) <= 1

    return {
        "present": True,
        "msgtype": msgtype,
        "header_frame_id_values": sorted(hf_vals),
        "child_frame_id_values": sorted([k for k in cf_vals if k != "(empty)"]),
        "header_frame_id_counts": dict(hf_counts),
        "child_frame_id_counts": dict(cf_counts),
        "stable_header_frame_id": stable_h,
        "stable_child_frame_id": stable_c,
        "empty_header_frame_id_msgs": empty_h,
        "empty_child_frame_id_msgs": empty_c,
        "messages_sampled": n,
    }


def _collect_tf_edges(reader: AnyReader, topic: str) -> tuple[Counter[tuple[str, str]], int]:
    """Return (edge counts parent->child, message count)."""
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        return Counter(), 0
    edges: Counter[tuple[str, str]] = Counter()
    n_msg = 0
    for c, _ts, raw in reader.messages(connections=conns):
        n_msg += 1
        msg = reader.deserialize(raw, c.msgtype)
        for t in msg.transforms:
            parent = t.header.frame_id
            child = t.child_frame_id
            edges[(parent, child)] += 1
    return edges, n_msg


def _bfs_reachable(start: str, forward: set[tuple[str, str]]) -> set[str]:
    """Children reachable from start following directed edges parent->child."""
    seen = {start}
    queue = [start]
    while queue:
        u = queue.pop(0)
        for p, c in forward:
            if p == u and c not in seen:
                seen.add(c)
                queue.append(c)
    return seen


def _analyze_tf_inventory(
    static_edges: Counter[tuple[str, str]],
    dynamic_edges: Counter[tuple[str, str]],
) -> dict[str, Any]:
    static_set = set(static_edges.keys())
    dynamic_set = set(dynamic_edges.keys())

    expected_found: list[dict[str, Any]] = []
    expected_missing: list[str] = []
    for par, ch in EXPECTED_STATIC_PARENT_CHILD:
        if (par, ch) in static_set:
            expected_found.append(
                {
                    "parent": par,
                    "child": ch,
                    "static_message_transforms": static_edges[(par, ch)],
                }
            )
        else:
            expected_missing.append(f"{par} -> {ch}")

    # Unique transforms (static vs dynamic)
    static_unique = sorted(static_set)
    dynamic_unique = sorted(dynamic_set)

    only_dynamic = sorted(dynamic_set - static_set)
    overlap = sorted(static_set & dynamic_set)

    forward_all = static_set | dynamic_set
    reachable_from_base = sorted(_bfs_reachable("base_link", forward_all)) if forward_all else []

    static_forward_only = static_set
    reachable_static_only = (
        sorted(_bfs_reachable("base_link", static_forward_only)) if static_forward_only else []
    )

    return {
        "static_transforms_unique": [{"parent": p, "child": c} for p, c in static_unique],
        "dynamic_transforms_unique": [{"parent": p, "child": c} for p, c in dynamic_unique],
        "dynamic_only_not_in_static": [{"parent": p, "child": c} for p, c in only_dynamic],
        "also_in_static": [{"parent": p, "child": c} for p, c in overlap],
        "expected_static_present": expected_found,
        "expected_static_missing": expected_missing,
        "frames_reachable_from_base_link_static_only": reachable_static_only,
        "frames_reachable_from_base_link_static_plus_dynamic": reachable_from_base,
    }


def _flags_for_topic(topic: str, info: dict[str, Any], tf_static_edges: set[tuple[str, str]]) -> list[dict[str, str]]:
    flags: list[dict[str, str]] = []
    if not info["present"]:
        if topic in ("/imu/data", "/sensors/dvl/odometry_cov", "/sensors/dvl/odometry"):
            flags.append(
                {
                    "issue": "missing_topic",
                    "consequence": "No messages on a core sensor topic; fusion or replay cannot use this stream.",
                }
            )
        return flags

    if info["empty_header_frame_id_msgs"] == info["messages_sampled"] and info["messages_sampled"] > 0:
        flags.append(
            {
                "issue": "header_frame_id_always_empty",
                "consequence": "TF and time-synchronized transforms cannot associate data with a coordinate frame.",
            }
        )
    elif info["empty_header_frame_id_msgs"] > 0:
        flags.append(
            {
                "issue": "some_empty_header_frame_id",
                "consequence": "Mixed empty/non-empty headers can break consumers that assume a fixed frame.",
            }
        )

    if not info["stable_header_frame_id"]:
        flags.append(
            {
                "issue": "changing_header_frame_id",
                "consequence": "Frame switches mid-bag confuse TF lookups and estimators that assume a fixed sensor frame.",
            }
        )

    if topic in ("/sensors/dvl/odometry_cov", "/sensors/dvl/odometry"):
        if not info["child_frame_id_values"] and info["messages_sampled"] > 0:
            flags.append(
                {
                    "issue": "missing_child_frame_id",
                    "consequence": "robot_localization and many TF tools require child_frame_id for odometry twist interpretation.",
                }
            )
        elif not info["stable_child_frame_id"]:
            flags.append(
                {
                    "issue": "changing_child_frame_id",
                    "consequence": "Velocity frame changes mid-bag; fusion and velocity plots become inconsistent.",
                }
            )
        else:
            hset = set(info["header_frame_id_values"])
            cset = set(info["child_frame_id_values"])
            if len(hset) == 1 and len(cset) == 1:
                hf = next(iter(hset))
                cf = next(iter(cset))
                if hf != cf:
                    flags.append(
                        {
                            "issue": "odom_header_child_mismatch",
                            "consequence": "Unusual for DVL driver; verify twist is expressed in the intended frame.",
                        }
                    )

    # Expected names vs static TF
    if topic == "/imu/data" and info["header_frame_id_values"]:
        exp = EXPECTED_IMU_HEADER_FRAME
        if exp not in info["header_frame_id_values"]:
            flags.append(
                {
                    "issue": "imu_header_not_expected_frame",
                    "consequence": f"EKF/launch expect IMU data in {exp!r} with base_link->{exp} in /tf_static; "
                    f"observed {info['header_frame_id_values']!r} may conflict with static TF or duplicate world links.",
                }
            )
        elif ("base_link", exp) not in tf_static_edges:
            flags.append(
                {
                    "issue": "imu_frame_not_connected_in_static_tf",
                    "consequence": f"No base_link->{exp} in recorded /tf_static; IMU cannot be rigidly tied to base without extrinsics.",
                }
            )

    if topic in ("/sensors/dvl/odometry_cov", "/sensors/dvl/odometry") and info["header_frame_id_values"]:
        exp = EXPECTED_DVL_SENSOR_FRAME
        hf_ok = exp in info["header_frame_id_values"]
        cf_vals = info["child_frame_id_values"]
        cf_ok = exp in cf_vals if cf_vals else False
        if not hf_ok or not cf_ok:
            flags.append(
                {
                    "issue": "dvl_frames_not_expected_sensor_frame",
                    "consequence": f"Driver/launch typically set header and child to {exp!r} to match static_tf_base_to_dvl; "
                    f"mismatch breaks EKF velocity updates and body-frame checks.",
                }
            )
        if hf_ok and cf_ok and ("base_link", exp) not in tf_static_edges:
            flags.append(
                {
                    "issue": "dvl_frame_not_connected_in_static_tf",
                    "consequence": f"No base_link->{exp} in /tf_static; DVL twist cannot be transformed to base without CAD extrinsics in the bag.",
                }
            )

    return flags


def _tf_flags(tf_inv: dict[str, Any], n_static_msgs: int, n_dynamic_msgs: int) -> list[dict[str, str]]:
    flags: list[dict[str, str]] = []
    if n_static_msgs == 0:
        flags.append(
            {
                "issue": "no_tf_static",
                "consequence": "No recorded static extrinsics; offline TF chains for IMU/DVL relative to base_link are unavailable from the bag alone.",
            }
        )
    if tf_inv["expected_static_missing"]:
        flags.append(
            {
                "issue": "expected_static_transform_missing",
                "consequence": "Missing CAD mounts (base_link->imu_link or base_link->dvl_a50_link); robot_model and fusion expecting those links will fail or use wrong geometry.",
            }
        )

    reach = tf_inv["frames_reachable_from_base_link_static_only"]
    reach_set = set(reach)
    static_frames: set[str] = set()
    for e in tf_inv["static_transforms_unique"]:
        static_frames.add(e["parent"])
        static_frames.add(e["child"])
    if n_static_msgs and static_frames and "base_link" not in static_frames:
        flags.append(
            {
                "issue": "base_link_absent_from_static_tf",
                "consequence": "No transform uses frame_id base_link; EKF and URDF expecting base_link as robot root will not align with this TF tree.",
            }
        )

    if n_dynamic_msgs > 0:
        flags.append(
            {
                "issue": "dynamic_tf_present",
                "consequence": "Recorded /tf usually comes from EKF or similar; for no-EKF pool tests verify whether these transforms are intended (e.g. odom->base_link) or stale.",
            }
        )

    if n_static_msgs and not tf_inv["expected_static_missing"]:
        need = {EXPECTED_IMU_HEADER_FRAME, EXPECTED_DVL_SENSOR_FRAME}
        missing_reach = sorted(need - reach_set)
        if missing_reach:
            flags.append(
                {
                    "issue": "expected_sensor_frames_not_reachable_static",
                    "consequence": f"Static tree from base_link does not reach {missing_reach}; check parent/child naming on transforms vs sensor headers.",
                }
            )

    return flags


def analyze_bag(bag_dir: Path) -> dict[str, Any]:
    topics_out: dict[str, Any] = {}
    with AnyReader([bag_dir]) as reader:
        static_ec, n_static = _collect_tf_edges(reader, "/tf_static")
        dynamic_ec, n_dynamic = _collect_tf_edges(reader, "/tf")
        tf_inv = _analyze_tf_inventory(static_ec, dynamic_ec)
        static_keys = set(static_ec.keys())

        for t in FOCUS_TOPICS:
            topics_out[t] = _collect_topic_frames(reader, t)
            topics_out[t]["flags"] = _flags_for_topic(t, topics_out[t], static_keys)

        tf_flags = _tf_flags(tf_inv, n_static, n_dynamic)

    return {
        "bag_dir": str(bag_dir),
        "bag_name": bag_dir.name,
        "tf": {
            "tf_static_messages": n_static,
            "tf_messages": n_dynamic,
            "inventory": tf_inv,
            "flags": tf_flags,
        },
        "topics": topics_out,
        "expected_reference": {
            "static_tf_edges": list(EXPECTED_STATIC_PARENT_CHILD),
            "imu_header_frame": EXPECTED_IMU_HEADER_FRAME,
            "dvl_sensor_frame": EXPECTED_DVL_SENSOR_FRAME,
            "notes": [
                "Expected values follow ekf_localization_pkg static mounts and dvl_a50_pkg frame convention.",
                "Pool bags documented without EKF often have no /tf; static mounts should still be in /tf_static.",
            ],
        },
    }


def _print_report(data: dict[str, Any]) -> None:
    name = data["bag_name"]
    print(f"## {name}\n")
    exp = data["expected_reference"]
    print("### Expected (repo convention)\n")
    print(f"- Static TF edges: {exp['static_tf_edges']}")
    print(f"- IMU header.frame_id: {exp['imu_header_frame']!r}")
    print(f"- DVL odometry header + child_frame_id: {exp['dvl_sensor_frame']!r}\n")

    print("### TF inventory\n")
    tf = data["tf"]
    print(f"- /tf_static messages: {tf['tf_static_messages']}, /tf messages: {tf['tf_messages']}")
    inv = tf["inventory"]
    print("- Static transforms (unique parent -> child):")
    for e in inv["static_transforms_unique"]:
        print(f"  - {e['parent']} -> {e['child']}")
    if not inv["static_transforms_unique"]:
        print("  - (none)")
    print("- Dynamic transforms (unique parent -> child):")
    for e in inv["dynamic_transforms_unique"]:
        print(f"  - {e['parent']} -> {e['child']}")
    if not inv["dynamic_transforms_unique"]:
        print("  - (none)")
    print(f"- Expected static present: {inv['expected_static_present']}")
    print(f"- Expected static missing: {inv['expected_static_missing'] or '[]'}")
    print(
        f"- Reachable from base_link (static only): {inv['frames_reachable_from_base_link_static_only']}"
    )
    print(
        f"- Reachable from base_link (static+dynamic): {inv['frames_reachable_from_base_link_static_plus_dynamic']}\n"
    )
    if tf["flags"]:
        print("#### TF flags\n")
        for f in tf["flags"]:
            print(f"- **{f['issue']}**: {f['consequence']}")
        print()

    print("### Per-topic frames\n")
    for t in FOCUS_TOPICS:
        info = data["topics"][t]
        print(f"- `{t}`")
        if not info["present"]:
            print("  - absent")
            continue
        print(f"  - msgtype: `{info['msgtype']}`, messages: {info['messages_sampled']}")
        print(f"  - header.frame_id values: {info['header_frame_id_values'] or ['(empty only)']}")
        if info["child_frame_id_counts"]:
            print(f"  - child_frame_id values: {info['child_frame_id_values'] or ['(empty)']}")
        print(
            f"  - stable header: {info['stable_header_frame_id']}, stable child: {info['stable_child_frame_id']}"
        )
        if info["empty_header_frame_id_msgs"]:
            print(f"  - empty header.frame_id msgs: {info['empty_header_frame_id_msgs']}")
        if info.get("flags"):
            for fl in info["flags"]:
                print(f"  - FLAG {fl['issue']}: {fl['consequence']}")
        print()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "roots",
        nargs="*",
        default=list(DEFAULT_ROOTS),
        help="Scan under these roots for rosbag2 dirs (default: recordings/rosbags)",
    )
    ap.add_argument("--json", action="store_true", help="Print JSON only")
    args = ap.parse_args()

    bags: list[Path] = []
    for r in args.roots:
        bags.extend(find_raw_bag_dirs(Path(r).resolve()))
    bags = sorted(set(bags))
    if not bags:
        print("No raw rosbag2 directories found.", file=sys.stderr)
        return 1

    reports = []
    for b in bags:
        try:
            reports.append(analyze_bag(b))
        except Exception as e:
            print(f"WARNING: skipping {b.name} — {e}", file=sys.stderr)
            reports.append({"bag_name": b.name, "bag_dir": str(b), "error": str(e), "bag_suggestion": "unreadable"})

    if args.json:
        print(json.dumps(reports, indent=2))
        return 0

    print("# Frame IDs and TF (raw rosbags only)\n")
    for d in reports:
        _print_report(d)
        print("---\n")

    print("## Summary: observed vs expected\n")
    print(
        "- **IMU** should use header.frame_id matching static child **imu_link** with **base_link -> imu_link** in /tf_static."
    )
    print(
        "- **DVL odometry** should use the same frame for header.frame_id and child_frame_id (**dvl_a50_link** here) and **base_link -> dvl_a50_link** in /tf_static."
    )
    print(
        "- **dynamic /tf** with no EKF is worth a quick manual check (duplicate world->sensor vs odom->base_link)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
