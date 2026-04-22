#!/usr/bin/env python3
"""
Compare raw rosbags under recordings/rosbags with __bodyframe companion bags.

Checks: companion exists, derived topics present, IMU/DVL message counts and log_time
sequences match sources, /tf_static raw payloads match, bag time span vs IMU span.

Dependencies: pip install rosbags
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

IMU_RAW = "/imu/data"
IMU_DER = "/imu/data_baselink"
DVL_RAW_DEFAULT = "/sensors/dvl/odometry_cov"
DVL_FALLBACK = "/sensors/dvl/odometry"
DVL_DER = "/sensors/dvl/odometry_cov_baselink"
TF_STATIC = "/tf_static"


def find_raw_bag_dirs(rosbags_root: Path) -> list[Path]:
    out: list[Path] = []
    for meta in rosbags_root.rglob("metadata.yaml"):
        d = meta.parent
        if d.name.endswith("__bodyframe"):
            continue
        if any(d.glob("*.mcap")):
            out.append(d)
    return sorted(set(out))


def companion_dir(raw_dir: Path, rosbags_root: Path, bodyframe_root: Path) -> Path:
    rel = raw_dir.resolve().relative_to(rosbags_root.resolve())
    return bodyframe_root / rel.parent / f"{rel.name}__bodyframe"


def iter_topic_times_and_raw(reader: AnyReader, topic: str) -> tuple[list[int], list[bytes]]:
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        return [], []
    times: list[int] = []
    blobs: list[bytes] = []
    for c, ts, raw in reader.messages(connections=conns):
        times.append(int(ts))
        blobs.append(bytes(raw))
    return times, blobs


def derived_log_extremes_ns(
    tf_t: list[int],
    imu_t: list[int],
    dvl_t: list[int],
) -> tuple[int | None, int | None]:
    """Min/max log_time over the topics that the companion bag carries (tf + imu + dvl)."""
    starts: list[int] = []
    ends: list[int] = []
    for seq in (tf_t, imu_t, dvl_t):
        if seq:
            starts.append(seq[0])
            ends.append(seq[-1])
    if not starts:
        return None, None
    return min(starts), max(ends)


def analyze_pair(
    raw_dir: Path,
    comp_dir: Path,
    *,
    dvl_raw_topic: str,
) -> dict:
    issues: list[str] = []
    notes: list[str] = []

    if not comp_dir.is_dir() or not (comp_dir / "metadata.yaml").exists():
        return {
            "raw_bag": str(raw_dir),
            "companion_bag": str(comp_dir),
            "status": "issue",
            "issues": ["missing companion bag directory or metadata.yaml"],
            "notes": [],
        }

    with AnyReader([raw_dir]) as raw_r, AnyReader([comp_dir]) as comp_r:
        imu_t_raw, _ = iter_topic_times_and_raw(raw_r, IMU_RAW)
        imu_t_comp, _ = iter_topic_times_and_raw(comp_r, IMU_DER)

        dvl_topic_used = dvl_raw_topic
        dvl_t_raw: list[int] = []
        with AnyReader([raw_dir]) as r2:
            dvl_t_raw, _ = iter_topic_times_and_raw(r2, dvl_raw_topic)
        if not dvl_t_raw and dvl_raw_topic == DVL_RAW_DEFAULT:
            with AnyReader([raw_dir]) as r2b:
                dvl_t_raw, _ = iter_topic_times_and_raw(r2b, DVL_FALLBACK)
            if dvl_t_raw:
                dvl_topic_used = DVL_FALLBACK
                notes.append(
                    f"raw bag has no {DVL_RAW_DEFAULT}; compared {DVL_FALLBACK} to {DVL_DER}"
                )

        with AnyReader([comp_dir]) as r3:
            dvl_t_comp, _ = iter_topic_times_and_raw(r3, DVL_DER)

        tf_raw_blobs: list[bytes] = []
        tf_comp_blobs: list[bytes] = []
        with AnyReader([raw_dir]) as r4:
            tf_t_raw, tf_raw_blobs = iter_topic_times_and_raw(r4, TF_STATIC)
        with AnyReader([comp_dir]) as r5:
            tf_t_comp, tf_comp_blobs = iter_topic_times_and_raw(r5, TF_STATIC)

    derived_raw_t0, derived_raw_t1 = derived_log_extremes_ns(tf_t_raw, imu_t_raw, dvl_t_raw)
    derived_comp_t0, derived_comp_t1 = derived_log_extremes_ns(tf_t_comp, imu_t_comp, dvl_t_comp)

    if not imu_t_raw:
        issues.append(f"missing {IMU_RAW} in raw bag")
    if not imu_t_comp:
        issues.append(f"missing {IMU_DER} in companion")
    if imu_t_raw and imu_t_comp:
        if len(imu_t_raw) != len(imu_t_comp):
            issues.append(
                f"IMU count mismatch: raw {len(imu_t_raw)} vs companion {len(imu_t_comp)}"
            )
        elif imu_t_raw != imu_t_comp:
            mism = sum(1 for a, b in zip(imu_t_raw, imu_t_comp, strict=True) if a != b)
            issues.append(f"IMU log_time sequence mismatch in {mism} / {len(imu_t_raw)} messages")
        else:
            notes.append("IMU log_time sequence identical to raw")

    if dvl_t_raw and not dvl_t_comp:
        issues.append(f"missing {DVL_DER} in companion while raw has DVL messages")
    if dvl_t_raw and dvl_t_comp:
        if len(dvl_t_raw) != len(dvl_t_comp):
            issues.append(
                f"DVL count mismatch ({dvl_topic_used}): raw {len(dvl_t_raw)} vs companion {len(dvl_t_comp)}"
            )
        elif dvl_t_raw != dvl_t_comp:
            mism = sum(1 for a, b in zip(dvl_t_raw, dvl_t_comp, strict=True) if a != b)
            issues.append(
                f"DVL log_time mismatch ({dvl_topic_used}): {mism} / {len(dvl_t_raw)} messages"
            )
        else:
            notes.append(f"DVL log_time sequence identical ({dvl_topic_used})")

    if not tf_raw_blobs:
        issues.append("missing /tf_static in raw")
    if not tf_comp_blobs:
        issues.append("missing /tf_static in companion")
    if tf_raw_blobs and tf_comp_blobs:
        if len(tf_raw_blobs) != len(tf_comp_blobs):
            issues.append(
                f"/tf_static message count mismatch: raw {len(tf_raw_blobs)} vs companion {len(tf_comp_blobs)}"
            )
        elif tf_raw_blobs != tf_comp_blobs:
            issues.append("/tf_static raw payload mismatch (not verbatim copy)")
        else:
            notes.append("/tf_static payloads verbatim")

    def ns_dur(t0: int | None, t1: int | None) -> float | None:
        if t0 is None or t1 is None:
            return None
        return (t1 - t0) / 1e9

    derived_raw_dur = ns_dur(derived_raw_t0, derived_raw_t1)
    derived_comp_dur = ns_dur(derived_comp_t0, derived_comp_t1)
    imu_span_dur = ns_dur(imu_t_raw[0], imu_t_raw[-1]) if len(imu_t_raw) >= 2 else None

    # Companion bags intentionally omit other topics; compare only tf+imu+DVL coverage.
    if (
        derived_raw_t0 is not None
        and derived_raw_t1 is not None
        and derived_comp_t0 is not None
        and derived_comp_t1 is not None
    ):
        if derived_raw_t0 != derived_comp_t0 or derived_raw_t1 != derived_comp_t1:
            issues.append(
                "derived-topic log_time envelope mismatch: "
                f"raw {derived_raw_t0}..{derived_raw_t1} vs companion {derived_comp_t0}..{derived_comp_t1}"
            )
        else:
            notes.append(
                f"derived-topic span (tf+imu+DVL) {derived_raw_dur:.6f}s; "
                "min/max log_time identical raw vs companion"
            )
    elif imu_t_raw:
        issues.append("could not compute derived-topic span (missing tf or imu timestamps)")

    if imu_t_raw and imu_t_comp:
        if imu_t_raw[0] != imu_t_comp[0] or imu_t_raw[-1] != imu_t_comp[-1]:
            issues.append(
                "IMU first/last log_time differs between raw and companion "
                f"(raw {imu_t_raw[0]}..{imu_t_raw[-1]} vs comp {imu_t_comp[0]}..{imu_t_comp[-1]})"
            )
        else:
            notes.append("IMU first/last log_time aligned with companion")

    status = "pass" if not issues else "issue"
    return {
        "raw_bag": str(raw_dir),
        "companion_bag": str(comp_dir),
        "dvl_raw_topic_compared": dvl_topic_used if dvl_t_raw else None,
        "counts": {
            "imu_raw": len(imu_t_raw),
            "imu_companion": len(imu_t_comp),
            "dvl_raw": len(dvl_t_raw),
            "dvl_companion": len(dvl_t_comp),
            "tf_static_raw": len(tf_raw_blobs),
            "tf_static_companion": len(tf_comp_blobs),
        },
        "span_log_time_sec": {
            "derived_topics_union_raw": derived_raw_dur,
            "derived_topics_union_companion": derived_comp_dur,
            "raw_imu_only": imu_span_dur,
        },
        "status": status,
        "issues": issues,
        "notes": notes,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--rosbags-root",
        type=Path,
        default=Path("recordings/rosbags"),
        help="Root of raw bags (default: recordings/rosbags)",
    )
    ap.add_argument(
        "--bodyframe-root",
        type=Path,
        default=Path("recordings/rosbags_bodyframe"),
        help="Root of companion bags (default: recordings/rosbags_bodyframe)",
    )
    ap.add_argument(
        "--dvl-raw-topic",
        default=DVL_RAW_DEFAULT,
        help=f"Raw DVL topic to compare (default: {DVL_RAW_DEFAULT})",
    )
    ap.add_argument("--json", action="store_true", help="Print JSON array of per-bag results")
    args = ap.parse_args()

    rosbags_root = args.rosbags_root.resolve()
    bodyframe_root = args.bodyframe_root.resolve()
    if not rosbags_root.is_dir():
        print(f"Missing rosbags root: {rosbags_root}", file=sys.stderr)
        return 1

    raw_dirs = find_raw_bag_dirs(rosbags_root)
    results: list[dict] = []
    missing_companion: list[str] = []

    for raw_dir in raw_dirs:
        comp = companion_dir(raw_dir, rosbags_root, bodyframe_root)
        if not comp.is_dir():
            missing_companion.append(str(raw_dir))
            results.append(
                {
                    "raw_bag": str(raw_dir),
                    "companion_bag": str(comp),
                    "status": "issue",
                    "issues": ["missing companion bag"],
                    "notes": [],
                }
            )
            continue
        results.append(
            analyze_pair(raw_dir, comp, dvl_raw_topic=args.dvl_raw_topic.rstrip("/"))
        )

    summary = {
        "raw_bag_count": len(raw_dirs),
        "pass_count": sum(1 for r in results if r.get("status") == "pass"),
        "issue_count": sum(1 for r in results if r.get("status") == "issue"),
        "missing_companion_paths": missing_companion,
    }

    if args.json:
        print(json.dumps({"summary": summary, "bags": results}, indent=2))
        return 0

    print(
        f"Summary: {summary['pass_count']}/{summary['raw_bag_count']} pass, "
        f"{summary['issue_count']} with issues"
    )
    if missing_companion:
        print("Missing companion bags:", len(missing_companion))
    for r in results:
        name = Path(r["raw_bag"]).name
        st = r.get("status", "?")
        print(f"\n[{st.upper()}] {name}")
        if r.get("counts"):
            c = r["counts"]
            print(
                f"  counts: imu {c['imu_raw']}->{c['imu_companion']}, "
                f"dvl {c['dvl_raw']}->{c['dvl_companion']}, "
                f"tf_static {c['tf_static_raw']}->{c['tf_static_companion']}"
            )
        if r.get("span_log_time_sec"):
            s = r["span_log_time_sec"]
            print(
                f"  span (s): derived_raw={s.get('derived_topics_union_raw')} "
                f"derived_comp={s.get('derived_topics_union_companion')} imu_only={s.get('raw_imu_only')}"
            )
        for n in r.get("notes", []):
            print(f"  note: {n}")
        for i in r.get("issues", []):
            print(f"  issue: {i}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
