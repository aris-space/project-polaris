#!/usr/bin/env python3
"""Recommend Water Linked DVL-A50 range_mode from one or more ROS 2 bags."""

import argparse
import math
from typing import List, Optional, Tuple

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

# Water Linked range mode table (lower, upper). Upper for mode 4 is unbounded.
RANGE_BANDS = {
    0: (0.05, 0.6),
    1: (0.3, 3.0),
    2: (1.5, 14.0),
    3: (7.7, 36.0),
    4: (15.0, float("inf")),
}


def percentile(sorted_vals: List[float], p: float) -> float:
    """Return linear-interpolated percentile for p in [0, 100]."""
    if not sorted_vals:
        return float("nan")
    if p <= 0.0:
        return sorted_vals[0]
    if p >= 100.0:
        return sorted_vals[-1]

    k = (len(sorted_vals) - 1) * (p / 100.0)
    floor_idx = math.floor(k)
    ceil_idx = math.ceil(k)
    if floor_idx == ceil_idx:
        return sorted_vals[int(k)]

    lower_part = sorted_vals[floor_idx] * (ceil_idx - k)
    upper_part = sorted_vals[ceil_idx] * (k - floor_idx)
    return lower_part + upper_part


def choose_range_mode(lo: float, hi: float) -> str:
    """Pick a compact range_mode that covers [lo, hi]."""
    best: Optional[Tuple[Tuple[int, int, int], int, int]] = None

    for a in range(0, 5):
        lower_a, _ = RANGE_BANDS[a]
        if lower_a > lo:
            continue

        for b in range(a, 5):
            _, upper_b = RANGE_BANDS[b]
            if upper_b >= hi:
                # Prefer: smallest span, then smallest b, then largest a.
                score = (b - a, b, -a)
                if best is None or score < best[0]:
                    best = (score, a, b)

    if best is None:
        return "auto"

    _, a, b = best
    return f"={a}" if a == b else f"{a}<={b}"


def detect_dvl_topic(topics_and_types, user_topic: Optional[str]) -> Tuple[str, str]:
    """Return (topic_name, type_name) for a Dvl message topic."""
    if user_topic is not None:
        for topic_type in topics_and_types:
            if topic_type.name == user_topic:
                return topic_type.name, topic_type.type

        available = "\n".join(
            [f"  - {topic_type.name} ({topic_type.type})" for topic_type in topics_and_types]
        )
        raise RuntimeError(f"Topic '{user_topic}' not found in bag. Available:\n{available}")

    dvl_candidates = [
        topic_type
        for topic_type in topics_and_types
        if topic_type.type in ("marine_acoustic_msgs/msg/Dvl", "marine_acoustic_msgs/Dvl")
    ]
    if not dvl_candidates:
        available = "\n".join(
            [f"  - {topic_type.name} ({topic_type.type})" for topic_type in topics_and_types]
        )
        raise RuntimeError(
            "No marine_acoustic_msgs/Dvl topic found in bag.\n"
            "Tip: record the DVL velocity topic (usually '/sensors/dvl/velocity').\n"
            f"Available topics:\n{available}"
        )

    preferred = [
        topic_type
        for topic_type in dvl_candidates
        if topic_type.name.endswith("dvl/velocity") or "dvl/velocity" in topic_type.name
    ]
    pick = preferred[0] if preferred else dvl_candidates[0]
    return pick.name, pick.type


def read_altitudes(bag_path: str, topic_name: Optional[str], storage_id: str) -> List[float]:
    """Read valid altitude samples from one bag."""
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    chosen_topic, chosen_type = detect_dvl_topic(topics_and_types, topic_name)
    msg_cls = get_message(chosen_type)

    altitudes: List[float] = []
    while reader.has_next():
        topic, data, _timestamp = reader.read_next()
        if topic != chosen_topic:
            continue

        msg = deserialize_message(data, msg_cls)

        altitude = float(getattr(msg, "altitude", float("nan")))
        if not math.isfinite(altitude) or altitude <= 0.0:
            continue

        # Keep only samples that indicate bottom-lock validity when available.
        valid = getattr(msg, "beam_velocities_valid", None)
        if valid is not None and not bool(valid):
            continue

        altitudes.append(altitude)

    return altitudes


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Recommend Water Linked DVL range_mode from ROS 2 bag altitude history."
    )
    parser.add_argument(
        "bags",
        nargs="+",
        help="One or more rosbag2 directories (for example: mission1_bag mission2_bag).",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="DVL velocity topic in the bag (default: auto-detect).",
    )
    parser.add_argument(
        "--storage-id",
        default="sqlite3",
        help="Bag storage id (default: sqlite3).",
    )
    parser.add_argument(
        "--lo-pct",
        type=float,
        default=1.0,
        help="Lower percentile for outlier trimming (default: 1.0).",
    )
    parser.add_argument(
        "--hi-pct",
        type=float,
        default=99.0,
        help="Upper percentile for outlier trimming (default: 99.0).",
    )
    parser.add_argument(
        "--margin-m",
        type=float,
        default=0.5,
        help="Safety margin added to trimmed bounds in meters (default: 0.5).",
    )
    args = parser.parse_args()

    if not (0.0 <= args.lo_pct < args.hi_pct <= 100.0):
        raise ValueError("Percentiles must satisfy: 0 <= lo-pct < hi-pct <= 100.")

    all_altitudes: List[float] = []
    for bag in args.bags:
        altitudes = read_altitudes(bag, args.topic, args.storage_id)
        print(f"{bag}: {len(altitudes)} valid altitude samples")
        all_altitudes.extend(altitudes)

    if not all_altitudes:
        raise RuntimeError(
            "No valid altitude samples found. Ensure the bag contains a DVL velocity topic."
        )

    all_altitudes.sort()
    trimmed_lo = percentile(all_altitudes, args.lo_pct)
    trimmed_hi = percentile(all_altitudes, args.hi_pct)

    cover_lo = max(0.0, trimmed_lo - args.margin_m)
    cover_hi = trimmed_hi + args.margin_m
    suggested_mode = choose_range_mode(cover_lo, cover_hi)

    p50 = percentile(all_altitudes, 50.0)
    p90 = percentile(all_altitudes, 90.0)
    p99 = percentile(all_altitudes, 99.0)

    print("\n=== Summary (combined bags) ===")
    print(f"samples: {len(all_altitudes)}")
    print(f"raw min/max: {all_altitudes[0]:.3f} / {all_altitudes[-1]:.3f} m")
    print(f"p50/p90/p99: {p50:.3f} / {p90:.3f} / {p99:.3f} m")
    print(
        f"trimmed [{args.lo_pct:.1f}%, {args.hi_pct:.1f}%]: "
        f"{trimmed_lo:.3f} / {trimmed_hi:.3f} m"
    )
    print(
        f"with margin +/-{args.margin_m:.2f} m -> cover: "
        f"{cover_lo:.3f} .. {cover_hi:.3f} m"
    )
    print(f"\nRECOMMENDED range_mode: {suggested_mode}")
    print(f'use in launch: ros2 launch dvl_a50_pkg launch_dvl.launch.py range_mode:="{suggested_mode}"')


if __name__ == "__main__":
    main()
