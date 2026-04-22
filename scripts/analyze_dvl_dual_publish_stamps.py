#!/usr/bin/env python3
"""
Explain duplicate / near-duplicate header.stamp on /sensors/dvl/odometry (and odometry_cov).

The bundled dvl_a50 driver (src/sensors/dvl_a50/src/dvl_a50_ros2.cpp) publishes
nav_msgs/Odometry twice per TCP cycle type:
  - after each velocity JSON (stamp from time_of_validity),
  - after each dead-reckoning JSON (stamp from ts).

So message_count(odometry) == message_count(velocity) + message_count(dead_reckoning)
unless the driver uses publish_odometry_on_dead_reckoning:=false (then n_odometry ≈ n_velocity).

/sensors/dvl/velocity uses one stamp per report (typically unique in the bag).
/sensors/dvl/dead_reckoning often reuses the same ts for several consecutive
reports → several odometry messages share one header.stamp → duplicate stamps
and zero Δt on a sorted odometry timeline. This is not NTP and not from
odometry_covariance_node (which forwards the same stamp as raw odometry).

Why pool "stationary_02" looks much cleaner than "stationary_01" in
stationary_tep_stats.json: lock-window + covariance gating drop segments where
DVL traffic and stamp reuse are worst; it does not mean the driver stopped
double-publishing.

Dependency: pip install rosbags
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

TOPIC_VEL = "/sensors/dvl/velocity"
TOPIC_DR = "/sensors/dvl/dead_reckoning"
TOPIC_ODO = "/sensors/dvl/odometry"
TOPIC_COV = "/sensors/dvl/odometry_cov"


def _stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)


def _collect_stamps(reader: AnyReader, topic: str) -> list[int]:
    conns = [c for c in reader.connections if c.topic == topic]
    out: list[int] = []
    for c, _, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, c.msgtype)
        out.append(_stamp_ns(msg))
    return out


def _multiplicity_hist(stamps: list[int]) -> dict[str, int]:
    ctr = Counter(stamps)
    return {str(k): v for k, v in sorted(Counter(ctr.values()).items())}


def _sorted_consecutive_equal(stamps: list[int]) -> int:
    if len(stamps) < 2:
        return 0
    s = sorted(stamps)
    return sum(1 for i in range(1, len(s)) if s[i] == s[i - 1])


def analyze_bag(bag_dir: Path) -> dict:
    with AnyReader([bag_dir]) as reader:
        vel = _collect_stamps(reader, TOPIC_VEL)
        dr = _collect_stamps(reader, TOPIC_DR)
        odo = _collect_stamps(reader, TOPIC_ODO)
        cov = _collect_stamps(reader, TOPIC_COV)

    n_v, n_d, n_o = len(vel), len(dr), len(odo)
    sum_ok = n_o == n_v + n_d

    def block(stamps: list[int]) -> dict:
        u = len(set(stamps)) if stamps else 0
        return {
            "n_messages": len(stamps),
            "n_unique_stamps": u,
            "sorted_consecutive_equal_stamp_pairs": _sorted_consecutive_equal(stamps),
            "stamp_multiplicity_histogram": _multiplicity_hist(stamps),
        }

    return {
        "bag": str(bag_dir.resolve()),
        "identity_n_odometry_eq_n_velocity_plus_n_dead_reckoning": sum_ok,
        "n_velocity": n_v,
        "n_dead_reckoning": n_d,
        "n_odometry": n_o,
        "n_odometry_cov": len(cov),
        "velocity": block(vel),
        "dead_reckoning": block(dr),
        "odometry": block(odo),
        "odometry_cov": block(cov),
        "note": (
            "If identity holds, duplicate odometry stamps follow from "
            "velocity (unique TOV) + dead_reckoning (reused ts) publishing "
            "two odometry messages per upstream message each."
        ),
        "note": (
            "If identity holds, duplicate odometry stamps follow from "
            "velocity (unique TOV) + dead_reckoning (reused ts) publishing "
            "two odometry messages per upstream message each."
        ),
    }


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument(
        "bag_dir",
        type=Path,
        help="Directory containing metadata.yaml and *.mcap",
    )
    p.add_argument("--json", action="store_true", help="Print JSON only")
    args = p.parse_args()
    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"Not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)
    out = analyze_bag(bag_dir)
    if args.json:
        print(json.dumps(out, indent=2))
    else:
        print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
