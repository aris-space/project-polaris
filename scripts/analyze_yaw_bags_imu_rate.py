#!/usr/bin/env python3
"""Summarize |angular_velocity.z| from /imu/data in yaw_turns* bags (offline).

Usage:
  python scripts/analyze_yaw_bags_imu_rate.py [recordings/rosbags/2026-04-19]

Dependencies: pip install rosbags numpy
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("pip install rosbags numpy", file=sys.stderr)
    raise


def _default_yaw_root() -> Path:
    base = Path("recordings/rosbags")
    if not base.is_dir():
        return Path("recordings/rosbags/2026-04-19")

    dated = sorted(
        d for d in base.iterdir() if d.is_dir() and d.name[:4].isdigit() and d.name.count("-") == 2
    )
    if dated:
        return dated[-1]
    return Path("recordings/rosbags/2026-04-19")


def load_abs_wz(bag_dir: Path) -> np.ndarray | None:
    wz: list[float] = []
    with AnyReader([bag_dir]) as reader:
        conns = [c for c in reader.connections if c.topic == "/imu/data"]
        if not conns:
            return None
        for c, _ts, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, c.msgtype)
            wz.append(float(m.angular_velocity.z))
    return np.abs(np.array(wz, dtype=np.float64))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "root",
        type=Path,
        nargs="?",
        default=_default_yaw_root(),
        help="Directory containing yaw_turns_* bag folders",
    )
    args = ap.parse_args()
    root = args.root.resolve()
    if not root.is_dir():
        print(f"Not a directory: {root}", file=sys.stderr)
        return 1

    dirs = sorted(d for d in root.iterdir() if d.is_dir() and "yaw_turns" in d.name)
    if not dirs:
        print(f"No yaw_turns* under {root}", file=sys.stderr)
        return 1

    thr = 0.08  # rad/s — rough "actively turning" floor
    all_peak: list[float] = []

    for d in dirs:
        aw = load_abs_wz(d)
        if aw is None or len(aw) == 0:
            print(f"{d.name}: NO /imu/data")
            continue
        sel = aw[aw > thr]
        p99 = float(np.percentile(aw, 99))
        p95 = float(np.percentile(aw, 95))
        p90 = float(np.percentile(aw, 90))
        mx = float(np.max(aw))
        med = float(np.median(aw))
        all_peak.append(mx)
        print(f"--- {d.name} --- n={len(aw)}")
        print(
            f"  |wz| all: max={mx:.4f} rad/s  p99={p99:.4f}  p95={p95:.4f}  "
            f"p90={p90:.4f}  median={med:.4f}"
        )
        if len(sel) > 0:
            print(
                f"  |wz|>{thr}: count={len(sel)}  max={float(np.max(sel)):.4f}  "
                f"p99={float(np.percentile(sel, 99)):.4f}  p90={float(np.percentile(sel, 90)):.4f}"
            )
        print()

    if all_peak:
        print(
            "Across bags: max(|wz|) per bag =",
            ", ".join(f"{x:.3f}" for x in all_peak),
            f" overall max={max(all_peak):.4f} rad/s ({np.degrees(max(all_peak)):.1f} deg/s)",
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
