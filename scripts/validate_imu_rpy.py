#!/usr/bin/env python3
"""
Validate quaternion_to_rpy_xyz (same logic as imu_to_rpy_node) with synthetic cases and
optional ROS 2 bag replay via rosbags (no live ROS required).

Dependencies: pip install rosbags numpy

Example:
  python scripts/validate_imu_rpy.py
  python scripts/validate_imu_rpy.py --bag-dir recordings/rosbags/2026-03-26/yaw_turns_02_2026_03_26-12_25_25
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_PKG_SRC = _REPO / "src" / "sensors" / "imu_orientation_pkg"
if str(_PKG_SRC) not in sys.path:
    sys.path.insert(0, str(_PKG_SRC))

from imu_orientation_pkg.quaternion_rpy import quaternion_to_rpy_xyz  # noqa: E402


def _synthetic_tests() -> None:
    r, p, y = quaternion_to_rpy_xyz(0.0, 0.0, 0.0, 1.0)
    assert abs(r) < 1e-9 and abs(p) < 1e-9 and abs(y) < 1e-9, "identity"

    s = math.sqrt(2.0) / 2.0
    r, p, y = quaternion_to_rpy_xyz(0.0, 0.0, s, s)
    assert abs(r) < 1e-6 and abs(p) < 1e-6, "pure yaw 90: roll/pitch"
    assert abs(y - math.pi / 2.0) < 1e-5, f"pure yaw 90: yaw={y}"

    r, p, y = quaternion_to_rpy_xyz(0.0, 0.0, 1.0, 0.0)
    assert abs(y - math.pi) < 1e-5 or abs(abs(y) - math.pi) < 1e-5, "180° yaw"

    print("Synthetic tests: OK")


def _bag_validation(bag_dir: Path) -> None:
    try:
        import numpy as np
        from rosbags.highlevel import AnyReader
    except ImportError as e:
        print("Install: pip install rosbags numpy", file=sys.stderr)
        raise e

    topic = "/imu/data"
    mcap = next(bag_dir.glob("*.mcap"), None)
    if mcap is None:
        raise FileNotFoundError(f"No .mcap under {bag_dir}")

    yaws: list[float] = []
    with AnyReader([bag_dir]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise RuntimeError(f"Topic {topic!r} not in bag")
        for c, _ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, c.msgtype)
            q = msg.orientation
            _r, _p, y = quaternion_to_rpy_xyz(
                float(q.x), float(q.y), float(q.z), float(q.w)
            )
            yaws.append(y)

    if len(yaws) < 10:
        raise RuntimeError("Too few IMU samples")

    ya = np.unwrap(np.array(yaws, dtype=np.float64))
    span = float(np.max(ya) - np.min(ya))
    # Yaw-turns pool test: expect multiple large rotations (e.g. ~90° turns); loose threshold.
    assert span > 0.5, f"Unwrapped yaw span {span:.3f} rad seems too small for a yaw_turns bag"

    print(
        f"Bag {bag_dir.name}: n={len(yaws)}, unwrapped yaw span = {span:.3f} rad "
        f"({math.degrees(span):.1f} deg)"
    )
    print("Bag validation: OK")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--bag-dir",
        type=Path,
        default=None,
        help="Directory containing metadata.yaml and *.mcap (optional)",
    )
    args = ap.parse_args()

    _synthetic_tests()
    if args.bag_dir is not None:
        d = args.bag_dir.resolve()
        if not d.is_dir():
            print(f"Not a directory: {d}", file=sys.stderr)
            return 1
        _bag_validation(d)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
