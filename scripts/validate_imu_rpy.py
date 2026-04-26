#!/usr/bin/env python3
"""
Validate IMU helpers used by imu_to_rpy_node:

  - quaternion_to_rpy_xyz (roll/pitch; quaternion yaw for optional comparison)
  - integrate_yaw_rate_forward_euler (same rule as live node: ω_z[i-1] * dt)

Optional bag: compare integrated-yaw span vs unwrapped quaternion-yaw span (differ if gyro bias / mag).

Dependencies: pip install rosbags numpy

Example:
  python scripts/validate_imu_rpy.py
  python scripts/validate_imu_rpy.py --bag-dir recordings/rosbags/2026-04-19/yaw_turns_02_2026_04_19-11_56_59
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
from imu_orientation_pkg.yaw_integration import (  # noqa: E402
    integrate_yaw_rate_forward_euler,
    stamp_to_seconds,
)


def _synthetic_quaternion_tests() -> None:
    r, p, y = quaternion_to_rpy_xyz(0.0, 0.0, 0.0, 1.0)
    assert abs(r) < 1e-9 and abs(p) < 1e-9 and abs(y) < 1e-9, "identity"

    s = math.sqrt(2.0) / 2.0
    r, p, y = quaternion_to_rpy_xyz(0.0, 0.0, s, s)
    assert abs(r) < 1e-6 and abs(p) < 1e-6, "pure yaw 90: roll/pitch"
    assert abs(y - math.pi / 2.0) < 1e-5, f"pure yaw 90: yaw={y}"

    r, p, y = quaternion_to_rpy_xyz(0.0, 0.0, 1.0, 0.0)
    assert abs(abs(y) - math.pi) < 1e-4, "180° yaw"

    print("Synthetic quaternion tests: OK")


def _synthetic_integration_tests() -> None:
    import numpy as np

    n = 100
    dt = 0.01
    w = 0.5  # rad/s
    stamps = np.arange(n, dtype=np.float64) * dt
    omega = np.full(n, w, dtype=np.float64)
    y = integrate_yaw_rate_forward_euler(stamps.tolist(), omega.tolist(), max_dt_sec=0.25)
    expected_end = (n - 1) * dt * w  # forward Euler with w[i-1]
    assert abs(y[-1] - expected_end) < 1e-9, (y[-1], expected_end)

    # Zero rate -> flat
    y0 = integrate_yaw_rate_forward_euler([0.0, 0.1, 0.2], [0.0, 0.0, 0.0])
    assert all(abs(v) < 1e-12 for v in y0)

    print("Synthetic yaw-integration tests: OK")


def _bag_validation(bag_dir: Path) -> None:
    try:
        import numpy as np
        from rosbags.highlevel import AnyReader
    except ImportError as e:
        print("Install: pip install rosbags numpy", file=sys.stderr)
        raise e

    topic = "/imu/data"
    if next(bag_dir.glob("*.mcap"), None) is None:
        raise FileNotFoundError(f"No .mcap under {bag_dir}")

    stamps: list[float] = []
    omega_z: list[float] = []
    yaws_quat: list[float] = []

    with AnyReader([bag_dir]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise RuntimeError(f"Topic {topic!r} not in bag")
        for c, _ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, c.msgtype)
            h = msg.header.stamp
            stamps.append(stamp_to_seconds(int(h.sec), int(h.nanosec)))
            omega_z.append(float(msg.angular_velocity.z))
            q = msg.orientation
            _r, _p, yq = quaternion_to_rpy_xyz(
                float(q.x), float(q.y), float(q.z), float(q.w)
            )
            yaws_quat.append(yq)

    if len(stamps) < 10:
        raise RuntimeError("Too few IMU samples")

    y_int = integrate_yaw_rate_forward_euler(stamps, omega_z, max_dt_sec=0.25)
    span_int = float(max(y_int) - min(y_int))

    ya = np.unwrap(np.array(yaws_quat, dtype=np.float64))
    span_quat = float(np.max(ya) - np.min(ya))

    # Yaw-turns maneuvers: both spans should show significant rotation
    assert span_int > 0.2, (
        f"Integrated yaw span {span_int:.3f} rad too small (check ω_z / stamps)"
    )
    assert span_quat > 0.5, (
        f"Unwrapped quaternion yaw span {span_quat:.3f} rad too small for yaw_turns"
    )

    diff = abs(span_int - span_quat)
    print(
        f"Bag {bag_dir.name}: n={len(stamps)}\n"
        f"  Integrated yaw span (max-min): {span_int:.3f} rad ({math.degrees(span_int):.1f} deg)\n"
        f"  Quaternion unwrap span: {span_quat:.3f} rad ({math.degrees(span_quat):.1f} deg)\n"
        f"  abs(span difference): {diff:.3f} rad (gyro vs mag can diverge)"
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

    _synthetic_quaternion_tests()
    _synthetic_integration_tests()
    if args.bag_dir is not None:
        d = args.bag_dir.resolve()
        if not d.is_dir():
            print(f"Not a directory: {d}", file=sys.stderr)
            return 1
        _bag_validation(d)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
