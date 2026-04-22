"""
Dead-reckoning ground truth evaluation: odom frame → lat/lon vs raw GNSS /fix.

Usage:
    python scripts/odom_to_gnss_overlay.py /path/to/bag_dir [--max-h-acc 2.0] [--output-dir ./output]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from mcap_ros2.reader import read_ros2_messages
import utm

try:
    import contextily as ctx
    _HAS_CONTEXTILY = True
except ImportError:
    _HAS_CONTEXTILY = False

# navsat_transform.yaml constants
_YAW_OFFSET = math.pi / 2.0      # 1.5708 rad
_MAG_DECL   = 0.0623             # rad
_IMU_TF_YAW = math.pi            # base_link ← imu_link static TF yaw

# NavSatFix covariance type constants
_COV_UNKNOWN  = 0
_COV_APPROX   = 1
_COV_DIAGONAL = 2
_COV_KNOWN    = 3


# ── dataclasses ──────────────────────────────────────────────────────────────

@dataclass
class FixMsg:
    t_ns: int
    lat: float
    lon: float
    status: int      # status.status field; ≥0 means fix
    cov: list        # 9-element row-major position_covariance
    cov_type: int    # COVARIANCE_TYPE_* constant

@dataclass
class UbxHpMsg:
    t_ns: int
    h_acc_raw: int   # raw 0.1 mm units from ublox

@dataclass
class ImuMsg:
    t_ns: int
    qx: float
    qy: float
    qz: float
    qw: float

@dataclass
class OdomMsg:
    t_ns: int
    x: float
    y: float
    cov_xx: float    # pose.covariance[0]  (x variance)
    cov_yy: float    # pose.covariance[7]  (y variance)

@dataclass
class BagData:
    fix_msgs:    list = field(default_factory=list)
    ubx_hp_msgs: list = field(default_factory=list)
    imu_msgs:    list = field(default_factory=list)
    odom_msgs:   list = field(default_factory=list)


# ── pure math helpers ────────────────────────────────────────────────────────

def _h_acc_from_navsatfix(cov: list, cov_type: int) -> float | None:
    """Largest horizontal 1-sigma from NavSatFix position_covariance (ENU, row-major 3×3)."""
    if cov_type == _COV_UNKNOWN:
        return None
    if cov_type == _COV_DIAGONAL:
        if len(cov) < 5:
            return None
        return math.sqrt(max(0.0, float(cov[0]), float(cov[4])))
    if len(cov) < 9:
        return None
    a, b, d = float(cov[0]), float(cov[1]), float(cov[4])
    tr = a + d
    det = a * d - b * b
    disc = max(0.0, tr * tr - 4.0 * det)
    return math.sqrt(max(0.0, 0.5 * (tr + math.sqrt(disc))))


def _h_acc_from_ubx(h_acc_raw: int) -> float | None:
    """Convert raw ublox h_acc (0.1 mm units) to meters. Returns None for sentinel 0xFFFFFFFF."""
    if h_acc_raw == 0xFFFFFFFF:
        return None
    return float(h_acc_raw) * 1e-4


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rad) from unit quaternion using scipy ZYX Euler convention."""
    return float(Rotation.from_quat([qx, qy, qz, qw]).as_euler("ZYX")[0])


def _wrap_pi(angle: float) -> float:
    """Wrap angle to [-π, π)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _compute_psi(theta_imu_rad: float) -> float:
    """Compute fixed rotation angle ψ (rad) from raw IMU yaw.

    Chain: θ_imu → +π (base_link←imu_link TF) → +yaw_offset → +mag_decl
    """
    theta_base = _wrap_pi(theta_imu_rad + _IMU_TF_YAW)
    return _wrap_pi(theta_base + _YAW_OFFSET + _MAG_DECL)


def _odom_to_utm(x: float, y: float, E0: float, N0: float, psi: float) -> tuple[float, float]:
    """Apply fixed rotation matrix T: odom (x, y) → UTM (E, N)."""
    E = E0 + math.cos(psi) * x - math.sin(psi) * y
    N = N0 + math.sin(psi) * x + math.cos(psi) * y
    return E, N


# ── bag reader ───────────────────────────────────────────────────────────────

_TOPIC_FIX  = "/fix"
_TOPIC_UBX  = "/ubx_nav_hp_pos_llh"
_TOPIC_IMU  = "/imu/data"
_TOPIC_ODOM = "/odometry/filtered/local"


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def read_bag(bag_dir: Path) -> BagData:
    """Read all relevant topics from the bag in a single pass."""
    mcap_path = bag_dir / f"{bag_dir.name}_0.mcap"
    if not mcap_path.exists():
        candidates = list(bag_dir.glob("*_0.mcap"))
        if not candidates:
            print(f"ERROR: no *_0.mcap found in {bag_dir}", file=sys.stderr)
            sys.exit(1)
        mcap_path = candidates[0]

    wanted = {_TOPIC_FIX, _TOPIC_UBX, _TOPIC_IMU, _TOPIC_ODOM}
    data = BagData()

    for msg in read_ros2_messages(str(mcap_path)):
        topic = msg.channel.topic
        if topic not in wanted:
            continue
        ros = msg.ros_msg
        try:
            t = _stamp_ns(ros.header.stamp)
        except AttributeError:
            continue
        if t == 0:
            continue

        if topic == _TOPIC_FIX:
            data.fix_msgs.append(FixMsg(
                t_ns=t,
                lat=float(ros.latitude),
                lon=float(ros.longitude),
                status=int(ros.status.status),
                cov=list(ros.position_covariance),
                cov_type=int(ros.position_covariance_type),
            ))

        elif topic == _TOPIC_UBX:
            data.ubx_hp_msgs.append(UbxHpMsg(
                t_ns=t,
                h_acc_raw=int(ros.h_acc),
            ))

        elif topic == _TOPIC_IMU:
            q = ros.orientation
            data.imu_msgs.append(ImuMsg(
                t_ns=t,
                qx=float(q.x), qy=float(q.y), qz=float(q.z), qw=float(q.w),
            ))

        elif topic == _TOPIC_ODOM:
            p = ros.pose.pose.position
            cov = ros.pose.covariance  # 36-element flat array
            data.odom_msgs.append(OdomMsg(
                t_ns=t,
                x=float(p.x),
                y=float(p.y),
                cov_xx=float(cov[0]),
                cov_yy=float(cov[7]),
            ))

    return data


def _parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path, help="Bag directory containing <name>_0.mcap")
    ap.add_argument("--max-h-acc", type=float, default=2.0,
                    help="Max horizontal accuracy (m) for GNSS ground truth gate (default: 2.0)")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output directory for PNGs and JSON (default: bag_dir/odom_gnss_analysis)")
    return ap.parse_args(argv)


def main():
    args = _parse_args()
    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)
    out_dir = args.output_dir or (bag_dir / "odom_gnss_analysis")
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Bag: {bag_dir.name}")
    print(f"Output: {out_dir}")


if __name__ == "__main__":
    main()
