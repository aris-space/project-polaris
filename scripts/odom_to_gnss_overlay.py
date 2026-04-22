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
