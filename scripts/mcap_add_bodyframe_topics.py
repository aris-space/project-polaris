#!/usr/bin/env python3
"""
Offline: read ROS 2 rosbag2 (MCAP) folders, extract static TF base_link->imu_link / base_link->dvl,
and write a *new* bag (originals untouched) with:
  - /tf_static                     (copied verbatim)
  - /imu/data_baselink             (sensor_msgs/Imu, vectors + orientation in base_link)
  - /sensors/dvl/odometry_cov_baselink (nav_msgs/Odometry, twist in base_link; see docstring)

Dependencies: pip install rosbags scipy

With --scan-root, output mirrors that tree: e.g. out_root/<date>/<test>__bodyframe/
(vs flat out_root/<test>__bodyframe/ when no mirror root applies).

Open the companion bag alongside the original in Foxglove (same time base).

Odometry caveat: only twist.linear / twist.angular are rotated; header.frame_id and pose are
unchanged from the source (often both dvl_a50_link). child_frame_id is set to base_link so
twist is self-consistent for body-frame velocity checks — do not use pose for navigation.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

try:
    from rosbags.highlevel import AnyReader
    from rosbags.interfaces import ConnectionExtRosbag2
    from rosbags.rosbag2 import Writer
    from rosbags.rosbag2.enums import StoragePlugin
except ImportError:
    print("Missing dependency. Install: pip install rosbags scipy", file=sys.stderr)
    raise


def _R_from_transform_rotation(q) -> np.ndarray:
    """geometry_msgs/Quaternion x,y,z,w -> 3x3 R maps child -> parent (v_parent = R @ v_child)."""
    return Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def _collect_static_Rs_from_bag(bag_dir: Path) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Return (R_base_imu, R_base_dvl) each 3x3, or None if missing."""
    R_imu: np.ndarray | None = None
    R_dvl: np.ndarray | None = None
    with AnyReader([bag_dir]) as reader:
        tf_conns = [c for c in reader.connections if c.topic == "/tf_static"]
        if not tf_conns:
            return None, None
        for _c, _ts, raw in reader.messages(connections=tf_conns):
            msg = reader.deserialize(raw, _c.msgtype)
            for t in msg.transforms:
                parent = t.header.frame_id
                child = t.child_frame_id
                if parent != "base_link":
                    continue
                R = _R_from_transform_rotation(t.transform.rotation)
                if child == "imu_link":
                    R_imu = R
                elif child == "dvl_a50_link":
                    R_dvl = R
    return R_imu, R_dvl


def _cov9_rotate(cov_row, R: np.ndarray) -> np.ndarray:
    if len(cov_row) < 9 or cov_row[0] < 0:
        return np.array(cov_row, dtype=np.float64, copy=True)
    C = np.array(cov_row, dtype=np.float64).reshape(3, 3)
    Cn = R @ C @ R.T
    return Cn.reshape(-1)


def _transform_imu(msg, R_bi: np.ndarray, typestore, typename: str) -> bytes:
    """R_bi: v_base = R_bi @ v_imu."""
    w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    wb = R_bi @ w
    ab = R_bi @ a
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = wb.tolist()
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ab.tolist()

    # Orientation: R_world_base = R_world_imu @ R_bi.T  (same world ref as source Imu)
    if msg.orientation_covariance[0] >= 0:
        q = msg.orientation
        R_wi = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        R_wb = R_wi @ R_bi.T
        qx, qy, qz, qw = Rot.from_matrix(R_wb).as_quat()
        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)
        msg.orientation_covariance = _cov9_rotate(msg.orientation_covariance, R_bi)

    msg.angular_velocity_covariance = _cov9_rotate(msg.angular_velocity_covariance, R_bi)
    msg.linear_acceleration_covariance = _cov9_rotate(msg.linear_acceleration_covariance, R_bi)
    msg.header.frame_id = "base_link"
    return typestore.serialize_cdr(msg, typename)


def _transform_odom_twist(msg, R_bd: np.ndarray, typestore, typename: str) -> bytes:
    """R_bd: v_base = R_bd @ v_dvl (velocity in DVL frame -> base_link)."""
    lin = np.array(
        [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
    )
    ang = np.array(
        [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
    )
    lin_b = R_bd @ lin
    ang_b = R_bd @ ang
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = lin_b.tolist()
    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = ang_b.tolist()
    msg.child_frame_id = "base_link"
    return typestore.serialize_cdr(msg, typename)


def _bodyframe_output_dir(bag_dir: Path, out_root: Path, mirror_under: Path | None) -> Path:
    """Match rosbags/(date)/(test)/ → out_root/(date)/(test)__bodyframe/ when mirror_under is set."""
    bag_dir = bag_dir.resolve()
    out_root = out_root.resolve()
    if mirror_under is not None:
        try:
            rel = bag_dir.relative_to(mirror_under.resolve())
            return out_root / rel.parent / f"{rel.name}__bodyframe"
        except ValueError:
            pass
    return out_root / f"{bag_dir.name}__bodyframe"


def process_one_bag(
    bag_dir: Path,
    out_root: Path,
    *,
    mirror_under: Path | None,
    imu_topic: str,
    dvl_topic: str,
    dry_run: bool,
) -> Path | None:
    bag_dir = bag_dir.resolve()
    R_imu, R_dvl = _collect_static_Rs_from_bag(bag_dir)
    if R_imu is None:
        print(f"[skip] {bag_dir.name}: no base_link->imu_link in /tf_static", file=sys.stderr)
        return None
    if R_dvl is None:
        print(f"[warn] {bag_dir.name}: no base_link->dvl_a50_link; DVL output skipped", file=sys.stderr)

    out_dir = _bodyframe_output_dir(bag_dir, out_root, mirror_under)
    if out_dir.exists():
        print(f"[skip] {out_dir} already exists (delete manually to regenerate)", file=sys.stderr)
        return None
    if dry_run:
        print(f"[dry-run] would create {out_dir}")
        return out_dir

    imu_out_topic = f"{imu_topic.rstrip('/')}_baselink"
    dvl_out_topic = f"{dvl_topic.rstrip('/')}_baselink"

    with AnyReader([bag_dir]) as reader:
        typestore = reader.typestore
        tf_conns = [c for c in reader.connections if c.topic == "/tf_static"]
        imu_conns = [c for c in reader.connections if c.topic == imu_topic]
        dvl_conns = [c for c in reader.connections if c.topic == dvl_topic]

        if not imu_conns:
            print(f"[skip] {bag_dir.name}: missing {imu_topic}", file=sys.stderr)
            return None
        if R_dvl is not None and not dvl_conns:
            print(f"[warn] {bag_dir.name}: missing {dvl_topic}", file=sys.stderr)

        imu_src = imu_conns[0]
        dvl_src = dvl_conns[0] if dvl_conns else None

        with Writer(
            out_dir,
            version=9,
            storage_plugin=StoragePlugin.MCAP,
        ) as writer:

            def add_like(src, topic: str) -> object:
                assert isinstance(src.ext, ConnectionExtRosbag2)
                return writer.add_connection(
                    topic,
                    src.msgtype,
                    typestore=typestore,
                    offered_qos_profiles=src.ext.offered_qos_profiles,
                )

            out_tf = add_like(tf_conns[0], "/tf_static")
            out_imu = add_like(imu_src, imu_out_topic)
            out_dvl = add_like(dvl_src, dvl_out_topic) if (R_dvl is not None and dvl_src) else None

            # Pass 1: tf_static verbatim
            for c, ts, raw in reader.messages(connections=tf_conns):
                writer.write(out_tf, ts, raw)

            # Pass 2: imu
            for c, ts, raw in reader.messages(connections=imu_conns):
                msg = reader.deserialize(raw, c.msgtype)
                blob = _transform_imu(msg, R_imu, typestore, imu_src.msgtype)
                writer.write(out_imu, ts, blob)

            # Pass 3: dvl
            if out_dvl is not None and dvl_conns:
                for c, ts, raw in reader.messages(connections=dvl_conns):
                    msg = reader.deserialize(raw, c.msgtype)
                    blob = _transform_odom_twist(msg, R_dvl, typestore, dvl_src.msgtype)
                    writer.write(out_dvl, ts, blob)

    print(f"[ok] {out_dir}")
    return out_dir


def find_bag_dirs(root: Path) -> list[Path]:
    dirs: list[Path] = []
    for meta in root.rglob("metadata.yaml"):
        d = meta.parent
        if d.name.endswith("__bodyframe"):
            continue
        if any(d.glob("*.mcap")):
            dirs.append(d)
    return sorted(set(dirs))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "inputs",
        nargs="*",
        default=[],
        help="Rosbag2 directories (with metadata.yaml). If empty, use --scan-root.",
    )
    ap.add_argument(
        "--scan-root",
        type=Path,
        default=None,
        help="Find all rosbag2 dirs under this path (e.g. recordings/rosbags).",
    )
    ap.add_argument(
        "--output-root",
        type=Path,
        default=Path("recordings/rosbags_bodyframe"),
        help="Parent directory for output bags (*__bodyframe). Created if missing.",
    )
    ap.add_argument(
        "--mirror-structure-under",
        type=Path,
        default=None,
        help=(
            "Preserve subdirs relative to this path (e.g. recordings/rosbags → "
            "out_root/<date>/<test>__bodyframe). Default: same as --scan-root when set; "
            "otherwise flat under --output-root."
        ),
    )
    ap.add_argument("--imu-topic", default="/imu/data")
    ap.add_argument("--dvl-topic", default="/sensors/dvl/odometry_cov")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    bags: list[Path] = []
    for p in args.inputs:
        p = Path(p)
        if not p.exists():
            print(f"Missing: {p}", file=sys.stderr)
            return 1
        bags.append(p)
    if args.scan_root:
        bags.extend(find_bag_dirs(args.scan_root.resolve()))
    if not bags:
        print("No bags: pass directories or --scan-root", file=sys.stderr)
        return 1

    mirror_under = args.mirror_structure_under
    if mirror_under is None and args.scan_root:
        mirror_under = args.scan_root

    # Stable order, unique paths
    bags = sorted({p.resolve() for p in bags})

    args.output_root.mkdir(parents=True, exist_ok=True)

    ok = 0
    for b in bags:
        if process_one_bag(
            b,
            args.output_root,
            mirror_under=mirror_under,
            imu_topic=args.imu_topic,
            dvl_topic=args.dvl_topic,
            dry_run=args.dry_run,
        ):
            ok += 1
    print(f"Done. {ok}/{len(bags)} written.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
