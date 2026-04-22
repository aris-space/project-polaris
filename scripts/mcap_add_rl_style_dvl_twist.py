#!/usr/bin/env python3
"""
Offline rosbag2 (MCAP) rewrite: DVL odometry twist adjusted like robot_localization prepareTwist().

robot_localization (ros_filter.cpp prepareTwist) does, for twist in child_frame_id:
  twist_lin = R * twist_lin + origin.cross(state_twist_rot)
where R,origin are base_link <- child from TF, and state_twist_rot is the filter's
body-frame angular velocity (roll/pitch/yaw rates).

This script approximates state_twist_rot by rotating /imu/data angular_velocity into
base_link (same static TF as the EKF uses) and picking the IMU sample whose header.stamp
is closest to each DVL message's header.stamp.

Output bag (new folder per input, originals untouched):
  - /tf_static                         (verbatim copy)
  - /imu/data_baselink (default)       same as mcap_add_bodyframe_topics: vectors + orientation in base_link
  - /sensors/dvl/odometry_cov_rltwist (default)  Odometry with prepareTwist-style linear twist

Use ``--skip-imu`` for DVL-only output. IMU rewrite does not change angular rates beyond a
fixed static rotation imu_link→base_link (it is not an EKF); it matches the bodyframe companion.

Pose and header.frame_id on DVL stay identical to the source message. DVL child_frame_id is
base_link so twist is self-consistent.

Dependencies: pip install rosbags numpy scipy

Example:
  python scripts/mcap_add_rl_style_dvl_twist.py --scan-root recordings/rosbags \\
      --output-root recordings/rosbags_rltwist
"""

from __future__ import annotations

import argparse
import bisect
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
    print("Missing dependency. Install: pip install rosbags numpy scipy", file=sys.stderr)
    raise


def _R_from_transform_rotation(q) -> np.ndarray:
    """geometry_msgs/Quaternion x,y,z,w -> 3x3 R: v_parent = R @ v_child."""
    return Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def _t_from_transform(t) -> np.ndarray:
    return np.array([t.x, t.y, t.z], dtype=np.float64)


def _stamp_to_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _collect_static_tf_from_bag(bag_dir: Path) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """Return (R_base_imu, R_base_dvl, t_base_dvl) where t is dvl origin in base_link coords."""
    R_imu: np.ndarray | None = None
    R_dvl: np.ndarray | None = None
    t_dvl: np.ndarray | None = None
    with AnyReader([bag_dir]) as reader:
        tf_conns = [c for c in reader.connections if c.topic == "/tf_static"]
        if not tf_conns:
            return None, None, None
        for _c, _ts, raw in reader.messages(connections=tf_conns):
            msg = reader.deserialize(raw, tf_conns[0].msgtype)
            for tr in msg.transforms:
                if tr.header.frame_id != "base_link":
                    continue
                R = _R_from_transform_rotation(tr.transform.rotation)
                t = _t_from_transform(tr.transform.translation)
                if tr.child_frame_id == "imu_link":
                    R_imu = R
                elif tr.child_frame_id == "dvl_a50_link":
                    R_dvl = R
                    t_dvl = t
    return R_imu, R_dvl, t_dvl


def _cov9_rotate(cov_row, R: np.ndarray) -> np.ndarray:
    if len(cov_row) < 9 or cov_row[0] < 0:
        return np.array(cov_row, dtype=np.float64, copy=True)
    C = np.array(cov_row, dtype=np.float64).reshape(3, 3)
    Cn = R @ C @ R.T
    return Cn.reshape(-1)


def _transform_imu(msg, R_bi: np.ndarray, typestore, typename: str) -> bytes:
    """R_bi: v_base = R_bi @ v_imu (identical to mcap_add_bodyframe_topics._transform_imu)."""
    w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    wb = R_bi @ w
    ab = R_bi @ a
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = wb.tolist()
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ab.tolist()

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


def _cov6_rotate(cov_row: list | np.ndarray, R: np.ndarray) -> list[float]:
    """Rotate nav_msgs/Odometry twist 6x6 covariance: linear and angular 3x3 blocks by R."""
    C = np.array(cov_row, dtype=np.float64).reshape(6, 6)
    Rb = np.zeros((6, 6), dtype=np.float64)
    Rb[0:3, 0:3] = R
    Rb[3:6, 3:6] = R
    return (Rb @ C @ Rb.T).reshape(-1).tolist()


def _transform_odom_twist_rl_style(
    msg,
    R_bd: np.ndarray,
    t_bd: np.ndarray,
    omega_base: np.ndarray,
    typestore,
    typename: str,
    *,
    rotate_covariance: bool,
) -> bytes:
    """
    Match prepareTwist linear part: twist_lin = R * twist_lin + t.cross(omega).
    t = dvl origin expressed in base_link (tf base_link from dvl lookup convention).
    omega = body angular velocity in base_link (rad/s), same frame as filter velocity state.
    """
    lin = np.array(
        [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
        dtype=np.float64,
    )
    ang = np.array(
        [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z],
        dtype=np.float64,
    )
    lin_b = R_bd @ lin + np.cross(t_bd, omega_base)
    ang_b = R_bd @ ang
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = lin_b.tolist()
    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = ang_b.tolist()
    msg.child_frame_id = "base_link"
    if rotate_covariance and len(msg.twist.covariance) >= 36:
        msg.twist.covariance = _cov6_rotate(msg.twist.covariance, R_bd)
    return typestore.serialize_cdr(msg, typename)


def _build_imu_stamp_index(reader, imu_conn) -> tuple[list[int], list[np.ndarray]]:
    times: list[int] = []
    omegas: list[np.ndarray] = []
    for c, _ts, raw in reader.messages(connections=[imu_conn]):
        msg = reader.deserialize(raw, c.msgtype)
        w = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float64,
        )
        times.append(_stamp_to_ns(msg))
        omegas.append(w)
    return times, omegas


def _nearest_omega_imu(times: list[int], omegas: list[np.ndarray], t_ns: int) -> np.ndarray | None:
    if not times:
        return None
    i = bisect.bisect_left(times, t_ns)
    candidates: list[int] = []
    if i < len(times):
        candidates.append(i)
    if i > 0:
        candidates.append(i - 1)
    best_i = min(candidates, key=lambda j: abs(times[j] - t_ns))
    return omegas[best_i]


def _rltwist_output_dir(bag_dir: Path, out_root: Path, mirror_under: Path | None) -> Path:
    bag_dir = bag_dir.resolve()
    out_root = out_root.resolve()
    if mirror_under is not None:
        try:
            rel = bag_dir.relative_to(mirror_under.resolve())
            return out_root / rel.parent / f"{rel.name}__rltwist"
        except ValueError:
            pass
    return out_root / f"{bag_dir.name}__rltwist"


def find_bag_dirs(root: Path) -> list[Path]:
    dirs: list[Path] = []
    for meta in root.rglob("metadata.yaml"):
        d = meta.parent
        if d.name.endswith("__bodyframe") or d.name.endswith("__rltwist"):
            continue
        if any(d.glob("*.mcap")):
            dirs.append(d)
    return sorted(set(dirs))


def process_one_bag(
    bag_dir: Path,
    out_root: Path,
    *,
    mirror_under: Path | None,
    imu_topic: str,
    dvl_topic: str,
    out_dvl_topic: str,
    out_imu_topic: str,
    skip_imu_baselink: bool,
    dry_run: bool,
    rotate_covariance: bool,
) -> Path | None:
    bag_dir = bag_dir.resolve()
    R_imu, R_dvl, t_dvl = _collect_static_tf_from_bag(bag_dir)
    if R_imu is None:
        print(f"[skip] {bag_dir.name}: no base_link->imu_link in /tf_static", file=sys.stderr)
        return None
    if R_dvl is None or t_dvl is None:
        print(f"[skip] {bag_dir.name}: no base_link->dvl_a50_link in /tf_static", file=sys.stderr)
        return None

    out_dir = _rltwist_output_dir(bag_dir, out_root, mirror_under)
    if out_dir.exists():
        print(f"[skip] {out_dir} already exists (delete manually to regenerate)", file=sys.stderr)
        return None
    if dry_run:
        print(f"[dry-run] would create {out_dir}")
        return out_dir

    with AnyReader([bag_dir]) as reader:
        typestore = reader.typestore
        tf_conns = [c for c in reader.connections if c.topic == "/tf_static"]
        imu_conns = [c for c in reader.connections if c.topic == imu_topic]
        dvl_conns = [c for c in reader.connections if c.topic == dvl_topic]

        if not tf_conns or not imu_conns or not dvl_conns:
            print(
                f"[skip] {bag_dir.name}: need /tf_static, {imu_topic}, {dvl_topic}",
                file=sys.stderr,
            )
            return None

        imu_src = imu_conns[0]
        dvl_src = dvl_conns[0]

        stamp_list, omega_imu_list = _build_imu_stamp_index(reader, imu_src)
        if not stamp_list:
            print(f"[skip] {bag_dir.name}: no IMU messages", file=sys.stderr)
            return None

    # Second pass: write output (AnyReader context closed; reopen)
    with AnyReader([bag_dir]) as reader:
        typestore = reader.typestore
        tf_conns = [c for c in reader.connections if c.topic == "/tf_static"]
        imu_conns = [c for c in reader.connections if c.topic == imu_topic]
        dvl_conns = [c for c in reader.connections if c.topic == dvl_topic]
        imu_src = imu_conns[0]
        dvl_src = dvl_conns[0]

        with Writer(out_dir, version=9, storage_plugin=StoragePlugin.MCAP) as writer:

            def add_like(src, topic: str) -> object:
                assert isinstance(src.ext, ConnectionExtRosbag2)
                return writer.add_connection(
                    topic,
                    src.msgtype,
                    typestore=typestore,
                    offered_qos_profiles=src.ext.offered_qos_profiles,
                )

            out_tf = add_like(tf_conns[0], "/tf_static")
            out_imu = add_like(imu_src, out_imu_topic) if not skip_imu_baselink else None
            out_dvl = add_like(dvl_src, out_dvl_topic)

            for c, ts, raw in reader.messages(connections=tf_conns):
                writer.write(out_tf, ts, raw)

            if out_imu is not None:
                for c, ts, raw in reader.messages(connections=imu_conns):
                    msg = reader.deserialize(raw, c.msgtype)
                    blob = _transform_imu(msg, R_imu, typestore, imu_src.msgtype)
                    writer.write(out_imu, ts, blob)

            for c, ts, raw in reader.messages(connections=dvl_conns):
                msg = reader.deserialize(raw, c.msgtype)
                t_ns = _stamp_to_ns(msg)
                w_imu = _nearest_omega_imu(stamp_list, omega_imu_list, t_ns)
                if w_imu is None:
                    continue
                omega_base = R_imu @ w_imu
                blob = _transform_odom_twist_rl_style(
                    msg,
                    R_dvl,
                    t_dvl,
                    omega_base,
                    typestore,
                    dvl_src.msgtype,
                    rotate_covariance=rotate_covariance,
                )
                writer.write(out_dvl, ts, blob)

    print(f"[ok] {out_dir}")
    return out_dir


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
        default=Path("recordings/rosbags_rltwist"),
        help="Parent directory for output bags (*__rltwist).",
    )
    ap.add_argument(
        "--mirror-structure-under",
        type=Path,
        default=None,
        help=(
            "Preserve subdirs relative to this path (e.g. recordings/rosbags → "
            "out_root/<date>/<test>__rltwist). Default: same as --scan-root when set."
        ),
    )
    ap.add_argument("--imu-topic", default="/imu/data")
    ap.add_argument("--dvl-topic", default="/sensors/dvl/odometry_cov")
    ap.add_argument(
        "--out-dvl-topic",
        default="/sensors/dvl/odometry_cov_rltwist",
        help="Topic name for the adjusted odometry in the output bag.",
    )
    ap.add_argument(
        "--out-imu-topic",
        default="/imu/data_baselink",
        help="Topic for IMU expressed in base_link (same transform as bodyframe script).",
    )
    ap.add_argument(
        "--skip-imu",
        action="store_true",
        help="Do not write IMU base_link topic (DVL + tf_static only).",
    )
    ap.add_argument(
        "--rotate-covariance",
        action="store_true",
        help="Rotate twist 6x6 covariance like robot_localization (linear/angular blocks by R).",
    )
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
            out_dvl_topic=args.out_dvl_topic,
            out_imu_topic=args.out_imu_topic,
            skip_imu_baselink=args.skip_imu,
            dry_run=args.dry_run,
            rotate_covariance=args.rotate_covariance,
        ):
            ok += 1
    print(f"Done. {ok}/{len(bags)} written.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
