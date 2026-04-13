#!/usr/bin/env python3
"""
Compare raw rosbags with __rltwist companion bags from mcap_add_rl_style_dvl_twist.py.

Structural checks (same spirit as validate_bodyframe_companion_bags.py):
  - companion exists; /tf_static, /imu/data_baselink, DVL rltwist topic
  - /tf_static payloads byte-identical to raw
  - IMU and DVL message counts and log_time sequences match raw sources
  - min/max log_time envelope over tf + imu + dvl matches raw vs companion

Kinematic checks:
  - Each IMU: re-apply generator _transform_imu to raw sample; compare to companion
  - Each DVL: recompute R*v + t x (R_imu*omega_nearest) with same stamp rule as generator

Dependencies: pip install rosbags numpy scipy

Example:
  python scripts/validate_rltwist_companion_bags.py \\
      --rosbags-root recordings/rosbags --rltwist-root recordings/rosbags_rltwist
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags numpy scipy", file=sys.stderr)
    raise


def _load_generator():
    path = Path(__file__).resolve().parent / "mcap_add_rl_style_dvl_twist.py"
    spec = importlib.util.spec_from_file_location("mcap_rltwist_gen", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_GEN = None


def _gen():
    global _GEN
    if _GEN is None:
        _GEN = _load_generator()
    return _GEN


IMU_RAW = "/imu/data"
IMU_DER = "/imu/data_baselink"
DVL_RAW_DEFAULT = "/sensors/dvl/odometry_cov"
DVL_FALLBACK = "/sensors/dvl/odometry"
DVL_DER_DEFAULT = "/sensors/dvl/odometry_cov_rltwist"
TF_STATIC = "/tf_static"


def find_raw_bag_dirs(rosbags_root: Path) -> list[Path]:
    out: list[Path] = []
    for meta in rosbags_root.rglob("metadata.yaml"):
        d = meta.parent
        if d.name.endswith("__bodyframe") or d.name.endswith("__rltwist"):
            continue
        if any(d.glob("*.mcap")):
            out.append(d)
    return sorted(set(out))


def companion_dir(raw_dir: Path, rosbags_root: Path, rltwist_root: Path) -> Path:
    rel = raw_dir.resolve().relative_to(rosbags_root.resolve())
    return rltwist_root / rel.parent / f"{rel.name}__rltwist"


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
    starts: list[int] = []
    ends: list[int] = []
    for seq in (tf_t, imu_t, dvl_t):
        if seq:
            starts.append(seq[0])
            ends.append(seq[-1])
    if not starts:
        return None, None
    return min(starts), max(ends)


def _quat_xyzw_close(a, b, atol: float = 1e-4) -> bool:
    qa = np.array([a.x, a.y, a.z, a.w], dtype=np.float64)
    qb = np.array([b.x, b.y, b.z, b.w], dtype=np.float64)
    return min(np.linalg.norm(qa - qb), np.linalg.norm(qa + qb)) <= atol


def _validate_imu_kinematics(
    raw_dir: Path,
    comp_dir: Path,
    R_imu: np.ndarray,
    *,
    rtol: float,
    atol: float,
) -> tuple[list[str], list[str]]:
    issues: list[str] = []
    notes: list[str] = []
    g = _gen()
    n_checked = 0
    bad = 0
    first_bad: int | None = None
    with AnyReader([raw_dir]) as raw_r, AnyReader([comp_dir]) as comp_r:
        raw_conns = [c for c in raw_r.connections if c.topic == IMU_RAW]
        comp_conns = [c for c in comp_r.connections if c.topic == IMU_DER]
        if not raw_conns or not comp_conns:
            return issues, notes
        raw_c, comp_c = raw_conns[0], comp_conns[0]
        typestore = raw_r.typestore
        typename = raw_c.msgtype
        for i, ((cr, tsr, raw_blob), (cc, tsc, comp_blob)) in enumerate(
            zip(
                raw_r.messages(connections=[raw_c]),
                comp_r.messages(connections=[comp_c]),
                strict=False,
            )
        ):
            if int(tsr) != int(tsc):
                issues.append(f"IMU kinematics: log_time mismatch at index {i}")
                return issues, notes
            msg_work = raw_r.deserialize(raw_blob, cr.msgtype)
            msg_comp = comp_r.deserialize(comp_blob, cc.msgtype)
            g._transform_imu(msg_work, R_imu, typestore, typename)
            n_checked = i + 1
            ok = True
            if not np.allclose(
                [msg_work.angular_velocity.x, msg_work.angular_velocity.y, msg_work.angular_velocity.z],
                [msg_comp.angular_velocity.x, msg_comp.angular_velocity.y, msg_comp.angular_velocity.z],
                rtol=rtol,
                atol=atol,
            ):
                ok = False
            if ok and not np.allclose(
                [
                    msg_work.linear_acceleration.x,
                    msg_work.linear_acceleration.y,
                    msg_work.linear_acceleration.z,
                ],
                [
                    msg_comp.linear_acceleration.x,
                    msg_comp.linear_acceleration.y,
                    msg_comp.linear_acceleration.z,
                ],
                rtol=rtol,
                atol=atol,
            ):
                ok = False
            if (
                ok
                and msg_work.orientation_covariance[0] >= 0
                and msg_comp.orientation_covariance[0] >= 0
                and not _quat_xyzw_close(msg_work.orientation, msg_comp.orientation, atol=1e-4)
            ):
                ok = False
            if not ok:
                bad += 1
                if first_bad is None:
                    first_bad = i
    if n_checked == 0:
        return issues, notes
    if bad:
        issues.append(
            f"IMU kinematics: {bad} / {n_checked} messages differ from recomputed _transform_imu "
            f"(first index {first_bad})"
        )
    else:
        notes.append(f"IMU kinematics: all {n_checked} messages match _transform_imu recomputation")
    return issues, notes


def _validate_dvl_kinematics(
    raw_dir: Path,
    comp_dir: Path,
    dvl_raw_topic: str,
    dvl_der_topic: str,
    R_imu: np.ndarray,
    R_dvl: np.ndarray,
    t_dvl: np.ndarray,
    *,
    rtol: float,
    atol: float,
    rotate_covariance: bool,
) -> tuple[list[str], list[str]]:
    issues: list[str] = []
    notes: list[str] = []
    g = _gen()
    with AnyReader([raw_dir]) as raw_r:
        imu_c = [c for c in raw_r.connections if c.topic == IMU_RAW][0]
        stamp_list, omega_list = g._build_imu_stamp_index(raw_r, imu_c)
    if not stamp_list:
        issues.append("DVL kinematics: no IMU index for omega lookup")
        return issues, notes

    n_checked = 0
    bad = 0
    first_bad: int | None = None
    with AnyReader([raw_dir]) as raw_r, AnyReader([comp_dir]) as comp_r:
        dvl_raw_c = [c for c in raw_r.connections if c.topic == dvl_raw_topic]
        dvl_comp_c = [c for c in comp_r.connections if c.topic == dvl_der_topic]
        if not dvl_raw_c or not dvl_comp_c:
            return issues, notes
        dr, dc = dvl_raw_c[0], dvl_comp_c[0]
        typestore = raw_r.typestore
        for i, ((cr, tsr, raw_blob), (cc, tsc, comp_blob)) in enumerate(
            zip(
                raw_r.messages(connections=[dr]),
                comp_r.messages(connections=[dc]),
                strict=False,
            )
        ):
            if int(tsr) != int(tsc):
                issues.append(f"DVL kinematics: log_time mismatch at index {i}")
                return issues, notes
            msg_raw = raw_r.deserialize(raw_blob, cr.msgtype)
            t_ns = g._stamp_to_ns(msg_raw)
            msg_comp = comp_r.deserialize(comp_blob, cc.msgtype)
            w_imu = g._nearest_omega_imu(stamp_list, omega_list, t_ns)
            if w_imu is None:
                issues.append(f"DVL kinematics: no omega at index {i}")
                return issues, notes
            omega_base = R_imu @ w_imu
            lin = np.array(
                [
                    msg_raw.twist.twist.linear.x,
                    msg_raw.twist.twist.linear.y,
                    msg_raw.twist.twist.linear.z,
                ],
                dtype=np.float64,
            )
            ang = np.array(
                [
                    msg_raw.twist.twist.angular.x,
                    msg_raw.twist.twist.angular.y,
                    msg_raw.twist.twist.angular.z,
                ],
                dtype=np.float64,
            )
            lin_e = R_dvl @ lin + np.cross(t_dvl, omega_base)
            ang_e = R_dvl @ ang
            n_checked = i + 1
            ok = np.allclose(
                lin_e,
                [
                    msg_comp.twist.twist.linear.x,
                    msg_comp.twist.twist.linear.y,
                    msg_comp.twist.twist.linear.z,
                ],
                rtol=rtol,
                atol=atol,
            ) and np.allclose(
                ang_e,
                [
                    msg_comp.twist.twist.angular.x,
                    msg_comp.twist.twist.angular.y,
                    msg_comp.twist.twist.angular.z,
                ],
                rtol=rtol,
                atol=atol,
            )
            if rotate_covariance and len(msg_raw.twist.covariance) >= 36:
                cov_e = np.array(g._cov6_rotate(msg_raw.twist.covariance, R_dvl), dtype=np.float64)
                cov_c = np.array(msg_comp.twist.covariance, dtype=np.float64)
                if not np.allclose(cov_e, cov_c, rtol=rtol, atol=atol):
                    ok = False
            if not ok:
                bad += 1
                if first_bad is None:
                    first_bad = i
    if n_checked == 0:
        return issues, notes
    if bad:
        issues.append(
            f"DVL kinematics: {bad} / {n_checked} messages differ from RL-style recomputation "
            f"(first index {first_bad})"
        )
    else:
        notes.append(
            f"DVL kinematics: all {n_checked} messages match R*v + t x omega (and angular R*v)"
            + ("; covariance checked" if rotate_covariance else "")
        )
    return issues, notes


def analyze_pair(
    raw_dir: Path,
    comp_dir: Path,
    *,
    dvl_raw_topic: str,
    dvl_der_topic: str,
    skip_kinematics: bool,
    rtol: float,
    atol: float,
    check_rotated_covariance: bool,
) -> dict:
    issues: list[str] = []
    notes: list[str] = []
    g = _gen()

    if not comp_dir.is_dir() or not (comp_dir / "metadata.yaml").exists():
        return {
            "raw_bag": str(raw_dir),
            "companion_bag": str(comp_dir),
            "status": "issue",
            "issues": ["missing companion bag directory or metadata.yaml"],
            "notes": [],
        }

    R_imu, R_dvl, t_dvl = g._collect_static_tf_from_bag(raw_dir)
    if R_imu is None or R_dvl is None or t_dvl is None:
        issues.append("raw bag: missing base_link→imu_link or base_link→dvl_a50_link in /tf_static")

    dvl_topic_used = dvl_raw_topic
    with AnyReader([raw_dir]) as raw_r, AnyReader([comp_dir]) as comp_r:
        imu_t_raw, _ = iter_topic_times_and_raw(raw_r, IMU_RAW)
        imu_t_comp, _ = iter_topic_times_and_raw(comp_r, IMU_DER)

    with AnyReader([raw_dir]) as r2:
        dvl_t_raw, _ = iter_topic_times_and_raw(r2, dvl_raw_topic)
    if not dvl_t_raw and dvl_raw_topic == DVL_RAW_DEFAULT:
        with AnyReader([raw_dir]) as r2b:
            dvl_t_raw, _ = iter_topic_times_and_raw(r2b, DVL_FALLBACK)
        if dvl_t_raw:
            dvl_topic_used = DVL_FALLBACK
            notes.append(
                f"raw bag has no {DVL_RAW_DEFAULT}; compared {DVL_FALLBACK} to {dvl_der_topic}"
            )

    with AnyReader([comp_dir]) as r3:
        dvl_t_comp, _ = iter_topic_times_and_raw(r3, dvl_der_topic)

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
        issues.append(f"missing {dvl_der_topic} in companion while raw has DVL messages")
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
            notes.append("derived-topic span (tf+imu+DVL) min/max log_time identical raw vs companion")

    if not skip_kinematics and R_imu is not None and R_dvl is not None and t_dvl is not None:
        ki, kn = _validate_imu_kinematics(
            raw_dir, comp_dir, R_imu, rtol=rtol, atol=atol
        )
        issues.extend(ki)
        notes.extend(kn)
        kd, nd = _validate_dvl_kinematics(
            raw_dir,
            comp_dir,
            dvl_topic_used,
            dvl_der_topic,
            R_imu,
            R_dvl,
            t_dvl,
            rtol=rtol,
            atol=atol,
            rotate_covariance=check_rotated_covariance,
        )
        issues.extend(kd)
        notes.extend(nd)

    def ns_dur(t0: int | None, t1: int | None) -> float | None:
        if t0 is None or t1 is None:
            return None
        return (t1 - t0) / 1e9

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
            "derived_topics_union_raw": ns_dur(derived_raw_t0, derived_raw_t1),
            "derived_topics_union_companion": ns_dur(derived_comp_t0, derived_comp_t1),
        },
        "status": status,
        "issues": issues,
        "notes": notes,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rosbags-root", type=Path, default=Path("recordings/rosbags"))
    ap.add_argument("--rltwist-root", type=Path, default=Path("recordings/rosbags_rltwist"))
    ap.add_argument("--dvl-raw-topic", default=DVL_RAW_DEFAULT)
    ap.add_argument(
        "--dvl-der-topic",
        default=DVL_DER_DEFAULT,
        help="Companion DVL topic (default: odometry_cov_rltwist)",
    )
    ap.add_argument(
        "--skip-kinematics",
        action="store_true",
        help="Only structural checks (counts, log_time, tf bytes, envelope)",
    )
    ap.add_argument("--rtol", type=float, default=1e-6)
    ap.add_argument("--atol", type=float, default=1e-8)
    ap.add_argument(
        "--expect-rotated-covariance",
        action="store_true",
        help="Companion was built with --rotate-covariance; verify DVL twist covariance too",
    )
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rosbags_root = args.rosbags_root.resolve()
    rltwist_root = args.rltwist_root.resolve()
    if not rosbags_root.is_dir():
        print(f"Missing rosbags root: {rosbags_root}", file=sys.stderr)
        return 1

    raw_dirs = find_raw_bag_dirs(rosbags_root)
    results: list[dict] = []
    missing: list[str] = []

    dvl_raw = args.dvl_raw_topic.rstrip("/")
    dvl_der = args.dvl_der_topic.rstrip("/")

    for raw_dir in raw_dirs:
        comp = companion_dir(raw_dir, rosbags_root, rltwist_root)
        if not comp.is_dir():
            missing.append(str(raw_dir))
            results.append(
                {
                    "raw_bag": str(raw_dir),
                    "companion_bag": str(comp),
                    "status": "issue",
                    "issues": ["missing __rltwist companion bag"],
                    "notes": [],
                }
            )
            continue
        results.append(
            analyze_pair(
                raw_dir,
                comp,
                dvl_raw_topic=dvl_raw,
                dvl_der_topic=dvl_der,
                skip_kinematics=args.skip_kinematics,
                rtol=args.rtol,
                atol=args.atol,
                check_rotated_covariance=args.expect_rotated_covariance,
            )
        )

    summary = {
        "raw_bag_count": len(raw_dirs),
        "pass_count": sum(1 for r in results if r.get("status") == "pass"),
        "issue_count": sum(1 for r in results if r.get("status") == "issue"),
        "missing_companion_paths": missing,
    }

    if args.json:
        print(json.dumps({"summary": summary, "bags": results}, indent=2))
        return 0

    print(
        f"Summary: {summary['pass_count']}/{summary['raw_bag_count']} pass, "
        f"{summary['issue_count']} with issues"
    )
    if missing:
        print(f"Missing __rltwist companions: {len(missing)}")
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
        for n in r.get("notes", []):
            print(f"  note: {n}")
        for i in r.get("issues", []):
            print(f"  issue: {i}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
