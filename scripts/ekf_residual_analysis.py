#!/usr/bin/env python3
"""
EKF residual analysis for process-noise (Q) tuning of the POLARIS local EKF.

The local EKF (robot_localization, 30 Hz, config: ekf_local.yaml) fuses:
    IMU       /imu/data                   -> roll, pitch, yaw          (indices 3-5)
    DVL       /sensors/dvl/odometry_cov   -> vx, vy, vz (in base_link) (indices 6-8)
    Pressure  /sensors/pressure/pose_enu  -> z                         (index  2)

Phase 1: Sanity checks (topic presence, rates, TF static, timestamp overlap).
Phase 2: Per-sensor-axis residuals (measurement - EKF). Plots + JSON per bag,
         aggregate summary for batch runs.

Residual computation:
  IMU orientation : quat->Euler(IMU in base_link via Rz(pi) from /tf_static)
                    minus quat->Euler(EKF); yaw wrapped to [-pi, pi].
  IMU omega (diag): R_base_from_imu @ omega_imu  minus  omega_ekf.
  DVL velocity    : prepareTwist replication
                    v_base = R_base_from_dvl @ v_dvl + t_base_from_dvl x omega_base
                    minus v_ekf.  omega_base = R_base_from_imu @ nearest IMU omega.
  Pressure z      : z_pressure - z_ekf (both already in 'odom' frame).

Bag-type -> axis selection (auto from prefix):
  vertical_*       : pressure.z, dvl.vz, imu.roll, imu.pitch
  straight_surge_* : dvl.vx, dvl.vy, imu.roll, imu.pitch
  yaw_turns_*      : imu.yaw, imu.omega_z, imu.roll, imu.pitch
  depth_hold_*     : imu.roll, imu.pitch, pressure.z, dvl.vz
  stationary_*     : all axes (baseline; not strictly rigid - small offsets OK)
  global_run_*     : skipped (reserved for dead-reckoning analysis)

Dependencies: rosbags, numpy, scipy, matplotlib

CLI:
    python scripts/ekf_residual_analysis.py <bag_dir>
        [--sensors all|dvl|imu|pressure[,...]] [--output-dir DIR]
        [--max-match-gap 0.05]

If <bag_dir> contains metadata.yaml it is processed as a single bag; otherwise
all child rosbag directories are discovered and processed.
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

try:
    from scipy.spatial.transform import Rotation as Rot
    from scipy import stats as sp_stats
except ImportError:
    print("Install: pip install scipy", file=sys.stderr)
    raise

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("Install: pip install matplotlib", file=sys.stderr)
    raise


# ──────────────────────────────────────────────────────────────────── constants

TOPIC_IMU = "/imu/data"
TOPIC_DVL_COV = "/sensors/dvl/odometry_cov"
TOPIC_DVL_VEL = "/sensors/dvl/velocity"
TOPIC_PRESSURE = "/sensors/pressure/pose_enu"
TOPIC_EKF = "/odometry/filtered/local"
TOPIC_TF_STATIC = "/tf_static"

REQUIRED_TOPICS = (
    TOPIC_IMU, TOPIC_DVL_COV, TOPIC_DVL_VEL, TOPIC_PRESSURE, TOPIC_EKF, TOPIC_TF_STATIC,
)

# Expected rates (Hz)
EXPECT_EKF_HZ = 30.0
EXPECT_IMU_HZ = 86.0
EXPECT_DVL_HZ = 9.5
EXPECT_PRESSURE_HZ = 10.0

# robot_localization sensor_timeout = 0.2 s
EKF_GAP_THRESHOLD_S = 0.2

# DVL lock-lost covariance sentinel (from measurement_noise_constants.py: no_lock_variance=1e6)
DVL_LOCK_LOST_COV = 1.0e4

# Reference Q diagonal (from ekf_local.yaml:61-77). 15-state.
Q_DIAG_REF = [
    0.12, 0.12, 0.08,   # x, y, z
    0.03, 0.03, 0.08,   # roll, pitch, yaw
    0.12, 0.12, 0.09,   # vx, vy, vz
    0.02, 0.02, 0.06,   # wx, wy, wz
    0.005, 0.005, 0.005  # ax, ay, az
]

# Fallback R values (from measurement_noise_constants.py) when message covariance
# is the -1 sentinel or zero.
R_FALLBACK_IMU_GYRO = (2.072e-06, 2.200e-06, 2.104e-06)
R_FALLBACK_DVL = (4.42e-06, 7.837e-06, 1.0e-06)  # vx, vy, vz (with vz floor)

# ASCII (no emoji, no unicode math) for Windows console friendliness.
PASS, WARN, FAIL = "PASS", "WARN", "FAIL"

# Dark theme (matches scripts/stationary_allan_variance.py)
plt.rcParams.update({
    "figure.facecolor": "#0c0f12",
    "axes.facecolor":   "#141a20",
    "axes.edgecolor":   "#1e2832",
    "axes.labelcolor":  "#8b9caa",
    "xtick.color":      "#8b9caa",
    "ytick.color":      "#8b9caa",
    "text.color":       "#e6edf3",
    "grid.color":       "#1e2832",
    "grid.linewidth":   0.5,
    "legend.facecolor": "#141a20",
    "legend.edgecolor": "#1e2832",
})
COLOR_RESID = "#3dd6c6"
COLOR_ENV   = "#e8b86d"
COLOR_ZERO  = "#8b9caa"


# ──────────────────────────────────────────────────────────────────── utilities

def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9

def _bag_type(bag_name: str) -> str:
    prefixes = ("vertical", "straight_surge", "yaw_turns", "depth_hold",
                "stationary", "global_run")
    low = bag_name.lower()
    for p in prefixes:
        if low.startswith(p):
            return p
    return "other"

def _axes_for_bag_type(bt: str) -> list[str]:
    if bt == "vertical":        return ["pressure.z", "dvl.vz", "imu.roll", "imu.pitch"]
    if bt == "straight_surge":  return ["dvl.vx", "dvl.vy", "imu.roll", "imu.pitch"]
    if bt == "yaw_turns":       return ["imu.yaw", "imu.omega_z", "imu.roll", "imu.pitch"]
    if bt == "depth_hold":      return ["imu.roll", "imu.pitch", "pressure.z", "dvl.vz"]
    if bt == "stationary":      return ALL_AXES[:]
    if bt == "global_run":      return []
    return ALL_AXES[:]

ALL_AXES = [
    "imu.roll", "imu.pitch", "imu.yaw",
    "imu.omega_x", "imu.omega_y", "imu.omega_z",
    "dvl.vx", "dvl.vy", "dvl.vz",
    "pressure.z",
]

# Per-axis location of the EKF posterior variance for that state.
# kind: "pose" = pose.covariance (x,y,z,roll,pitch,yaw)
#       "twist" = twist.covariance (vx,vy,vz,wx,wy,wz)
# diag_idx is the [k,k] index in the row-major 6x6 (k*6+k).
AXIS_TO_EKF_COV: dict[str, tuple[str, int]] = {
    "pressure.z":  ("pose",  2 * 6 + 2),   # 14
    "imu.roll":    ("pose",  3 * 6 + 3),   # 21
    "imu.pitch":   ("pose",  4 * 6 + 4),   # 28
    "imu.yaw":     ("pose",  5 * 6 + 5),   # 35
    "dvl.vx":      ("twist", 0 * 6 + 0),   # 0
    "dvl.vy":      ("twist", 1 * 6 + 1),   # 7
    "dvl.vz":      ("twist", 2 * 6 + 2),   # 14
    "imu.omega_x": ("twist", 3 * 6 + 3),   # 21
    "imu.omega_y": ("twist", 4 * 6 + 4),   # 28
    "imu.omega_z": ("twist", 5 * 6 + 5),   # 35
}


def _ekf_state_var(bag: BagRead, axis: str, j: int) -> float:
    """Posterior state variance from EKF output (HPH^T for identity-H measurements)."""
    if axis not in AXIS_TO_EKF_COV:
        return float("nan")
    kind, idx = AXIS_TO_EKF_COV[axis]
    arr = bag.ekf_pose_cov if kind == "pose" else bag.ekf_twist_cov
    if j < 0 or j >= arr.shape[0] or idx >= arr.shape[1]:
        return float("nan")
    v = float(arr[j, idx])
    return v if (np.isfinite(v) and v > 0) else float("nan")

def _sensors_filter(axes: list[str], keep: set[str]) -> list[str]:
    if not keep or "all" in keep:
        return axes
    out = []
    for a in axes:
        sensor = a.split(".")[0]
        if sensor in keep:
            out.append(a)
    return out


def _find_bag_dirs(root: Path) -> list[Path]:
    dirs: list[Path] = []
    for meta in root.rglob("metadata.yaml"):
        d = meta.parent.resolve()
        if any(d.glob("*.mcap")):
            dirs.append(d)
    return sorted(set(dirs))


def _wrap_angle(x):
    return (x + math.pi) % (2.0 * math.pi) - math.pi


# ──────────────────────────────────────────────────────────────────── static TF

@dataclass
class StaticTF:
    R_base_imu: np.ndarray | None = None  # v_base = R @ v_imu
    R_base_dvl: np.ndarray | None = None  # v_base = R @ v_dvl
    t_base_dvl: np.ndarray | None = None  # dvl origin expressed in base_link
    rpy_imu: tuple[float, float, float] | None = None
    rpy_dvl: tuple[float, float, float] | None = None
    t_imu: np.ndarray | None = None


def _extract_static_tf(reader: AnyReader) -> StaticTF:
    out = StaticTF()
    conns = [c for c in reader.connections if c.topic == TOPIC_TF_STATIC]
    if not conns:
        return out
    for c, _ts, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, c.msgtype)
        for tr in msg.transforms:
            if tr.header.frame_id != "base_link":
                continue
            q = tr.transform.rotation
            t = tr.transform.translation
            R = Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            rpy = tuple(Rot.from_matrix(R).as_euler("xyz"))
            tvec = np.array([t.x, t.y, t.z], dtype=np.float64)
            if tr.child_frame_id == "imu_link":
                out.R_base_imu = R
                out.rpy_imu = rpy
                out.t_imu = tvec
            elif tr.child_frame_id == "dvl_a50_link":
                out.R_base_dvl = R
                out.t_base_dvl = tvec
                out.rpy_dvl = rpy
    return out


# ──────────────────────────────────────────────────────────────────── bag read

@dataclass
class BagRead:
    ekf_t_ns: np.ndarray       # (N,)
    ekf_z: np.ndarray          # (N,)
    ekf_v: np.ndarray          # (N, 3)  vx, vy, vz in base_link (child_frame_id)
    ekf_euler: np.ndarray      # (N, 3)  roll, pitch, yaw (xyz Euler of odom-frame quat)
    ekf_omega: np.ndarray      # (N, 3)  wx, wy, wz in base_link
    ekf_pose_cov: np.ndarray   # (N, 36) row-major 6x6: [x,y,z,r,p,y]
    ekf_twist_cov: np.ndarray  # (N, 36) row-major 6x6: [vx,vy,vz,wx,wy,wz]

    imu_t_ns: np.ndarray       # (M,)
    imu_q_xyzw: np.ndarray     # (M, 4) raw quat in imu_link
    imu_omega: np.ndarray      # (M, 3) raw ang vel in imu_link
    imu_ori_cov: np.ndarray    # (M, 9) row-major
    imu_gyro_cov: np.ndarray   # (M, 9)
    imu_q_norm: np.ndarray     # (M,)

    dvl_t_ns: np.ndarray       # (K,) from /sensors/dvl/odometry_cov
    dvl_v_raw: np.ndarray      # (K, 3) twist.linear in dvl_a50_link
    dvl_twist_cov: np.ndarray  # (K, 36) 6x6 row-major
    dvl_lock: np.ndarray       # (K,) bool — True if not lock-lost

    dvl_vel_t_ns: np.ndarray       # (L,) from /sensors/dvl/velocity
    dvl_vel_valid: np.ndarray      # (L,) bool

    pressure_t_ns: np.ndarray  # (P,)
    pressure_z: np.ndarray     # (P,)
    pressure_z_cov: np.ndarray # (P,) scalar: cov[2,2]

    topics_found: dict[str, int]  # topic -> message count (all seen)
    static_tf: StaticTF


def _read_bag(bag_dir: Path) -> BagRead:
    # Buckets
    ekf_t, ekf_z, ekf_v, ekf_eu, ekf_w = [], [], [], [], []
    ekf_pc, ekf_tc = [], []
    imu_t, imu_q, imu_w, imu_oc, imu_gc, imu_qn = [], [], [], [], [], []
    dvl_t, dvl_v, dvl_tc, dvl_lk = [], [], [], []
    dvl_vel_t, dvl_vel_val = [], []
    pre_t, pre_z, pre_zc = [], [], []
    topics_found: dict[str, int] = {}
    static_tf = StaticTF()

    with AnyReader([bag_dir]) as reader:
        static_tf = _extract_static_tf(reader)

        conns_by_topic = {t: [c for c in reader.connections if c.topic == t]
                          for t in REQUIRED_TOPICS}
        for t, cs in conns_by_topic.items():
            topics_found[t] = sum(c.msgcount for c in cs) if cs else 0

        all_conns = [c for cs in conns_by_topic.values() for c in cs]
        if not all_conns:
            return _empty_bagread(topics_found, static_tf)

        for c, _log_ts, raw in reader.messages(connections=all_conns):
            try:
                msg = reader.deserialize(raw, c.msgtype)
            except Exception:
                continue
            if c.topic == TOPIC_EKF:
                t_ns = _stamp_ns(msg.header.stamp)
                p = msg.pose.pose.position
                v = msg.twist.twist.linear
                q = msg.pose.pose.orientation
                w = msg.twist.twist.angular
                eu = Rot.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
                ekf_t.append(t_ns)
                ekf_z.append(float(p.z))
                ekf_v.append([float(v.x), float(v.y), float(v.z)])
                ekf_eu.append(eu.tolist())
                ekf_w.append([float(w.x), float(w.y), float(w.z)])
                ekf_pc.append(list(msg.pose.covariance))
                ekf_tc.append(list(msg.twist.covariance))
            elif c.topic == TOPIC_IMU:
                t_ns = _stamp_ns(msg.header.stamp)
                q = msg.orientation
                w = msg.angular_velocity
                imu_t.append(t_ns)
                imu_q.append([float(q.x), float(q.y), float(q.z), float(q.w)])
                imu_w.append([float(w.x), float(w.y), float(w.z)])
                imu_oc.append(list(msg.orientation_covariance))
                imu_gc.append(list(msg.angular_velocity_covariance))
                imu_qn.append(math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w))
            elif c.topic == TOPIC_DVL_COV:
                t_ns = _stamp_ns(msg.header.stamp)
                v = msg.twist.twist.linear
                tc = list(msg.twist.covariance)
                # lock-lost sentinel: twist cov diagonals > 1e4
                diag0 = float(tc[0]) if tc else 0.0
                locked = diag0 < DVL_LOCK_LOST_COV
                dvl_t.append(t_ns)
                dvl_v.append([float(v.x), float(v.y), float(v.z)])
                dvl_tc.append(tc)
                dvl_lk.append(locked)
            elif c.topic == TOPIC_DVL_VEL:
                t_ns = _stamp_ns(msg.header.stamp)
                dvl_vel_t.append(t_ns)
                dvl_vel_val.append(bool(getattr(msg, "velocity_valid", True)))
            elif c.topic == TOPIC_PRESSURE:
                t_ns = _stamp_ns(msg.header.stamp)
                z = msg.pose.pose.position.z
                cov = msg.pose.covariance
                zc = float(cov[14]) if len(cov) >= 15 else 0.0
                pre_t.append(t_ns)
                pre_z.append(float(z))
                pre_zc.append(zc)

    def _to_arr(x, dt=np.float64):
        return np.asarray(x, dtype=dt) if x else np.empty(0, dtype=dt)

    return BagRead(
        ekf_t_ns = _to_arr(ekf_t, np.int64),
        ekf_z    = _to_arr(ekf_z),
        ekf_v    = _to_arr(ekf_v).reshape(-1, 3) if ekf_v else np.empty((0, 3)),
        ekf_euler= _to_arr(ekf_eu).reshape(-1, 3) if ekf_eu else np.empty((0, 3)),
        ekf_omega= _to_arr(ekf_w).reshape(-1, 3) if ekf_w else np.empty((0, 3)),
        ekf_pose_cov  = _to_arr(ekf_pc).reshape(-1, 36) if ekf_pc else np.empty((0, 36)),
        ekf_twist_cov = _to_arr(ekf_tc).reshape(-1, 36) if ekf_tc else np.empty((0, 36)),
        imu_t_ns = _to_arr(imu_t, np.int64),
        imu_q_xyzw = _to_arr(imu_q).reshape(-1, 4) if imu_q else np.empty((0, 4)),
        imu_omega  = _to_arr(imu_w).reshape(-1, 3) if imu_w else np.empty((0, 3)),
        imu_ori_cov  = _to_arr(imu_oc).reshape(-1, 9) if imu_oc else np.empty((0, 9)),
        imu_gyro_cov = _to_arr(imu_gc).reshape(-1, 9) if imu_gc else np.empty((0, 9)),
        imu_q_norm   = _to_arr(imu_qn),
        dvl_t_ns = _to_arr(dvl_t, np.int64),
        dvl_v_raw= _to_arr(dvl_v).reshape(-1, 3) if dvl_v else np.empty((0, 3)),
        dvl_twist_cov = _to_arr(dvl_tc).reshape(-1, 36) if dvl_tc else np.empty((0, 36)),
        dvl_lock = np.asarray(dvl_lk, dtype=bool) if dvl_lk else np.empty(0, dtype=bool),
        dvl_vel_t_ns  = _to_arr(dvl_vel_t, np.int64),
        dvl_vel_valid = np.asarray(dvl_vel_val, dtype=bool) if dvl_vel_val else np.empty(0, dtype=bool),
        pressure_t_ns = _to_arr(pre_t, np.int64),
        pressure_z    = _to_arr(pre_z),
        pressure_z_cov= _to_arr(pre_zc),
        topics_found = topics_found,
        static_tf = static_tf,
    )


def _empty_bagread(topics_found, static_tf) -> BagRead:
    z3 = np.empty((0, 3))
    z36 = np.empty((0, 36))
    return BagRead(
        ekf_t_ns=np.empty(0, dtype=np.int64), ekf_z=np.empty(0),
        ekf_v=z3, ekf_euler=z3, ekf_omega=z3,
        ekf_pose_cov=z36, ekf_twist_cov=z36,
        imu_t_ns=np.empty(0, dtype=np.int64),
        imu_q_xyzw=np.empty((0, 4)), imu_omega=z3,
        imu_ori_cov=np.empty((0, 9)), imu_gyro_cov=np.empty((0, 9)),
        imu_q_norm=np.empty(0),
        dvl_t_ns=np.empty(0, dtype=np.int64), dvl_v_raw=z3,
        dvl_twist_cov=np.empty((0, 36)),
        dvl_lock=np.empty(0, dtype=bool),
        dvl_vel_t_ns=np.empty(0, dtype=np.int64),
        dvl_vel_valid=np.empty(0, dtype=bool),
        pressure_t_ns=np.empty(0, dtype=np.int64),
        pressure_z=np.empty(0), pressure_z_cov=np.empty(0),
        topics_found=topics_found, static_tf=static_tf,
    )


# ─────────────────────────────────────────────────────────────── sanity checks

@dataclass
class SanityCheck:
    name: str
    status: str
    detail: str

@dataclass
class SanityReport:
    checks: list[SanityCheck] = field(default_factory=list)
    ekf_rate_hz: float = 0.0
    ekf_gaps: int = 0
    ekf_duration_s: float = 0.0
    ekf_jumps: int = 0
    imu_rate_hz: float = 0.0
    imu_nan_inf_count: int = 0
    imu_quat_norm_violations: int = 0
    dvl_rate_hz: float = 0.0
    dvl_vel_rate_hz: float = 0.0
    dvl_bottom_lock_fraction: float = 0.0
    dvl_odomcov_lock_fraction: float = 0.0
    dvl_nonmonotonic_count: int = 0
    pressure_rate_hz: float = 0.0
    pressure_nan_count: int = 0
    tf_imu_yaw: float | None = None
    tf_dvl_yaw: float | None = None
    tf_dvl_roll: float | None = None
    sensors_reliable: dict[str, bool] = field(default_factory=dict)

def _status_for_sensor(report: SanityReport, sensor: str) -> bool:
    for c in report.checks:
        if c.status == FAIL and sensor in c.name.lower():
            return False
    return True


def _rate_hz(t_ns: np.ndarray) -> float:
    if t_ns.size < 2:
        return 0.0
    dur = float(t_ns[-1] - t_ns[0]) * 1e-9
    return (t_ns.size - 1) / dur if dur > 0 else 0.0


def _run_sanity(bag: BagRead) -> SanityReport:
    rep = SanityReport()

    missing = [t for t in REQUIRED_TOPICS if bag.topics_found.get(t, 0) == 0]
    if missing:
        rep.checks.append(SanityCheck("topic_presence", FAIL,
            f"missing: {', '.join(missing)}"))
    else:
        rep.checks.append(SanityCheck("topic_presence", PASS,
            "all required topics present"))

    # EKF continuity
    if bag.ekf_t_ns.size >= 2:
        dt_ns = np.diff(bag.ekf_t_ns)
        rep.ekf_rate_hz = _rate_hz(bag.ekf_t_ns)
        rep.ekf_duration_s = float(bag.ekf_t_ns[-1] - bag.ekf_t_ns[0]) * 1e-9
        gaps = int(np.sum(dt_ns > int(EKF_GAP_THRESHOLD_S * 1e9)))
        rep.ekf_gaps = gaps
        # simple 5-sigma jump check on z and vx
        if bag.ekf_v.shape[0] >= 3:
            dvx = np.abs(np.diff(bag.ekf_v[:, 0]))
            thr = 5.0 * np.std(dvx) if np.std(dvx) > 0 else np.inf
            rep.ekf_jumps = int(np.sum(dvx > thr))
        status = PASS
        detail = (f"{bag.ekf_t_ns.size} msgs, {rep.ekf_rate_hz:.2f} Hz "
                  f"(expect ~{EXPECT_EKF_HZ}), gaps>{EKF_GAP_THRESHOLD_S}s = {gaps}")
        if gaps > 0 or abs(rep.ekf_rate_hz - EXPECT_EKF_HZ) > 5.0:
            status = WARN
        rep.checks.append(SanityCheck("ekf_continuity", status, detail))
    else:
        rep.checks.append(SanityCheck("ekf_continuity", FAIL,
            "fewer than 2 EKF messages"))

    # IMU health
    if bag.imu_t_ns.size >= 2:
        rep.imu_rate_hz = _rate_hz(bag.imu_t_ns)
        nan_q = int(np.sum(~np.isfinite(bag.imu_q_xyzw).all(axis=1)))
        nan_w = int(np.sum(~np.isfinite(bag.imu_omega).all(axis=1)))
        rep.imu_nan_inf_count = nan_q + nan_w
        rep.imu_quat_norm_violations = int(np.sum(np.abs(bag.imu_q_norm - 1.0) > 0.01))
        status = PASS
        detail = (f"{bag.imu_t_ns.size} msgs, {rep.imu_rate_hz:.2f} Hz "
                  f"(expect ~{EXPECT_IMU_HZ}), NaN/Inf={rep.imu_nan_inf_count}, "
                  f"|q|!=1 count={rep.imu_quat_norm_violations}")
        if rep.imu_nan_inf_count or rep.imu_quat_norm_violations \
                or abs(rep.imu_rate_hz - EXPECT_IMU_HZ) > 10.0:
            status = WARN
        rep.checks.append(SanityCheck("imu_health", status, detail))
    else:
        rep.checks.append(SanityCheck("imu_health", FAIL, "no IMU messages"))

    # DVL health
    if bag.dvl_t_ns.size >= 2:
        rep.dvl_rate_hz = _rate_hz(bag.dvl_t_ns)
        rep.dvl_odomcov_lock_fraction = float(np.mean(bag.dvl_lock))
        # monotonicity
        dt = np.diff(bag.dvl_t_ns)
        rep.dvl_nonmonotonic_count = int(np.sum(dt <= 0))
    if bag.dvl_vel_t_ns.size >= 2:
        rep.dvl_vel_rate_hz = _rate_hz(bag.dvl_vel_t_ns)
        rep.dvl_bottom_lock_fraction = float(np.mean(bag.dvl_vel_valid)) \
            if bag.dvl_vel_valid.size else 0.0
    if bag.dvl_t_ns.size >= 2 or bag.dvl_vel_t_ns.size >= 2:
        status = PASS
        detail = (f"odom_cov: {bag.dvl_t_ns.size} msgs, {rep.dvl_rate_hz:.2f} Hz "
                  f"(expect ~{EXPECT_DVL_HZ}), lock frac={rep.dvl_odomcov_lock_fraction:.2%} "
                  f"| velocity: bottom lock={rep.dvl_bottom_lock_fraction:.2%}, "
                  f"non-mono jumps={rep.dvl_nonmonotonic_count}")
        if rep.dvl_nonmonotonic_count or rep.dvl_odomcov_lock_fraction < 0.5:
            status = WARN
        rep.checks.append(SanityCheck("dvl_health", status, detail))
    else:
        rep.checks.append(SanityCheck("dvl_health", FAIL, "no DVL messages"))

    # Pressure health
    if bag.pressure_t_ns.size >= 2:
        rep.pressure_rate_hz = _rate_hz(bag.pressure_t_ns)
        rep.pressure_nan_count = int(np.sum(~np.isfinite(bag.pressure_z)))
        status = PASS
        detail = (f"{bag.pressure_t_ns.size} msgs, {rep.pressure_rate_hz:.2f} Hz "
                  f"(expect ~{EXPECT_PRESSURE_HZ}), NaN={rep.pressure_nan_count}")
        if rep.pressure_nan_count \
                or abs(rep.pressure_rate_hz - EXPECT_PRESSURE_HZ) > 3.0:
            status = WARN
        rep.checks.append(SanityCheck("pressure_health", status, detail))
    else:
        rep.checks.append(SanityCheck("pressure_health", FAIL, "no pressure messages"))

    # TF static
    tf = bag.static_tf
    tf_bits = []
    ok = True
    if tf.rpy_imu is not None:
        rep.tf_imu_yaw = tf.rpy_imu[2]
        tf_bits.append(f"imu rpy=({tf.rpy_imu[0]:.3f}, {tf.rpy_imu[1]:.3f}, "
                       f"{tf.rpy_imu[2]:.3f})")
        if abs(tf.rpy_imu[2] - math.pi) > 0.01:
            ok = False
            tf_bits.append("imu yaw != pi!")
    else:
        ok = False
        tf_bits.append("no base_link->imu_link")
    if tf.rpy_dvl is not None:
        rep.tf_dvl_yaw = tf.rpy_dvl[2]
        rep.tf_dvl_roll = tf.rpy_dvl[0]
        tf_bits.append(f"dvl rpy=({tf.rpy_dvl[0]:.3f}, {tf.rpy_dvl[1]:.3f}, "
                       f"{tf.rpy_dvl[2]:.3f})")
    else:
        ok = False
        tf_bits.append("no base_link->dvl_a50_link")
    rep.checks.append(SanityCheck("tf_static", PASS if ok else WARN, "; ".join(tf_bits)))

    # Timestamp overlap
    def _overlap(t: np.ndarray) -> bool:
        if t.size == 0 or bag.ekf_t_ns.size == 0:
            return False
        return not (t[-1] < bag.ekf_t_ns[0] or t[0] > bag.ekf_t_ns[-1])
    overlaps = {
        "imu":      _overlap(bag.imu_t_ns),
        "dvl":      _overlap(bag.dvl_t_ns),
        "pressure": _overlap(bag.pressure_t_ns),
    }
    ok_all = all(overlaps.values())
    rep.checks.append(SanityCheck("timestamp_overlap",
        PASS if ok_all else FAIL,
        ", ".join(f"{k}={'ok' if v else 'no'}" for k, v in overlaps.items())))

    # Per-sensor reliability (downstream flag in JSON)
    rep.sensors_reliable = {
        "imu":      overlaps["imu"]      and rep.imu_nan_inf_count == 0 and rep.imu_quat_norm_violations == 0,
        "dvl":      overlaps["dvl"]      and rep.dvl_nonmonotonic_count == 0,
        "pressure": overlaps["pressure"] and rep.pressure_nan_count == 0,
    }
    return rep


def _print_sanity(bag_name: str, rep: SanityReport) -> None:
    print(f"\n=== Phase 1: Sanity checks for {bag_name} ===")
    width = max(len(c.name) for c in rep.checks) + 2
    for c in rep.checks:
        print(f"  [{c.status:4s}] {c.name:<{width}s} {c.detail}")


# ─────────────────────────────────────────────────────────────── EKF matching

def _build_ekf_index(bag: BagRead) -> np.ndarray:
    return bag.ekf_t_ns  # already sorted (publish order)

def _nearest_ekf_idx(ekf_t: np.ndarray, t_ns: int, max_gap_ns: int) -> int:
    if ekf_t.size == 0:
        return -1
    i = int(np.searchsorted(ekf_t, t_ns))
    cands = []
    if i < ekf_t.size:
        cands.append(i)
    if i > 0:
        cands.append(i - 1)
    best = min(cands, key=lambda j: abs(int(ekf_t[j]) - t_ns))
    if abs(int(ekf_t[best]) - t_ns) > max_gap_ns:
        return -1
    return best


def _prior_ekf_idx(ekf_t: np.ndarray, t_ns: int, max_age_ns: int) -> int:
    """Largest j with ekf_t[j] < t_ns AND (t_ns - ekf_t[j]) <= max_age_ns.

    Used for the prior_approx residual mode: this is NOT a true EKF prior
    state; it is the previous published /odometry/filtered/local sample.
    Returns -1 if no such sample exists.
    """
    if ekf_t.size == 0:
        return -1
    j = int(np.searchsorted(ekf_t, t_ns, side="left")) - 1
    if j < 0:
        return -1
    age = t_ns - int(ekf_t[j])
    if age <= 0 or age > max_age_ns:
        return -1
    return j


def _match_ekf_idx(ekf_t: np.ndarray, t_ns: int, mode: str,
                   max_gap_ns: int, max_prior_age_ns: int) -> int:
    """Dispatch helper used by _compute_residual."""
    if mode == "prior_approx":
        return _prior_ekf_idx(ekf_t, t_ns, max_prior_age_ns)
    return _nearest_ekf_idx(ekf_t, t_ns, max_gap_ns)

def _nearest_idx_sorted(t_arr: np.ndarray, t_ns: int) -> int:
    """Closest index in sorted t_arr; no gap check."""
    if t_arr.size == 0:
        return -1
    i = int(np.searchsorted(t_arr, t_ns))
    cands = []
    if i < t_arr.size:
        cands.append(i)
    if i > 0:
        cands.append(i - 1)
    return min(cands, key=lambda j: abs(int(t_arr[j]) - t_ns))


# ─────────────────────────────────────────────────────────────── residuals

@dataclass
class AxisResult:
    axis: str
    t_rel_s: np.ndarray
    resid: np.ndarray
    R: np.ndarray            # per-sample measurement variance (sensor message)
    P: np.ndarray            # per-sample EKF state variance (posterior, or
                             # P_prior_approx in prior_approx mode — see mode)
    stats: dict[str, Any]    # var_over_S etc.
    mode: str = "posterior"
    dt_s: np.ndarray = field(default_factory=lambda: np.empty(0))
    n_skipped: int = 0
    n_total: int = 0         # total sensor samples considered (matched + skipped)


def _compute_residual(
    axis: str,
    bag: BagRead,
    max_gap_ns: int,
    reliability: dict[str, bool],
    mode: str = "posterior",
    max_prior_age_ns: int | None = None,
) -> AxisResult | None:
    """Compute per-sample residuals and matched-sample diagnostics.

    mode:
      - "posterior" (default, preserves existing behavior): use the nearest
        EKF /odometry/filtered/local sample within ``max_gap_ns``. The matched
        EKF state may already include the same sensor update, so this is
        post-update by construction.
      - "prior_approx": use the most recent EKF sample with timestamp
        STRICTLY BEFORE the sensor sample, within ``max_prior_age_ns``. This
        is not the true Kalman-filter prior — it is the previously-published
        EKF odometry sample. Useful as an offline diagnostic against the
        posterior collapse, especially for fused IMU axes.
    """
    sensor = axis.split(".")[0]
    if not reliability.get(sensor, True):
        # still compute but caller marks unreliable
        pass

    if bag.ekf_t_ns.size == 0:
        return None

    if max_prior_age_ns is None:
        max_prior_age_ns = max_gap_ns

    tf = bag.static_tf
    t0_ns = int(bag.ekf_t_ns[0])

    # IMU-based residuals
    if sensor == "imu":
        if bag.imu_t_ns.size == 0 or tf.R_base_imu is None:
            return None
        R_base_imu = tf.R_base_imu
        t_rel, resid, R_arr, P_arr, dt_arr = [], [], [], [], []
        n_skipped = 0
        n_total = int(bag.imu_t_ns.size)
        for k in range(bag.imu_t_ns.size):
            t_ns = int(bag.imu_t_ns[k])
            j = _match_ekf_idx(bag.ekf_t_ns, t_ns, mode,
                               max_gap_ns, max_prior_age_ns)
            if j < 0:
                n_skipped += 1
                continue
            dt_ns_signed = t_ns - int(bag.ekf_t_ns[j])  # >0 if EKF sample is older
            if axis in ("imu.roll", "imu.pitch", "imu.yaw"):
                qx, qy, qz, qw = bag.imu_q_xyzw[k]
                R_world_imu = Rot.from_quat([qx, qy, qz, qw]).as_matrix()
                R_world_base = R_world_imu @ R_base_imu.T
                eu_imu = Rot.from_matrix(R_world_base).as_euler("xyz")
                eu_ekf = bag.ekf_euler[j]
                idx = {"imu.roll": 0, "imu.pitch": 1, "imu.yaw": 2}[axis]
                r = eu_imu[idx] - eu_ekf[idx]
                if axis == "imu.yaw":
                    r = _wrap_angle(r)
                cov = bag.imu_ori_cov[k]
                r_var = float(cov[idx * 3 + idx]) if len(cov) >= 9 else 0.0
                if not np.isfinite(r_var) or r_var <= 0:
                    r_var = R_FALLBACK_IMU_GYRO[idx]
            else:  # imu.omega_*
                w_imu = bag.imu_omega[k]
                w_base = R_base_imu @ w_imu
                w_ekf = bag.ekf_omega[j]
                idx = {"imu.omega_x": 0, "imu.omega_y": 1, "imu.omega_z": 2}[axis]
                r = float(w_base[idx] - w_ekf[idx])
                cov = bag.imu_gyro_cov[k]
                r_var = float(cov[idx * 3 + idx]) if len(cov) >= 9 else 0.0
                if not np.isfinite(r_var) or r_var <= 0:
                    r_var = R_FALLBACK_IMU_GYRO[idx]
            p_var = _ekf_state_var(bag, axis, j)
            t_rel.append((t_ns - t0_ns) * 1e-9)
            resid.append(float(r))
            R_arr.append(r_var)
            P_arr.append(p_var)
            dt_arr.append(dt_ns_signed * 1e-9)
        return _finalize(axis, t_rel, resid, R_arr, P_arr,
                         dt_arr, mode, n_skipped, n_total)

    # DVL velocity residuals (prepareTwist)
    if sensor == "dvl":
        if bag.dvl_t_ns.size == 0 or tf.R_base_dvl is None or tf.t_base_dvl is None \
                or tf.R_base_imu is None or bag.imu_t_ns.size == 0:
            return None
        R_bd, t_bd = tf.R_base_dvl, tf.t_base_dvl
        R_bi = tf.R_base_imu
        t_rel, resid, R_arr, P_arr, dt_arr = [], [], [], [], []
        n_skipped = 0
        n_total = int(bag.dvl_t_ns.size)
        idx = {"dvl.vx": 0, "dvl.vy": 1, "dvl.vz": 2}[axis]
        for k in range(bag.dvl_t_ns.size):
            if not bool(bag.dvl_lock[k]):
                n_skipped += 1
                continue
            t_ns = int(bag.dvl_t_ns[k])
            j = _match_ekf_idx(bag.ekf_t_ns, t_ns, mode,
                               max_gap_ns, max_prior_age_ns)
            if j < 0:
                n_skipped += 1
                continue
            dt_ns_signed = t_ns - int(bag.ekf_t_ns[j])
            m = _nearest_idx_sorted(bag.imu_t_ns, t_ns)
            if m < 0:
                n_skipped += 1
                continue
            w_base = R_bi @ bag.imu_omega[m]
            v_dvl = bag.dvl_v_raw[k]
            v_base = R_bd @ v_dvl + np.cross(t_bd, w_base)
            v_ekf = bag.ekf_v[j]
            r = float(v_base[idx] - v_ekf[idx])
            tc = bag.dvl_twist_cov[k]
            r_var = float(tc[idx * 6 + idx]) if tc.size >= 36 else 0.0
            if not np.isfinite(r_var) or r_var <= 0 or r_var >= DVL_LOCK_LOST_COV:
                r_var = R_FALLBACK_DVL[idx]
            p_var = _ekf_state_var(bag, axis, j)
            t_rel.append((t_ns - t0_ns) * 1e-9)
            resid.append(r)
            R_arr.append(r_var)
            P_arr.append(p_var)
            dt_arr.append(dt_ns_signed * 1e-9)
        return _finalize(axis, t_rel, resid, R_arr, P_arr,
                         dt_arr, mode, n_skipped, n_total)

    # Pressure z residual
    if sensor == "pressure":
        if bag.pressure_t_ns.size == 0:
            return None
        t_rel, resid, R_arr, P_arr, dt_arr = [], [], [], [], []
        n_skipped = 0
        n_total = int(bag.pressure_t_ns.size)
        for k in range(bag.pressure_t_ns.size):
            t_ns = int(bag.pressure_t_ns[k])
            j = _match_ekf_idx(bag.ekf_t_ns, t_ns, mode,
                               max_gap_ns, max_prior_age_ns)
            if j < 0:
                n_skipped += 1
                continue
            dt_ns_signed = t_ns - int(bag.ekf_t_ns[j])
            r = float(bag.pressure_z[k] - bag.ekf_z[j])
            r_var = float(bag.pressure_z_cov[k])
            if not np.isfinite(r_var) or r_var <= 0:
                r_var = 4.0e-06  # launch param default
            p_var = _ekf_state_var(bag, axis, j)
            t_rel.append((t_ns - t0_ns) * 1e-9)
            resid.append(r)
            R_arr.append(r_var)
            P_arr.append(p_var)
            dt_arr.append(dt_ns_signed * 1e-9)
        return _finalize(axis, t_rel, resid, R_arr, P_arr,
                         dt_arr, mode, n_skipped, n_total)

    return None


def _finalize(axis: str, t_rel, resid, R_arr, P_arr,
              dt_arr=None, mode: str = "posterior",
              n_skipped: int = 0, n_total: int = 0) -> AxisResult | None:
    if not resid:
        return None
    t = np.asarray(t_rel, dtype=np.float64)
    r = np.asarray(resid, dtype=np.float64)
    R = np.asarray(R_arr, dtype=np.float64)
    P = np.asarray(P_arr, dtype=np.float64)
    dt = np.asarray(dt_arr if dt_arr is not None else [], dtype=np.float64)
    stats = _axis_stats(r, R, P, dt, mode, n_skipped, n_total)
    return AxisResult(axis=axis, t_rel_s=t, resid=r, R=R, P=P, stats=stats,
                      mode=mode, dt_s=dt, n_skipped=n_skipped, n_total=n_total)


def _axis_stats(r: np.ndarray, R: np.ndarray, P: np.ndarray,
                dt: np.ndarray | None = None, mode: str = "posterior",
                n_skipped: int = 0, n_total: int = 0) -> dict[str, Any]:
    n = int(r.size)
    if n == 0:
        return {"n": 0}
    mean = float(np.mean(r))
    median = float(np.median(r))
    std  = float(np.std(r, ddof=1)) if n > 1 else 0.0
    var  = float(std ** 2)
    # Median absolute deviation, scaled to be a consistent estimator of std under
    # a Gaussian model. Robust alternative to std for heavy-tailed residuals.
    mad = float(np.median(np.abs(r - median)))
    mad_std = mad * 1.4826
    skew = float(sp_stats.skew(r)) if n > 2 else 0.0
    R_mean = float(np.mean(R)) if R.size else 0.0
    P_finite = P[np.isfinite(P)] if P.size else P
    P_mean = float(np.mean(P_finite)) if P_finite.size else 0.0
    # S = HPH^T + R, scalar for these identity-H measurements.
    S = R + P
    S_finite = S[np.isfinite(S)]
    S_mean = float(np.mean(S_finite)) if S_finite.size else 0.0
    var_over_R = var / R_mean if R_mean > 0 else float("inf")
    var_over_P = var / P_mean if P_mean > 0 else float("inf")
    var_over_S = var / S_mean if S_mean > 0 else float("inf")
    # NIS: per-sample r^2/S. Mean is the textbook NIS test statistic (chi-square),
    # but for real-world bags the median is more robust to outlier samples
    # (lock blips, brief sensor glitches, transient EKF discontinuities).
    nis_mask = np.isfinite(S) & (S > 0)
    if np.any(nis_mask):
        nis_samples = (r[nis_mask] ** 2) / S[nis_mask]
        nis_mean = float(np.mean(nis_samples))
        nis_median = float(np.median(nis_samples))
    else:
        nis_mean = float("nan")
        nis_median = float("nan")
    # Ljung-Box at lag 20
    lb_p = float("nan")
    try:
        from statsmodels.stats.diagnostic import acorr_ljungbox
        lags = min(20, max(1, n // 10))
        res = acorr_ljungbox(r, lags=[lags], return_df=True)
        lb_p = float(res["lb_pvalue"].iloc[0])
    except Exception:
        # fallback: rough approximation with Box-Pierce Q at lag 20
        lags = min(20, max(1, n // 10))
        try:
            ac = _acf(r, lags)
            Q = n * np.sum(ac[1:lags + 1] ** 2)
            lb_p = float(1.0 - sp_stats.chi2.cdf(Q, df=lags))
        except Exception:
            pass
    # Match-quality diagnostics. For posterior, dt is signed (sensor - EKF) and
    # the absolute value is what bounds the match. For prior_approx, dt is
    # always >=0 (sensor strictly newer than EKF).
    if dt is None or dt.size == 0:
        dt_median_s = dt_p95_s = dt_max_s = float("nan")
    else:
        dt_abs = np.abs(dt)
        dt_median_s = float(np.median(dt_abs))
        dt_p95_s = float(np.percentile(dt_abs, 95))
        dt_max_s = float(np.max(dt_abs))
    return {
        "n": n, "n_skipped": int(n_skipped), "n_total": int(n_total),
        "residual_mode": mode,
        "mean": mean, "median": median,
        "std": std, "mad_std": mad_std, "variance": var,
        "skewness": skew,
        "R_mean": R_mean, "P_mean": P_mean, "S_mean": S_mean,
        # In prior_approx mode, P_mean is the previous-sample EKF state variance.
        # It is NOT the true Kalman prior — see _compute_residual docstring.
        "var_over_R": var_over_R, "var_over_P": var_over_P,
        "var_over_S": var_over_S,
        "nis_mean": nis_mean, "nis_median": nis_median,
        "dt_median_s": dt_median_s, "dt_p95_s": dt_p95_s, "dt_max_s": dt_max_s,
        "ljung_box_p": lb_p,
    }


def _acf(x: np.ndarray, nlags: int) -> np.ndarray:
    x = x - np.mean(x)
    var = np.dot(x, x)
    if var <= 0 or x.size < 2:
        return np.zeros(nlags + 1)
    out = np.empty(nlags + 1)
    out[0] = 1.0
    for k in range(1, nlags + 1):
        out[k] = np.dot(x[:-k], x[k:]) / var
    return out


# ─────────────────────────────────────────────────────────────── plotting

def _plot_axis(result: AxisResult, bag_name: str, out_dir: Path,
               reliable: bool) -> Path:
    fig = plt.figure(figsize=(12, 10))
    gs = fig.add_gridspec(3, 1, hspace=0.5)
    axis_title = result.axis
    suffix = "" if reliable else "  [UNRELIABLE: see sanity report]"
    fig.suptitle(
        f"{bag_name}  residual: {axis_title}  [mode={result.mode}]"
        f"{suffix}", fontsize=13)

    # 1) time series with +-2 sqrt(S) envelope, where S = R + HPH^T (innovation cov)
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(result.t_rel_s, result.resid, color=COLOR_RESID, lw=0.8,
             label="residual")
    S = result.R + np.where(np.isfinite(result.P), result.P, 0.0)
    env = 2.0 * np.sqrt(np.maximum(S, 0.0))
    ax1.plot(result.t_rel_s, env, color=COLOR_ENV, lw=0.5, alpha=0.7,
             label="+-2 sqrt(S)  (S = R + P)")
    ax1.plot(result.t_rel_s, -env, color=COLOR_ENV, lw=0.5, alpha=0.7)
    ax1.axhline(0.0, color=COLOR_ZERO, lw=0.5, alpha=0.7)
    ax1.set_xlabel("t - t0 [s]")
    ax1.set_ylabel(f"residual [{_axis_unit(result.axis)}]")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best", fontsize=8)

    # 2) histogram with Gaussian fit
    ax2 = fig.add_subplot(gs[1])
    bins = min(80, max(20, result.resid.size // 30))
    ax2.hist(result.resid, bins=bins, color=COLOR_RESID, alpha=0.7, density=True,
             label="residual")
    mu, sd = result.stats["mean"], result.stats["std"]
    if sd > 0:
        xs = np.linspace(mu - 4 * sd, mu + 4 * sd, 400)
        pdf = (1.0 / (sd * math.sqrt(2 * math.pi))) * np.exp(
            -0.5 * ((xs - mu) / sd) ** 2)
        ax2.plot(xs, pdf, color=COLOR_ENV, lw=1.3,
                 label=f"N(mu={mu:.3g}, sd={sd:.3g})")
    ax2.set_xlabel(f"residual [{_axis_unit(result.axis)}]")
    ax2.set_ylabel("density")
    s = result.stats
    ax2.set_title(
        f"median={s['median']:.3g} (mean={mu:.3g})  "
        f"MADstd={s['mad_std']:.3g} (std={sd:.3g})  skew={s['skewness']:.2f}\n"
        f"NIS median={s['nis_median']:.2f} (mean={s['nis_mean']:.2f})  "
        f"var/S={s['var_over_S']:.2f}  "
        f"(var/R={s['var_over_R']:.2g}, var/P={s['var_over_P']:.2g})",
        fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best", fontsize=8)

    # 3) autocorrelation, first 50 lags
    ax3 = fig.add_subplot(gs[2])
    nlags = min(50, max(5, result.resid.size // 20))
    acf = _acf(result.resid, nlags)
    lags = np.arange(nlags + 1)
    ax3.stem(lags, acf, linefmt=COLOR_RESID, markerfmt=" ", basefmt=" ")
    # 95% white-noise band
    if result.resid.size > 0:
        band = 1.96 / math.sqrt(result.resid.size)
        ax3.axhline(band, color=COLOR_ENV, lw=0.5, alpha=0.7, linestyle="--",
                    label="95% band")
        ax3.axhline(-band, color=COLOR_ENV, lw=0.5, alpha=0.7, linestyle="--")
    ax3.axhline(0.0, color=COLOR_ZERO, lw=0.5, alpha=0.7)
    ax3.set_xlabel("lag")
    ax3.set_ylabel("ACF")
    ax3.set_title(f"autocorrelation (first {nlags} lags)  "
                  f"Ljung-Box p={result.stats['ljung_box_p']:.3g}", fontsize=10)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="best", fontsize=8)

    axis_fname = result.axis.replace(".", "_")
    out_path = out_dir / f"{bag_name}_residual_{axis_fname}.png"
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path


def _axis_unit(axis: str) -> str:
    if axis.startswith("imu.") and axis in ("imu.roll", "imu.pitch", "imu.yaw"):
        return "rad"
    if axis.startswith("imu.omega"):
        return "rad/s"
    if axis.startswith("dvl."):
        return "m/s"
    if axis.startswith("pressure."):
        return "m"
    return ""


# ─────────────────────────────────────────────────────────────── per-bag run

def _process_one_bag(bag_dir: Path, out_dir: Path, sensors_keep: set[str],
                     max_match_gap_s: float,
                     mode: str = "posterior",
                     max_prior_age_s: float = 0.10) -> dict[str, Any] | None:
    bag_name = bag_dir.name
    bag_type = _bag_type(bag_name)
    if bag_type == "global_run":
        print(f"\n=== Skipping {bag_name} (global_run: reserved for DR analysis) ===")
        return None

    print(f"\n{'=' * 72}\nBag: {bag_name}   (type: {bag_type})  mode: {mode}")
    print(f"Output: {out_dir}")

    bag = _read_bag(bag_dir)
    rep = _run_sanity(bag)
    _print_sanity(bag_name, rep)

    axes = _sensors_filter(_axes_for_bag_type(bag_type), sensors_keep)
    if not axes:
        print(f"No residual axes selected for bag_type={bag_type} "
              f"with --sensors={sorted(sensors_keep) if sensors_keep else 'auto'}")
        axes_json = {}
    else:
        max_gap_ns = int(max_match_gap_s * 1e9)
        max_prior_age_ns = int(max_prior_age_s * 1e9)
        axes_json: dict[str, Any] = {}
        print(f"\n=== Phase 2: Residuals ({len(axes)} axes, mode={mode}) ===")
        out_dir.mkdir(parents=True, exist_ok=True)
        for axis in axes:
            result = _compute_residual(
                axis, bag, max_gap_ns, rep.sensors_reliable,
                mode=mode, max_prior_age_ns=max_prior_age_ns,
            )
            if result is None or result.resid.size == 0:
                print(f"  [skip] {axis}: no residuals (missing data or TF)")
                axes_json[axis] = {"n": 0, "reliable": False, "reason": "no_data",
                                   "residual_mode": mode}
                continue
            sensor = axis.split(".")[0]
            reliable = bool(rep.sensors_reliable.get(sensor, True))
            png_path = _plot_axis(result, bag_name, out_dir, reliable)
            entry = dict(result.stats)
            entry["reliable"] = reliable
            entry["plot"] = png_path.name
            axes_json[axis] = entry
            print(f"  [{axis:16s}] mode={mode:13s}  "
                  f"n={entry['n']:6d}/{entry.get('n_total', entry['n']):6d}  "
                  f"skip={entry.get('n_skipped', 0):5d}  "
                  f"dt med/p95/max={entry['dt_median_s']*1e3:.1f}/"
                  f"{entry['dt_p95_s']*1e3:.1f}/{entry['dt_max_s']*1e3:.1f} ms  "
                  f"NIS med={entry['nis_median']:.2g} (mean={entry['nis_mean']:.2g})  "
                  f"var/S={entry['var_over_S']:.2f}"
                  f"{'  (UNRELIABLE)' if not reliable else ''}")

    # JSON per bag
    sanity_json = {
        "ekf_rate_hz":              rep.ekf_rate_hz,
        "ekf_gaps":                 rep.ekf_gaps,
        "ekf_duration_s":           rep.ekf_duration_s,
        "ekf_5sigma_jumps":         rep.ekf_jumps,
        "imu_rate_hz":              rep.imu_rate_hz,
        "imu_nan_inf_count":        rep.imu_nan_inf_count,
        "imu_quat_norm_violations": rep.imu_quat_norm_violations,
        "dvl_odomcov_rate_hz":      rep.dvl_rate_hz,
        "dvl_velocity_rate_hz":     rep.dvl_vel_rate_hz,
        "dvl_bottom_lock_fraction": rep.dvl_bottom_lock_fraction,
        "dvl_odomcov_lock_fraction": rep.dvl_odomcov_lock_fraction,
        "dvl_nonmonotonic_count":   rep.dvl_nonmonotonic_count,
        "pressure_rate_hz":         rep.pressure_rate_hz,
        "pressure_nan_count":       rep.pressure_nan_count,
        "tf_imu_yaw_rad":           rep.tf_imu_yaw,
        "tf_dvl_yaw_rad":           rep.tf_dvl_yaw,
        "tf_dvl_roll_rad":          rep.tf_dvl_roll,
        "sensors_reliable":         rep.sensors_reliable,
        "checks": [{"name": c.name, "status": c.status, "detail": c.detail}
                   for c in rep.checks],
    }
    bag_json = {
        "bag": bag_name,
        "bag_type": bag_type,
        "Q_diagonal_ref": Q_DIAG_REF,
        "residual_mode": mode,
        "max_match_gap_s": max_match_gap_s,
        "max_prior_age_s": max_prior_age_s,
        "residual_mode_note": (
            "prior_approx uses the previous published EKF odometry sample "
            "before the sensor timestamp. This is not an exact EKF "
            "innovation, but avoids comparing against a posterior state "
            "that may already include the same measurement."
        ),
        "sanity": sanity_json,
        "residuals": axes_json,
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    out_json = out_dir / f"{bag_name}_ekf_residuals.json"
    out_json.write_text(json.dumps(bag_json, indent=2))
    print(f"Saved: {out_json.name}")
    return bag_json


# ─────────────────────────────────────────────────────────────── CLI

def _parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dir", type=Path,
                    help="Bag directory (single bag or parent of many)")
    ap.add_argument("--sensors", default="auto",
                    help="Comma list of {all,dvl,imu,pressure} or 'auto' (by bag type)")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output dir (default: <bag>/ekf_residual_analysis or "
                         "<root>/ekf_residual_analysis for batch)")
    ap.add_argument("--max-match-gap", type=float, default=0.05,
                    help="Max time gap sensor<->EKF in seconds, posterior mode "
                         "(default 0.05)")
    ap.add_argument("--residual-mode", default="posterior",
                    choices=["posterior", "prior_approx"],
                    help="posterior: nearest EKF sample (default; preserves "
                         "existing behavior). prior_approx: latest EKF sample "
                         "with timestamp strictly before the sensor stamp; "
                         "approximate, NOT a true Kalman prior.")
    ap.add_argument("--max-prior-age-sec", type=float, default=0.10,
                    help="Max age of the prior EKF sample in prior_approx mode "
                         "(default 0.10).")
    ap.add_argument("--include",
                    help="Comma-separated substrings; only bags whose dir name "
                         "contains at least one are processed. Batch mode only.")
    ap.add_argument("--exclude",
                    help="Comma-separated substrings; bags whose dir name contains "
                         "any are skipped. Applied after --include.")
    return ap.parse_args(argv)


def _split_csv(s: str | None) -> list[str]:
    if not s:
        return []
    return [tok.strip() for tok in s.split(",") if tok.strip()]


def _filter_bags(bags: list[Path], include: list[str], exclude: list[str]) -> list[Path]:
    out = bags
    if include:
        out = [b for b in out if any(tok in b.name for tok in include)]
    if exclude:
        out = [b for b in out if not any(tok in b.name for tok in exclude)]
    return out


def _resolve_sensors(arg: str) -> set[str]:
    if arg.strip().lower() in ("auto", ""):
        return set()
    tokens = {tok.strip().lower() for tok in arg.split(",") if tok.strip()}
    valid = {"all", "dvl", "imu", "pressure"}
    bad = tokens - valid
    if bad:
        print(f"WARNING: unknown --sensors tokens ignored: {sorted(bad)}",
              file=sys.stderr)
    return tokens & valid


def main(argv=None) -> int:
    args = _parse_args(argv)
    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        return 1

    sensors_keep = _resolve_sensors(args.sensors)
    is_single = (bag_dir / "metadata.yaml").exists()
    mode = args.residual_mode
    # Default output-dir name carries the mode so posterior/prior_approx runs
    # never clobber each other. If the user passes --output-dir explicitly we
    # leave it untouched (their responsibility to keep modes separated).
    default_dirname = f"ekf_residual_analysis_{mode}"

    if is_single:
        out_dir = args.output_dir or (bag_dir / default_dirname)
        entry = _process_one_bag(
            bag_dir, out_dir, sensors_keep, args.max_match_gap,
            mode=mode, max_prior_age_s=args.max_prior_age_sec,
        )
        return 0 if entry is not None else 2

    # Batch
    bags = _find_bag_dirs(bag_dir)
    if not bags:
        print(f"ERROR: no rosbag2 dirs under {bag_dir}", file=sys.stderr)
        return 1
    include = _split_csv(args.include)
    exclude = _split_csv(args.exclude)
    n_pre = len(bags)
    bags = _filter_bags(bags, include, exclude)
    if not bags:
        print(f"ERROR: filter excluded all {n_pre} bags "
              f"(include={include or 'any'}, exclude={exclude or 'none'})",
              file=sys.stderr)
        return 1
    batch_root = args.output_dir or (bag_dir / default_dirname)
    batch_root.mkdir(parents=True, exist_ok=True)
    print(f"Batch: {len(bags)}/{n_pre} bags under {bag_dir} "
          f"(include={include or 'any'}, exclude={exclude or 'none'})  "
          f"mode={mode}")
    print(f"Output root: {batch_root}")

    aggregate: dict[str, Any] = {
        "root": str(bag_dir),
        "n_bags_total_found": n_pre,
        "n_bags_processed": len(bags),
        "residual_mode": mode,
        "max_match_gap_s": args.max_match_gap,
        "max_prior_age_s": args.max_prior_age_sec,
        "residual_mode_note": (
            "prior_approx uses the previous published EKF odometry sample "
            "before the sensor timestamp. This is not an exact EKF "
            "innovation, but avoids comparing against a posterior state "
            "that may already include the same measurement."
        ),
        "include_filter": include or None,
        "exclude_filter": exclude or None,
        "Q_diagonal_ref": Q_DIAG_REF,
        "bags": {},
    }
    for b in bags:
        sub = batch_root / b.name
        entry = _process_one_bag(
            b, sub, sensors_keep, args.max_match_gap,
            mode=mode, max_prior_age_s=args.max_prior_age_sec,
        )
        if entry is not None:
            aggregate["bags"][b.name] = {
                "bag_type": entry["bag_type"],
                "sensors_reliable": entry["sanity"]["sensors_reliable"],
                "var_over_S":  {ax: d.get("var_over_S")
                                for ax, d in entry["residuals"].items()
                                if isinstance(d, dict) and "var_over_S" in d},
                "nis_median": {ax: d.get("nis_median")
                                for ax, d in entry["residuals"].items()
                                if isinstance(d, dict) and "nis_median" in d},
                "nis_mean":    {ax: d.get("nis_mean")
                                for ax, d in entry["residuals"].items()
                                if isinstance(d, dict) and "nis_mean" in d},
                "dt_median_s": {ax: d.get("dt_median_s")
                                for ax, d in entry["residuals"].items()
                                if isinstance(d, dict) and "dt_median_s" in d},
                "dt_p95_s":    {ax: d.get("dt_p95_s")
                                for ax, d in entry["residuals"].items()
                                if isinstance(d, dict) and "dt_p95_s" in d},
                "n_skipped":   {ax: d.get("n_skipped")
                                for ax, d in entry["residuals"].items()
                                if isinstance(d, dict) and "n_skipped" in d},
            }

    out_summary = batch_root / "ekf_residuals_summary.json"
    out_summary.write_text(json.dumps(aggregate, indent=2))
    print(f"\nSaved aggregate: {out_summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
