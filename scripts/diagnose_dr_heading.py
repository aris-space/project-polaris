#!/usr/bin/env python3
"""Diagnose dead-reckoning vs GNSS heading error across multiple bags.

Decomposes the visible DR-vs-GNSS rotation in /gps/filtered into:
  Analysis 1 - measured heading error during the first straight segment
  Analysis 2 - psi reconstruction (imu yaw + TF + yaw_offset + mag declination)
  Analysis 3 - what psi should be (from GNSS bearing)
  Analysis 4 - whether the Xsens driver is applying an internal rotation
  Analysis 5 - heading stability (does the magnetometer drift?)
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import utm
from mcap_ros2.reader import read_ros2_messages

T_FIX           = "/fix"
T_GPS_FILT      = "/gps/filtered"
T_IMU           = "/imu/data"
T_FILTER_EULER  = "/filter/euler"
T_FILTER_QUAT   = "/filter/quaternion"
T_TF_STATIC     = "/tf_static"
T_ROSOUT        = "/rosout"
T_PARAM_EVENTS  = "/parameter_events"
T_UBX_PVT       = "/ubx_nav_pvt"

WANTED = {T_FIX, T_GPS_FILT, T_IMU, T_FILTER_EULER, T_FILTER_QUAT,
          T_TF_STATIC, T_ROSOUT, T_PARAM_EVENTS, T_UBX_PVT}


@dataclass
class FixSample:
    t_ns: int; lat: float; lon: float


@dataclass
class TimedQuat:
    t_ns: int; qx: float; qy: float; qz: float; qw: float


@dataclass
class TimedRPY:
    t_ns: int; r: float; p: float; y: float


@dataclass
class StaticTransform:
    parent: str; child: str
    qx: float; qy: float; qz: float; qw: float


@dataclass
class RosoutLine:
    t_ns: int; name: str; msg: str; level: int


@dataclass
class ParamEvent:
    t_ns: int; node: str; new_params: dict; changed_params: dict


@dataclass
class UbxPvtSample:
    t_ns: int; head_mot_raw: int; g_speed_mm_s: int


@dataclass
class BagDiag:
    fix:           list = field(default_factory=list)
    dr:            list = field(default_factory=list)
    imu:           list = field(default_factory=list)
    filt_euler:    list = field(default_factory=list)
    filt_quat:     list = field(default_factory=list)
    tf_static:     list = field(default_factory=list)
    rosout:        list = field(default_factory=list)
    param_events:  list = field(default_factory=list)
    ubx_pvt:       list = field(default_factory=list)


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quat_to_yaw_rad(qx, qy, qz, qw) -> float:
    """ZYX yaw from quaternion (x,y,z,w). For ENU body quats, yaw is CCW from East."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_signed_deg(deg: float) -> float:
    x = ((deg + 180.0) % 360.0) - 180.0
    return x + 360.0 if x <= -180.0 else x


def wrap_compass_deg(deg: float) -> float:
    return deg % 360.0


def enu_yaw_to_compass(enu_deg: float) -> float:
    return wrap_compass_deg(90.0 - enu_deg)


def utm_xy(lat, lon):
    e, n, _, _ = utm.from_latlon(lat, lon)
    return float(e), float(n)


def _param_value_to_str(v) -> str:
    t = int(v.type)
    if t == 1: return str(bool(v.bool_value))
    if t == 2: return str(int(v.integer_value))
    if t == 3: return repr(float(v.double_value))
    if t == 4: return str(v.string_value)
    return f"<type={t}>"


def read_bag(bag_dir: Path) -> BagDiag:
    candidates = list(bag_dir.glob("*_0.mcap"))
    if not candidates:
        raise FileNotFoundError(f"no *_0.mcap in {bag_dir}")
    mcap_path = candidates[0]

    data = BagDiag()
    for msg in read_ros2_messages(str(mcap_path)):
        topic = msg.channel.topic
        if topic not in WANTED:
            continue
        ros = msg.ros_msg
        try:
            if topic == T_FIX:
                t = stamp_ns(ros.header.stamp)
                if t == 0: continue
                data.fix.append(FixSample(t, float(ros.latitude), float(ros.longitude)))
            elif topic == T_GPS_FILT:
                t = stamp_ns(ros.header.stamp)
                if t == 0: continue
                data.dr.append(FixSample(t, float(ros.latitude), float(ros.longitude)))
            elif topic == T_IMU:
                t = stamp_ns(ros.header.stamp)
                if t == 0: continue
                q = ros.orientation
                data.imu.append(TimedQuat(t, float(q.x), float(q.y), float(q.z), float(q.w)))
            elif topic == T_FILTER_EULER:
                t = stamp_ns(ros.header.stamp)
                if t == 0: continue
                v = ros.vector
                data.filt_euler.append(TimedRPY(t, float(v.x), float(v.y), float(v.z)))
            elif topic == T_FILTER_QUAT:
                t = stamp_ns(ros.header.stamp)
                if t == 0: continue
                q = ros.quaternion
                data.filt_quat.append(TimedQuat(t, float(q.x), float(q.y), float(q.z), float(q.w)))
            elif topic == T_TF_STATIC:
                for tr in ros.transforms:
                    q = tr.transform.rotation
                    data.tf_static.append(StaticTransform(
                        parent=str(tr.header.frame_id),
                        child=str(tr.child_frame_id),
                        qx=float(q.x), qy=float(q.y), qz=float(q.z), qw=float(q.w),
                    ))
            elif topic == T_ROSOUT:
                t = stamp_ns(ros.stamp) if hasattr(ros, "stamp") else 0
                data.rosout.append(RosoutLine(t, str(ros.name), str(ros.msg), int(ros.level)))
            elif topic == T_PARAM_EVENTS:
                t = stamp_ns(ros.stamp)
                np_d = {p.name: _param_value_to_str(p.value) for p in ros.new_parameters}
                cp_d = {p.name: _param_value_to_str(p.value) for p in ros.changed_parameters}
                data.param_events.append(ParamEvent(t, str(ros.node), np_d, cp_d))
            elif topic == T_UBX_PVT:
                t = stamp_ns(ros.header.stamp)
                if t == 0: continue
                data.ubx_pvt.append(UbxPvtSample(
                    t_ns=t,
                    head_mot_raw=int(ros.head_mot),
                    g_speed_mm_s=int(ros.g_speed),
                ))
        except (AttributeError, ValueError):
            continue
    return data


def find_straight_segment(fix_samples, min_dist_m=5.0, ratio_max=1.4):
    if len(fix_samples) < 2:
        return None
    e0, n0 = utm_xy(fix_samples[0].lat, fix_samples[0].lon)
    last_e, last_n = e0, n0
    path_len = 0.0
    for i in range(1, len(fix_samples)):
        e, n = utm_xy(fix_samples[i].lat, fix_samples[i].lon)
        path_len += math.hypot(e - last_e, n - last_n)
        last_e, last_n = e, n
        chord = math.hypot(e - e0, n - n0)
        if chord >= min_dist_m and chord > 0 and (path_len / chord) <= ratio_max:
            return 0, i, fix_samples[0].t_ns, fix_samples[i].t_ns
    return None


def measure_heading_error(fix_samples, dr_samples, ubx_pvt, t0, t1):
    fixes = [f for f in fix_samples if t0 <= f.t_ns <= t1]
    drs   = [d for d in dr_samples  if t0 <= d.t_ns <= t1]
    if len(fixes) < 2 or len(drs) < 2:
        return None
    ef0, nf0 = utm_xy(fixes[0].lat, fixes[0].lon)
    ef1, nf1 = utm_xy(fixes[-1].lat, fixes[-1].lon)
    ed0, nd0 = utm_xy(drs[0].lat, drs[0].lon)
    ed1, nd1 = utm_xy(drs[-1].lat, drs[-1].lon)

    gnss_b = math.degrees(math.atan2(ef1 - ef0, nf1 - nf0))
    dr_b   = math.degrees(math.atan2(ed1 - ed0, nd1 - nd0))
    err = wrap_signed_deg(dr_b - gnss_b)

    pvts = [p for p in ubx_pvt if t0 <= p.t_ns <= t1 and p.g_speed_mm_s > 200]
    ubx_compass = None
    if pvts:
        rads = [math.radians(p.head_mot_raw * 1e-5) for p in pvts]
        s = sum(math.sin(r) for r in rads); c = sum(math.cos(r) for r in rads)
        ubx_compass = wrap_compass_deg(math.degrees(math.atan2(s, c)))

    return {
        "gnss_bearing_compass_deg":    wrap_compass_deg(gnss_b),
        "dr_bearing_compass_deg":      wrap_compass_deg(dr_b),
        "gnss_bearing_enu_yaw_deg":    wrap_signed_deg(90.0 - gnss_b),
        "dr_bearing_enu_yaw_deg":      wrap_signed_deg(90.0 - dr_b),
        "heading_error_deg_signed":    err,
        "ubx_head_motion_compass_deg": ubx_compass,
        "n_fix_in_window":             len(fixes),
        "n_dr_in_window":              len(drs),
        "n_ubx_in_window":             len(pvts),
        "segment_chord_m":             math.hypot(ef1 - ef0, nf1 - nf0),
    }


def reconstruct_psi(imu_msgs, tf_static, rosout_lines, param_events, t_query_ns,
                    fallback_yaml):
    imu_msg = next((m for m in imu_msgs if m.t_ns >= t_query_ns),
                   imu_msgs[0] if imu_msgs else None)
    imu_yaw_rad = (quat_to_yaw_rad(imu_msg.qx, imu_msg.qy, imu_msg.qz, imu_msg.qw)
                   if imu_msg else None)

    tf_match = next((t for t in tf_static
                     if t.parent == "base_link" and t.child == "imu_link"), None)
    if tf_match is None:
        tf_match = next((t for t in tf_static if t.child == "imu_link"), None)
    tf_yaw_rad = (quat_to_yaw_rad(tf_match.qx, tf_match.qy, tf_match.qz, tf_match.qw)
                  if tf_match is not None else None)

    theta_base_rad = (imu_yaw_rad - tf_yaw_rad
                      if imu_yaw_rad is not None and tf_yaw_rad is not None else None)

    yaw_offset = None
    mag_decl = None
    psi_source = None

    for ev in param_events:
        if "navsat_transform" not in ev.node:
            continue
        for params in (ev.new_params, ev.changed_params):
            if "yaw_offset" in params and yaw_offset is None:
                try: yaw_offset = float(params["yaw_offset"])
                except ValueError: pass
            if "magnetic_declination_radians" in params and mag_decl is None:
                try: mag_decl = float(params["magnetic_declination_radians"])
                except ValueError: pass
        if (yaw_offset is not None or mag_decl is not None) and psi_source is None:
            psi_source = "from_/parameter_events"

    if (yaw_offset is None or mag_decl is None) and rosout_lines:
        nav_lines = [r for r in rosout_lines if "navsat" in r.name.lower()]
        for r in nav_lines:
            if yaw_offset is None:
                m = re.search(r"yaw_offset[\s:=]+(-?[\d\.eE+-]+)", r.msg)
                if m:
                    try: yaw_offset = float(m.group(1))
                    except ValueError: pass
            if mag_decl is None:
                m = re.search(r"magnetic_declination_radians[\s:=]+(-?[\d\.eE+-]+)", r.msg)
                if m:
                    try: mag_decl = float(m.group(1))
                    except ValueError: pass
        if (yaw_offset is not None or mag_decl is not None) and psi_source is None:
            psi_source = "from_/rosout"

    if (yaw_offset is None or mag_decl is None) and fallback_yaml:
        if yaw_offset is None and "yaw_offset" in fallback_yaml:
            yaw_offset = float(fallback_yaml["yaw_offset"])
        if mag_decl is None and "magnetic_declination_radians" in fallback_yaml:
            mag_decl = float(fallback_yaml["magnetic_declination_radians"])
        if psi_source is None:
            psi_source = "repo_yaml_fallback"

    psi_rad = (theta_base_rad + (yaw_offset or 0.0) + (mag_decl or 0.0)
               if theta_base_rad is not None and yaw_offset is not None and mag_decl is not None
               else None)

    return {
        "imu_yaw_enu_deg":             None if imu_yaw_rad is None else math.degrees(imu_yaw_rad),
        "imu_yaw_compass_deg":         None if imu_yaw_rad is None else enu_yaw_to_compass(math.degrees(imu_yaw_rad)),
        "tf_yaw_baselink_to_imu_deg":  None if tf_yaw_rad is None else math.degrees(tf_yaw_rad),
        "theta_base_enu_deg":          None if theta_base_rad is None else wrap_signed_deg(math.degrees(theta_base_rad)),
        "theta_base_compass_deg":      None if theta_base_rad is None else enu_yaw_to_compass(math.degrees(theta_base_rad)),
        "yaw_offset_rad":              yaw_offset,
        "yaw_offset_deg":              None if yaw_offset is None else math.degrees(yaw_offset),
        "magnetic_declination_rad":    mag_decl,
        "magnetic_declination_deg":    None if mag_decl is None else math.degrees(mag_decl),
        "psi_total_enu_deg":           None if psi_rad is None else wrap_signed_deg(math.degrees(psi_rad)),
        "psi_total_compass_deg":       None if psi_rad is None else enu_yaw_to_compass(math.degrees(psi_rad)),
        "psi_source":                  psi_source,
        "tf_static_n_transforms":      len(tf_static),
        "tf_match_found":              tf_match is not None,
        "imu_msg_t_ns":                None if imu_msg is None else imu_msg.t_ns,
    }


def expected_psi(gnss_bearing_compass_deg):
    return {
        "psi_correct_compass_deg": wrap_compass_deg(gnss_bearing_compass_deg),
        "psi_correct_enu_deg":     wrap_signed_deg(90.0 - gnss_bearing_compass_deg),
    }


def _evenly_spaced_times(msgs, n=5):
    if not msgs:
        return []
    if len(msgs) <= n:
        return [m.t_ns for m in msgs]
    return [msgs[int(i * (len(msgs) - 1) / (n - 1))].t_ns for i in range(n)]


def _nearest(msgs, t_ns):
    if not msgs:
        return None
    arr = np.array([m.t_ns for m in msgs], dtype=np.int64)
    idx = int(np.argmin(np.abs(arr - t_ns)))
    return msgs[idx]


def check_xsens_rotation(rosout_lines, imu_msgs, filter_quat_msgs, filt_euler_msgs):
    keywords = ("rotsensor", "frame_config", "180", "enable_rotsensor")
    evidence = []
    for r in rosout_lines:
        nm = r.name.lower()
        if "xsens" in nm or "mti" in nm:
            for kw in keywords:
                if kw in r.msg.lower():
                    evidence.append(f"{r.name}: {r.msg.strip()}")
                    break

    diffs_q, diffs_e = [], []
    for sample_t in _evenly_spaced_times(imu_msgs, n=5):
        imu = _nearest(imu_msgs, sample_t)
        if imu is None: continue
        imu_yaw = math.degrees(quat_to_yaw_rad(imu.qx, imu.qy, imu.qz, imu.qw))
        if filter_quat_msgs:
            fq = _nearest(filter_quat_msgs, sample_t)
            if fq is not None:
                fq_yaw = math.degrees(quat_to_yaw_rad(fq.qx, fq.qy, fq.qz, fq.qw))
                diffs_q.append(wrap_signed_deg(imu_yaw - fq_yaw))
        if filt_euler_msgs:
            fe = _nearest(filt_euler_msgs, sample_t)
            if fe is not None:
                fe_yaw = math.degrees(fe.y)  # Vector3.z is yaw in Xsens convention
                diffs_e.append(wrap_signed_deg(imu_yaw - fe_yaw))

    mean_q = float(np.mean(diffs_q)) if diffs_q else None
    mean_e = float(np.mean(diffs_e)) if diffs_e else None

    if mean_q is not None and abs(mean_q) < 1.0:
        conclusion = "no_extra_rotation"
    elif mean_q is not None and abs(abs(mean_q) - 180.0) < 5.0:
        conclusion = "180_degree_in_driver"
    elif mean_e is not None and abs(abs(mean_e) - 180.0) < 5.0:
        conclusion = "180_degree_in_driver"
    elif mean_q is None and mean_e is not None and abs(mean_e) < 2.0:
        conclusion = "no_extra_rotation"
    else:
        conclusion = "ambiguous"

    return {
        "rosout_evidence_lines":              evidence[:10],
        "imu_vs_filter_quat_yaw_diff_deg":    mean_q,
        "imu_vs_filter_euler_yaw_diff_deg":   mean_e,
        "conclusion":                         conclusion,
    }


def heading_stability(imu_msgs, dr_samples):
    if not imu_msgs:
        return None
    t_imu = np.array([m.t_ns for m in imu_msgs], dtype=np.int64)
    yaw_deg = np.array([math.degrees(quat_to_yaw_rad(m.qx, m.qy, m.qz, m.qw))
                        for m in imu_msgs])
    yaw_unwrapped = np.degrees(np.unwrap(np.radians(yaw_deg)))

    stationary = []
    if len(dr_samples) >= 2:
        speeds_t, speeds_v = [], []
        last_e, last_n = utm_xy(dr_samples[0].lat, dr_samples[0].lon)
        last_t = dr_samples[0].t_ns
        for d in dr_samples[1:]:
            e, n = utm_xy(d.lat, d.lon)
            dt = (d.t_ns - last_t) / 1e9
            if dt > 0:
                speeds_t.append(d.t_ns)
                speeds_v.append(math.hypot(e - last_e, n - last_n) / dt)
            last_e, last_n, last_t = e, n, d.t_ns
        speed_thr = 0.3
        in_stat = False; start_t = 0
        for ti, vi in zip(speeds_t, speeds_v):
            if vi < speed_thr and not in_stat:
                start_t = ti; in_stat = True
            elif vi >= speed_thr and in_stat:
                if (ti - start_t) > 3e9:
                    stationary.append((int(start_t), int(ti)))
                in_stat = False
        if in_stat and (speeds_t[-1] - start_t) > 3e9:
            stationary.append((int(start_t), int(speeds_t[-1])))

    phases = []
    for s, e in stationary:
        mask = (t_imu >= s) & (t_imu <= e)
        if mask.sum() < 5:
            continue
        ys = yaw_unwrapped[mask]
        phases.append({
            "duration_s":    (e - s) / 1e9,
            "yaw_min_deg":   float(ys.min()),
            "yaw_max_deg":   float(ys.max()),
            "yaw_std_deg":   float(ys.std()),
            "yaw_range_deg": float(ys.max() - ys.min()),
        })

    return {
        "n_imu_samples":           len(imu_msgs),
        "yaw_overall_range_deg":   float(yaw_unwrapped.max() - yaw_unwrapped.min()),
        "stationary_phases":       phases,
        "stationary_phases_t":     stationary,
        "_t_imu":                  t_imu,
        "_yaw_unwrapped_deg":      yaw_unwrapped,
    }


def _load_repo_yaml_fallback(repo_root: Path) -> dict:
    p = repo_root / "src/navigation/ekf_localization_pkg/config/navsat_transform.yaml"
    if not p.exists():
        return {}
    try:
        text = p.read_text()
    except OSError:
        return {}
    out = {}
    for key in ("yaw_offset", "magnetic_declination_radians"):
        m = re.search(rf"{key}\s*:\s*(-?[\d\.eE+-]+)", text)
        if m:
            try: out[key] = float(m.group(1))
            except ValueError: pass
    return out


def plot_yaw_timeseries(stab, straight_window, out_path: Path, bag_name: str):
    if stab is None or len(stab["_t_imu"]) == 0:
        return
    t0 = int(stab["_t_imu"][0])
    t = (stab["_t_imu"] - t0) / 1e9
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, stab["_yaw_unwrapped_deg"], color="steelblue", lw=0.8, label="IMU yaw (ENU, unwrapped)")
    for s, e in stab["stationary_phases_t"]:
        ax.axvspan((s - t0) / 1e9, (e - t0) / 1e9, color="lightgray", alpha=0.4)
    if straight_window is not None:
        s, e = straight_window
        ax.axvspan((s - t0) / 1e9, (e - t0) / 1e9, color="lightgreen", alpha=0.4,
                   label="straight segment")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("yaw (deg)")
    ax.set_title(f"{bag_name} - heading timeseries", fontsize=10)
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, alpha=0.3)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_bearings_summary(fix_samples, dr_samples, straight_window, a1,
                          out_path: Path, bag_name: str):
    fig, ax = plt.subplots(figsize=(8, 8))
    if fix_samples:
        e_n = [utm_xy(f.lat, f.lon) for f in fix_samples]
        ax.plot([p[0] for p in e_n], [p[1] for p in e_n],
                color="crimson", lw=1.8, label="/fix (GT)")
    if dr_samples:
        e_n = [utm_xy(d.lat, d.lon) for d in dr_samples]
        ax.plot([p[0] for p in e_n], [p[1] for p in e_n],
                color="royalblue", lw=1.8, label="/gps/filtered (DR)")
    if straight_window is not None and a1 is not None:
        s, e = straight_window
        fixes = [f for f in fix_samples if s <= f.t_ns <= e]
        drs   = [d for d in dr_samples if s <= d.t_ns <= e]
        if fixes:
            e0, n0 = utm_xy(fixes[0].lat, fixes[0].lon)
            e1, n1 = utm_xy(fixes[-1].lat, fixes[-1].lon)
            ax.annotate("", xy=(e1, n1), xytext=(e0, n0),
                        arrowprops=dict(arrowstyle="->", color="darkred", lw=2.5))
        if drs:
            e0, n0 = utm_xy(drs[0].lat, drs[0].lon)
            e1, n1 = utm_xy(drs[-1].lat, drs[-1].lon)
            ax.annotate("", xy=(e1, n1), xytext=(e0, n0),
                        arrowprops=dict(arrowstyle="->", color="darkblue", lw=2.5))
    ax.set_aspect("equal")
    ax.set_xlabel("UTM E (m)"); ax.set_ylabel("UTM N (m)")
    title = bag_name
    if a1:
        title += (f"\nGNSS={a1['gnss_bearing_compass_deg']:.1f}deg  "
                  f"DR={a1['dr_bearing_compass_deg']:.1f}deg  "
                  f"err={a1['heading_error_deg_signed']:+.1f}deg")
    ax.set_title(title, fontsize=9)
    ax.legend(fontsize=8); ax.ticklabel_format(useOffset=False, style="plain")
    ax.grid(True, alpha=0.3)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _strip_arrays(d):
    return {k: v for k, v in d.items() if not k.startswith("_")}


def _json_default(o):
    if isinstance(o, (np.integer,)): return int(o)
    if isinstance(o, (np.floating,)): return float(o)
    if isinstance(o, np.ndarray): return o.tolist()
    if isinstance(o, tuple): return list(o)
    raise TypeError(f"not serializable: {type(o)}")


def diagnose_bag(bag_dir: Path, out_dir: Path, min_segment_m: float, fallback_yaml: dict) -> dict:
    print(f"\n=== {bag_dir.name} ===")
    data = read_bag(bag_dir)
    print(f"  /fix={len(data.fix)}  /gps/filtered={len(data.dr)}  "
          f"/imu/data={len(data.imu)}  /tf_static={len(data.tf_static)}  "
          f"/rosout={len(data.rosout)}  /parameter_events={len(data.param_events)}  "
          f"/ubx_nav_pvt={len(data.ubx_pvt)}  "
          f"/filter/euler={len(data.filt_euler)}  /filter/quaternion={len(data.filt_quat)}")
    if not data.fix or not data.dr or not data.imu:
        print("  WARN: missing core topics, skipping")
        return {"bag": bag_dir.name, "error": "missing core topics"}

    seg = find_straight_segment(data.fix, min_dist_m=min_segment_m)
    if seg is None:
        print("  WARN: no straight segment found")
        a1, a3 = None, None
        t_query = data.fix[len(data.fix)//2].t_ns
        straight_window = None
        seg_info = None
    else:
        i0, i1, t0, t1 = seg
        straight_window = (t0, t1)
        a1 = measure_heading_error(data.fix, data.dr, data.ubx_pvt, t0, t1)
        a3 = expected_psi(a1["gnss_bearing_compass_deg"]) if a1 else None
        t_query = t0
        seg_info = {"i_start": i0, "i_end": i1, "t_start_ns": t0, "t_end_ns": t1,
                    "duration_s": (t1 - t0) / 1e9}

    a2 = reconstruct_psi(data.imu, data.tf_static, data.rosout, data.param_events,
                         t_query, fallback_yaml)
    a4 = check_xsens_rotation(data.rosout, data.imu, data.filt_quat, data.filt_euler)
    a5 = heading_stability(data.imu, data.dr)

    out_subdir = out_dir / bag_dir.name
    out_subdir.mkdir(parents=True, exist_ok=True)
    plot_yaw_timeseries(a5, straight_window,
                        out_subdir / "heading_yaw_timeseries.png", bag_dir.name)
    plot_bearings_summary(data.fix, data.dr, straight_window, a1,
                          out_subdir / "bearings_summary.png", bag_dir.name)

    result = {
        "bag":               bag_dir.name,
        "straight_segment":  seg_info,
        "analysis_1":        a1,
        "analysis_2":        a2,
        "analysis_3":        a3,
        "analysis_4":        a4,
        "analysis_5":        _strip_arrays(a5) if a5 else None,
    }
    json_path = out_subdir / "diagnosis.json"
    json_path.write_text(json.dumps(result, indent=2, default=_json_default))
    print(f"  Saved: {json_path}")
    return result


def build_top_diagnosis(results) -> str:
    valid = [r for r in results if "error" not in r and r.get("analysis_1")]
    if not valid:
        return "No valid bags."
    errs = np.array([r["analysis_1"]["heading_error_deg_signed"] for r in valid])
    spread = float(errs.max() - errs.min())
    mean_err = float(errs.mean())
    parts = [f"heading error mean = {mean_err:+.1f} deg, "
             f"spread = {spread:.1f} deg across {len(valid)} bags."]
    if spread <= 4.0:
        parts.append(f"Consistent systematic offset of {mean_err:+.1f} deg.")
    else:
        parts.append("Inconsistent across bags - magnetometer-dependent.")
    rotations = {r["analysis_4"]["conclusion"] for r in valid}
    if "180_degree_in_driver" in rotations:
        parts.append("Xsens driver appears to apply 180 deg rotation; static TF may be doubling it.")
    deltas = []
    for r in valid:
        a2 = r.get("analysis_2", {}); a3 = r.get("analysis_3", {})
        if a2.get("psi_total_enu_deg") is not None and a3 and a3.get("psi_correct_enu_deg") is not None:
            deltas.append(wrap_signed_deg(a2["psi_total_enu_deg"] - a3["psi_correct_enu_deg"]))
    if deltas:
        parts.append(f"Residual psi error after applying yaw_offset+mag_decl: mean = {np.mean(deltas):+.1f} deg.")
    return " ".join(parts)


def print_report(results):
    print()
    print(f"Heading diagnosis ({len(results)} bags)")
    print()
    fmt = "  {:<48s} {:>7s} {:>7s} {:>7s} {:>9s}"
    print(fmt.format("bag", "GNSS", "DR", "err", "UBX-mot"))
    for r in results:
        if "error" in r:
            print(fmt.format(r["bag"][:46], "-", "-", "ERR", "-")); continue
        a1 = r.get("analysis_1")
        if not a1:
            print(fmt.format(r["bag"][:46], "-", "-", "no_seg", "-")); continue
        ubx = a1["ubx_head_motion_compass_deg"]
        print(fmt.format(r["bag"][:46],
                         f"{a1['gnss_bearing_compass_deg']:.1f}",
                         f"{a1['dr_bearing_compass_deg']:.1f}",
                         f"{a1['heading_error_deg_signed']:+.1f}",
                         f"{ubx:.1f}" if ubx is not None else "-"))
    print()
    print("  Stationary IMU yaw per bag (longest stationary phase):")
    print("  " + "-" * 88)
    fmt2 = "  {:<48s} {:>12s} {:>12s} {:>10s} {:>9s}"
    print(fmt2.format("bag", "imu_yaw_ENU", "imu_yaw_cmps", "yaw_range", "duration"))
    for r in results:
        if "error" in r or not r.get("analysis_5"):
            continue
        sps = r["analysis_5"]["stationary_phases"]
        if not sps:
            print(fmt2.format(r["bag"][:46], "(no stationary)", "-", "-", "-"))
            continue
        longest = max(sps, key=lambda s: s["duration_s"])
        yaw_mid = 0.5 * (longest["yaw_min_deg"] + longest["yaw_max_deg"])
        yaw_mid_w = wrap_signed_deg(yaw_mid)
        print(fmt2.format(r["bag"][:46],
                          f"{yaw_mid_w:+.1f}",
                          f"{enu_yaw_to_compass(yaw_mid_w):.1f}",
                          f"{longest['yaw_range_deg']:.2f}",
                          f"{longest['duration_s']:.1f}s"))

    rep = next((r for r in results if "error" not in r and r.get("analysis_2")), None)
    if rep:
        a2, a3 = rep["analysis_2"], rep.get("analysis_3")
        print()
        print(f"  psi breakdown ({rep['bag']}):")
        print(f"    theta_imu (raw, imu_link, ENU)    : {_fdeg(a2.get('imu_yaw_enu_deg'))}")
        print(f"    TF yaw (base_link -> imu_link)    : {_fdeg(a2.get('tf_yaw_baselink_to_imu_deg'))}")
        print(f"    theta_base (ENU)                  : {_fdeg(a2.get('theta_base_enu_deg'))}")
        print(f"    yaw_offset                        : {_fdeg(a2.get('yaw_offset_deg'))} (source: {a2.get('psi_source')})")
        print(f"    magnetic_declination              : {_fdeg(a2.get('magnetic_declination_deg'))}")
        print(f"    psi total (ENU)                   : {_fdeg(a2.get('psi_total_enu_deg'))}")
        if a3:
            print(f"    psi correct (ENU, from GNSS)      : {_fdeg(a3.get('psi_correct_enu_deg'))}")
            if a2.get("psi_total_enu_deg") is not None and a3.get("psi_correct_enu_deg") is not None:
                d = wrap_signed_deg(a2["psi_total_enu_deg"] - a3["psi_correct_enu_deg"])
                print(f"    Delta psi                         : {d:+.1f} deg")
        print()
        a4 = rep.get("analysis_4", {})
        print(f"  Xsens driver rotation: {a4.get('conclusion')}  "
              f"(imu_vs_filter_quat={_fdeg(a4.get('imu_vs_filter_quat_yaw_diff_deg'))}, "
              f"imu_vs_filter_euler={_fdeg(a4.get('imu_vs_filter_euler_yaw_diff_deg'))})")
        if a4.get("rosout_evidence_lines"):
            print("  Xsens rosout evidence:")
            for line in a4["rosout_evidence_lines"]:
                print(f"    - {line}")
        if rep["analysis_5"] and rep["analysis_5"]["stationary_phases"]:
            longest = max(rep["analysis_5"]["stationary_phases"],
                          key=lambda s: s["duration_s"])
            print(f"  Stationary yaw range  : {longest['yaw_range_deg']:.2f} deg "
                  f"(over {longest['duration_s']:.1f} s)")
    print()
    print("  Diagnosis: " + build_top_diagnosis(results))


def _fdeg(v):
    return "n/a" if v is None else f"{v:.2f}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bags", type=Path, nargs="+")
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--min-segment-m", type=float, default=5.0)
    ap.add_argument("--repo-root", type=Path, default=Path.cwd())
    args = ap.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    fallback = _load_repo_yaml_fallback(args.repo_root)
    if fallback:
        print(f"Repo YAML fallback values: {fallback}")

    results = []
    for b in args.bags:
        try:
            results.append(diagnose_bag(b, args.output_dir, args.min_segment_m, fallback))
        except Exception as exc:
            print(f"ERROR processing {b}: {exc}", file=sys.stderr)
            results.append({"bag": b.name, "error": str(exc)})

    summary = {
        "n_bags":              len(results),
        "fallback_yaml_used":  fallback,
        "results":             results,
        "diagnosis":           build_top_diagnosis(results),
    }
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, default=_json_default))
    print(f"\nSaved summary: {summary_path}")

    print_report(results)
    return 0 if any("error" not in r for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
