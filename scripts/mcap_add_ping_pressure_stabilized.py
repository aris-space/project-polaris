#!/usr/bin/env python3
"""
Offline: copy a ROS 2 rosbag2 (MCAP) folder and add heave-compensated ping range:

  /ping_sonar/distance_pressure_stabilized (custom_msgs/msg/Distance)

**Model:** We remove the **linear** coupling between reported ping range and
``z`` (from ``pose_enu`` or pressure-derived ``z_enu``)::

  distance_mm' = distance_mm - slope_mm_per_m * (z(t) - z_ref)

``z_ref`` is the median ``z`` over the first ``--ref-duration-sec``.

**Why ``--heave-coupling auto`` (default):** On real ice/pool data, range and
``pose_enu.z`` are often **negatively** correlated (e.g. ~-1000 mm/m) because
of horizontal motion over varying bottom, sensor frame effects, or ice
geometry—not a pure “flat bottom + vertical heave” model. Using the textbook
``slope = +1000`` then **adds** the wrong correction and can **amplify** wiggles
(``corr(stabilized, z)`` becomes *more* negative). **Auto** fits ``slope`` with
least squares on ping samples (see ``--heave-fit-d-min-mm`` / ``d-max-mm``) so
``distance'`` has near-zero linear correlation with ``z`` on that segment.

**Fixed slope:** ``--heave-coupling fixed --heave-slope-mm-per-m 1000`` matches
the naive ENU flat-bottom sign; use only when you know the coupling.

**Depth source:** default ``auto`` uses ``/sensors/pressure/pose_enu`` if present
else ``/pixhawk/scaled_pressure`` with ``--p-surface-pa`` (same as the pressure
node, default 101325 Pa).

**Reference level:** After ``d_stab = d - slope*Δz``, the dive segment can sit
near the OLS intercept (often ~0 mm) while the surface hold still matches raw.
Default **anchor** uses the **same distance band as heave auto-fit**
(``--heave-fit-d-min-mm`` … ``--heave-fit-d-max-mm``): median(raw) − median(stab)
on those pings, so the **ice / dive** band keeps the real ~2 m level, not zero.
Use ``--anchor-mode first-window`` to anchor on the first ``--ref-duration-sec``
only (e.g. single-phase bags).

**Sensor height (optional):** If pressure depth and ping are not co-located and
you need a constant range correction, set ``--ping-above-pressure-m`` (meters,
+ if ping is above the pressure port in +z ENU). Default **0** — the **anchor**
already matches hold median to raw sonar range; add this only if you still need
a known lever-arm bias after that.

**Optional:** ``--smooth-median-half-width-sec`` adds ``…_smooth``.

**Units:** Ping1D publishes ``Distance.distance`` in **millimeters** (default
``--ping-distance-units mm``).

Dependencies: pip install rosbags numpy mcap pyyaml

Example::

  python scripts/mcap_add_ping_pressure_stabilized.py \\
    recordings/rosbags/2026-04-01/ping01_2026_04_01-14_46_23 \\
    --output-root recordings/rosbags_ping_pressure_stable --overwrite
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml

try:
    from mcap.reader import make_reader
except ImportError:
    print("Missing dependency. Install: pip install mcap", file=sys.stderr)
    raise

try:
    from rosbags.highlevel import AnyReader
    from rosbags.interfaces import ConnectionExtRosbag2
    from rosbags.rosbag2 import Writer
    from rosbags.rosbag2.enums import StoragePlugin
except ImportError:
    print("Missing dependency. Install: pip install rosbags", file=sys.stderr)
    raise

_DEFAULT_QOS = (
    "- history: 3\n"
    "  depth: 0\n"
    "  reliability: 1\n"
    "  durability: 2\n"
    "  deadline:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  lifespan:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  liveliness: 1\n"
    "  liveliness_lease_duration:\n"
    "    sec: 9223372036\n"
    "    nsec: 854775807\n"
    "  avoid_ros_namespace_conventions: false"
)


def _metadata_is_valid(path: Path) -> bool:
    if not path.is_file() or path.stat().st_size == 0:
        return False
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception:
        return False
    if not isinstance(data, dict):
        return False
    info = data.get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        return False
    topics = info.get("topics_with_message_count")
    return isinstance(topics, list) and len(topics) > 0


def repair_metadata_from_mcap(bag_dir: Path) -> None:
    """Write metadata.yaml by scanning *.mcap (needed when metadata is empty)."""
    mcaps = sorted(bag_dir.glob("*.mcap"))
    if not mcaps:
        raise FileNotFoundError(f"No .mcap in {bag_dir}")

    from collections import defaultdict

    counts: dict[int, int] = defaultdict(int)
    t_min: int | None = None
    t_max: int | None = None

    # Single MCAP is the common case; multi-file bags need a fuller merger.
    mcap_path = mcaps[0]
    with mcap_path.open("rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages():
            counts[channel.id] += 1
            lt = message.log_time
            t_min = lt if t_min is None else min(t_min, lt)
            t_max = lt if t_max is None else max(t_max, lt)
        summary = reader.get_summary()

    topics: list[dict] = []
    for cid, ch in summary.channels.items():
        sch = summary.schemas[ch.schema_id]
        topics.append(
            {
                "topic_metadata": {
                    "name": ch.topic,
                    "type": sch.name,
                    "serialization_format": "cdr",
                    "offered_qos_profiles": _DEFAULT_QOS,
                },
                "message_count": counts.get(cid, 0),
            }
        )

    total = sum(counts.values())
    info = {
        "version": 9,
        "storage_identifier": "mcap",
        "duration": {"nanoseconds": max(0, (t_max or 0) - (t_min or 0))},
        "starting_time": {"nanoseconds_since_epoch": t_min or 0},
        "message_count": total,
        "topics_with_message_count": topics,
        "compression_format": "",
        "compression_mode": "",
        "relative_file_paths": [mcap_path.name],
    }
    meta_path = bag_dir / "metadata.yaml"
    meta_path.write_text(
        yaml.dump({"rosbag2_bagfile_information": info}, default_flow_style=False, sort_keys=False),
        encoding="utf-8",
    )
    print(f"[repair] wrote {meta_path} ({len(topics)} topics, {total} messages)", file=sys.stderr)


def _output_dir(bag_dir: Path, out_root: Path, mirror_under: Path | None) -> Path:
    bag_dir = bag_dir.resolve()
    out_root = out_root.resolve()
    suffix = "__pressure_stabilized"
    if mirror_under is not None:
        try:
            rel = bag_dir.relative_to(mirror_under.resolve())
            return out_root / rel.parent / f"{rel.name}{suffix}"
        except ValueError:
            pass
    return out_root / f"{bag_dir.name}{suffix}"


def _collect_pressure_series(reader: AnyReader, pressure_topic: str) -> tuple[np.ndarray, np.ndarray]:
    conns = [c for c in reader.connections if c.topic == pressure_topic]
    if not conns:
        raise ValueError(f"No connection for {pressure_topic}")
    c0 = conns[0]
    times: list[int] = []
    pressures: list[float] = []
    for _c, ts, raw in reader.messages(connections=[c0]):
        msg = reader.deserialize(raw, c0.msgtype)
        times.append(ts)
        pressures.append(float(msg.fluid_pressure))
    return np.asarray(times, dtype=np.int64), np.asarray(pressures, dtype=np.float64)


def _collect_pose_z_series(reader: AnyReader, pose_topic: str) -> tuple[np.ndarray, np.ndarray]:
    """Log-time + pose.pose.position.z (ENU, m), same convention as pressure_z_ned_to_pose_node."""
    conns = [c for c in reader.connections if c.topic == pose_topic]
    if not conns:
        raise ValueError(f"No connection for {pose_topic}")
    c0 = conns[0]
    times: list[int] = []
    zs: list[float] = []
    for _c, ts, raw in reader.messages(connections=[c0]):
        msg = reader.deserialize(raw, c0.msgtype)
        times.append(ts)
        zs.append(float(msg.pose.pose.position.z))
    t = np.asarray(times, dtype=np.int64)
    z = np.asarray(zs, dtype=np.float64)
    order = np.argsort(t)
    return t[order], z[order]


def _pressure_to_z_enu(
    t_ns: np.ndarray,
    p_pa: np.ndarray,
    *,
    p_surface_pa: float,
    rho: float,
    g: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Match pressure_z_ned_to_pose_node (gauge=false): z_enu = -(p - p_surface)/(rho*g)."""
    z = -(p_pa - float(p_surface_pa)) / (rho * g)
    order = np.argsort(t_ns)
    return t_ns[order], z[order]


def _z_ref_from_window(t_ns: np.ndarray, z: np.ndarray, ref_duration_sec: float) -> float:
    if t_ns.size == 0:
        raise ValueError("No depth samples")
    t0 = int(t_ns[0])
    window_end = t0 + int(ref_duration_sec * 1e9)
    mask = t_ns <= window_end
    if not np.any(mask):
        mask = np.ones_like(t_ns, dtype=bool)
    return float(np.median(z[mask]))


def _interp_1d(t_query_ns: int, t_ns: np.ndarray, vals: np.ndarray) -> float:
    if t_ns.size == 0:
        return float("nan")
    if t_query_ns <= t_ns[0]:
        return float(vals[0])
    if t_query_ns >= t_ns[-1]:
        return float(vals[-1])
    idx = np.searchsorted(t_ns, t_query_ns)
    i0 = idx - 1
    i1 = idx
    t0, t1 = float(t_ns[i0]), float(t_ns[i1])
    if t1 == t0:
        return float(vals[i0])
    alpha = (t_query_ns - t0) / (t1 - t0)
    return float(vals[i0] * (1.0 - alpha) + vals[i1] * alpha)


def _time_median_smooth(t_ns: np.ndarray, y: np.ndarray, half_width_ns: float) -> np.ndarray:
    out = np.empty_like(y)
    for i in range(len(t_ns)):
        m = np.abs(t_ns - t_ns[i]) <= half_width_ns
        out[i] = float(np.median(y[m]))
    return out


def _distance_anchor_same_units(
    d_raw: np.ndarray,
    d_stab: np.ndarray,
    *,
    mode: str,
    ping_ts_ns: np.ndarray,
    t0_ns: int,
    ref_duration_sec: float,
    d_eff_mm: np.ndarray,
    d_min_mm: float,
    d_max_mm: float,
) -> float:
    """Shift so median(d_stab)+anchor matches median(d_raw) on chosen ping subset."""
    if mode == "first_window":
        t1 = t0_ns + int(ref_duration_sec * 1e9)
        m = (ping_ts_ns >= t0_ns) & (ping_ts_ns <= t1)
    else:
        m = (
            np.isfinite(d_eff_mm)
            & (d_eff_mm >= d_min_mm)
            & (d_eff_mm <= d_max_mm)
        )
    if np.count_nonzero(m) < 5:
        return 0.0
    return float(np.median(d_raw[m]) - np.median(d_stab[m]))


def _estimate_heave_slope_mm_per_m(
    d_mm: np.ndarray,
    z_m: np.ndarray,
    z_ref: float,
    *,
    d_min_mm: float,
    d_max_mm: float,
) -> tuple[float, int]:
    """OLS: d_mm ≈ slope * (z_m - z_ref) + b. Return (slope, n_used)."""
    zd = z_m - float(z_ref)
    m = (
        np.isfinite(zd)
        & np.isfinite(d_mm)
        & (d_mm >= d_min_mm)
        & (d_mm <= d_max_mm)
    )
    n = int(np.count_nonzero(m))
    if n < 30:
        return 0.0, n
    A = np.column_stack([zd[m], np.ones(n)])
    sol, _res, rank, _s = np.linalg.lstsq(A, d_mm[m], rcond=None)
    if rank < 1:
        return 0.0, n
    return float(sol[0]), n


def _bag_has_topic(reader: AnyReader, topic: str) -> bool:
    return any(c.topic == topic for c in reader.connections)


def process_bag(
    bag_dir: Path,
    out_root: Path,
    *,
    mirror_under: Path | None,
    depth_source: str,
    pose_topic: str,
    pressure_topic: str,
    p_surface_pa: float,
    ping_topic: str,
    out_topic: str,
    out_topic_smooth: str | None,
    smooth_half_width_sec: float,
    ref_duration_sec: float,
    rho: float,
    g: float,
    dry_run: bool,
    skip_repair: bool,
    ping_distance_units: str,
    overwrite: bool,
    heave_coupling: str,
    heave_slope_mm_per_m: float,
    heave_fit_d_min_mm: float,
    heave_fit_d_max_mm: float,
    ping_above_pressure_m: float,
    anchor_mode: str,
) -> Path | None:
    bag_dir = bag_dir.resolve()
    meta = bag_dir / "metadata.yaml"
    if not skip_repair and not _metadata_is_valid(meta):
        repair_metadata_from_mcap(bag_dir)

    out_dir = _output_dir(bag_dir, out_root, mirror_under)
    if out_dir.exists():
        if not overwrite:
            print(f"[skip] {out_dir} already exists (use --overwrite)", file=sys.stderr)
            return None
        shutil.rmtree(out_dir)
    if dry_run:
        print(f"[dry-run] would create {out_dir}")
        return out_dir

    with AnyReader([bag_dir]) as reader:
        ping_preview = [c for c in reader.connections if c.topic == ping_topic]
        has_pose = _bag_has_topic(reader, pose_topic)
        use_pose = depth_source == "pose_enu" or (depth_source == "auto" and has_pose)
        if depth_source == "pose_enu" and not has_pose:
            print(f"[skip] {bag_dir.name}: --depth-source pose_enu but missing {pose_topic}", file=sys.stderr)
            return None
        if not use_pose and not _bag_has_topic(reader, pressure_topic):
            print(f"[skip] {bag_dir.name}: missing {pressure_topic}", file=sys.stderr)
            return None

        if use_pose:
            t_z, z_arr = _collect_pose_z_series(reader, pose_topic)
            depth_label = f"{pose_topic} z_enu"
        else:
            t_p, p_pa = _collect_pressure_series(reader, pressure_topic)
            t_z, z_arr = _pressure_to_z_enu(t_p, p_pa, p_surface_pa=p_surface_pa, rho=rho, g=g)
            depth_label = f"{pressure_topic} -> z_enu (p_surface={p_surface_pa:g} Pa)"

    if not ping_preview:
        print(f"[skip] {bag_dir.name}: missing {ping_topic}", file=sys.stderr)
        return None

    z_ref = _z_ref_from_window(t_z, z_arr, ref_duration_sec)
    t0_ns = int(t_z[0])

    ping_ts: list[int] = []
    ping_d_raw: list[float] = []
    ping_z: list[float] = []
    with AnyReader([bag_dir]) as reader:
        ping_conns = [c for c in reader.connections if c.topic == ping_topic]
        ping_src = ping_conns[0]
        ping_type = ping_src.msgtype
        for _c, ts, raw in reader.messages(connections=ping_conns):
            msg = reader.deserialize(raw, ping_type)
            z_i = _interp_1d(ts, t_z, z_arr)
            d_raw = float(msg.distance)
            ping_ts.append(ts)
            ping_d_raw.append(d_raw)
            ping_z.append(float(z_i) if not np.isnan(z_i) else float("nan"))

    if not ping_ts:
        print(f"[skip] {bag_dir.name}: no ping messages", file=sys.stderr)
        return None

    d_arr = np.asarray(ping_d_raw, dtype=np.float64)
    z_arr_ping = np.asarray(ping_z, dtype=np.float64)
    # OLS is always in mm vs z[m]; convert ping distances if stored in meters.
    d_eff_mm = d_arr * (1000.0 if ping_distance_units == "m" else 1.0)

    if heave_coupling == "auto":
        slope_mm_m, n_fit = _estimate_heave_slope_mm_per_m(
            d_eff_mm,
            z_arr_ping,
            z_ref,
            d_min_mm=heave_fit_d_min_mm,
            d_max_mm=heave_fit_d_max_mm,
        )
        if n_fit < 30:
            print(
                f"[warn] heave auto-fit: only {n_fit} samples in "
                f"[{heave_fit_d_min_mm:g},{heave_fit_d_max_mm:g}] mm (effective); slope=0",
                file=sys.stderr,
            )
            slope_mm_m = 0.0
        else:
            m_fit = (
                np.isfinite(z_arr_ping)
                & np.isfinite(d_eff_mm)
                & (d_eff_mm >= heave_fit_d_min_mm)
                & (d_eff_mm <= heave_fit_d_max_mm)
            )
            zd = z_arr_ping - z_ref
            stab_try_mm = d_eff_mm - slope_mm_m * zd
            cz = float(np.corrcoef(stab_try_mm[m_fit], z_arr_ping[m_fit])[0, 1])
            print(
                f"[heave] auto slope={slope_mm_m:.2f} mm/m  n_fit={n_fit}  "
                f"corr(stab,z)|fit_mask={cz:.3f}",
                file=sys.stderr,
            )
    else:
        slope_mm_m = float(heave_slope_mm_per_m)
        print(f"[heave] fixed slope={slope_mm_m:.2f} mm/m", file=sys.stderr)

    d_stab_pre: list[float] = []
    for d_raw, z_i in zip(ping_d_raw, ping_z):
        if np.isnan(z_i):
            d_stab_pre.append(float(d_raw))
        else:
            zd = z_i - z_ref
            if ping_distance_units == "mm":
                d_stab_pre.append(float(d_raw - slope_mm_m * zd))
            else:
                d_stab_pre.append(float(d_raw - (slope_mm_m / 1000.0) * zd))

    ts_arr = np.asarray(ping_ts, dtype=np.int64)
    d_raw_arr = np.asarray(ping_d_raw, dtype=np.float64)
    d_stab_arr = np.asarray(d_stab_pre, dtype=np.float64)
    anchor = _distance_anchor_same_units(
        d_raw_arr,
        d_stab_arr,
        mode=anchor_mode,
        ping_ts_ns=ts_arr,
        t0_ns=t0_ns,
        ref_duration_sec=ref_duration_sec,
        d_eff_mm=d_eff_mm,
        d_min_mm=heave_fit_d_min_mm,
        d_max_mm=heave_fit_d_max_mm,
    )
    if ping_distance_units == "mm":
        static_bias = float(ping_above_pressure_m) * 1000.0
    else:
        static_bias = float(ping_above_pressure_m)
    d_final_arr = d_stab_arr + anchor + static_bias
    print(
        f"[heave] anchor_mode={anchor_mode}  anchor={anchor:.2f} {ping_distance_units}  "
        f"static_bias={static_bias:.2f} (ping_above_pressure_m={ping_above_pressure_m:g})",
        file=sys.stderr,
    )

    ping_plan: list[tuple[int, float, float | None]] = []
    for ts, d_raw, d_out in zip(ping_ts, ping_d_raw, d_final_arr):
        ping_plan.append((ts, d_raw, float(d_out)))

    smooth_vals: list[float | None] = [None] * len(ping_plan)
    if smooth_half_width_sec > 0.0 and out_topic_smooth:
        t_ping = np.array([p[0] for p in ping_plan], dtype=np.int64)
        d_stab_arr = np.array([p[2] for p in ping_plan], dtype=np.float64)
        half_ns = float(smooth_half_width_sec * 1e9)
        sm = _time_median_smooth(t_ping, d_stab_arr, half_ns)
        smooth_vals = [float(x) for x in sm]

    ping_iter = iter(zip(ping_plan, smooth_vals))

    wrote_smooth = smooth_half_width_sec > 0.0 and bool(out_topic_smooth)

    with AnyReader([bag_dir]) as reader:
        typestore = reader.typestore
        ping_conns = [c for c in reader.connections if c.topic == ping_topic]
        ping_src = ping_conns[0]
        ping_type = ping_src.msgtype

        with Writer(out_dir, version=9, storage_plugin=StoragePlugin.MCAP) as writer:

            def add_like(src, topic: str):
                assert isinstance(src.ext, ConnectionExtRosbag2)
                return writer.add_connection(
                    topic,
                    src.msgtype,
                    typestore=typestore,
                    offered_qos_profiles=src.ext.offered_qos_profiles,
                )

            out_by_id: dict[int, object] = {}
            for c in reader.connections:
                out_by_id[c.id] = add_like(c, c.topic)

            out_stab = add_like(ping_src, out_topic)
            out_smooth = add_like(ping_src, out_topic_smooth) if wrote_smooth else None

            for c, ts, raw in reader.messages():
                writer.write(out_by_id[c.id], ts, raw)
                if c.topic != ping_topic:
                    continue
                try:
                    (ts_e, _d_raw, d_stab), d_smo = next(ping_iter)
                except StopIteration:
                    print("[warn] ping count mismatch", file=sys.stderr)
                    break
                if ts_e != ts:
                    print(f"[warn] ping timestamp mismatch {ts_e} vs {ts}", file=sys.stderr)
                msg = reader.deserialize(raw, ping_type)
                msg.distance = float(np.float32(d_stab))
                writer.write(out_stab, ts, typestore.serialize_cdr(msg, ping_type))
                if out_smooth is not None and d_smo is not None:
                    msg.distance = float(np.float32(d_smo))
                    writer.write(out_smooth, ts, typestore.serialize_cdr(msg, ping_type))

    print(
        f"[ok] {out_dir}  depth={depth_label}  z_ref={z_ref:.4f} m  "
        f"slope={slope_mm_m:.2f} mm/m  ref_window={ref_duration_sec:g}s  "
        f"ping_units={ping_distance_units}",
        file=sys.stderr,
    )
    if wrote_smooth and out_topic_smooth:
        print(
            f"     smooth -> {out_topic_smooth} (median +/- {smooth_half_width_sec:g} s)",
            file=sys.stderr,
        )
    return out_dir


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path, help="Source rosbag2 directory (with .mcap)")
    ap.add_argument(
        "--output-root",
        type=Path,
        default=Path("recordings/rosbags_ping_pressure_stable"),
        help="Parent directory for output * __pressure_stabilized",
    )
    ap.add_argument(
        "--mirror-structure-under",
        type=Path,
        default=None,
        help="e.g. recordings/rosbags → out_root/<date>/<bag>__pressure_stabilized",
    )
    ap.add_argument(
        "--depth-source",
        choices=("auto", "pose_enu", "fluid_pressure"),
        default="auto",
        help="auto: use pose_enu if in bag else scaled_pressure",
    )
    ap.add_argument(
        "--pose-topic",
        default="/sensors/pressure/pose_enu",
        help="PoseWithCovarianceStamped; z = position.z (ENU m)",
    )
    ap.add_argument("--pressure-topic", default="/pixhawk/scaled_pressure")
    ap.add_argument(
        "--p-surface-pa",
        type=float,
        default=101325.0,
        help="Only for fluid_pressure path; same as pressure_z_ned_to_pose_node",
    )
    ap.add_argument("--ping-topic", default="/ping_sonar/distance")
    ap.add_argument(
        "--out-topic",
        default="/ping_sonar/distance_pressure_stabilized",
        help="Heave-compensated Distance (same units as source ping)",
    )
    ap.add_argument(
        "--out-topic-smooth",
        default="/ping_sonar/distance_pressure_stabilized_smooth",
        help="Optional time-median series (see --smooth-median-half-width-sec)",
    )
    ap.add_argument(
        "--smooth-median-half-width-sec",
        type=float,
        default=0.0,
        help="If >0, write out-topic-smooth (median over +/- this window, seconds)",
    )
    ap.add_argument(
        "--ref-duration-sec",
        type=float,
        default=120.0,
        help="Median z (or derived z) over [t0, t0+window] defines z_ref (surface hold)",
    )
    ap.add_argument("--water-density-kg-m3", type=float, default=1000.0)
    ap.add_argument("--gravity-m-s2", type=float, default=9.80665)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument(
        "--skip-metadata-repair",
        action="store_true",
        help="Do not scan MCAP to rebuild empty metadata.yaml",
    )
    ap.add_argument(
        "--ping-distance-units",
        choices=("mm", "m"),
        default="mm",
        help="Units of custom_msgs/Distance.distance in the bag (Ping1D uses mm).",
    )
    ap.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove existing output bag directory and regenerate.",
    )
    ap.add_argument(
        "--heave-coupling",
        choices=("auto", "fixed"),
        default="auto",
        help="auto: least-squares slope (mm/m) ping vs z; fixed: use --heave-slope-mm-per-m",
    )
    ap.add_argument(
        "--heave-slope-mm-per-m",
        type=float,
        default=1000.0,
        help="With --heave-coupling fixed: d' = d - slope*(z-z_ref)",
    )
    ap.add_argument(
        "--heave-fit-d-min-mm",
        type=float,
        default=200.0,
        help="Auto-fit: use pings with effective range in [min,max] mm",
    )
    ap.add_argument(
        "--heave-fit-d-max-mm",
        type=float,
        default=6000.0,
        help="Auto-fit: exclude long-range / dropout spikes (mm in effective units)",
    )
    ap.add_argument(
        "--ping-above-pressure-m",
        type=float,
        default=0.0,
        help=(
            "Optional constant added to output after anchoring (meters of range). "
            "E.g. 0.05 adds +50 mm when ping distance is in mm. Use if lever-arm "
            "bias remains after median anchor. Default 0."
        ),
    )
    ap.add_argument(
        "--anchor-mode",
        choices=("fit_mask", "first_window"),
        default="fit_mask",
        help=(
            "fit_mask: anchor median using pings in heave-fit distance band (default "
            "for surface+dive bags). first_window: anchor on first ref-duration only."
        ),
    )
    args = ap.parse_args()

    b = args.bag_dir.resolve()
    if not b.is_dir():
        print(f"Not a directory: {b}", file=sys.stderr)
        return 1

    args.output_root.mkdir(parents=True, exist_ok=True)

    if args.mirror_structure_under is not None:
        mirror: Path | None = args.mirror_structure_under.resolve()
    else:
        rosbags_root = Path("recordings/rosbags").resolve()
        try:
            b.relative_to(rosbags_root)
            mirror = rosbags_root
        except ValueError:
            mirror = None

    smooth_w = float(args.smooth_median_half_width_sec)
    smooth_topic = args.out_topic_smooth if smooth_w > 0.0 else None

    r = process_bag(
        b,
        args.output_root,
        mirror_under=mirror,
        depth_source=args.depth_source,
        pose_topic=args.pose_topic,
        pressure_topic=args.pressure_topic,
        p_surface_pa=args.p_surface_pa,
        ping_topic=args.ping_topic,
        out_topic=args.out_topic,
        out_topic_smooth=smooth_topic,
        smooth_half_width_sec=smooth_w,
        ref_duration_sec=args.ref_duration_sec,
        rho=args.water_density_kg_m3,
        g=args.gravity_m_s2,
        dry_run=args.dry_run,
        skip_repair=args.skip_metadata_repair,
        ping_distance_units=args.ping_distance_units,
        overwrite=args.overwrite,
        heave_coupling=args.heave_coupling,
        heave_slope_mm_per_m=args.heave_slope_mm_per_m,
        heave_fit_d_min_mm=args.heave_fit_d_min_mm,
        heave_fit_d_max_mm=args.heave_fit_d_max_mm,
        ping_above_pressure_m=args.ping_above_pressure_m,
        anchor_mode=args.anchor_mode,
    )
    if r is None and not args.dry_run:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
