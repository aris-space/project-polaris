#!/usr/bin/env python3
"""
Bag reading, timestamp verification and augmented-bag writing for the sysid validation
scripts. Wraps `rosbags` so the rest of scripts/sysid/ stays numpy-only.

Reading: pulls the EKF odometry and the raw servo PWM into numpy arrays. The servo topic
(`std_msgs/Int16MultiArray`) carries no header, so its MCAP log_time is the only time
source available for it; the EKF uses header.stamp. The BMS pack voltage is read separately
from /diagnostics (read_voltage), also on log_time.

Verification: before any propagation, check that the two time bases are usable together -
monotonic, overlapping, free of large gaps, and that header.stamp does not drift from
log_time. Findings are reported, not silently corrected.

Writing: copies every message of the input bag unchanged into a new rosbag2/MCAP bag and
appends the `/sysid/*` topics. The output is a directory containing a single `.mcap`;
Foxglove opens either the directory or that file.

Dependency: pip install rosbags numpy scipy pyproj

Example:
    odom, servo = read_tracks(Path("recordings/run.mcap"))
    report = verify_timestamps(odom, servo)
    report.print()
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rosbags.rosbag2.storage_mcap as _storage_mcap
from rosbags.highlevel import AnyReader
from rosbags.interfaces import ConnectionExtRosbag2
from rosbags.rosbag2 import Writer
from rosbags.rosbag2.enums import StoragePlugin


def _allow_empty_mcap_profile() -> None:
    """Accept bags whose MCAP header leaves the `profile` field empty.

    Bags written by libmcap set the header profile to 'ros2'; bags written by mcap-rust
    (the aut_* recordings from 2026-06 onwards) leave it empty, and rosbags rejects
    anything that is not literally 'ros2'. The payloads are ordinary ROS 2 CDR with
    ros2msg schemas either way, so only the empty case is relaxed here - any other
    profile still raises, so a genuinely non-ROS 2 bag cannot be misread as one.
    """
    original_open = _storage_mcap.McapReader.open

    def open_relaxed(self) -> None:
        original_read_string = _storage_mcap.read_string
        first = []

        def read_string(bio):
            value = original_read_string(bio)
            # The first string open() reads out of the header record is the profile.
            if not first:
                first.append(value)
                if value == "":
                    return "ros2"
            return value

        _storage_mcap.read_string = read_string
        try:
            original_open(self)
        finally:
            _storage_mcap.read_string = original_read_string

    _storage_mcap.McapReader.open = open_relaxed


_allow_empty_mcap_profile()

DEFAULT_EKF_TOPIC = "/odometry/filtered/local"
DEFAULT_SERVO_TOPIC = "/pixhawk/servo_output_raw"
DEFAULT_DIAG_TOPIC = "/diagnostics"

# bms_node publishes the pack voltage once per second as a DiagnosticStatus with this name,
# carrying the value as text (e.g. "14.70V") under the key "voltage".
BMS_VOLTAGE_STATUS = "Battery: Voltage"
BMS_VOLTAGE_KEY = "voltage"

# NavSatFix tracks the odom -> UTM anchor is fitted against, best first. /gps/filtered is
# navsat_transform re-projecting the local EKF; the June bags have it, the April ones only
# carry the global EKF's /gps/filtered/global.
GEO_REF_TOPICS = ("/gps/filtered", "/gps/filtered/global")

NS = 1e-9


@dataclass
class OdomTrack:
    """EKF odometry sampled on its own header.stamp clock."""

    t: np.ndarray  # header.stamp [s]
    log_t: np.ndarray  # MCAP log_time [s]
    pos: np.ndarray  # (N, 3) in frame_id
    yaw: np.ndarray  # (N,) [rad]
    vel_body: np.ndarray  # (N, 3) twist.twist.linear, in child_frame_id
    yaw_rate: np.ndarray  # (N,) [rad/s]
    frame_id: str
    child_frame_id: str

    def __len__(self) -> int:
        return self.t.size


@dataclass
class ServoTrack:
    """Raw servo PWM. Headerless message: log_time is the only timestamp."""

    log_t: np.ndarray  # MCAP log_time [s]
    pwm: np.ndarray  # (N, 6) microseconds

    def __len__(self) -> int:
        return self.log_t.size


@dataclass
class VoltageTrack:
    """Battery pack voltage from the BMS diagnostics, ~1 Hz, on MCAP log_time like the servo."""

    log_t: np.ndarray  # MCAP log_time [s]
    volts: np.ndarray  # (N,) [V]

    def __len__(self) -> int:
        return self.log_t.size

    def at(self, t: float | np.ndarray) -> float | np.ndarray:
        """Zero-order hold. Before the first sample the first value is used: the pack voltage
        changes over minutes, so holding it backwards for a second is harmless."""
        i = np.clip(np.searchsorted(self.log_t, t, side="right") - 1, 0, self.log_t.size - 1)
        return self.volts[i]


def parse_bms_voltage(text: str) -> float | None:
    """'14.70V' -> 14.7. Returns None for anything that is not a number with an optional V."""
    try:
        return float(text.strip().rstrip("Vv").strip())
    except ValueError:
        return None


def read_voltage(bag: Path, topic: str = DEFAULT_DIAG_TOPIC) -> VoltageTrack | None:
    """Read the BMS pack voltage from /diagnostics. Returns None (with a printed reason) when
    the topic or the BMS status is absent, so the caller can fall back to a fixed voltage."""
    log_t, volts = [], []
    unparsable = 0
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            print(f"  [voltage] {topic} not in {bag.name}")
            return None
        for conn, ts, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            for status in m.status:
                if status.name != BMS_VOLTAGE_STATUS:
                    continue
                for kv in status.values:
                    if kv.key != BMS_VOLTAGE_KEY:
                        continue
                    v = parse_bms_voltage(kv.value)
                    if v is None:
                        unparsable += 1
                    else:
                        log_t.append(ts * NS)
                        volts.append(v)
    if unparsable:
        print(f"  [voltage] skipped {unparsable} unparsable '{BMS_VOLTAGE_STATUS}' value(s)")
    if not volts:
        print(f"  [voltage] no '{BMS_VOLTAGE_STATUS}' status on {topic} in {bag.name}")
        return None
    order = np.argsort(log_t, kind="stable")
    return VoltageTrack(log_t=np.asarray(log_t)[order], volts=np.asarray(volts)[order])


def read_tracks(
    bag: Path,
    ekf_topic: str = DEFAULT_EKF_TOPIC,
    servo_topic: str = DEFAULT_SERVO_TOPIC,
) -> tuple[OdomTrack, ServoTrack]:
    """Read the EKF odometry and servo PWM from a bare .mcap or a rosbag2 directory."""
    t, log_t, pos, quat, vel, yrate = [], [], [], [], [], []
    frame_id = child_frame_id = ""
    s_log_t, s_pwm = [], []

    with AnyReader([bag]) as reader:
        conns = {c.topic: c for c in reader.connections}
        missing = [x for x in (ekf_topic, servo_topic) if x not in conns]
        if missing:
            available = "\n  ".join(sorted(conns))
            raise KeyError(f"topic(s) not in {bag.name}: {missing}\navailable:\n  {available}")

        c = conns[ekf_topic]
        for conn, ts, raw in reader.messages(connections=[c]):
            m = reader.deserialize(raw, conn.msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * NS)
            log_t.append(ts * NS)
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            lin = m.twist.twist.linear
            pos.append((p.x, p.y, p.z))
            quat.append((q.x, q.y, q.z, q.w))
            vel.append((lin.x, lin.y, lin.z))
            yrate.append(m.twist.twist.angular.z)
            frame_id = m.header.frame_id
            child_frame_id = m.child_frame_id

        c = conns[servo_topic]
        for conn, ts, raw in reader.messages(connections=[c]):
            m = reader.deserialize(raw, conn.msgtype)
            s_log_t.append(ts * NS)
            s_pwm.append(list(m.data))

    if not t:
        raise ValueError(f"no messages on {ekf_topic}")
    if not s_log_t:
        raise ValueError(f"no messages on {servo_topic}")

    quat_arr = np.asarray(quat, dtype=np.float64)
    yaw = Rot.from_quat(quat_arr).as_euler("xyz")[:, 2]

    odom = OdomTrack(
        t=np.asarray(t, dtype=np.float64),
        log_t=np.asarray(log_t, dtype=np.float64),
        pos=np.asarray(pos, dtype=np.float64),
        yaw=yaw,
        vel_body=np.asarray(vel, dtype=np.float64),
        yaw_rate=np.asarray(yrate, dtype=np.float64),
        frame_id=frame_id,
        child_frame_id=child_frame_id,
    )
    servo = ServoTrack(
        log_t=np.asarray(s_log_t, dtype=np.float64),
        pwm=np.asarray(s_pwm, dtype=np.float64),
    )
    return odom, servo


# ============================================================
# Timestamp verification
# ============================================================


@dataclass
class TimestampReport:
    lines: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    overlap: tuple[float, float] = (0.0, 0.0)

    @property
    def ok(self) -> bool:
        return not self.errors

    def print(self) -> None:
        print("\n[timestamps]")
        for line in self.lines:
            print(f"  {line}")
        for w in self.warnings:
            print(f"  WARN  {w}")
        for e in self.errors:
            print(f"  ERROR {e}")


def _rate_stats(t: np.ndarray, label: str, lines: list[str], warnings: list[str]) -> None:
    dt = np.diff(t)
    if dt.size == 0:
        return
    span = t[-1] - t[0]
    med = float(np.median(dt))
    lines.append(
        f"{label:<28} n={t.size:<7} span={span:8.1f}s  rate={1.0/med if med > 0 else 0:6.2f} Hz  "
        f"dt med={med*1e3:7.1f} ms  max={dt.max()*1e3:9.1f} ms"
    )
    if np.any(dt < 0):
        warnings.append(f"{label}: {int((dt < 0).sum())} backwards time step(s)")
    if np.any(dt == 0):
        warnings.append(f"{label}: {int((dt == 0).sum())} duplicate timestamp(s)")
    gap_thresh = max(5.0 * med, 0.5)
    gaps = dt > gap_thresh
    if np.any(gaps):
        warnings.append(
            f"{label}: {int(gaps.sum())} gap(s) > {gap_thresh:.2f}s, largest {dt.max():.2f}s"
        )


def verify_timestamps(
    odom: OdomTrack,
    servo: ServoTrack,
    max_stamp_offset_s: float = 1.0,
) -> TimestampReport:
    """Check that the EKF header.stamp clock and the servo log_time clock can be used together."""
    lines: list[str] = []
    warnings: list[str] = []
    errors: list[str] = []

    _rate_stats(odom.t, "EKF header.stamp", lines, warnings)
    _rate_stats(odom.log_t, "EKF log_time", lines, warnings)
    _rate_stats(servo.log_t, "servo log_time", lines, warnings)

    # Does the EKF stamp its own messages close to when they were recorded? If these two
    # clocks disagree, pairing servo log_time with EKF header.stamp is not meaningful.
    offset = odom.log_t - odom.t
    lines.append(
        f"{'EKF log_time - header.stamp':<28} median={np.median(offset)*1e3:+8.1f} ms  "
        f"p95={np.percentile(offset, 95)*1e3:+8.1f} ms  max={offset.max()*1e3:+9.1f} ms"
    )
    if abs(float(np.median(offset))) > max_stamp_offset_s:
        warnings.append(
            f"EKF header.stamp differs from log_time by {np.median(offset):.2f}s on median; "
            "servo log_time and EKF header.stamp may be on different clocks"
        )

    lo = max(odom.t[0], servo.log_t[0])
    hi = min(odom.t[-1], servo.log_t[-1])
    lines.append(
        f"{'overlap window':<28} {hi - lo:8.1f}s  "
        f"(EKF starts {odom.t[0] - servo.log_t[0]:+.1f}s vs servo, "
        f"ends {odom.t[-1] - servo.log_t[-1]:+.1f}s)"
    )
    if hi <= lo:
        errors.append("EKF and servo time ranges do not overlap - cannot propagate")

    return TimestampReport(lines=lines, warnings=warnings, errors=errors, overlap=(lo, hi))


# ============================================================
# Writing
# ============================================================


@dataclass
class GeoAnchor:
    """Rigid odom -> UTM transform, recovered from the bag's own recorded NavSatFix track.

    `/gps/filtered` is `/odometry/filtered/local` pushed through a single fixed transform that
    navsat_transform established once at GNSS lock and never corrects afterwards. Fitting that
    transform from the recorded pairs reproduces it exactly (measured: zero residual over 51k
    samples), and recovers the map-to-UTM yaw without having to assume the map frame is
    ENU-aligned - it is not, by 17 degrees on the June bags.
    """

    R: np.ndarray  # 2x2 rotation, odom -> UTM
    t: np.ndarray  # (2,) translation in UTM metres
    epsg: str
    alt_offset: float
    residual_m: float
    ref_topic: str
    n_samples: int

    @property
    def yaw_deg(self) -> float:
        return float(np.degrees(np.arctan2(self.R[1, 0], self.R[0, 0])))

    def to_latlon(self, xy: np.ndarray, z: np.ndarray):
        """Map odom XY (and z) onto latitude, longitude, altitude."""
        from pyproj import Transformer

        en = (self.R @ np.asarray(xy, dtype=np.float64).T).T + self.t
        inv = Transformer.from_crs(self.epsg, "EPSG:4326", always_xy=True)
        lon, lat = inv.transform(en[:, 0], en[:, 1])
        return np.asarray(lat), np.asarray(lon), self.alt_offset + np.asarray(z)


def _utm_epsg(lat: float, lon: float) -> str:
    zone = int((lon + 180.0) / 6.0) + 1
    hemisphere = "6" if lat >= 0.0 else "7"
    return f"EPSG:32{hemisphere}{zone:02d}"


def fit_geo_anchor(bag: Path, odom: OdomTrack, ref_topics=GEO_REF_TOPICS,
                   max_dt: float = 0.02) -> GeoAnchor | None:
    """Recover the odom -> UTM transform by fitting odom XY onto a recorded NavSatFix track.

    Returns None (with a printed reason) when no reference topic is present, too few samples
    match, or pyproj is unavailable - the caller then simply skips the geographic output.
    """
    try:
        from pyproj import Transformer
    except ImportError:
        print("  [geo] pyproj not installed - skipping NavSatFix output")
        return None

    with AnyReader([bag]) as reader:
        conns = {c.topic: c for c in reader.connections}
        topic = next((t for t in ref_topics if t in conns), None)
        if topic is None:
            print(f"  [geo] none of {list(ref_topics)} in bag - skipping NavSatFix output")
            return None
        ref = []
        c = conns[topic]
        for conn, ts, raw in reader.messages(connections=[c]):
            m = reader.deserialize(raw, conn.msgtype)
            ref.append((m.header.stamp.sec + m.header.stamp.nanosec * NS,
                        m.latitude, m.longitude, m.altitude))

    ref_arr = np.asarray(ref, dtype=np.float64)
    if ref_arr.shape[0] < 10:
        print(f"  [geo] only {ref_arr.shape[0]} messages on {topic} - skipping NavSatFix output")
        return None

    # Nearest-timestamp match of each reference fix to an odom sample.
    j = np.clip(np.searchsorted(odom.t, ref_arr[:, 0]), 1, odom.t.size - 1)
    pick = np.where(np.abs(odom.t[j - 1] - ref_arr[:, 0]) < np.abs(odom.t[j] - ref_arr[:, 0]),
                    j - 1, j)
    good = np.abs(odom.t[pick] - ref_arr[:, 0]) < max_dt
    if good.sum() < 10:
        print(f"  [geo] only {int(good.sum())} of {ref_arr.shape[0]} {topic} samples align "
              f"with the EKF within {max_dt*1e3:.0f} ms - skipping NavSatFix output")
        return None

    A = odom.pos[pick[good], :2]
    lat, lon, alt = ref_arr[good, 1], ref_arr[good, 2], ref_arr[good, 3]
    epsg = _utm_epsg(float(lat.mean()), float(lon.mean()))
    east, north = Transformer.from_crs("EPSG:4326", epsg, always_xy=True).transform(lon, lat)
    B = np.column_stack([east, north])

    # Kabsch: best rotation+translation with scale pinned to 1, since the true relationship
    # between odom metres and UTM metres is rigid.
    Ac, Bc = A - A.mean(0), B - B.mean(0)
    U, _, Vt = np.linalg.svd(Ac.T @ Bc)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = B.mean(0) - R @ A.mean(0)
    residual = float(np.linalg.norm((R @ A.T).T + t - B, axis=1).mean())

    return GeoAnchor(
        R=R, t=t, epsg=epsg,
        alt_offset=float(np.mean(alt - odom.pos[pick[good], 2])),
        residual_m=residual, ref_topic=topic, n_samples=int(good.sum()),
    )


@dataclass
class OutputSeries:
    """Everything written to the /sysid/* topics, on the EKF output grid."""

    t: np.ndarray  # [s], the EKF header.stamp grid
    model_pos: np.ndarray  # (N, 3)
    model_yaw: np.ndarray
    model_surge: np.ndarray
    err_pos: np.ndarray
    err_surge: np.ndarray
    horizon: np.ndarray
    info_text: str
    base_pos: np.ndarray | None = None
    base_yaw: np.ndarray | None = None
    base_surge: np.ndarray | None = None
    base_err_pos: np.ndarray | None = None
    base_err_surge: np.ndarray | None = None
    model_lat: np.ndarray | None = None
    model_lon: np.ndarray | None = None
    model_alt: np.ndarray | None = None
    thruster_forces: np.ndarray | None = None  # (N, 6) [N] per servo channel, NaN = unknown
    battery_voltage: np.ndarray | None = None  # (N,) [V] the voltage the thrust was evaluated at


def _sec_to_time(ts, t: float):
    ns = int(round(t * 1e9))
    return ts.types["builtin_interfaces/msg/Time"](sec=int(ns // 10**9), nanosec=int(ns % 10**9))


def _make_odom(ts, t: float, pos: np.ndarray, yaw: float, surge: float,
               frame_id: str, child_frame_id: str):
    T = ts.types
    q = Rot.from_euler("z", yaw).as_quat()
    return T["nav_msgs/msg/Odometry"](
        header=T["std_msgs/msg/Header"](stamp=_sec_to_time(ts, t), frame_id=frame_id),
        child_frame_id=child_frame_id,
        pose=T["geometry_msgs/msg/PoseWithCovariance"](
            pose=T["geometry_msgs/msg/Pose"](
                position=T["geometry_msgs/msg/Point"](x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
                orientation=T["geometry_msgs/msg/Quaternion"](
                    x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
                ),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        twist=T["geometry_msgs/msg/TwistWithCovariance"](
            twist=T["geometry_msgs/msg/Twist"](
                linear=T["geometry_msgs/msg/Vector3"](x=float(surge), y=0.0, z=0.0),
                angular=T["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )


def _make_navsatfix(ts, t: float, lat: float, lon: float, alt: float):
    """Same shape as navsat_transform's /gps/filtered: frame base_link, covariance unknown."""
    T = ts.types
    status = T["sensor_msgs/msg/NavSatStatus"]
    fix = T["sensor_msgs/msg/NavSatFix"]
    return fix(
        header=T["std_msgs/msg/Header"](stamp=_sec_to_time(ts, t), frame_id="base_link"),
        status=status(status=0, service=1),  # STATUS_FIX, SERVICE_GPS
        latitude=float(lat),
        longitude=float(lon),
        altitude=float(alt),
        position_covariance=np.zeros(9, dtype=np.float64),
        position_covariance_type=0,  # COVARIANCE_TYPE_UNKNOWN
    )


def write_augmented_bag(src: Path, out_path: Path, series: OutputSeries,
                        frame_id: str, child_frame_id: str) -> Path:
    """Copy every message of `src` into a new bag and append the /sysid/* topics.

    `out_path` is the final bare .mcap file. The rosbags writer can only produce a rosbag2
    directory, so it writes into a temporary one alongside the target and the inner .mcap is
    moved out afterwards; this keeps the output in the same bare-file form as the recordings
    in recordings/, which both this script and Foxglove read directly.

    Returns the path of the written .mcap file.
    """
    if out_path.exists():
        raise FileExistsError(f"{out_path} already exists (delete it to regenerate)")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_dir = out_path.parent / f"{out_path.stem}.tmp_rosbag2"
    if out_dir.exists():
        shutil.rmtree(out_dir)

    has_base = series.base_pos is not None

    with AnyReader([src]) as reader:
        ts = reader.typestore
        with Writer(out_dir, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
            conn_map = {}
            for c in reader.connections:
                qos = c.ext.offered_qos_profiles if isinstance(c.ext, ConnectionExtRosbag2) else ""
                conn_map[c.id] = writer.add_connection(
                    c.topic, c.msgtype, typestore=ts, offered_qos_profiles=qos
                )

            def new(topic: str, msgtype: str):
                return writer.add_connection(topic, msgtype, typestore=ts)

            c_model = new("/sysid/model_odom", "nav_msgs/msg/Odometry")
            c_base = new("/sysid/powerlaw_odom", "nav_msgs/msg/Odometry") if has_base else None
            has_geo = series.model_lat is not None
            c_navsat = new("/sysid/model_navsatfix", "sensor_msgs/msg/NavSatFix") if has_geo else None
            c_info = new("/sysid/model_info", "std_msgs/msg/String")
            c_errp = new("/sysid/error/position_m", "std_msgs/msg/Float64")
            c_erru = new("/sysid/error/surge_mps", "std_msgs/msg/Float64")
            c_hor = new("/sysid/horizon_s", "std_msgs/msg/Float64")
            has_forces = series.thruster_forces is not None
            c_forces = (new("/sysid/thruster_forces", "std_msgs/msg/Float64MultiArray")
                        if has_forces else None)
            has_volts = series.battery_voltage is not None
            c_volts = new("/sysid/battery_voltage", "std_msgs/msg/Float64") if has_volts else None

            for c, t_ns, raw in reader.messages():
                writer.write(conn_map[c.id], t_ns, raw)

            f64 = ts.types["std_msgs/msg/Float64"]
            string = ts.types["std_msgs/msg/String"]
            if has_forces:
                n_ch = series.thruster_forces.shape[1]
                f64_array = ts.types["std_msgs/msg/Float64MultiArray"]
                layout = ts.types["std_msgs/msg/MultiArrayLayout"](
                    dim=[ts.types["std_msgs/msg/MultiArrayDimension"](
                        label="servo_channel", size=n_ch, stride=n_ch)],
                    data_offset=0,
                )

            def put(conn, t: float, msg, typename: str) -> None:
                writer.write(conn, int(round(t * 1e9)), ts.serialize_cdr(msg, typename))

            for i, t in enumerate(series.t):
                put(c_model,
                    t,
                    _make_odom(ts, t, series.model_pos[i], series.model_yaw[i],
                               series.model_surge[i], frame_id, child_frame_id),
                    "nav_msgs/msg/Odometry")
                if has_base:
                    put(c_base,
                        t,
                        _make_odom(ts, t, series.base_pos[i], series.base_yaw[i],
                                   series.base_surge[i], frame_id, child_frame_id),
                        "nav_msgs/msg/Odometry")
                if has_geo:
                    put(c_navsat, t,
                        _make_navsatfix(ts, t, series.model_lat[i], series.model_lon[i],
                                        series.model_alt[i]),
                        "sensor_msgs/msg/NavSatFix")
                put(c_errp, t, f64(data=float(series.err_pos[i])), "std_msgs/msg/Float64")
                put(c_erru, t, f64(data=float(series.err_surge[i])), "std_msgs/msg/Float64")
                put(c_hor, t, f64(data=float(series.horizon[i])), "std_msgs/msg/Float64")
                if has_forces:
                    put(c_forces, t,
                        f64_array(layout=layout,
                                  data=np.asarray(series.thruster_forces[i], dtype=np.float64)),
                        "std_msgs/msg/Float64MultiArray")
                if has_volts:
                    put(c_volts, t, f64(data=float(series.battery_voltage[i])),
                        "std_msgs/msg/Float64")

            # model_info at 1 Hz across the whole output span, so it is visible wherever
            # the user scrubs to in Foxglove.
            for t in np.arange(series.t[0], series.t[-1] + 1.0, 1.0):
                put(c_info, float(t), string(data=series.info_text), "std_msgs/msg/String")

    written = list(out_dir.glob("*.mcap"))
    if not written:
        raise RuntimeError(f"writer produced no .mcap in {out_dir}")
    written[0].replace(out_path)
    shutil.rmtree(out_dir)  # drops metadata.yaml; nothing offline reads it
    return out_path
