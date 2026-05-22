"""
Online diagnostic for offline-replay EKF debugging.

Subscribes to the inputs and outputs of the global pose source
(`ekf_global_node` or `gnss_anchored_pose`) during offline replay, computes
per-axis innovations, and writes a CSV row per `/odometry/filtered/global`
message. On shutdown, prints a one-page summary.

Designed to run inside `offline_ekf_replay.launch.py` or
`offline_anchored_replay.launch.py` with `use_sim_time:=true`. Pure subscriber;
does not publish any topic, does not modify any other node's behaviour.

Why this exists
---------------
The 5-day debugging campaign in `POLARIS/research/EKF_RESEARCH_NOTES.md`
tested every parameter knob and architectural variant; the residual
~10–60 km offline-replay divergence has not been isolated *in time*. This
node logs everything the global EKF sees and emits, so a single replay run
produces enough evidence to localise where the divergence enters the state.

Output
------
`output_dir/diag.csv` — one row per `/odometry/filtered/global` message; see
`CSV_HEADER` for columns. Empty cells indicate the corresponding upstream
topic hasn't arrived yet (e.g. SBL before datum is locked, or GPS before
`gnss_datum_watchdog` has spawned `navsat_transform`).

A summary block is printed at shutdown: median / p95 / |max| of each
innovation channel, max P diagonal with timestamps, dt distribution.

Conventions
-----------
- yaw is extracted from the quaternion's z-axis rotation (valid for level
  AUVs).
- "world velocity innovation" rotates the local-EKF body-frame twist by the
  global EKF's yaw and compares to the global EKF's twist. If the global
  EKF is correctly slaved to local, this should be zero-mean small.
- SBL is reprojected to map frame using the first valid `/gps/validated`
  fix as the datum (matches what `gnss_datum_watchdog` hands to
  `navsat_transform`). Requires `pyproj`; otherwise SBL columns stay empty.
"""
from __future__ import annotations

import csv
import math
import signal
import statistics
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


# RELIABLE high-depth QoS for the global-odom subscriber. Earlier the diag
# subscribed with qos_profile_sensor_data (BEST_EFFORT, depth=5); under bag
# replay at 2× rate with the EKF publishing at 30 Hz, the diag's CSV-write
# callback couldn't keep up and most messages were dropped — making the
# "publish rate" derived from CSV row count an artifact of the diag's
# throughput, not the EKF's actual rate. Using RELIABLE with a deep queue
# forces the diag to capture every message; if the diag stalls under the
# load that's still informative (means the EKF really is publishing fast).
_GLOBAL_ODOM_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=200,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _utm_epsg(lat: float, lon: float) -> str:
    zone = int((lon + 180.0) / 6.0) + 1
    hemisphere = "6" if lat >= 0.0 else "7"
    return f"EPSG:32{hemisphere}{zone:02d}"


CSV_HEADER = [
    "t_sim",
    "dt_since_prev_global",
    "x", "y", "z", "yaw",
    "vx_world", "vy_world", "vz_world",
    "P_xx", "P_yy", "P_zz", "P_yaw", "P_vx", "P_vy",
    "lx", "ly", "lz", "lyaw",
    "lvx_body", "lvy_body", "lvz_body",
    "P_lvx", "P_lvy",
    "gx", "gy", "gps_age_s", "R_gps_x", "R_gps_y",
    "innov_x", "innov_y", "innov_yaw",
    "innov_vx_world", "innov_vy_world",
    "sbl_x", "sbl_y", "sbl_age_s",
    "sbl_err_x", "sbl_err_y", "sbl_err_m",
]


class EkfOfflineDiagnostic(Node):

    def __init__(self) -> None:
        super().__init__("ekf_offline_diagnostic")

        self.declare_parameter("output_dir", "/tmp/ekf_diag")
        self.declare_parameter("global_odom_topic", "/odometry/filtered/global")
        self.declare_parameter("local_odom_topic", "/odometry/filtered/local_validated")
        self.declare_parameter("gps_odom_topic", "/odometry/gps")
        self.declare_parameter("sbl_topic", "/waterlinked_ugps/navsatfix")
        # Preferred datum source: the NavSatFix the global track itself emits
        # (gnss_anchored_pose publishes /gps/filtered/global; the global EKF
        # stack publishes /gps/filtered via global_ekf_to_navsatfix_node).
        # Its first message's lat/lon IS the algorithm's anchor datum, so the
        # diagnostic's SBL projection uses the same datum as the algorithm,
        # eliminating any "first-/gps/selected vs first-h_acc-good" timing
        # mismatch that would otherwise show up as a constant offset between
        # SBL and global tracks. Fallback: datum_topic_fallback (a raw GPS
        # NavSatFix topic) if the global-track NavSatFix never publishes —
        # that lets the diagnostic still produce SBL columns in pure-input
        # debug runs without an algorithm node.
        self.declare_parameter("datum_navsatfix_topic", "/gps/filtered/global")
        self.declare_parameter("datum_topic_fallback", "/gps/validated")
        self.declare_parameter("gps_max_age_s", 5.0)
        self.declare_parameter("sbl_max_age_s", 5.0)

        out_dir = Path(str(self.get_parameter("output_dir").value))
        out_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = out_dir / "diag.csv"
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_HEADER)
        self._csv_file.flush()

        self._gps_max_age_s: float = float(self.get_parameter("gps_max_age_s").value)
        self._sbl_max_age_s: float = float(self.get_parameter("sbl_max_age_s").value)
        self._global_topic: str = str(self.get_parameter("global_odom_topic").value)
        self._local_topic: str = str(self.get_parameter("local_odom_topic").value)
        self._gps_topic: str = str(self.get_parameter("gps_odom_topic").value)
        self._sbl_topic: str = str(self.get_parameter("sbl_topic").value)
        self._datum_navsatfix_topic: str = str(self.get_parameter("datum_navsatfix_topic").value)
        self._datum_fallback_topic: str = str(self.get_parameter("datum_topic_fallback").value)
        # Track which source actually locked the datum so the summary log
        # makes the bookkeeping unambiguous.
        self._datum_source: str = "<unset>"

        self._latest_local: Optional[Odometry] = None
        self._latest_gps: Optional[Odometry] = None
        self._latest_sbl: Optional[NavSatFix] = None
        self._prev_global_t: Optional[float] = None
        self._n_rows = 0

        self._datum_lat: Optional[float] = None
        self._datum_lon: Optional[float] = None
        self._datum_utm_x: Optional[float] = None
        self._datum_utm_y: Optional[float] = None
        self._utm_transformer = None

        self._innov_x: list[float] = []
        self._innov_y: list[float] = []
        self._innov_yaw: list[float] = []
        self._innov_vx: list[float] = []
        self._innov_vy: list[float] = []
        self._dt_global: list[float] = []
        self._sbl_err: list[float] = []

        self._max_pxx = 0.0
        self._max_pyy = 0.0
        self._max_pyaw = 0.0
        self._max_pxx_t = float("nan")
        self._max_pyy_t = float("nan")
        self._max_pyaw_t = float("nan")
        self._t_first: Optional[float] = None
        self._t_last: Optional[float] = None

        self._sub_global = self.create_subscription(
            Odometry, self._global_topic, self._on_global, _GLOBAL_ODOM_QOS
        )
        self._sub_local = self.create_subscription(
            Odometry, self._local_topic, self._on_local, qos_profile_sensor_data
        )
        self._sub_gps = self.create_subscription(
            Odometry, self._gps_topic, self._on_gps_odom, qos_profile_sensor_data
        )
        self._sub_sbl = self.create_subscription(
            NavSatFix, self._sbl_topic, self._on_sbl, qos_profile_sensor_data
        )
        # Primary datum source: the global track's own NavSatFix output.
        # First message's lat/lon = algorithm's anchor datum, by definition.
        self._sub_datum_primary = self.create_subscription(
            NavSatFix,
            self._datum_navsatfix_topic,
            lambda m: self._on_datum(m, self._datum_navsatfix_topic),
            qos_profile_sensor_data,
        )
        # Fallback: only used if the primary topic has not delivered any
        # message by the time the first /odometry/filtered/global arrives.
        # Useful for runs where the algorithm doesn't emit a NavSatFix.
        self._sub_datum_fallback = self.create_subscription(
            NavSatFix,
            self._datum_fallback_topic,
            self._on_datum_fallback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"writing {self._csv_path} "
            f"global={self._global_topic} local={self._local_topic} "
            f"gps={self._gps_topic} sbl={self._sbl_topic} "
            f"datum_primary={self._datum_navsatfix_topic} "
            f"datum_fallback={self._datum_fallback_topic}"
        )

    def _on_local(self, msg: Odometry) -> None:
        self._latest_local = msg

    def _on_gps_odom(self, msg: Odometry) -> None:
        self._latest_gps = msg

    def _on_sbl(self, msg: NavSatFix) -> None:
        self._latest_sbl = msg

    def _on_datum(self, msg: NavSatFix, source: str) -> None:
        if self._datum_lat is not None:
            return
        if msg.status.status < 0:
            return
        if abs(msg.latitude) < 0.1 and abs(msg.longitude) < 0.1:
            return
        self._datum_lat = msg.latitude
        self._datum_lon = msg.longitude
        self._datum_source = source
        self._init_utm()
        self.get_logger().info(
            f"datum locked from {source}: "
            f"lat={self._datum_lat:.7f} lon={self._datum_lon:.7f}"
        )

    def _on_datum_fallback(self, msg: NavSatFix) -> None:
        # Only honor the fallback if the primary topic hasn't delivered.
        # This avoids the timing race where /gps/validated and
        # /gps/filtered/global both have valid messages but /gps/validated
        # arrives first because it's published earlier in the pipeline.
        if self._datum_lat is not None:
            return
        # Defer for ~5 s to give the primary a chance.
        if (self._t_first is None
                or (self.get_clock().now().nanoseconds * 1e-9
                    - (self._t_first or 0.0)) < 5.0):
            return
        self._on_datum(msg, f"fallback:{self._datum_fallback_topic}")

    def _init_utm(self) -> None:
        try:
            from pyproj import Transformer
        except ImportError:
            self.get_logger().warn(
                "pyproj not available; sbl_* columns will stay empty"
            )
            return
        epsg = _utm_epsg(self._datum_lat, self._datum_lon)
        self._utm_transformer = Transformer.from_crs(
            "EPSG:4326", epsg, always_xy=True
        )
        x, y = self._utm_transformer.transform(self._datum_lon, self._datum_lat)
        self._datum_utm_x = x
        self._datum_utm_y = y

    def _on_global(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t_first is None:
            self._t_first = t
        self._t_last = t

        if self._prev_global_t is None:
            dt = float("nan")
        else:
            dt = t - self._prev_global_t
            self._dt_global.append(dt)
        self._prev_global_t = t

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        # Pose covariance is row-major 6x6. Diagonals for x,y,z,roll,pitch,yaw
        # live at indices 0, 7, 14, 21, 28, 35. Twist covariance similar for
        # vx,vy,vz,vroll,vpitch,vyaw.
        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance
        P_xx = pose_cov[0]
        P_yy = pose_cov[7]
        P_zz = pose_cov[14]
        P_yaw = pose_cov[35]
        P_vx = twist_cov[0]
        P_vy = twist_cov[7]

        if P_xx > self._max_pxx:
            self._max_pxx = P_xx
            self._max_pxx_t = t
        if P_yy > self._max_pyy:
            self._max_pyy = P_yy
            self._max_pyy_t = t
        if P_yaw > self._max_pyaw:
            self._max_pyaw = P_yaw
            self._max_pyaw_t = t

        lx = ly = lz = lyaw = float("nan")
        lvx_body = lvy_body = lvz_body = float("nan")
        P_lvx = P_lvy = float("nan")
        if self._latest_local is not None:
            lp = self._latest_local.pose.pose.position
            lq = self._latest_local.pose.pose.orientation
            lvb = self._latest_local.twist.twist.linear
            lx, ly, lz = lp.x, lp.y, lp.z
            lyaw = _yaw_from_quat(lq.x, lq.y, lq.z, lq.w)
            lvx_body, lvy_body, lvz_body = lvb.x, lvb.y, lvb.z
            ltc = self._latest_local.twist.covariance
            P_lvx = ltc[0]
            P_lvy = ltc[7]

        gx = gy = float("nan")
        gps_age = float("nan")
        R_gps_x = R_gps_y = float("nan")
        innov_x = innov_y = float("nan")
        if self._latest_gps is not None:
            gt = (
                self._latest_gps.header.stamp.sec
                + self._latest_gps.header.stamp.nanosec * 1e-9
            )
            age = t - gt
            if 0.0 <= age <= self._gps_max_age_s:
                gp = self._latest_gps.pose.pose.position
                gx, gy = gp.x, gp.y
                gps_age = age
                gpc = self._latest_gps.pose.covariance
                R_gps_x = gpc[0]
                R_gps_y = gpc[7]
                innov_x = gx - p.x
                innov_y = gy - p.y
                self._innov_x.append(innov_x)
                self._innov_y.append(innov_y)

        innov_yaw = float("nan")
        innov_vx_world = innov_vy_world = float("nan")
        if not math.isnan(lyaw):
            innov_yaw = _wrap_pi(lyaw - yaw)
            self._innov_yaw.append(innov_yaw)
            if not math.isnan(lvx_body):
                lvx_w = lvx_body * math.cos(yaw) - lvy_body * math.sin(yaw)
                lvy_w = lvx_body * math.sin(yaw) + lvy_body * math.cos(yaw)
                innov_vx_world = lvx_w - v.x
                innov_vy_world = lvy_w - v.y
                self._innov_vx.append(innov_vx_world)
                self._innov_vy.append(innov_vy_world)

        sbl_x = sbl_y = float("nan")
        sbl_age = float("nan")
        sbl_err_x = sbl_err_y = sbl_err_m = float("nan")
        if (
            self._latest_sbl is not None
            and self._datum_utm_x is not None
            and self._utm_transformer is not None
        ):
            st = (
                self._latest_sbl.header.stamp.sec
                + self._latest_sbl.header.stamp.nanosec * 1e-9
            )
            sbl_age = t - st
            if abs(sbl_age) < self._sbl_max_age_s and self._latest_sbl.status.status >= 0:
                sx_utm, sy_utm = self._utm_transformer.transform(
                    self._latest_sbl.longitude, self._latest_sbl.latitude
                )
                sbl_x = sx_utm - self._datum_utm_x
                sbl_y = sy_utm - self._datum_utm_y
                sbl_err_x = sbl_x - p.x
                sbl_err_y = sbl_y - p.y
                sbl_err_m = math.hypot(sbl_err_x, sbl_err_y)
                self._sbl_err.append(sbl_err_m)

        def fmt(x: float, prec: int) -> str:
            return "" if math.isnan(x) else f"{x:.{prec}f}"

        def fmtsci(x: float) -> str:
            return "" if math.isnan(x) else f"{x:.6e}"

        self._csv.writerow([
            f"{t:.6f}",
            fmt(dt, 6),
            f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}", f"{yaw:.6f}",
            f"{v.x:.4f}", f"{v.y:.4f}", f"{v.z:.4f}",
            f"{P_xx:.6e}", f"{P_yy:.6e}", f"{P_zz:.6e}",
            f"{P_yaw:.6e}", f"{P_vx:.6e}", f"{P_vy:.6e}",
            fmt(lx, 4), fmt(ly, 4), fmt(lz, 4), fmt(lyaw, 6),
            fmt(lvx_body, 4), fmt(lvy_body, 4), fmt(lvz_body, 4),
            fmtsci(P_lvx), fmtsci(P_lvy),
            fmt(gx, 4), fmt(gy, 4), fmt(gps_age, 3),
            fmtsci(R_gps_x), fmtsci(R_gps_y),
            fmt(innov_x, 4), fmt(innov_y, 4), fmt(innov_yaw, 6),
            fmt(innov_vx_world, 4), fmt(innov_vy_world, 4),
            fmt(sbl_x, 4), fmt(sbl_y, 4), fmt(sbl_age, 3),
            fmt(sbl_err_x, 4), fmt(sbl_err_y, 4), fmt(sbl_err_m, 4),
        ])
        self._n_rows += 1
        if self._n_rows % 200 == 0:
            self._csv_file.flush()

    def _summary_lines(self) -> list[str]:
        def stats(xs: list[float], unit: str) -> str:
            if not xs:
                return "   (no samples)"
            xs_sorted = sorted(xs)
            n = len(xs_sorted)
            med = statistics.median(xs_sorted)
            mean = statistics.fmean(xs_sorted)
            try:
                p95 = xs_sorted[int(0.95 * (n - 1))]
            except IndexError:
                p95 = xs_sorted[-1]
            mx = max(xs_sorted, key=abs)
            return (
                f"   n={n}  mean={mean:+.4f} {unit}  median={med:+.4f} {unit}  "
                f"|p95|={abs(p95):.4f} {unit}  |max|={abs(mx):.4f} {unit}"
            )

        L = ["=" * 72]
        duration = (self._t_last - self._t_first) if self._t_first else float("nan")
        L.append(
            f"ekf_offline_diagnostic SUMMARY  rows={self._n_rows}  "
            f"duration_s={duration:.1f}  csv={self._csv_path}"
        )
        L.append("=" * 72)
        L.append("Innovation x  (gps_x - global_x) — biased mean = GPS pulling on x")
        L.append(stats(self._innov_x, "m"))
        L.append("Innovation y  (gps_y - global_y)")
        L.append(stats(self._innov_y, "m"))
        L.append(
            "Innovation yaw  (local_yaw - global_yaw) — non-zero mean = orientation"
        )
        L.append("                                          fusion is not happening.")
        L.append(stats(self._innov_yaw, "rad"))
        L.append(
            "Innovation vx_world  (R(global_yaw)*local_v_body - global_vx) — non-zero"
        )
        L.append("                                                              mean ⇒ velocity")
        L.append("                                                              fusion drift.")
        L.append(stats(self._innov_vx, "m/s"))
        L.append("Innovation vy_world")
        L.append(stats(self._innov_vy, "m/s"))
        L.append("dt between consecutive /odometry/filtered/global publishes")
        L.append(stats(self._dt_global, "s"))
        if self._sbl_err:
            L.append("|global - SBL| (independent ground truth, when datum locked)")
            L.append(stats(self._sbl_err, "m"))
        else:
            L.append("|global - SBL|: (no SBL samples — datum not locked or no SBL topic)")
        L.append(f"max P_xx  = {self._max_pxx:.4e}  at t_sim = {self._max_pxx_t:.3f}")
        L.append(f"max P_yy  = {self._max_pyy:.4e}  at t_sim = {self._max_pyy_t:.3f}")
        L.append(f"max P_yaw = {self._max_pyaw:.4e}  at t_sim = {self._max_pyaw_t:.3f}")
        L.append(f"datum source: {self._datum_source}")
        L.append("Validator drops are logged separately at end of run by odometry_validator.")
        L.append("=" * 72)
        return L

    def destroy_node(self) -> bool:
        try:
            for line in self._summary_lines():
                self.get_logger().info(line)
        except Exception as e:
            self.get_logger().warn(f"summary print failed: {e}")
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfOfflineDiagnostic()

    # Convert SIGTERM into KeyboardInterrupt so the finally block runs and
    # destroy_node() flushes the CSV + prints the summary. replay_ekf_bag.sh
    # sends SIGTERM (default `kill`) for fast shutdown; without this handler,
    # Python exits immediately on SIGTERM and skips finally clauses.
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
