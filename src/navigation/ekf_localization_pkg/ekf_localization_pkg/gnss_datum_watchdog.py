import math
import signal
import subprocess
import tempfile
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

_DEG2RAD = math.pi / 180.0
_EARTH_R_M = 6_371_000.0


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = (lat2 - lat1) * _DEG2RAD
    dlon = (lon2 - lon1) * _DEG2RAD
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1 * _DEG2RAD) * math.cos(lat2 * _DEG2RAD) * math.sin(dlon / 2) ** 2
    )
    return 2.0 * _EARTH_R_M * math.asin(math.sqrt(a))

try:
    from ublox_ubx_msgs.msg import UBXNavHPPosLLH as _UBXNavHPPosLLH
    _HPPOSLLH_AVAILABLE = True
except ImportError:
    _UBXNavHPPosLLH = None
    _HPPOSLLH_AVAILABLE = False

# UBX-NAV-HPPOSLLH h_acc field is in 0.1 mm units
_HPPOSLLH_H_ACC_TO_M = 1e-4


class GnssDatumWatchdog(Node):
    """
    Waits for a quality-gated GNSS fix, then spawns navsat_transform_node and
    ekf_global_node via navsat_global_ekf.launch.py with that fix as the datum.

    This guarantees navsat_transform never starts at null-island (0°, 0°): the
    GPS-dependent nodes are only launched once a valid position is confirmed.
    The local EKF (IMU + DVL + pressure) runs from system start, independently.

    Quality gate (both must pass when h_acc topic is available):
      - NavSatFix.status >= 0   (valid fix, not STATUS_NO_FIX)
      - |lat| > 0.1° and |lon| > 0.1°   (null-island guard)
      - UBX-NAV-HPPOSLLH h_acc > 0 and <= h_acc_max_m

    Parameters
    ----------
    fix_topic            NavSatFix topic                      (default /gps/selected)
    h_acc_topic          UBXNavHPPosLLH accuracy topic        (default /ubx_nav_hp_pos_llh)
    h_acc_max_m          Max acceptable horizontal accuracy   (default 0.50 m)
    max_fix_distance_m   Max haversine distance from datum    (default 100000 m = 100 km)
    imu_topic            IMU topic forwarded to navsat        (default /imu/data)
    odom_topic           Local odom forwarded to navsat       (default /odometry/filtered/local)
    navsat_params_file   Base navsat_transform YAML path      (default: package navsat_transform.yaml)
    global_ekf_params_file  Global EKF YAML path             (default: package ekf_global.yaml)
    use_global_ekf       Also launch ekf_global_node         (default true)

    After the datum is set, every incoming fix is validated by:
      - status >= 0 and |lat| > 0.1° (null-island guard)
      - haversine distance from datum <= max_fix_distance_m
    Valid fixes are republished on /gps/validated; navsat_transform subscribes
    to that topic instead of the raw fix topic so garbage fixes never reach it.
    """

    def __init__(self) -> None:
        super().__init__("gnss_datum_watchdog")

        self.declare_parameter("fix_topic", "/gps/selected")
        self.declare_parameter("h_acc_topic", "/ubx_nav_hp_pos_llh")
        self.declare_parameter("h_acc_max_m", 0.50)
        self.declare_parameter("max_fix_distance_m", 100_000.0)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odometry/filtered/local")
        self.declare_parameter("navsat_params_file", "")
        self.declare_parameter("global_ekf_params_file", "")
        self.declare_parameter("use_global_ekf", True)

        fix_topic: str = self.get_parameter("fix_topic").value
        h_acc_topic: str = self.get_parameter("h_acc_topic").value
        self._h_acc_max_m: float = self.get_parameter("h_acc_max_m").value
        self._max_fix_distance_m: float = self.get_parameter("max_fix_distance_m").value
        self._imu_topic: str = self.get_parameter("imu_topic").value
        self._odom_topic: str = self.get_parameter("odom_topic").value
        self._navsat_params: str = self.get_parameter("navsat_params_file").value
        self._global_ekf_params: str = self.get_parameter("global_ekf_params_file").value
        self._use_global_ekf: bool = bool(self.get_parameter("use_global_ekf").value)
        self._fix_topic = fix_topic
        self._use_sim_time: bool = bool(self.get_parameter("use_sim_time").value)

        self._latest_fix: NavSatFix | None = None
        self._latest_h_acc_m: float | None = None
        self._launched = False
        self._datum_lat: float | None = None
        self._datum_lon: float | None = None
        # Onset-tracking: log the first time global EKF magnitude crosses each
        # threshold so we can pin down when divergence starts (and what local
        # EKF reported at that moment).
        self._diag_levels_hit: dict[float, bool] = {}
        # Onset-tracking: stamp at first crossing for the [DIAG-summary] report.
        self._diag_levels_t: dict[float, float] = {}

        # Counters for the [DIAG-summary] log line.
        self._n_gps_received = 0
        self._n_gps_validated = 0
        self._n_gps_rejected_status = 0
        self._n_gps_rejected_null = 0
        self._n_gps_rejected_haversine = 0
        self._t_node_start: float = self.get_clock().now().nanoseconds * 1e-9
        self._t_lock_s: float | None = None

        # Validated GPS fixes forwarded to navsat_transform (replaces raw topic).
        self._validated_pub = self.create_publisher(
            NavSatFix, "/gps/validated", qos_profile_sensor_data
        )

        # Diagnostic monitors — warn the moment a position exceeds 10 km (way beyond
        # any lake test) to identify whether GPS or global EKF causes the crash.
        self._diag_warn_m = 10_000.0
        self._diag_odom_gps_sub = self.create_subscription(
            Odometry, "/odometry/gps", self._on_diag_odom_gps, 10
        )
        self._diag_global_sub = self.create_subscription(
            Odometry, "/odometry/filtered/global", self._on_diag_global_odom, 10
        )

        # Cache latest local EKF odom so we can log its position at datum-set time.
        self._latest_local_odom: Odometry | None = None
        self._diag_local_sub = self.create_subscription(
            Odometry, self._odom_topic, self._on_local_odom, 10
        )

        self._fix_sub = self.create_subscription(
            NavSatFix, fix_topic, self._on_fix, qos_profile_sensor_data
        )

        if h_acc_topic and _HPPOSLLH_AVAILABLE:
            self._h_acc_sub = self.create_subscription(
                _UBXNavHPPosLLH,
                h_acc_topic,
                self._on_hp_pos,
                qos_profile_sensor_data,
            )
        else:
            self._h_acc_sub = None
            if h_acc_topic and not _HPPOSLLH_AVAILABLE:
                self.get_logger().warn(
                    "ublox_ubx_msgs not available — h_acc quality gate disabled, "
                    "falling back to coordinate + status check only."
                )

        self.get_logger().info(
            f"Waiting for RTK fix on '{fix_topic}' "
            f"(h_acc ≤ {self._h_acc_max_m * 100:.0f} cm via '{h_acc_topic}') "
            f"before launching navsat_transform + global EKF"
        )

        # Periodic [DIAG-summary] every 60 s so a long replay log has periodic
        # checkpoints, not just an end-of-run summary.
        self._summary_timer = self.create_timer(60.0, self._log_diag_summary)

    # ------------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        self._n_gps_received += 1
        if self._launched:
            self._validate_and_republish(msg)
            return
        if msg.status.status < 0:
            self._n_gps_rejected_status += 1
            return
        self._latest_fix = msg
        self._check_and_launch()

    def _on_hp_pos(self, msg: _UBXNavHPPosLLH) -> None:
        self._latest_h_acc_m = msg.h_acc * _HPPOSLLH_H_ACC_TO_M
        if not self._launched:
            self._check_and_launch()

    def _check_and_launch(self) -> None:
        if self._launched or self._latest_fix is None:
            return

        # Reject coordinates within ~11 km of null island.
        fix = self._latest_fix
        if abs(fix.latitude) < 0.1 and abs(fix.longitude) < 0.1:
            return

        # h_acc gate
        if self._h_acc_sub is not None:
            if self._latest_h_acc_m is None:
                return
            if self._latest_h_acc_m <= 0.0 or self._latest_h_acc_m > self._h_acc_max_m:
                return

        self._launched = True
        self._spawn_navsat_and_global(fix)

    def _validate_and_republish(self, msg: NavSatFix) -> None:
        # Log every incoming fix so we can see exactly what arrives near the crash.
        self.get_logger().info(
            f"[GPS raw] status={msg.status.status} "
            f"lat={msg.latitude:.6f}° lon={msg.longitude:.6f}°",
            throttle_duration_sec=5.0,
        )
        if msg.status.status < 0:
            self._n_gps_rejected_status += 1
            self.get_logger().warn(
                f"[GPS reject] status={msg.status.status} "
                f"lat={msg.latitude:.6f}° lon={msg.longitude:.6f}° — bad fix status"
            )
            return
        if abs(msg.latitude) < 0.1 and abs(msg.longitude) < 0.1:
            self._n_gps_rejected_null += 1
            self.get_logger().warn(
                f"[GPS reject] lat={msg.latitude:.6f}° lon={msg.longitude:.6f}° — null island"
            )
            return
        if self._datum_lat is not None:
            dist = _haversine_m(
                self._datum_lat, self._datum_lon, msg.latitude, msg.longitude
            )
            if dist > self._max_fix_distance_m:
                self._n_gps_rejected_haversine += 1
                self.get_logger().warn(
                    f"[GPS reject] lat={msg.latitude:.6f}° lon={msg.longitude:.6f}° "
                    f"dist={dist / 1000.0:.1f} km from datum "
                    f"(max {self._max_fix_distance_m / 1000.0:.0f} km) — "
                    "likely corrupt/mislabeled NavSatFix; not forwarded to navsat_transform",
                )
                return
            self.get_logger().info(
                f"[GPS valid] lat={msg.latitude:.6f}° lon={msg.longitude:.6f}° "
                f"dist={dist:.0f} m from datum — forwarding to navsat_transform",
                throttle_duration_sec=10.0,
            )
        self._n_gps_validated += 1
        self._validated_pub.publish(msg)

    def _on_local_odom(self, msg: Odometry) -> None:
        self._latest_local_odom = msg

    def _on_diag_odom_gps(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(
            f"[GPS odom] x={x:.2f} m  y={y:.2f} m — navsat_transform → /odometry/gps",
            throttle_duration_sec=5.0,
        )
        if abs(x) > self._diag_warn_m or abs(y) > self._diag_warn_m:
            self.get_logger().warn(
                f"[DIAG] /odometry/gps out of range: "
                f"x={x / 1000.0:.2f} km, y={y / 1000.0:.2f} km — "
                "navsat_transform computed a far GPS odometry; crash imminent. "
                "A bad GPS fix likely slipped through the gateway.",
            )

    def _on_diag_global_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        r = (x * x + y * y) ** 0.5
        for level in (100.0, 1_000.0, 10_000.0):
            if r >= level and not self._diag_levels_hit.get(level, False):
                self._diag_levels_hit[level] = True
                t_now = self.get_clock().now().nanoseconds * 1e-9
                self._diag_levels_t[level] = t_now - self._t_node_start
                lx, ly = (None, None)
                if self._latest_local_odom is not None:
                    lx = self._latest_local_odom.pose.pose.position.x
                    ly = self._latest_local_odom.pose.pose.position.y
                self.get_logger().warn(
                    f"[DIAG-onset] /odometry/filtered/global crossed {level:.0f} m: "
                    f"global=({x:.1f}, {y:.1f}), local=({lx}, {ly}) — "
                    "first crossing of this level"
                )
        if abs(x) > self._diag_warn_m or abs(y) > self._diag_warn_m:
            self.get_logger().warn(
                f"[DIAG] /odometry/filtered/global out of range: "
                f"x={x / 1000.0:.2f} km, y={y / 1000.0:.2f} km — "
                "global EKF diverged; publish_filtered_gps crash imminent. "
                "Check Q values for unsensored states.",
            )

    # ------------------------------------------------------------------

    def _log_diag_summary(self) -> None:
        t_now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = t_now - self._t_node_start
        datum = (
            f"({self._datum_lat:.7f}, {self._datum_lon:.7f})"
            if self._datum_lat is not None
            else "unset"
        )
        time_to_lock = (
            f"{self._t_lock_s:.1f}s" if self._t_lock_s is not None else "not yet"
        )
        thresholds = (
            ", ".join(
                f"{int(level)}m@{self._diag_levels_t[level]:.1f}s"
                for level in sorted(self._diag_levels_hit)
                if self._diag_levels_hit.get(level)
            )
            or "none"
        )
        self.get_logger().info(
            f"[DIAG-summary] elapsed={elapsed:.1f}s "
            f"datum={datum} time_to_lock={time_to_lock} "
            f"gps received={self._n_gps_received} "
            f"validated={self._n_gps_validated} "
            f"rejected(status/null/haversine)="
            f"{self._n_gps_rejected_status}/"
            f"{self._n_gps_rejected_null}/"
            f"{self._n_gps_rejected_haversine} "
            f"diag_thresholds_hit=[{thresholds}]"
        )

    def _spawn_navsat_and_global(self, fix: NavSatFix) -> None:
        self._datum_lat = fix.latitude
        self._datum_lon = fix.longitude
        t_now = self.get_clock().now().nanoseconds * 1e-9
        self._t_lock_s = t_now - self._t_node_start

        h_acc_str = (
            f", h_acc={self._latest_h_acc_m * 100:.1f} cm"
            if self._latest_h_acc_m is not None
            else ""
        )
        local_pos = ""
        if self._latest_local_odom is not None:
            lx = self._latest_local_odom.pose.pose.position.x
            ly = self._latest_local_odom.pose.pose.position.y
            local_pos = f" | local EKF odom at datum-set: x={lx:.2f} m  y={ly:.2f} m"
        self.get_logger().info(
            f"Valid fix: lat={fix.latitude:.7f}° lon={fix.longitude:.7f}° "
            f"alt={fix.altitude:.1f} m{h_acc_str} — spawning navsat_transform + global EKF"
            f"{local_pos}"
        )

        # Write a minimal YAML with just the datum; loaded second in the launch
        # so it overrides any placeholder datum in the base navsat params file.
        datum_yaml = (
            "navsat_transform_node:\n"
            "  ros__parameters:\n"
            f"    datum: [{fix.latitude}, {fix.longitude}, {fix.altitude}]\n"
        )
        datum_file = Path(tempfile.mkdtemp()) / "gnss_datum.yaml"
        datum_file.write_text(datum_yaml)

        # navsat_transform subscribes to /gps/validated so that bad/corrupt
        # NavSatFix messages (e.g. swapped lat/lon, projected coords, garbage
        # floats from a patched bag) never reach it and cause a UTM-range crash.
        cmd = [
            "ros2", "launch", "ekf_localization_pkg", "navsat_global_ekf.launch.py",
            f"datum_yaml:={datum_file}",
            f"datum_lat:={fix.latitude}",
            f"datum_lon:={fix.longitude}",
            f"datum_alt:={fix.altitude}",
            f"gps_fix_topic:=/gps/validated",
            f"imu_topic:={self._imu_topic}",
            f"odom_topic:={self._odom_topic}",
            f"use_global_ekf:={'true' if self._use_global_ekf else 'false'}",
            f"use_sim_time:={'true' if self._use_sim_time else 'false'}",
        ]
        if self._navsat_params:
            cmd.append(f"navsat_params_file:={self._navsat_params}")
        if self._global_ekf_params:
            cmd.append(f"global_ekf_params_file:={self._global_ekf_params}")

        subprocess.Popen(cmd)
        self.get_logger().info(
            "navsat_transform_node + ekf_global_node launched with precise datum."
        )

    def destroy_node(self) -> bool:
        try:
            self._log_diag_summary()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssDatumWatchdog()

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
