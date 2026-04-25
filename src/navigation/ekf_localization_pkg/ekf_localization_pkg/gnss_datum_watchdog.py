import subprocess
import tempfile
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

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
    imu_topic            IMU topic forwarded to navsat        (default /imu/data)
    odom_topic           Local odom forwarded to navsat       (default /odometry/filtered/local)
    navsat_params_file   Base navsat_transform YAML path      (default: package navsat_transform.yaml)
    global_ekf_params_file  Global EKF YAML path             (default: package ekf_global.yaml)
    use_global_ekf       Also launch ekf_global_node         (default true)
    """

    def __init__(self) -> None:
        super().__init__("gnss_datum_watchdog")

        self.declare_parameter("fix_topic", "/gps/selected")
        self.declare_parameter("h_acc_topic", "/ubx_nav_hp_pos_llh")
        self.declare_parameter("h_acc_max_m", 0.50)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odometry/filtered/local")
        self.declare_parameter("navsat_params_file", "")
        self.declare_parameter("global_ekf_params_file", "")
        self.declare_parameter("use_global_ekf", True)

        fix_topic: str = self.get_parameter("fix_topic").value
        h_acc_topic: str = self.get_parameter("h_acc_topic").value
        self._h_acc_max_m: float = self.get_parameter("h_acc_max_m").value
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

    # ------------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        if self._launched:
            return
        if msg.status.status < 0:
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

    # ------------------------------------------------------------------

    def _spawn_navsat_and_global(self, fix: NavSatFix) -> None:
        h_acc_str = (
            f", h_acc={self._latest_h_acc_m * 100:.1f} cm"
            if self._latest_h_acc_m is not None
            else ""
        )
        self.get_logger().info(
            f"Valid fix: lat={fix.latitude:.7f}° lon={fix.longitude:.7f}° "
            f"alt={fix.altitude:.1f} m{h_acc_str} — spawning navsat_transform + global EKF"
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

        cmd = [
            "ros2", "launch", "ekf_localization_pkg", "navsat_global_ekf.launch.py",
            f"datum_yaml:={datum_file}",
            f"datum_lat:={fix.latitude}",
            f"datum_lon:={fix.longitude}",
            f"datum_alt:={fix.altitude}",
            f"gps_fix_topic:={self._fix_topic}",
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssDatumWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
