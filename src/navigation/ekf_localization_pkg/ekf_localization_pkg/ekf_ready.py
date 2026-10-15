"""
ekf_ready — one-shot (or --wait / --watch) readiness check of the localization stack.

Tells you whether it is safe to activate Nav2 and switch to GUIDED:

  NAV2 READY   local EKF publishing, GNSS datum locked, global EKF publishing,
               TF map -> odom -> base_link available
  GUIDED READY additionally: the mavlink bridge forwards external nav to the Pixhawk
               and the Pixhawk's yaw agrees with ours (else EKF3 latched an old heading
               and GUIDED is refused with "requires position" -> reboot Pixhawk disarmed)
  advisory     heading calibrated, heading_check error, GNSS gate state

Usage:
  ros2 run ekf_localization_pkg ekf_ready            # check once (~3 s), exit 0 if NAV2 READY
  ros2 run ekf_localization_pkg ekf_ready --wait     # block until NAV2 READY (timeout 300 s)
  ros2 run ekf_localization_pkg ekf_ready --watch    # print the status every 2 s
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)


class EkfReady(Node):
    def __init__(self) -> None:
        super().__init__("ekf_ready")
        self.t = {}       # topic -> monotonic time of last message
        self.v = {}       # topic -> last value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        def stamp(name, keep=None):
            def cb(msg):
                self.t[name] = time.monotonic()
                self.v[name] = keep(msg) if keep else msg
            return cb

        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, "/odometry/filtered/local", stamp("local"), sd)
        self.create_subscription(Odometry, "/odometry/filtered/global", stamp("global"), sd)
        self.create_subscription(NavSatFix, "/gnss_datum", stamp("datum"), LATCHED)
        self.create_subscription(String, "/imu_yaw_correction/calibration_result",
                                 stamp("cal", lambda m: json.loads(m.data)), LATCHED)
        self.create_subscription(Float64, "/imu_yaw_correction/yaw_offset_deg",
                                 stamp("offset", lambda m: m.data), LATCHED)
        self.create_subscription(Float64, "/heading_check/error_deg",
                                 stamp("hc", lambda m: m.data), LATCHED)
        self.create_subscription(String, "/gnss_datum_watchdog/gnss_gate",
                                 stamp("gate", lambda m: json.loads(m.data)), LATCHED)
        self.create_subscription(String, "/mavlink_bridge/extnav_status",
                                 stamp("extnav", lambda m: json.loads(m.data)), LATCHED)

    def fresh(self, name, max_age=1.0) -> bool:
        return name in self.t and time.monotonic() - self.t[name] < max_age

    def tf_ok(self) -> bool:
        try:
            return self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return False

    def evaluate(self):
        """Return (nav2_ready, guided_ready, lines)."""
        lines = []

        def row(ok, label, detail=""):
            mark = {True: "OK  ", False: "FAIL", None: "WARN"}[ok]
            lines.append(f"  [{mark}] {label}{(' — ' + detail) if detail else ''}")
            return ok

        local = row(self.fresh("local"), "local EKF publishing (/odometry/filtered/local)",
                    "" if self.fresh("local") else "no message in the last 1 s")
        datum = row("datum" in self.v, "GNSS datum locked (/gnss_datum)",
                    "" if "datum" in self.v else "watchdog still waiting for an RTK fix")
        glob = row(self.fresh("global"), "global EKF publishing (/odometry/filtered/global)",
                   "" if self.fresh("global") else "not yet (starts after the datum lock)")
        tf = row(self.tf_ok(), "TF map -> base_link available",
                 "" if self.tf_ok() else "map frame missing (global EKF / map_odom bootstrap)")
        nav2 = bool(local and datum and glob and tf)

        ex = self.v.get("extnav")
        if ex is None:
            fwd = row(False, "bridge forwards external nav to Pixhawk",
                      "no /mavlink_bridge/extnav_status (bridge not running / old version)")
            yaw_ok = False
        else:
            fwd = row(bool(ex.get("forwarding")), "bridge forwards external nav to Pixhawk",
                      ex.get("reason") or "")
            d = ex.get("pixhawk_yaw_minus_extnav_deg")
            if d is None:
                yaw_ok = row(None, "Pixhawk yaw agrees with ours", "no comparison yet")
                yaw_ok = bool(fwd)
            else:
                yaw_ok = row(abs(d) <= 10.0, "Pixhawk yaw agrees with ours",
                             f"{d:+.1f} deg" + ("" if abs(d) <= 10.0 else
                                                 " -> reboot the Pixhawk while DISARMED"))
        guided = bool(nav2 and fwd and yaw_ok)

        cal = self.v.get("cal")
        off = self.v.get("offset")
        if cal is None and off is None:
            row(None, "heading calibration", "no imu_yaw_correction status")
        else:
            st = (cal or {}).get("status", "none")
            row(None if st not in ("ok",) else True, "heading calibration",
                f"last result '{st}', yaw_offset_deg={off}")
        hc = self.v.get("hc")
        if hc is None:
            row(None, "heading_check", "not running (ros2 run ekf_localization_pkg heading_check)")
        else:
            row(abs(hc) <= 5.0 or None, "heading_check error", f"{hc:+.1f} deg (want |e| <= 5)")
        gate = self.v.get("gate")
        if gate is not None:
            row(True if gate.get("gate_open") else None, "GNSS gate",
                "open" if gate.get("gate_open") else f"closed ({gate.get('last_reason')})")
        return nav2, guided, lines


def main(args=None) -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--wait", action="store_true", help="block until NAV2 READY")
    p.add_argument("--watch", action="store_true", help="print status every 2 s until Ctrl+C")
    p.add_argument("--timeout", type=float, default=300.0, help="--wait timeout [s]")
    a, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = EkfReady()
    start = time.monotonic()
    nav2 = guided = False
    try:
        # let latched topics and a few odometry messages arrive
        while time.monotonic() - start < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        while True:
            nav2, guided, lines = node.evaluate()
            head = ("NAV2 READY" if nav2 else "NAV2 NOT READY") + " | " + (
                "GUIDED READY" if guided else "GUIDED NOT READY")
            print(f"\n=== {head} ===")
            print("\n".join(lines), flush=True)
            if not (a.wait or a.watch) or (a.wait and nav2):
                break
            if a.wait and time.monotonic() - start > a.timeout:
                print("timeout", file=sys.stderr)
                break
            t_next = time.monotonic() + 2.0
            while time.monotonic() < t_next:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if nav2 else 1)


if __name__ == "__main__":
    main()
