#!/usr/bin/env python3
"""SBL quick check: live sanity report of the raw SBL ranges (thesis lake test).

Only LISTENS to topics (no HTTP, no writes to the box) -> run it on the laptop, in parallel to the
stations / floating stops / missions. Every `window_s` seconds it prints, without any filter:

    ping rate, per receiver: share of valid pings, median range, ping-to-ping scatter, frozen share,
    rssi / nsd; pings with >= 3 / 4 receivers; box position valid; box fix - RTK (at the surface)
    VERDICT: OK / DEGRADED / BAD   (absolute sanity limits for gross failures only)
    vs reference: COMPARABLE / CLEARLY WORSE   (if a reference label is given)

It is a sanity check, not a proof: it rules out a gross failure. A small systematic range error is
invisible here. Maths: sbl_quick_math.py (unit-tested in test/test_sbl_quick_math.py).

Labels and references (no fixed quality thresholds; the morning's own numbers are the reference):
    label      name of what is being recorded now, e.g. station1_top. Each window is written to
               <out_dir>/<label>.json (latest window wins) and appended to <out_dir>/log.jsonl.
    reference  label of an earlier window to compare with, e.g. station1_top while testing the
               underside mount at the same station.
Both can be changed while running:
    ros2 param set /sbl_quick_check label station1_top
    ros2 param set /sbl_quick_check reference station1_top

Run:
    ros2 launch uwgpsg2_ros2_interface sbl_quick_check.launch.py label:=station1_top
    ros2 launch uwgpsg2_ros2_interface sbl_quick_check.launch.py label:=float1_under reference:=station1_top
    ros2 run uwgpsg2_ros2_interface sbl_quick_check
"""

import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix
from custom_msgs.msg import SblRaw

from .sbl_geo import haversine_m
from . import sbl_quick_math as qm

try:  # RTK gate is optional: without the ublox messages the fix is used ungated (and said so)
    from ublox_ubx_msgs.msg import UBXNavPVT
except ImportError:  # pragma: no cover
    UBXNavPVT = None

_CARR_SOLN_FIXED = 2


class SblQuickCheck(Node):
    def __init__(self):
        super().__init__("sbl_quick_check")
        p = self.declare_parameter
        # "auto": find the topic of type custom_msgs/msg/SblRaw on the graph (the driver's namespace differs
        # between launches: /waterlinked/ugps/... on dev, /waterlinked_ugps/... in older bags) and take
        # locator_position_global from the same namespace.
        self.raw_topic = str(p("sbl_raw_topic", "auto").value)
        self.global_topic = str(p("locator_global_topic", "auto").value)
        self.fix_topic = p("fix_topic", "/fix").value
        self.pvt_topic = p("pvt_topic", "/ubx_nav_pvt").value
        self.window_s = float(p("window_s", 30.0).value)
        self.require_rtk_fixed = bool(p("require_rtk_fixed", True).value)
        self.fix_max_age_s = float(p("fix_max_age_s", 1.5).value)
        self.out_dir = os.path.expanduser(str(p("out_dir", "~/sbl_quick_check").value))
        p("label", "")
        p("reference", "")

        self._pings = []            # dicts for sbl_quick_math.window_stats
        self._fix_errors = []       # (t_wall, distance box fix - RTK [m])
        self._rtk = None            # (t_wall, lat, lon)
        self._carr_soln = None

        self._sbl_subscribed = False
        self._try_subscribe_sbl()
        self.create_subscription(NavSatFix, self.fix_topic, self._on_fix, qos_profile_sensor_data)
        if UBXNavPVT is not None:
            self.create_subscription(UBXNavPVT, self.pvt_topic, self._on_pvt, qos_profile_sensor_data)
        else:
            self.get_logger().warn("ublox_ubx_msgs not available: RTK fix quality is not checked")
        self.create_timer(self.window_s, self._report)
        os.makedirs(self.out_dir, exist_ok=True)
        print(f"\n=== SBL quick check === window {self.window_s:.0f} s, reports -> {self.out_dir}\n", flush=True)

    def _try_subscribe_sbl(self):
        """Subscribe to sbl_raw and the box's global fix; with "auto", wait until the driver's topic shows up."""
        raw = self.raw_topic
        if raw == "auto":
            found = [name for name, types in self.get_topic_names_and_types()
                     if "custom_msgs/msg/SblRaw" in types]
            if not found:
                return False
            raw = sorted(found)[0]
            if len(found) > 1:
                self.get_logger().warn(f"several SblRaw topics {found}; using {raw}")
        glob = self.global_topic
        if glob == "auto":
            glob = raw.rsplit("/", 1)[0] + "/locator_position_global"
        self.create_subscription(SblRaw, raw, self._on_raw, 10)
        self.create_subscription(GeoPointStamped, glob, self._on_global, 10)
        self._sbl_subscribed = True
        print(f"listening to {raw} and {glob}", flush=True)
        return True

    # --- callbacks ---
    def _on_raw(self, msg):
        self._pings.append({
            "t": time.monotonic(),
            "distance": [float(x) for x in msg.receiver_distance],
            "valid": [int(x) for x in msg.receiver_valid],
            "rssi": [float(x) for x in msg.receiver_rssi],
            "nsd": [float(x) for x in msg.receiver_nsd],
            "position_valid": bool(msg.position_valid),
        })

    def _on_pvt(self, msg):
        try:
            self._carr_soln = int(msg.carr_soln.status)
        except AttributeError:
            self._carr_soln = None

    def _on_fix(self, msg):
        lat, lon = float(msg.latitude), float(msg.longitude)
        if abs(lat) < 0.1 and abs(lon) < 0.1:      # null-island guard
            return
        if UBXNavPVT is not None and self.require_rtk_fixed and self._carr_soln != _CARR_SOLN_FIXED:
            return
        self._rtk = (time.monotonic(), lat, lon)

    def _on_global(self, msg):
        if self._rtk is None or time.monotonic() - self._rtk[0] > self.fix_max_age_s:
            return                                  # no fresh RTK fix (submerged, or not FIXED)
        lat, lon = float(msg.position.latitude), float(msg.position.longitude)
        if abs(lat) < 0.1 and abs(lon) < 0.1:
            return
        self._fix_errors.append((time.monotonic(), haversine_m(lat, lon, self._rtk[1], self._rtk[2])))

    # --- report ---
    def _report(self):
        if not self._sbl_subscribed and not self._try_subscribe_sbl():
            print(time.strftime("%H:%M:%S ") + "no topic of type custom_msgs/msg/SblRaw on the graph yet "
                  "(is the Waterlinked driver running with publish_sbl_raw, same ROS_DOMAIN_ID?)", flush=True)
            return
        t0 = time.monotonic() - self.window_s
        self._pings = [x for x in self._pings if x["t"] >= t0]
        self._fix_errors = [x for x in self._fix_errors if x[0] >= t0]
        stats = qm.window_stats(self._pings, [e for _, e in self._fix_errors])

        label = str(self.get_parameter("label").value).strip()
        ref_label = str(self.get_parameter("reference").value).strip()
        ref = None
        if ref_label:
            path = os.path.join(self.out_dir, ref_label + ".json")
            try:
                ref = qm.load_stats(path)
            except (OSError, ValueError):
                print(f"  (reference '{ref_label}' not found at {path})", flush=True)
        print(time.strftime("%H:%M:%S ") + qm.format_report(stats, label, ref, ref_label) + "\n", flush=True)

        if stats["n_pings"] > 0:
            if label:
                qm.save_stats(os.path.join(self.out_dir, label + ".json"), stats)
            with open(os.path.join(self.out_dir, "log.jsonl"), "a", encoding="utf-8") as f:
                f.write(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "label": label,
                                    "reference": ref_label, "stats": stats}) + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = SblQuickCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\nstopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
