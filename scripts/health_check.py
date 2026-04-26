#!/usr/bin/env python3
"""Polaris System Health Check.

Run after start_system.launch to verify all sensors are operational.
Reports device presence (udev symlinks) and ROS topic health (rates and field values).
Exits 0 if all checks pass, 1 otherwise.

Usage:
    python3 scripts/health_check.py
    python3 scripts/health_check.py --no-delay
    python3 scripts/health_check.py --delay 30 --window 10
"""

import argparse
import os
import stat
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
    FluidPressure,
    Imu,
    NavSatFix,
)
from std_msgs.msg import Float32
from mavros_msgs.msg import State
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from custom_msgs.msg import Distance
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


# ============================================================
# Configuration
# ============================================================

DEFAULT_STARTUP_DELAY_SEC = 20
DEFAULT_MEASUREMENT_SEC = 5

# h_acc field is uint32 in 0.1 mm units. 10 m == 100_000 raw.
H_ACC_MAX_RAW = 100_000

# Device presence checks: (display name, /dev symlink)
DEVICE_CHECKS: List[Tuple[str, str]] = [
    ("IMU (Xsens)",   "/dev/xsens_imu"),
    ("Keller sensor", "/dev/keller"),
    ("Camera front",  "/dev/cam_front"),
    ("Camera tube",   "/dev/cam_tube"),
]

# Parameter toggle for the ice measurement publisher
ICE_MEASUREMENT_NODE = "/ice_measurement_publisher"
ICE_MEASUREMENT_PARAM = "recording"
ICE_MEASUREMENT_TOPIC = "/ping_sonar/distance"

# Topic checks: (display, topic, msg_type, kind, threshold)
#   kind = "min_rate"   -> threshold is float (Hz lower bound)
#   kind = "rate_band"  -> threshold is (lo, hi) tuple in Hz
#   kind = "received"   -> threshold ignored, just need >=1 msg
#   kind = "h_acc"      -> threshold is max raw h_acc value
TOPIC_CHECKS: List[Tuple[str, str, Any, str, Any]] = [
    ("IMU",                "/imu/data",                       Imu,             "min_rate",  50.0),
    ("Pixhawk heartbeat",  "/pixhawk/heartbeat",              State,           "rate_band", (0.5, 2.0)),
    ("BlueRobotics press", "/pixhawk/scaled_pressure",        FluidPressure,   "min_rate",  40.0),
    ("Keller pressure",    "/sensors/keller26x/abs_pressure", FluidPressure,   "min_rate",  10.0),
    ("SBL waterlinked",    "/waterlinked_ugps/navsatfix",     NavSatFix,       "min_rate",   1.0),
    ("Ultrasonic front",   "/front/ultrasonic/distance",      Float32,         "min_rate",   5.0),
    ("Ultrasonic top",     "/top/ultrasonic/distance",        Float32,         "min_rate",   5.0),
    ("Camera front info",  "/front/camera_info",              CameraInfo,      "received",  None),
    ("Camera front image", "/front/image_raw/compressed",     CompressedImage, "received",  None),
    ("Camera tube info",   "/tube/camera_info",               CameraInfo,      "received",  None),
    ("Camera tube image",  "/tube/image_raw/compressed",      CompressedImage, "received",  None),
    ("Ice measurement",    ICE_MEASUREMENT_TOPIC,             Distance,        "min_rate",   2.0),
    ("ublox DGNSS h_acc",  "/ubx_nav_hp_pos_llh",             UBXNavHPPosLLH,  "h_acc",     H_ACC_MAX_RAW),
]


# ============================================================
# Output formatting
# ============================================================

USE_COLOR = sys.stdout.isatty()


def _c(code: str) -> str:
    return code if USE_COLOR else ""


GREEN  = _c("\033[32m")
RED    = _c("\033[31m")
YELLOW = _c("\033[33m")
CYAN   = _c("\033[36m")
BOLD   = _c("\033[1m")
DIM    = _c("\033[2m")
RESET  = _c("\033[0m")

OK   = f"{GREEN}  OK  {RESET}"
FAIL = f"{RED} FAIL {RESET}"
WARN = f"{YELLOW} WARN {RESET}"


def _is(status: str, target: str) -> bool:
    return target in status


def banner(text: str) -> None:
    print()
    print(f"{BOLD}{CYAN}{'=' * 72}{RESET}")
    print(f"{BOLD}{CYAN}  {text}{RESET}")
    print(f"{BOLD}{CYAN}{'=' * 72}{RESET}")


def section(text: str) -> None:
    print(f"\n{BOLD}[{text}]{RESET}")


# ============================================================
# Phase 1 — device presence
# ============================================================

def check_devices() -> List[Tuple[str, str, str, str]]:
    """Returns (status, display_name, path, detail) per device."""
    results = []
    for name, path in DEVICE_CHECKS:
        if not os.path.exists(path):
            results.append((FAIL, name, path, "symlink missing"))
            continue
        try:
            st = os.stat(path)
        except OSError as exc:
            results.append((FAIL, name, path, f"stat failed: {exc}"))
            continue
        if not (stat.S_ISCHR(st.st_mode) or stat.S_ISBLK(st.st_mode)):
            results.append((WARN, name, path, "exists but not a device"))
            continue
        results.append((OK, name, path, "present"))
    return results


def print_device_results(results) -> Tuple[int, int]:
    section("DEVICES")
    name_w = max(len(n) for _, n, _, _ in results) + 2
    fails = warns = 0
    for status, name, path, detail in results:
        if _is(status, "FAIL"):
            fails += 1
        elif _is(status, "WARN"):
            warns += 1
        print(f" {status}  {name:<{name_w}} {DIM}{path}{RESET}  {detail}")
    return fails, warns


# ============================================================
# Phase 2 — ROS topic health
# ============================================================

@dataclass
class TopicMonitor:
    display_name: str
    topic: str
    msg_type: Any
    check_kind: str
    threshold: Any
    timestamps: List[float] = field(default_factory=list)
    last_msg: Optional[Any] = None


class HealthCheckNode(Node):
    def __init__(self, monitors: List[TopicMonitor]):
        super().__init__("polaris_health_check")
        self.monitors = monitors
        self._subs = []
        for mon in monitors:
            sub = self.create_subscription(
                mon.msg_type,
                mon.topic,
                self._make_callback(mon),
                qos_profile_sensor_data,
            )
            self._subs.append(sub)

    @staticmethod
    def _make_callback(mon: TopicMonitor) -> Callable:
        def cb(msg):
            mon.timestamps.append(time.monotonic())
            mon.last_msg = msg
        return cb


def set_ice_recording(node: Node, value: bool, timeout_sec: float = 3.0) -> Optional[str]:
    """Toggle the `recording` parameter on the ice measurement publisher.

    Returns None on success, or an error string on failure.
    """
    service_name = f"{ICE_MEASUREMENT_NODE}/set_parameters"
    client = node.create_client(SetParameters, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return f"service {service_name} not available"
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name=ICE_MEASUREMENT_PARAM,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL,
                    bool_value=value,
                ),
            )
        ]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            return "service call timed out"
        result = future.result()
        if result is None or not result.results:
            return "no result returned"
        for r in result.results:
            if not r.successful:
                return r.reason or "rejected by node"
        return None
    finally:
        node.destroy_client(client)


def evaluate(mon: TopicMonitor, window_sec: float) -> Tuple[str, str]:
    n = len(mon.timestamps)

    if mon.check_kind == "received":
        if n == 0:
            return FAIL, "no messages received"
        return OK, f"received ({n} msgs)"

    rate = n / window_sec

    if mon.check_kind == "min_rate":
        thr = mon.threshold
        if n == 0:
            return FAIL, f"  0.0 Hz   (min {thr:g})  no messages"
        if rate >= thr:
            return OK, f"{rate:5.1f} Hz   (min {thr:g})"
        return FAIL, f"{rate:5.1f} Hz   (min {thr:g})"

    if mon.check_kind == "rate_band":
        lo, hi = mon.threshold
        if n == 0:
            return FAIL, f"  0.0 Hz   ({lo}-{hi})  no messages"
        if lo <= rate <= hi:
            return OK, f"{rate:5.1f} Hz   ({lo}-{hi})"
        return WARN, f"{rate:5.1f} Hz   ({lo}-{hi})"

    if mon.check_kind == "h_acc":
        if mon.last_msg is None:
            return FAIL, "no messages received"
        h_acc_raw = int(mon.last_msg.h_acc)
        h_acc_m = h_acc_raw * 0.1 / 1000.0
        thr_m = mon.threshold * 0.1 / 1000.0
        detail = f"h_acc={h_acc_m:5.2f} m  (max {thr_m:g} m, n={n})"
        if h_acc_raw <= mon.threshold:
            return OK, detail
        return WARN, detail

    return WARN, f"unknown check kind: {mon.check_kind}"


def print_topic_results(results, window_sec: float) -> Tuple[int, int]:
    section(f"TOPICS  ({window_sec:g}s window)")
    name_w = max(len(name) for name, _, _, _ in results) + 2
    topic_w = max(len(t) for _, t, _, _ in results) + 2
    fails = warns = 0
    for name, topic, status, detail in results:
        if _is(status, "FAIL"):
            fails += 1
        elif _is(status, "WARN"):
            warns += 1
        print(f" {status}  {name:<{name_w}} {DIM}{topic:<{topic_w}}{RESET} {detail}")
    return fails, warns


# ============================================================
# Main flow
# ============================================================

def countdown(seconds: int, label: str) -> None:
    if seconds <= 0:
        return
    for remaining in range(seconds, 0, -1):
        print(f"\r  {label}: {remaining:3d}s ", end="", flush=True)
        time.sleep(1)
    print(f"\r  {label}: done.    ")


def sample_topics(node: Node, window_sec: float) -> None:
    end = time.monotonic() + window_sec
    while rclpy.ok() and time.monotonic() < end:
        remaining = max(0.0, end - time.monotonic())
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Polaris system health check (devices + ROS topics).",
    )
    parser.add_argument("--delay", type=int, default=DEFAULT_STARTUP_DELAY_SEC,
                        help="seconds to wait for nodes to stabilize (default: %(default)s)")
    parser.add_argument("--no-delay", action="store_true",
                        help="skip the startup delay")
    parser.add_argument("--window", type=float, default=DEFAULT_MEASUREMENT_SEC,
                        help="topic measurement window in seconds (default: %(default)s)")
    args = parser.parse_args(argv)

    delay = 0 if args.no_delay else max(0, args.delay)
    window = max(0.5, args.window)

    banner("POLARIS SYSTEM HEALTH CHECK")

    if delay:
        print()
        countdown(delay, "Waiting for nodes to stabilize")

    # Phase 1 — devices
    device_results = check_devices()
    dev_fails, dev_warns = print_device_results(device_results)

    # Phase 2 — ROS topics
    rclpy.init(args=None)
    monitors = [
        TopicMonitor(name, topic, mtype, kind, thr)
        for (name, topic, mtype, kind, thr) in TOPIC_CHECKS
    ]
    node = HealthCheckNode(monitors)

    ice_set_err: Optional[str] = None
    ice_recording_enabled = False

    try:
        ice_set_err = set_ice_recording(node, True)
        if ice_set_err is None:
            ice_recording_enabled = True

        print(f"\n  {DIM}Sampling topics for {window:g}s ...{RESET}")
        sample_topics(node, window)
    except KeyboardInterrupt:
        print(f"\n  {YELLOW}Interrupted by user{RESET}")
    finally:
        if ice_recording_enabled:
            set_ice_recording(node, False)

    # Evaluate
    topic_results = []
    for mon in monitors:
        if mon.topic == ICE_MEASUREMENT_TOPIC and ice_set_err is not None:
            status = FAIL
            detail = f"could not enable recording: {ice_set_err}"
        else:
            status, detail = evaluate(mon, window)
        topic_results.append((mon.display_name, mon.topic, status, detail))

    topic_fails, topic_warns = print_topic_results(topic_results, window)

    node.destroy_node()
    rclpy.shutdown()

    # Summary
    fails = dev_fails + topic_fails
    warns = dev_warns + topic_warns

    print()
    print(f"{BOLD}{CYAN}{'=' * 72}{RESET}")
    if fails == 0 and warns == 0:
        print(f"  {GREEN}{BOLD}RESULT: all checks passed{RESET}")
    elif fails == 0:
        print(f"  {YELLOW}{BOLD}RESULT: passed with {warns} warning(s){RESET}")
    else:
        print(f"  {RED}{BOLD}RESULT: {fails} FAIL, {warns} WARN — system not fully operational{RESET}")
    print(f"{BOLD}{CYAN}{'=' * 72}{RESET}\n")

    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
