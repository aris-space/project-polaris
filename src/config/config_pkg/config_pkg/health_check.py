#!/usr/bin/env python3
"""Polaris System Health Check.

Run after start_system.launch to verify all sensors are operational.
Reports device presence (udev symlinks) and ROS topic health (rates and field values).

Two modes:
  * One-shot (default): runs the checks once, prints the report and exits
    0 if all checks pass, 1 otherwise.
  * Service (--serve): stays up as node `polaris_health_check` and runs the
    checks on every call to /health_check (std_srvs/srv/Trigger).
    `success` is true when no check FAILed (warnings allowed); `message`
    is a one-line summary, e.g. "2 sensors failed (IMU, Keller pressure)".
    The full report is printed to the node's output. Each run is also published on /diagnostics
    as a single `health_check` status (worst level, failing sensors named in
    the message, per-check results as emoji-badged key/values, worst first).

The service mode is started with the system by start_system.launch.py
(via launch/launch_health_check.launch.py), so /health_check can be called
straight from the Foxglove Service Call panel.

Usage:
    ros2 run config_pkg health_check
    ros2 run config_pkg health_check --no-delay
    ros2 run config_pkg health_check --delay 30 --window 10
    ros2 launch config_pkg launch_health_check.launch.py
    ros2 service call /health_check std_srvs/srv/Trigger
"""

import argparse
import os
import stat
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

from sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
    FluidPressure,
    Imu,
    NavSatFix,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from ublox_ubx_msgs.msg import UBXNavHPPosLLH
from custom_msgs.msg import Distance
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# ============================================================
# Configuration
# ============================================================

DEFAULT_STARTUP_DELAY_SEC = 10
DEFAULT_MEASUREMENT_SEC = 5

SERVICE_NAME = "/health_check"

# In --serve mode every run is also published as ONE DiagnosticStatus with
# this name, so it occupies a single row in the Foxglove Diagnostics panels
# and is overwritten by the next run. hardware_id is left empty (like the
# other /diagnostics publishers): Foxglove adds an extra entry per hardware_id.
DIAGNOSTICS_TOPIC = "/diagnostics"
DIAGNOSTIC_NAME = "polaris: health_check"

# h_acc field is uint32 in 0.1 mm units. 10 m == 100_000 raw.
H_ACC_MAX_RAW = 100_000

# Device presence checks: (display name, /dev symlink)
DEVICE_CHECKS: List[Tuple[str, str]] = [
    ("IMU (Xsens)", "/dev/xsens_imu"),
    ("Keller sensor", "/dev/keller"),
    ("Camera front", "/dev/cam_front"),
    ("Camera tube", "/dev/cam_tube"),
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
    ("IMU", "/imu/data", Imu, "min_rate", 50.0),
    ("Pixhawk heartbeat", "/pixhawk/heartbeat", State, "rate_band", (0.5, 2.0)),
    ("BlueRobotics press", "/pixhawk/scaled_pressure", FluidPressure, "min_rate", 40.0),
    (
        "Keller pressure",
        "/sensors/keller26x/abs_pressure",
        FluidPressure,
        "min_rate",
        10.0,
    ),
    ("SBL waterlinked", "/waterlinked_ugps/navsatfix", NavSatFix, "min_rate", 1.0),
    # ("Ultrasonic front", "/front/ultrasonic/distance", Float32, "min_rate", 5.0),
    ("Ultrasonic top", "/top/ultrasonic/distance", Float32, "min_rate", 5.0),
    ("Camera front info", "/front/camera/camera_info", CameraInfo, "received", None),
    (
        "Camera front image",
        "/front/camera/image_raw/compressed",
        CompressedImage,
        "received",
        None,
    ),
    # ("Camera tube info", "/tube/camera/camera_info", CameraInfo, "received", None),
    # (
    #     "Camera tube image",
    #     "/tube/camera/image_raw/compressed",
    #     CompressedImage,
    #     "received",
    #     None,
    # ),
    ("Ice measurement", ICE_MEASUREMENT_TOPIC, Distance, "min_rate", 2.0),
    (
        "ublox DGNSS h_acc",
        "/ubx_nav_hp_pos_llh",
        UBXNavHPPosLLH,
        "h_acc",
        H_ACC_MAX_RAW,
    ),
]


# ============================================================
# Output formatting
# ============================================================
# Check results carry plain status strings ("OK" / "WARN" / "FAIL").
# Colour is applied only when rendering, so the same report can be printed
# to a terminal or returned in the service response without ANSI codes.

GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
CYAN = "\033[36m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"

STATUS_COLOR = {"OK": GREEN, "WARN": YELLOW, "FAIL": RED}

# (status, display_name, path_or_topic, detail)
CheckResult = Tuple[str, str, str, str]


def count_results(results: List[CheckResult]) -> Tuple[int, int]:
    fails = sum(1 for r in results if r[0] == "FAIL")
    warns = sum(1 for r in results if r[0] == "WARN")
    return fails, warns


def banner_text(color: bool) -> str:
    def c(code: str) -> str:
        return code if color else ""

    rule = f"{c(BOLD)}{c(CYAN)}{'=' * 72}{c(RESET)}"
    title = f"{c(BOLD)}{c(CYAN)}  POLARIS SYSTEM HEALTH CHECK{c(RESET)}"
    return "\n".join(["", rule, title, rule])


def format_report(
    device_results: List[CheckResult],
    topic_results: List[CheckResult],
    window_sec: float,
    color: bool,
) -> str:
    def c(code: str) -> str:
        return code if color else ""

    def badge(status: str) -> str:
        return f"{c(STATUS_COLOR[status])} {status:^4} {c(RESET)}"

    lines: List[str] = []

    lines += ["", f"{c(BOLD)}[DEVICES]{c(RESET)}"]
    name_w = max(len(r[1]) for r in device_results) + 2
    for status, name, path, detail in device_results:
        lines.append(f" {badge(status)}  {name:<{name_w}} {c(DIM)}{path}{c(RESET)}  {detail}")

    lines += ["", f"{c(BOLD)}[TOPICS  ({window_sec:g}s window)]{c(RESET)}"]
    name_w = max(len(r[1]) for r in topic_results) + 2
    topic_w = max(len(r[2]) for r in topic_results) + 2
    for status, name, topic, detail in topic_results:
        lines.append(
            f" {badge(status)}  {name:<{name_w}} {c(DIM)}{topic:<{topic_w}}{c(RESET)} {detail}"
        )

    fails, warns = count_results(device_results + topic_results)
    rule = f"{c(BOLD)}{c(CYAN)}{'=' * 72}{c(RESET)}"
    lines += ["", rule]
    if fails == 0 and warns == 0:
        lines.append(f"  {c(GREEN)}{c(BOLD)}RESULT: all checks passed{c(RESET)}")
    elif fails == 0:
        lines.append(
            f"  {c(YELLOW)}{c(BOLD)}RESULT: passed with {warns} warning(s){c(RESET)}"
        )
    else:
        lines.append(
            f"  {c(RED)}{c(BOLD)}RESULT: {fails} FAIL, {warns} WARN"
            f" — system not fully operational{c(RESET)}"
        )
    lines.append(rule)
    return "\n".join(lines)


def summary_text(results: List[CheckResult]) -> str:
    """One-line result for the service response, e.g. "2 sensors failed (IMU, Keller pressure)"."""
    fail_names = [r[1] for r in results if r[0] == "FAIL"]
    warn_names = [r[1] for r in results if r[0] == "WARN"]

    def counted(names: List[str], what: str) -> str:
        noun = "sensor" if len(names) == 1 else "sensors"
        return f"{len(names)} {noun} {what} ({', '.join(names)})"

    parts = []
    if fail_names:
        parts.append(counted(fail_names, "failed"))
    if warn_names:
        parts.append(counted(warn_names, "with warnings"))
    return ", ".join(parts) if parts else "all sensors OK"


# Diagnostics rendering: Foxglove only colours the overall level, so the
# per-check values carry coloured emoji badges and are sorted worst-first.
STATUS_EMOJI = {"OK": "🟢", "WARN": "🟡", "FAIL": "🔴"}
STATUS_RANK = {"FAIL": 0, "WARN": 1, "OK": 2}


def build_diagnostic_fields(
    device_results: List[CheckResult],
    topic_results: List[CheckResult],
    run_time: str,
) -> Tuple[str, str, List[Tuple[str, str]]]:
    """Returns (worst_status, summary_message, [(key, value), ...]).

    The summary names the failing / warning checks so the problem is visible
    in the Summary panel row without opening the Detail panel.
    """
    results = device_results + topic_results
    fail_names = [r[1] for r in results if r[0] == "FAIL"]
    warn_names = [r[1] for r in results if r[0] == "WARN"]

    if fail_names:
        worst = "FAIL"
    elif warn_names:
        worst = "WARN"
    else:
        worst = "OK"

    parts = []
    if fail_names:
        parts.append(f"{STATUS_EMOJI['FAIL']} FAIL: {', '.join(fail_names)}")
    if warn_names:
        parts.append(f"{STATUS_EMOJI['WARN']} WARN: {', '.join(warn_names)}")
    if not parts:
        parts.append(f"{STATUS_EMOJI['OK']} all checks passed")
    message = f"{' | '.join(parts)} (run {run_time})"

    ordered = sorted(results, key=lambda r: STATUS_RANK[r[0]])  # stable sort
    values = [
        (name, f"{STATUS_EMOJI[status]} {status:<4}  {' '.join(detail.split())}")
        for status, name, _, detail in ordered
    ]
    return worst, message, values


# ============================================================
# Phase 1 — device presence
# ============================================================


def check_devices() -> List[CheckResult]:
    """Returns (status, display_name, path, detail) per device."""
    results = []
    for name, path in DEVICE_CHECKS:
        if not os.path.exists(path):
            results.append(("FAIL", name, path, "symlink missing"))
            continue
        try:
            st = os.stat(path)
        except OSError as exc:
            results.append(("FAIL", name, path, f"stat failed: {exc}"))
            continue
        if not (stat.S_ISCHR(st.st_mode) or stat.S_ISBLK(st.st_mode)):
            results.append(("WARN", name, path, "exists but not a device"))
            continue
        results.append(("OK", name, path, "present"))
    return results


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


def evaluate(mon: TopicMonitor, window_sec: float) -> Tuple[str, str]:
    n = len(mon.timestamps)

    if mon.check_kind == "received":
        if n == 0:
            return "FAIL", "no messages received"
        return "OK", f"received ({n} msgs)"

    rate = n / window_sec

    if mon.check_kind == "min_rate":
        thr = mon.threshold
        if n == 0:
            return "FAIL", f"  0.0 Hz   (min {thr:g})  no messages"
        if rate >= thr:
            return "OK", f"{rate:5.1f} Hz   (min {thr:g})"
        return "FAIL", f"{rate:5.1f} Hz   (min {thr:g})"

    if mon.check_kind == "rate_band":
        lo, hi = mon.threshold
        if n == 0:
            return "FAIL", f"  0.0 Hz   ({lo}-{hi})  no messages"
        if lo <= rate <= hi:
            return "OK", f"{rate:5.1f} Hz   ({lo}-{hi})"
        return "WARN", f"{rate:5.1f} Hz   ({lo}-{hi})"

    if mon.check_kind == "h_acc":
        if mon.last_msg is None:
            return "FAIL", "no messages received"
        h_acc_raw = int(mon.last_msg.h_acc)
        h_acc_m = h_acc_raw * 0.1 / 1000.0
        thr_m = mon.threshold * 0.1 / 1000.0
        detail = f"h_acc={h_acc_m:5.2f} m  (max {thr_m:g} m, n={n})"
        if h_acc_raw <= mon.threshold:
            return "OK", detail
        return "WARN", detail

    return "WARN", f"unknown check kind: {mon.check_kind}"


class HealthCheckNode(Node):
    """Holds the topic subscriptions and runs the checks.

    The node is always spun by a MultiThreadedExecutor and every callback
    lives in one reentrant group, so `run_checks()` can block (from the main
    thread or from inside the service callback) while subscription and
    parameter-client callbacks keep being processed on other threads.
    """

    def __init__(self, window_sec: float, serve: bool):
        super().__init__("polaris_health_check")
        self.window_sec = window_sec
        self._cbg = ReentrantCallbackGroup()
        self._sampling = False
        self._run_lock = threading.Lock()

        self.monitors = [
            TopicMonitor(name, topic, mtype, kind, thr)
            for (name, topic, mtype, kind, thr) in TOPIC_CHECKS
        ]
        self._subs = [
            self.create_subscription(
                mon.msg_type,
                mon.topic,
                self._make_callback(mon),
                qos_profile_sensor_data,
                callback_group=self._cbg,
            )
            for mon in self.monitors
        ]
        self._ice_client = self.create_client(
            SetParameters,
            f"{ICE_MEASUREMENT_NODE}/set_parameters",
            callback_group=self._cbg,
        )

        if serve:
            self._diag_pub = self.create_publisher(DiagnosticArray, DIAGNOSTICS_TOPIC, 10)
            self._srv = self.create_service(
                Trigger, SERVICE_NAME, self._on_trigger, callback_group=self._cbg
            )

    def publish_diagnostics(
        self, device_results: List[CheckResult], topic_results: List[CheckResult]
    ) -> None:
        worst, message, values = build_diagnostic_fields(
            device_results, topic_results, time.strftime("%H:%M:%S")
        )
        level = {
            "OK": DiagnosticStatus.OK,
            "WARN": DiagnosticStatus.WARN,
            "FAIL": DiagnosticStatus.ERROR,
        }[worst]
        status = DiagnosticStatus(
            level=level,
            name=DIAGNOSTIC_NAME,
            message=message,
            values=[KeyValue(key=k, value=v) for k, v in values],
        )
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)

    def _make_callback(self, mon: TopicMonitor) -> Callable:
        # Only record while a check is sampling; in --serve mode the
        # subscriptions stay up indefinitely and must not accumulate.
        def cb(msg):
            if self._sampling:
                mon.timestamps.append(time.monotonic())
                mon.last_msg = msg

        return cb

    def set_ice_recording(self, value: bool, timeout_sec: float = 3.0) -> Optional[str]:
        """Toggle the `recording` parameter on the ice measurement publisher.

        Returns None on success, or an error string on failure.
        """
        if not self._ice_client.wait_for_service(timeout_sec=timeout_sec):
            return f"service {self._ice_client.srv_name} not available"
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
        future = self._ice_client.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return "service call timed out"
        result = future.result()
        if result is None or not result.results:
            return "no result returned"
        for r in result.results:
            if not r.successful:
                return r.reason or "rejected by node"
        return None

    def run_checks(self) -> Tuple[List[CheckResult], List[CheckResult], bool]:
        """Run device + topic checks once.

        Returns (device_results, topic_results, interrupted).
        """
        device_results = check_devices()

        for mon in self.monitors:
            mon.timestamps = []
            mon.last_msg = None

        ice_set_err = self.set_ice_recording(True)
        interrupted = False
        try:
            self._sampling = True
            time.sleep(self.window_sec)
        except KeyboardInterrupt:
            interrupted = True
        finally:
            self._sampling = False
            if ice_set_err is None:
                self.set_ice_recording(False)

        topic_results: List[CheckResult] = []
        for mon in self.monitors:
            if mon.topic == ICE_MEASUREMENT_TOPIC and ice_set_err is not None:
                status = "FAIL"
                detail = f"could not enable recording: {ice_set_err}"
            else:
                status, detail = evaluate(mon, self.window_sec)
            topic_results.append((status, mon.display_name, mon.topic, detail))

        return device_results, topic_results, interrupted

    def _on_trigger(self, request, response):
        if not self._run_lock.acquire(blocking=False):
            response.success = False
            response.message = "health check already running"
            return response
        try:
            self.get_logger().info(f"Running health check ({self.window_sec:g}s window) ...")
            device_results, topic_results, _ = self.run_checks()
            fails, warns = count_results(device_results + topic_results)

            print(banner_text(sys.stdout.isatty()))
            print(format_report(device_results, topic_results, self.window_sec,
                                color=sys.stdout.isatty()), flush=True)

            self.publish_diagnostics(device_results, topic_results)

            response.success = fails == 0
            response.message = summary_text(device_results + topic_results)
            self.get_logger().info(f"Health check done: {fails} FAIL, {warns} WARN")
        except Exception as exc:  # report, don't kill the server
            self.get_logger().error(f"Health check error: {exc}")
            response.success = False
            response.message = f"health check error: {exc}"
        finally:
            self._run_lock.release()
        return response


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


def run_once(node: HealthCheckNode, delay: int) -> int:
    color = sys.stdout.isatty()
    print(banner_text(color))
    if delay:
        print()
        countdown(delay, "Waiting for nodes to stabilize")

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        dim = DIM if color else ""
        reset = RESET if color else ""
        print(f"\n  {dim}Sampling topics for {node.window_sec:g}s ...{reset}")
        device_results, topic_results, interrupted = node.run_checks()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)

    if interrupted:
        yellow = YELLOW if color else ""
        reset = RESET if color else ""
        print(f"\n  {yellow}Interrupted by user{reset}")

    print(format_report(device_results, topic_results, node.window_sec, color))
    print()
    fails, _ = count_results(device_results + topic_results)
    return 0 if fails == 0 else 1


def serve(node: HealthCheckNode) -> int:
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    node.get_logger().info(
        f"Ready: call {SERVICE_NAME} (std_srvs/srv/Trigger), "
        f"{node.window_sec:g}s sampling window"
    )
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
    return 0


def main(argv=None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    full_argv = ["health_check"] + list(argv)

    parser = argparse.ArgumentParser(
        description="Polaris system health check (devices + ROS topics).",
    )
    parser.add_argument(
        "--delay",
        type=int,
        default=DEFAULT_STARTUP_DELAY_SEC,
        help="seconds to wait for nodes to stabilize, one-shot mode only "
        "(default: %(default)s)",
    )
    parser.add_argument(
        "--no-delay", action="store_true", help="skip the startup delay"
    )
    parser.add_argument(
        "--window",
        type=float,
        default=DEFAULT_MEASUREMENT_SEC,
        help="topic measurement window in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--serve",
        action="store_true",
        help=f"stay up and run the checks on each {SERVICE_NAME} "
        "(std_srvs/srv/Trigger) call",
    )
    args = parser.parse_args(remove_ros_args(full_argv)[1:])

    delay = 0 if args.no_delay else max(0, args.delay)
    window = max(0.5, args.window)

    rclpy.init(args=full_argv)
    node = HealthCheckNode(window, serve=args.serve)
    try:
        if args.serve:
            return serve(node)
        return run_once(node, delay)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
