#!/usr/bin/env python3
"""Polaris System Health Check.

Run after start_system.launch to verify all sensors are operational.
Reports device presence (udev symlinks) and ROS topic health (rates and field values).
After the sensor checks, prompts to optionally cycle each thruster with a low PWM
(verifiable on /pixhawk/servo_output_raw). Exits 0 if all checks pass, 1 otherwise.

Usage:
    python3 scripts/health_check.py
    python3 scripts/health_check.py --no-delay
    python3 scripts/health_check.py --delay 30 --window 10
    python3 scripts/health_check.py --skip-thruster-test
    python3 scripts/health_check.py --thruster-pwm 1600 --thruster-duration 3
"""

import argparse
import os
import stat
import sys
import termios
import time
import tty
from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus as DiagStatus
from nav_msgs.msg import Odometry
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

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil
from config_pkg.constants import Comms


# ============================================================
# Configuration
# ============================================================

DEFAULT_STARTUP_DELAY_SEC = 10
DEFAULT_MEASUREMENT_SEC = 5

# Thruster test defaults (MAV_CMD_DO_MOTOR_TEST, throttle_type=PWM)
DEFAULT_NUM_THRUSTERS = 6
DEFAULT_THRUSTER_PWM = 1600
DEFAULT_THRUSTER_DURATION_SEC = 3.0
DEFAULT_THRUSTER_BREAK_SEC = 1.0

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
#   kind = "diag_ok"    -> threshold is DiagnosticStatus name prefix string;
#                          filters a DiagnosticArray by that prefix and maps
#                          the worst status level to OK / WARN / FAIL
TOPIC_CHECKS: List[Tuple[str, str, Any, str, Any]] = [
    ("IMU",                "/imu/data",                       Imu,             "min_rate",  50.0),
    ("Pixhawk heartbeat",  "/pixhawk/heartbeat",              State,           "rate_band", (0.5, 2.0)),
    ("BlueRobotics press", "/pixhawk/scaled_pressure",        FluidPressure,   "min_rate",  30.0),
    ("Keller pressure",    "/sensors/keller26x/abs_pressure", FluidPressure,   "min_rate",  10.0),
    ("SBL waterlinked",    "/waterlinked_ugps/navsatfix",     NavSatFix,       "min_rate",   1.0),
    ("Ultrasonic front",   "/front/ultrasonic/distance",      Float32,         "min_rate",   5.0),
    ("Ultrasonic top",     "/top/ultrasonic/distance",        Float32,         "min_rate",   5.0),
    ("Camera front info",  "/front/camera/camera_info",              CameraInfo,      "received",  None),
    ("Camera front image", "/front/camera/image_raw/compressed",     CompressedImage, "received",  None),
    ("Camera tube info",   "/tube/camera/camera_info",               CameraInfo,      "received",  None),
    ("Camera tube image",  "/tube/camera/image_raw/compressed",      CompressedImage, "received",  None),
    ("BMS",                "/diagnostics",                    DiagnosticArray, "diag_ok",   "Battery:"),
    ("DVL A50",            "/sensors/dvl/odometry",           Odometry,        "min_rate",   8.0),
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
    last_diag_statuses: List[Any] = field(default_factory=list)


class HealthCheckNode(Node):
    def __init__(self, monitors: List[TopicMonitor]):
        super().__init__("polaris_health_check")
        self.monitors = monitors
        self._subs = []
        for mon in monitors:
            if mon.check_kind == "diag_ok":
                cb = self._make_diag_callback(mon, mon.threshold)
            else:
                cb = self._make_callback(mon)
            sub = self.create_subscription(
                mon.msg_type,
                mon.topic,
                cb,
                qos_profile_sensor_data,
            )
            self._subs.append(sub)

    @staticmethod
    def _make_callback(mon: TopicMonitor) -> Callable:
        def cb(msg):
            mon.timestamps.append(time.monotonic())
            mon.last_msg = msg
        return cb

    @staticmethod
    def _make_diag_callback(mon: TopicMonitor, name_prefix: str) -> Callable:
        def cb(msg):
            mon.timestamps.append(time.monotonic())
            mon.last_msg = msg
            matching = [s for s in msg.status if s.name.startswith(name_prefix)]
            if matching:
                mon.last_diag_statuses = matching
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

    if mon.check_kind == "diag_ok":
        prefix = mon.threshold
        if not mon.last_diag_statuses:
            n = len(mon.timestamps)
            if n == 0:
                return FAIL, f"no messages on {mon.topic}"
            return FAIL, f"no '{prefix}' statuses in {n} received messages"
        worst = max(s.level for s in mon.last_diag_statuses)
        if worst == DiagStatus.OK:
            names = ", ".join(s.name for s in mon.last_diag_statuses)
            return OK, f"all OK  ({names})"
        elif worst == DiagStatus.WARN:
            msgs = "; ".join(
                f"{s.name}: {s.message}"
                for s in mon.last_diag_statuses
                if s.level >= DiagStatus.WARN
            )
            return WARN, msgs
        else:
            msgs = "; ".join(
                f"{s.name}: {s.message}"
                for s in mon.last_diag_statuses
                if s.level >= DiagStatus.ERROR
            )
            return FAIL, msgs

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
# Phase 3 — interactive thruster test
# ============================================================

def prompt_thruster_test() -> bool:
    """Block until the user presses ENTER (run) or ESC (skip).

    Falls back to skipping the test if stdin is not a TTY.
    """
    if not sys.stdin.isatty():
        print(f"\n  {DIM}Skipping thruster test (non-interactive stdin).{RESET}")
        return False

    print()
    print(f"{BOLD}{CYAN}Press ENTER to run thruster test, ESC to exit.{RESET}")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch in ("\r", "\n"):
                return True
            if ch == "\x1b":
                return False
            # ignore other keys; keep waiting
    finally:
        termios.tcsetattr(fd, old, termios.TCSADRAIN)


def run_thruster_test(
    num_thrusters: int,
    pwm: int,
    duration: float,
    break_sec: float,
) -> Tuple[int, int]:
    """Drive each thruster individually via MAV_CMD_DO_MOTOR_TEST.

    Returns (fails, warns) for inclusion in the overall summary.
    """
    section("THRUSTER TEST")
    fails = warns = 0

    print(f"  {DIM}Connecting to mavlink-router at {Comms.MAVLINK_ROUTER_TCP} ...{RESET}")
    try:
        port = mavutil.mavlink_connection(Comms.MAVLINK_ROUTER_TCP)
        if port.wait_heartbeat(timeout=5) is None:
            print(f" {FAIL}  no heartbeat from Pixhawk within 5 s")
            return 1, 0
    except Exception as exc:
        print(f" {FAIL}  could not connect: {exc}")
        return 1, 0

    print(f"  {DIM}Heartbeat received from system {port.target_system}.{RESET}")
    print(f"  {DIM}PWM={pwm}us, {duration:g}s on, {break_sec:g}s break — "
          f"watch /pixhawk/servo_output_raw to verify.{RESET}\n")

    name_w = len(f"Thruster {num_thrusters}") + 2
    for motor in range(1, num_thrusters + 1):
        label = f"Thruster {motor}"
        print(f" {CYAN}->{RESET} {label:<{name_w}} PWM={pwm}us for {duration:g}s ... ",
              end="", flush=True)
        port.mav.command_long_send(
            port.target_system,
            port.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,                  # confirmation
            float(motor),       # param1: motor instance (1-based)
            1.0,                # param2: throttle_type (1 = PWM in microseconds)
            float(pwm),         # param3: throttle value
            float(duration),    # param4: timeout (seconds)
            0.0,                # param5: motor count (0 → just this one)
            0.0,                # param6: test order (default)
            0.0,                # param7: unused
        )

        ack = port.recv_match(type="COMMAND_ACK", blocking=True, timeout=2.0)
        if ack is None:
            print(f"{WARN}  no COMMAND_ACK")
            warns += 1
        elif ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"{FAIL}  rejected (MAV_RESULT={ack.result})")
            fails += 1
        else:
            print(f"{OK}  accepted")

        # Wait for the motor test to finish, then pause between thrusters.
        time.sleep(duration + break_sec)

    print(f"\n  {DIM}Thruster test complete.{RESET}")
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
    parser.add_argument("--skip-thruster-test", action="store_true",
                        help="skip the interactive thruster test prompt")
    parser.add_argument("--num-thrusters", type=int, default=DEFAULT_NUM_THRUSTERS,
                        help="number of thrusters to cycle through (default: %(default)s)")
    parser.add_argument("--thruster-pwm", type=int, default=DEFAULT_THRUSTER_PWM,
                        help="PWM (us) sent to each thruster (default: %(default)s)")
    parser.add_argument("--thruster-duration", type=float, default=DEFAULT_THRUSTER_DURATION_SEC,
                        help="seconds each thruster runs (default: %(default)s)")
    parser.add_argument("--thruster-break", type=float, default=DEFAULT_THRUSTER_BREAK_SEC,
                        help="seconds of pause between thrusters (default: %(default)s)")
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

    # Phase 3 — optional thruster test
    thr_fails = thr_warns = 0
    if not args.skip_thruster_test and prompt_thruster_test():
        thr_fails, thr_warns = run_thruster_test(
            num_thrusters=max(1, args.num_thrusters),
            pwm=args.thruster_pwm,
            duration=max(0.1, args.thruster_duration),
            break_sec=max(0.0, args.thruster_break),
        )

    # Summary
    fails = dev_fails + topic_fails + thr_fails
    warns = dev_warns + topic_warns + thr_warns

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
