#!/usr/bin/env python3
"""
Synthetic ground-truth bags for the sysid pipeline.

Simulates the surge model of dynamics.py with KNOWN coefficients, delays and noise, drives it
with the same manoeuvre programs as command_scripts/ (including the /sysid/event markers), and
writes an MCAP bag with the same topics and message types as a real POLARIS recording. Fit
scripts can then be checked against the truth before any lake data exists: a fit that does not
recover the coefficients of a synthetic bag will not recover them from a real one.

Written alongside every bag is <stem>_truth.yaml with every true value used. The same content
is embedded in the bag as JSON on /sysid/synthetic_truth (one message at the start).

What is simulated (surge only, constant heading, no current unless --offset-n):
  * plant      (m - X_udot) du/dt = k*T(pwm, V) + F_offset - X_u u - X_uu |u| u, coefficients
               and thrust map taken from the --model file (the "truth" model);
  * actuation  the thrust follows the LOGGED servo PWM after a pure delay (--actuation-lag-s)
               and a first-order motor spin-up (--spinup-tau-s);
  * battery    linear sag between --v-start and --v-end, reported by the BMS like the real one;
  * EKF        surge velocity through a first-order lag (--ekf-lag-s) plus white noise; the
               pose is the integral of that velocity, so pose and twist stay consistent;
  * IMU        specific force in FLU: f_x = du/dt - g sin(theta_0), f_z = g cos(theta_0);
  * DVL        body velocity rotated into dvl_a50_link with the real mount from /tf_static.

Sign convention of the delay. --actuation-lag-s is physical: thrust acts this long AFTER the PWM
is logged. propagate_model.py and coupling_check.py SUBTRACT thrust.servo_delay_s from the servo
log time, so the value that compensates a lag L is servo_delay_s = -L. The truth file records
both numbers.

Command-to-PWM mapping (pwm = 1500 + --pwm-per-cmd * surge) is an ASSUMPTION (default 0.4 us per
unit, i.e. command 500 -> 1700 us); the real ArduSub gain must be read off a lake bag.

Topics written (types identical to the real bags, ROS 2 Jazzy):
  /odometry/filtered/local   nav_msgs/Odometry           30 Hz, odom -> base_link
  /pixhawk/servo_output_raw  std_msgs/Int16MultiArray    10 Hz, no header
  /pixhawk/manual_control    std_msgs/Int16MultiArray    20 Hz while a script runs
  /imu/data                  sensor_msgs/Imu             100 Hz, imu_link
  /sensors/dvl/velocity      marine_acoustic_msgs/Dvl    7.6 Hz, dvl_a50_link
  /diagnostics               diagnostic_msgs/DiagnosticArray  1 Hz, BMS statuses
  /sysid/event               std_msgs/String             JSON markers as _step_common.py
  /sysid/synthetic_truth     std_msgs/String             truth as JSON, once
  /tf_static                 tf2_msgs/TFMessage          base_link -> imu_link, dvl_a50_link

Dependencies: pip install rosbags numpy scipy pyyaml

Examples:
    # default imperfections, the full suite (four steps and a coast-down)
    python scripts/sysid/simulation/synth_bag.py --program suite

    # everything ideal (no delay, no noise): the propagation must reproduce it exactly
    python scripts/sysid/simulation/synth_bag.py --program steps --ideal

    # ideal except a 0.2 s actuation lag
    python scripts/sysid/simulation/synth_bag.py --program steps --ideal --actuation-lag-s 0.2
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from dataclasses import asdict, dataclass, fields, replace
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot

SYSID = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SYSID))

from actuation import N_SERVO_CHANNELS, ThrustSource  # noqa: E402
from dynamics import SurgeParams, surge_accel  # noqa: E402
from model_io import load_model  # noqa: E402

G = 9.80665
GENERATOR_VERSION = "1"
DVL_MSG = Path(__file__).with_name("Dvl.msg")
DVL_MSGTYPE = "marine_acoustic_msgs/msg/Dvl"

# Static transforms as recorded on the vehicle (/tf_static, June 2026 bags), FLU.
# dvl_a50_link: 180 deg about an axis in the xy plane at -22.5 deg (looks down, x at 45 deg).
_C, _S = math.cos(math.radians(22.5)), math.sin(math.radians(22.5))
TF_STATIC = {
    "dvl_a50_link": ((0.722, 0.0, -0.075), (_C, -_S, 0.0, 0.0)),
    "imu_link": ((0.049, -0.009, 0.076), (0.0, 0.0, 0.0, 1.0)),
}

SIM_RATE_HZ = 1000.0  # truth integration grid; every sensor time lies on it
RATES_HZ = {"ekf": 30.0, "servo": 10.0, "imu": 100.0, "dvl": 7.6, "bms": 1.0}
PUBLISH_HZ = 20.0  # manual_control rate of _step_common.py
NEUTRAL_BURST = 9  # neutral messages after the cut in _step_common.py (plus the first one)
DONE_AFTER_CUT_S = NEUTRAL_BURST / PUBLISH_HZ + 0.2  # when _step_common.py publishes 'done'

REQUIRED = ["rigid_body.m", "added_mass.X_udot", "damping_linear.X_u", "damping_quadratic.X_uu"]


# ============================================================
# Configuration and manoeuvre programs
# ============================================================


@dataclass(frozen=True)
class SynthConfig:
    """Every imperfection of the synthetic vehicle and its sensors. Defaults are plausible
    guesses, not measurements; ideal() switches them all off."""

    actuation_lag_s: float = 0.15  # thrust acts this long after the PWM is logged
    spinup_tau_s: float = 0.05  # first-order motor spin-up time constant
    thrust_scale: float = 1.0  # true thrust = scale * table (models a datasheet error)
    offset_n: float = 0.0  # constant extra surge force (tether, current)
    v_start: float = 16.2  # battery voltage at the start [V]
    v_end: float = 15.6  # battery voltage at the end [V]
    ekf_lag_s: float = 0.10  # first-order lag of the EKF surge velocity
    ekf_noise_mps: float = 0.01  # white noise on the EKF twist
    imu_noise_mps2: float = 0.02  # white noise on the IMU specific force
    dvl_noise_mps: float = 0.005  # white noise on the DVL velocity
    dvl_lag_s: float = 0.0  # pure delay of the DVL velocity (ping averaging)
    pwm_per_cmd: float = 0.4  # ASSUMED ArduSub mapping: pwm = 1500 + this * surge command
    heading_rad: float = 0.5  # constant yaw in odom (ENU, counter-clockwise from x)
    trim_pitch_rad: float = 0.02  # constant pitch (FLU: positive nose down)
    depth_m: float = 0.0  # constant z in odom
    seed: int = 0
    t0: float = 1_790_000_000.0  # start time [s since epoch]

    @classmethod
    def ideal(cls) -> SynthConfig:
        return cls(actuation_lag_s=0.0, spinup_tau_s=0.0, ekf_lag_s=0.0, ekf_noise_mps=0.0,
                   imu_noise_mps2=0.0, dvl_noise_mps=0.0, dvl_lag_s=0.0, v_end=16.2)


@dataclass(frozen=True)
class Segment:
    """One run of a command script: constant surge for hold_s, then neutral for rest_s."""

    maneuver: str  # 'step' or 'coastdown', as the scripts label it
    node: str  # node name the script would use
    surge: int  # manual_control surge command
    hold_s: float
    rest_s: float


def build_program(name: str, levels: list[int] | None = None, hold_s: float | None = None,
                  rest_s: float | None = None) -> list[Segment]:
    """Manoeuvre sequences matching command_scripts/: 'steps' (step_surge_200..500, 10 s each),
    'coastdown' (coastdown_surge_400, 20 s hold, 20 s coast) and 'suite' (both)."""
    steps = [Segment("step", f"sysid_step_surge_{lv}", lv, hold_s or 10.0, rest_s or 15.0)
             for lv in (levels or [200, 300, 400, 500])]
    coast = [Segment("coastdown", f"sysid_coastdown_surge_{lv}", lv, hold_s or 20.0,
                     rest_s or 20.0) for lv in (levels or [400])]
    programs = {"steps": steps, "coastdown": coast, "suite": steps + coast}
    if name not in programs:
        raise ValueError(f"unknown program '{name}', choose from {sorted(programs)}")
    return programs[name]


# ============================================================
# Simulation
# ============================================================


@dataclass
class Truth:
    """Continuous-time truth on the SIM_RATE_HZ grid plus the logged/derived sample streams."""

    t: np.ndarray  # absolute time [s]
    u: np.ndarray  # true surge speed
    du: np.ndarray  # true surge acceleration
    thrust_n: np.ndarray  # force actually acting (after lag, spin-up, scale)
    u_ekf: np.ndarray  # EKF surge velocity before noise (lagged)
    x_ekf: np.ndarray  # EKF position along the heading (integral of u_ekf)
    volts: np.ndarray  # true battery voltage
    pwm_cmd: np.ndarray  # PWM the Pixhawk outputs (before logging)
    servo_idx: np.ndarray  # indices into t of the servo samples
    events: list[tuple[float, dict]]  # (time, payload)
    manual: list[tuple[float, list[int]]]  # (time, data) of manual_control messages


def _sample_idx(n: int, rate_hz: float) -> np.ndarray:
    """Indices on the simulation grid closest to a regular sampling at rate_hz."""
    k = np.arange(0, int(n * rate_hz / SIM_RATE_HZ))
    return np.unique(np.round(k * SIM_RATE_HZ / rate_hz).astype(int))


def _first_order(x: np.ndarray, tau: float, dt: float) -> np.ndarray:
    """Exact discrete first-order lag of a piecewise-constant signal (tau = 0: identity)."""
    if tau <= 0.0:
        return x.copy()
    a = 1.0 - math.exp(-dt / tau)
    y = np.empty_like(x)
    y[0] = x[0]
    for k in range(1, x.size):
        y[k] = y[k - 1] + a * (x[k - 1] - y[k - 1])
    return y


def simulate(model_path: Path, program: list[Segment], cfg: SynthConfig,
             lead_s: float = 5.0, tail_s: float = 5.0) -> tuple[Truth, dict]:
    """Integrate the truth model over the programme. Returns the truth and the parameters used."""
    model = load_model(model_path)
    missing = model.missing_for(REQUIRED)
    if missing:
        raise ValueError(f"truth model {model_path.name} has null parameters: {missing}")
    params = SurgeParams(m_tot=model.get("rigid_body.m") - model.get("added_mass.X_udot"),
                         X_u=model.get("damping_linear.X_u"),
                         X_uu=model.get("damping_quadratic.X_uu"))
    channel = int(model.thrust.get("channel", 0))
    neutral = float(model.thrust.get("pwm_neutral", 1500))
    sign = int(model.thrust.get("sign", 1))
    source = ThrustSource(model, channel, neutral, sign)
    if not source.has(channel):
        raise ValueError(f"truth model {model_path.name} has no thrust curve for ch{channel}")

    # Timeline of the scripts: commands, neutral bursts and event markers.
    events: list[tuple[float, dict]] = []
    manual: list[tuple[float, list[int]]] = []
    holds: list[tuple[float, float, int]] = []
    cursor = lead_s
    for seg in program:
        on, cut = cursor, cursor + seg.hold_s
        base = {"maneuver": seg.maneuver, "node": seg.node, "surge": seg.surge,
                "hold_s": seg.hold_s}
        events.append((on, {**base, "event": "thrust_on"}))
        events.append((cut, {**base, "event": "thrust_cut", "held_s": seg.hold_s,
                             "interrupted": False}))
        events.append((cut + DONE_AFTER_CUT_S, {**base, "event": "done"}))
        n_pub = int(seg.hold_s * PUBLISH_HZ)
        manual += [(on + i / PUBLISH_HZ, [seg.surge, 0, 500, 0, 0, 0]) for i in range(n_pub)]
        manual += [(cut + i / PUBLISH_HZ, [0, 0, 500, 0, 0, 0]) for i in range(NEUTRAL_BURST + 1)]
        holds.append((on, cut, seg.surge))
        cursor = cut + seg.rest_s
    duration = cursor + tail_s

    dt = 1.0 / SIM_RATE_HZ
    n = int(round(duration * SIM_RATE_HZ)) + 1
    t_rel = np.arange(n) * dt

    pwm_cmd = np.full(n, neutral)
    for on, cut, surge in holds:
        pwm_cmd[(t_rel >= on) & (t_rel < cut)] = neutral + round(cfg.pwm_per_cmd * surge)
    volts = cfg.v_start + (cfg.v_end - cfg.v_start) * t_rel / t_rel[-1]

    # The servo topic logs the Pixhawk output at 10 Hz; the thrust follows the LOGGED value
    # (zero-order hold) after the actuation lag, which is what servo_delay_s is defined against.
    servo_idx = _sample_idx(n, RATES_HZ["servo"])
    logged_pwm = pwm_cmd[servo_idx]
    held = np.searchsorted(t_rel[servo_idx], t_rel - cfg.actuation_lag_s, side="right") - 1
    pwm_applied = np.where(held >= 0, logged_pwm[np.clip(held, 0, None)], neutral)
    # Thrust is evaluated only where the applied PWM or the rounded voltage changes.
    thrust_target = np.empty(n)
    cache: dict[tuple[float, float], float] = {}
    for k in range(n):
        key = (float(pwm_applied[k]), round(float(volts[k]), 3))
        if key not in cache:
            cache[key] = source.force(channel, key[0], key[1])
        thrust_target[k] = cfg.thrust_scale * cache[key]
    thrust = _first_order(thrust_target, cfg.spinup_tau_s, dt)

    # Truth: RK4 per grid step with the acting force held over the step.
    u = np.zeros(n)
    du = np.zeros(n)
    for k in range(n - 1):
        f = thrust[k] + cfg.offset_n
        k1 = surge_accel(u[k], f, params)
        k2 = surge_accel(u[k] + 0.5 * dt * k1, f, params)
        k3 = surge_accel(u[k] + 0.5 * dt * k2, f, params)
        k4 = surge_accel(u[k] + dt * k3, f, params)
        du[k] = k1
        u[k + 1] = u[k] + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    du[-1] = surge_accel(u[-1], thrust[-1] + cfg.offset_n, params)

    u_ekf = _first_order(u, cfg.ekf_lag_s, dt)
    x_ekf = np.concatenate([[0.0], np.cumsum(0.5 * (u_ekf[1:] + u_ekf[:-1]) * dt)])

    truth = Truth(t=cfg.t0 + t_rel, u=u, du=du, thrust_n=thrust + cfg.offset_n, u_ekf=u_ekf,
                  x_ekf=x_ekf, volts=volts, pwm_cmd=pwm_cmd, servo_idx=servo_idx,
                  events=[(cfg.t0 + te, p) for te, p in events],
                  manual=[(cfg.t0 + tm, d) for tm, d in manual])
    info = {
        "model_file": str(model_path),
        "model_version": model.version_tag(),
        "thrust": source.describe(),
        "channel": channel,
        "pwm_neutral": neutral,
        "sign": sign,
        "m": model.get("rigid_body.m"),
        "X_udot": model.get("added_mass.X_udot"),
        "m_tot": params.m_tot,
        "X_u": params.X_u,
        "X_uu": params.X_uu,
        "duration_s": float(t_rel[-1]),
    }
    return truth, info


# ============================================================
# Writing
# ============================================================


def _typestore():
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS2_JAZZY)
    ts.register(get_types_from_msg(DVL_MSG.read_text(encoding="utf-8"), DVL_MSGTYPE))
    return ts


def _stamp(ts, t: float):
    ns = int(round(t * 1e9))
    return ts.types["builtin_interfaces/msg/Time"](sec=ns // 10**9, nanosec=ns % 10**9)


def _header(ts, t: float, frame: str):
    return ts.types["std_msgs/msg/Header"](stamp=_stamp(ts, t), frame_id=frame)


def _vec(ts, x: float, y: float, z: float):
    return ts.types["geometry_msgs/msg/Vector3"](x=float(x), y=float(y), z=float(z))


def _quat(ts, q):
    return ts.types["geometry_msgs/msg/Quaternion"](x=float(q[0]), y=float(q[1]),
                                                    z=float(q[2]), w=float(q[3]))


def write_bag(truth: Truth, info: dict, cfg: SynthConfig, program: list[Segment],
              out_path: Path) -> tuple[Path, Path]:
    """Write the synthetic bag (bare .mcap) and its _truth.yaml. Returns both paths."""
    from rosbags.rosbag2 import Writer
    from rosbags.rosbag2.enums import StoragePlugin

    if out_path.suffix != ".mcap":
        out_path = out_path.with_suffix(".mcap")
    if out_path.exists():
        raise FileExistsError(f"{out_path} already exists (delete it to regenerate)")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_dir = out_path.parent / f"{out_path.stem}.tmp_rosbag2"
    if tmp_dir.exists():
        shutil.rmtree(tmp_dir)

    ts = _typestore()
    T = ts.types
    rng = np.random.default_rng(cfg.seed)
    n = truth.t.size
    neutral = info["pwm_neutral"]
    psi, theta = cfg.heading_rad, cfg.trim_pitch_rad
    q_att = Rot.from_euler("xyz", [0.0, theta, psi]).as_quat()  # same convention as bag_io
    zeros36 = np.zeros(36)
    msgs: list[tuple[int, str, object, str]] = []  # (log_ns, topic, msg, type)

    def put(t_log: float, topic: str, msg, typename: str) -> None:
        msgs.append((int(round(t_log * 1e9)), topic, msg, typename))

    truth_doc = make_truth_doc(info, cfg, program, truth)
    truth_text = ("# SYNTHETIC BAG - ground truth written by scripts/sysid/simulation/synth_bag.py.\n"
                  "# Not a recording. Values here are what a fit of the bag should recover.\n"
                  + yaml.safe_dump(truth_doc, sort_keys=False))
    put(truth.t[0], "/sysid/synthetic_truth", T["std_msgs/msg/String"](data=json.dumps(truth_doc)),
        "std_msgs/msg/String")

    tf = T["tf2_msgs/msg/TFMessage"](transforms=[
        T["geometry_msgs/msg/TransformStamped"](
            header=_header(ts, truth.t[0], "base_link"), child_frame_id=child,
            transform=T["geometry_msgs/msg/Transform"](translation=_vec(ts, *xyz),
                                                         rotation=_quat(ts, q)))
        for child, (xyz, q) in TF_STATIC.items()])
    put(truth.t[0], "/tf_static", tf, "tf2_msgs/msg/TFMessage")

    # EKF odometry: twist in base_link, pose in odom.
    c, s = math.cos(psi), math.sin(psi)
    for k in _sample_idx(n, RATES_HZ["ekf"]):
        t = truth.t[k]
        noise = rng.normal(0.0, cfg.ekf_noise_mps, 3) if cfg.ekf_noise_mps > 0 else np.zeros(3)
        msg = T["nav_msgs/msg/Odometry"](
            header=_header(ts, t, "odom"), child_frame_id="base_link",
            pose=T["geometry_msgs/msg/PoseWithCovariance"](
                pose=T["geometry_msgs/msg/Pose"](
                    position=T["geometry_msgs/msg/Point"](x=float(c * truth.x_ekf[k]),
                                                          y=float(s * truth.x_ekf[k]),
                                                          z=float(cfg.depth_m)),
                    orientation=_quat(ts, q_att)),
                covariance=zeros36),
            twist=T["geometry_msgs/msg/TwistWithCovariance"](
                twist=T["geometry_msgs/msg/Twist"](
                    linear=_vec(ts, truth.u_ekf[k] + noise[0], noise[1], noise[2]),
                    angular=_vec(ts, 0.0, 0.0, 0.0)),
                covariance=zeros36))
        put(t + 0.004, "/odometry/filtered/local", msg, "nav_msgs/msg/Odometry")

    # Servo output: headerless, log time only. Non-surge channels at neutral.
    for k in truth.servo_idx:
        pwm = np.full(N_SERVO_CHANNELS, int(neutral), dtype=np.int16)
        pwm[info["channel"]] = int(truth.pwm_cmd[k])
        put(truth.t[k], "/pixhawk/servo_output_raw",
            T["std_msgs/msg/Int16MultiArray"](
                layout=T["std_msgs/msg/MultiArrayLayout"](dim=[], data_offset=0), data=pwm),
            "std_msgs/msg/Int16MultiArray")

    for t, data in truth.manual:
        put(t, "/pixhawk/manual_control",
            T["std_msgs/msg/Int16MultiArray"](
                layout=T["std_msgs/msg/MultiArrayLayout"](dim=[], data_offset=0),
                data=np.asarray(data, dtype=np.int16)),
            "std_msgs/msg/Int16MultiArray")

    for t, payload in truth.events:
        put(t, "/sysid/event", T["std_msgs/msg/String"](data=json.dumps(payload)),
            "std_msgs/msg/String")

    # IMU: specific force f = a - g_body in FLU, with g_body = R^T [0, 0, -g].
    g_body = Rot.from_euler("xyz", [0.0, theta, psi]).inv().apply([0.0, 0.0, -G])
    cov3 = np.diag([cfg.imu_noise_mps2**2] * 3).ravel()
    for k in _sample_idx(n, RATES_HZ["imu"]):
        t = truth.t[k]
        f = np.array([truth.du[k], 0.0, 0.0]) - g_body
        if cfg.imu_noise_mps2 > 0:
            f = f + rng.normal(0.0, cfg.imu_noise_mps2, 3)
        msg = T["sensor_msgs/msg/Imu"](
            header=_header(ts, t, "imu_link"), orientation=_quat(ts, q_att),
            orientation_covariance=np.zeros(9), angular_velocity=_vec(ts, 0.0, 0.0, 0.0),
            angular_velocity_covariance=np.zeros(9), linear_acceleration=_vec(ts, *f),
            linear_acceleration_covariance=cov3)
        put(t + 0.002, "/imu/data", msg, "sensor_msgs/msg/Imu")

    # DVL: body velocity rotated into dvl_a50_link (v_dvl = R^T v_body), optional pure delay.
    r_dvl = Rot.from_quat(TF_STATIC["dvl_a50_link"][1])
    lag_steps = int(round(cfg.dvl_lag_s * SIM_RATE_HZ))
    for k in _sample_idx(n, RATES_HZ["dvl"]):
        t = truth.t[k]
        v_body = np.array([truth.u[max(0, k - lag_steps)], 0.0, 0.0])
        if cfg.dvl_noise_mps > 0:
            v_body = v_body + rng.normal(0.0, cfg.dvl_noise_mps, 3)
        v = r_dvl.inv().apply(v_body)
        msg = T[DVL_MSGTYPE](
            header=_header(ts, t, "dvl_a50_link"), velocity_mode=1, dvl_type=0,
            velocity=_vec(ts, *v), velocity_covar=np.diag([cfg.dvl_noise_mps**2] * 3).ravel(),
            altitude=2.0, course_gnd=float(math.atan2(v[1], v[0])),
            speed_gnd=float(math.hypot(v[0], v[1])), num_good_beams=4, sound_speed=1500.0,
            beam_ranges_valid=True, beam_velocities_valid=False,
            beam_unit_vec=[_vec(ts, 0.0, 0.0, 0.0) for _ in range(4)],
            range=np.full(4, 2.3), range_covar=np.full(4, -1.0, dtype=np.float32),
            beam_quality=np.zeros(4, dtype=np.float32),
            beam_velocity=np.zeros(4, dtype=np.float32),
            beam_velocity_covar=np.full(4, -1.0, dtype=np.float32))
        put(t + 0.005, "/sensors/dvl/velocity", msg, DVL_MSGTYPE)

    # BMS: same three statuses as bms_node, voltage as text with two decimals.
    kv, st = T["diagnostic_msgs/msg/KeyValue"], T["diagnostic_msgs/msg/DiagnosticStatus"]
    for k in _sample_idx(n, RATES_HZ["bms"]):
        t, v = truth.t[k], f"{truth.volts[k]:.2f}V"
        msg = T["diagnostic_msgs/msg/DiagnosticArray"](header=_header(ts, t, ""), status=[
            st(level=0, name="Battery: Current", message="-2.10A", hardware_id="",
               values=[kv(key="current_A", value="-2.10")]),
            st(level=0, name="Battery: Voltage", message=v, hardware_id="",
               values=[kv(key="voltage", value=v)]),
            st(level=0, name="Battery: Percentage", message="synthetic", hardware_id="",
               values=[kv(key="soc_percent", value="80.00%")]),
        ])
        put(t, "/diagnostics", msg, "diagnostic_msgs/msg/DiagnosticArray")

    msgs.sort(key=lambda m: m[0])
    with Writer(tmp_dir, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        conns = {}
        for _, topic, _, typename in msgs:
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, typename, typestore=ts)
        for t_ns, topic, msg, typename in msgs:
            writer.write(conns[topic], t_ns, ts.serialize_cdr(msg, typename))
    written = list(tmp_dir.glob("*.mcap"))
    if not written:
        raise RuntimeError(f"writer produced no .mcap in {tmp_dir}")
    written[0].replace(out_path)
    shutil.rmtree(tmp_dir)

    truth_path = out_path.with_name(f"{out_path.stem}_truth.yaml")
    truth_path.write_text(
        "# SYNTHETIC BAG - ground truth written by scripts/sysid/simulation/synth_bag.py.\n"
        "# Not a recording. Values here are what a fit of the bag should recover.\n"
        + yaml.safe_dump(truth_doc, sort_keys=False), encoding="utf-8")
    return out_path, truth_path


def make_truth_doc(info: dict, cfg: SynthConfig, program: list[Segment], truth: Truth) -> dict:
    """Everything a fit of the bag should recover, plus how the bag was made."""
    return {
        "synthetic": True,
        "generator": f"synth_bag.py v{GENERATOR_VERSION}",
        "created": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "truth_model": {k: info[k] for k in ("model_file", "model_version", "thrust")},
        "surge": {
            "m": info["m"], "X_udot": info["X_udot"], "m_tot": info["m_tot"],
            "X_u": info["X_u"], "X_uu": info["X_uu"],
            "thrust_scale": cfg.thrust_scale, "offset_n": cfg.offset_n,
        },
        "timing": {
            "actuation_lag_s": cfg.actuation_lag_s,
            "servo_delay_s_equivalent": -cfg.actuation_lag_s,
            "spinup_tau_s": cfg.spinup_tau_s,
            "ekf_lag_s": cfg.ekf_lag_s,
            "dvl_lag_s": cfg.dvl_lag_s,
        },
        "config": asdict(cfg),
        "rates_hz": dict(RATES_HZ, sim=SIM_RATE_HZ),
        "program": [asdict(seg) for seg in program],
        "events": [{"t_rel_s": round(float(t - truth.t[0]), 6), **p} for t, p in truth.events],
        "duration_s": float(info["duration_s"]),
    }


def generate(model_path: Path, program: list[Segment], cfg: SynthConfig,
             out_path: Path) -> tuple[Path, Path, Truth, dict]:
    truth, info = simulate(model_path, program, cfg)
    bag, truth_file = write_bag(truth, info, cfg, program, out_path)
    return bag, truth_file, truth, info


# ============================================================
# CLI
# ============================================================


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--program", default="suite", choices=["steps", "coastdown", "suite"])
    ap.add_argument("--levels", type=int, nargs="+", default=None,
                    help="surge commands of the program (default as in command_scripts/)")
    ap.add_argument("--hold-s", type=float, default=None, help="override every hold time")
    ap.add_argument("--rest-s", type=float, default=None, help="override every rest time")
    ap.add_argument("--model", type=Path, default=SYSID / "dummy_bluerov2_heavy_t500.yaml",
                    help="TRUTH model: coefficients and thrust map of the simulated vehicle")
    ap.add_argument("--out", type=Path, default=None,
                    help="output .mcap (default recordings/sysid/synthetic/synth_<program>_"
                         "seed<seed>.mcap)")
    ap.add_argument("--ideal", action="store_true",
                    help="switch every delay and noise off; individual flags still apply on top")
    for f in fields(SynthConfig):
        flag = "--" + f.name.replace("_", "-")
        ap.add_argument(flag, type=type(f.default), default=None,
                        help=f"(default {f.default}, ideal "
                             f"{getattr(SynthConfig.ideal(), f.name)})")
    args = ap.parse_args(argv)

    cfg = SynthConfig.ideal() if args.ideal else SynthConfig()
    overrides = {f.name: getattr(args, f.name) for f in fields(SynthConfig)
                 if getattr(args, f.name) is not None}
    cfg = replace(cfg, **overrides)
    program = build_program(args.program, args.levels, args.hold_s, args.rest_s)
    out = args.out or (Path("recordings/sysid/synthetic")
                       / f"synth_{args.program}_seed{cfg.seed}.mcap")

    bag, truth_file, truth, info = generate(args.model, program, cfg, out)

    print(f"[synth] truth model {info['model_version']}")
    print(f"[synth] m_tot={info['m_tot']:g} kg  X_u={info['X_u']:g} N s/m  "
          f"X_uu={info['X_uu']:g} N s^2/m^2  thrust_scale={cfg.thrust_scale:g}")
    print(f"[synth] actuation lag {cfg.actuation_lag_s:g} s (servo_delay_s equivalent "
          f"{-cfg.actuation_lag_s:g}), spin-up {cfg.spinup_tau_s:g} s, EKF lag "
          f"{cfg.ekf_lag_s:g} s")
    print(f"[synth] {info['duration_s']:.1f} s, {len(program)} manoeuvre(s):")
    for seg in program:
        on = next(t for t, p in truth.events
                  if p["node"] == seg.node and p["event"] == "thrust_on")
        k_end = int(round((on - truth.t[0] + seg.hold_s) * SIM_RATE_HZ)) - 1
        print(f"  {seg.maneuver:<9} surge {seg.surge:4d} "
              f"(pwm {info['pwm_neutral'] + round(cfg.pwm_per_cmd * seg.surge):.0f} us) "
              f"hold {seg.hold_s:g} s -> u at cut {truth.u[k_end]:.3f} m/s")
    print(f"[out] bag:   {bag}")
    print(f"[out] truth: {truth_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
