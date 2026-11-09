#!/usr/bin/env python3
"""
Tests for simulation/synth_bag.py: the synthetic bag must be readable by bag_io exactly like a
real recording, and the propagation must reproduce its truth. Together they check the generator
and the whole read -> thrust -> integrate path end to end.

The servo-delay tests pin down the sign convention: thrust that acts L seconds AFTER the PWM is
logged is compensated by thrust.servo_delay_s = -L, because the scripts subtract it.

Run: python -m pytest scripts/sysid/tests/test_synth_bag.py
"""
from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rosbags")

SYSID = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SYSID))
sys.path.insert(0, str(SYSID / "simulation"))
sys.path.insert(0, str(SYSID / "validation"))

import yaml  # noqa: E402
from rosbags.highlevel import AnyReader  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

from actuation import ThrustSource  # noqa: E402
from bag_io import read_tracks, read_voltage  # noqa: E402
from dynamics import SurgeParams  # noqa: E402
from model_io import load_model  # noqa: E402
from propagate_model import default_pwm_stale_s, propagate  # noqa: E402
from synth_bag import G, SynthConfig, build_program, generate  # noqa: E402

TRUTH_MODEL = SYSID / "dummy_bluerov2_heavy_t500.yaml"
LAG = 0.2


def _program():
    return build_program("steps", levels=[300, 500], hold_s=6.0, rest_s=6.0)


@pytest.fixture(scope="module")
def ideal(tmp_path_factory):
    out = tmp_path_factory.mktemp("synth") / "ideal.mcap"
    return generate(TRUTH_MODEL, _program(), SynthConfig.ideal(), out)


@pytest.fixture(scope="module")
def lagged(tmp_path_factory):
    out = tmp_path_factory.mktemp("synth") / "lagged.mcap"
    cfg = replace(SynthConfig.ideal(), actuation_lag_s=LAG)
    return generate(TRUTH_MODEL, _program(), cfg, out)


def _surge_rms(bag: Path, servo_delay_s: float) -> float:
    """Free-run propagation of the truth model over the bag, as propagate_model.main does it."""
    model = load_model(TRUTH_MODEL)
    params = SurgeParams(m_tot=model.get("rigid_body.m") - model.get("added_mass.X_udot"),
                         X_u=model.get("damping_linear.X_u"),
                         X_uu=model.get("damping_quadratic.X_uu"))
    source = ThrustSource(model, 0, 1500.0, 1)
    odom, servo = read_tracks(bag)
    voltage = read_voltage(bag)
    servo.log_t = servo.log_t - servo_delay_s
    lo, hi = max(odom.t[0], servo.log_t[0]), min(odom.t[-1], servo.log_t[-1])
    keep = (odom.t >= lo) & (odom.t <= hi)
    for name in ("t", "log_t", "pos", "yaw", "vel_body", "yaw_rate", "ang_vel", "roll", "pitch"):
        setattr(odom, name, getattr(odom, name)[keep])
    _, _, surge, *_ = propagate(
        odom, servo, source, lambda t: float(voltage.at(t)), params, channel=0, neutral=1500.0,
        sign=1, reinit_s=0.0, dt=0.01, with_baseline=False,
        pwm_stale_s=default_pwm_stale_s(servo.log_t))
    err = surge - odom.vel_body[:, 0]
    return float(np.sqrt(np.mean(err**2)))


def test_ideal_bag_is_reproduced_by_the_propagation(ideal):
    bag, *_ = ideal
    assert _surge_rms(bag, 0.0) < 1e-6


def test_actuation_lag_is_compensated_by_a_negative_servo_delay(lagged):
    bag, *_ = lagged
    uncompensated = _surge_rms(bag, 0.0)
    assert uncompensated > 1e-3  # the lag is visible
    assert _surge_rms(bag, -LAG) < 1e-6  # and removed exactly by servo_delay_s = -lag
    assert _surge_rms(bag, +LAG) > uncompensated  # the intuitive sign makes it worse


def test_truth_file_records_the_model_and_the_delay_convention(lagged):
    _, truth_file, _, info = lagged
    doc = yaml.safe_load(truth_file.read_text(encoding="utf-8"))
    assert doc["synthetic"] is True
    assert doc["surge"]["m_tot"] == pytest.approx(info["m"] - info["X_udot"])
    assert doc["timing"]["actuation_lag_s"] == LAG
    assert doc["timing"]["servo_delay_s_equivalent"] == -LAG


def test_events_bracket_the_pwm_steps(ideal):
    bag, _, truth, info = ideal
    odom, servo = read_tracks(bag)
    events = []
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == "/sysid/event"]
        for conn, t_ns, raw in reader.messages(connections=conns):
            events.append((t_ns * 1e-9, json.loads(reader.deserialize(raw, conn.msgtype).data)))
    assert [p["event"] for _, p in events] == ["thrust_on", "thrust_cut", "done"] * 2
    period = float(np.median(np.diff(servo.log_t)))
    above = servo.pwm[:, 0] > info["pwm_neutral"]
    rises = servo.log_t[1:][above[1:] & ~above[:-1]]
    falls = servo.log_t[1:][~above[1:] & above[:-1]]
    ons = [t for t, p in events if p["event"] == "thrust_on"]
    cuts = [t for t, p in events if p["event"] == "thrust_cut"]
    assert np.all(np.abs(rises - ons) <= period)
    assert np.all(np.abs(falls - cuts) <= period)


def test_voltage_and_frames_match_a_real_recording(ideal):
    bag, *_ = ideal
    odom, _ = read_tracks(bag)
    voltage = read_voltage(bag)
    assert odom.frame_id == "odom" and odom.child_frame_id == "base_link"
    assert np.all(voltage.volts == pytest.approx(SynthConfig.ideal().v_start))


def test_imu_reports_specific_force_at_rest(ideal):
    bag, *_ = ideal
    theta = SynthConfig.ideal().trim_pitch_rad
    f = []
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == "/imu/data"]
        t0 = None
        for conn, t_ns, raw in reader.messages(connections=conns):
            t0 = t0 or t_ns
            if (t_ns - t0) * 1e-9 > 4.0:  # before the first thrust_on (lead 5 s)
                break
            a = reader.deserialize(raw, conn.msgtype).linear_acceleration
            f.append((a.x, a.y, a.z))
    f = np.mean(f, axis=0)
    # FLU, positive pitch nose down: gravity tilts forward, the accelerometer reads -g sin(theta).
    assert f[0] == pytest.approx(-G * np.sin(theta), abs=1e-9)
    assert f[2] == pytest.approx(G * np.cos(theta), abs=1e-9)


def test_dvl_rotates_back_to_the_body_velocity(ideal):
    bag, _, truth, _ = ideal
    with AnyReader([bag]) as reader:
        tf = next(reader.deserialize(raw, c.msgtype)
                  for c, _, raw in reader.messages(
                      connections=[c for c in reader.connections if c.topic == "/tf_static"]))
        q = next(t.transform.rotation for t in tf.transforms if t.child_frame_id == "dvl_a50_link")
        r_dvl = Rot.from_quat([q.x, q.y, q.z, q.w])
        conns = [c for c in reader.connections if c.topic == "/sensors/dvl/velocity"]
        t, v_body = [], []
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            v = m.velocity
            v_body.append(r_dvl.apply([v.x, v.y, v.z]))
    v_body = np.asarray(v_body)
    # Timestamps near 1.8e9 s resolve only ~0.2 us as float64, hence 1e-6 rather than 1e-9.
    assert v_body[:, 0] == pytest.approx(np.interp(t, truth.t, truth.u), abs=1e-6)
    assert np.abs(v_body[:, 1:]).max() < 1e-9


def test_propagate_cli_runs_on_a_synthetic_bag(ideal, tmp_path):
    from propagate_model import main

    bag, *_ = ideal
    out = tmp_path / "propagated.mcap"
    assert main([str(bag), "--model", str(TRUTH_MODEL), "--out", str(out),
                 "--no-navsatfix"]) == 0
    assert out.exists()
    summary = out.with_name(f"{out.stem}_summary.txt").read_text(encoding="utf-8")
    assert "surge RMS error    :   0.000 m/s" in summary
