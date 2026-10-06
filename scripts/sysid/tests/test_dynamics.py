#!/usr/bin/env python3
"""
Tests for scripts/sysid/dynamics.py, checking the integrator against closed-form solutions.

Run:  python -m pytest scripts/sysid/tests/ -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dynamics import (  # noqa: E402
    KGF_TO_N,
    SurgeParams,
    ThrusterCurve,
    powerlaw_speed,
    rk4_step,
    steady_state_speed,
    surge_accel,
)


def _roll(u0: float, thrust: float, p: SurgeParams, t_end: float, dt: float = 1e-3) -> float:
    u = u0
    for _ in range(int(round(t_end / dt))):
        u = rk4_step(u, thrust, dt, p)
    return u


def test_quadratic_decay_matches_closed_form():
    """Zero thrust, quadratic drag only: u(t) = u0 / (1 + X_uu*u0*t/m_tot)."""
    p = SurgeParams(m_tot=24.0, X_u=0.0, X_uu=30.0)
    u0 = 0.8
    for t_end in (0.5, 2.0, 5.0):
        expected = u0 / (1.0 + p.X_uu * u0 * t_end / p.m_tot)
        assert _roll(u0, 0.0, p, t_end) == pytest.approx(expected, rel=1e-6)


def test_linear_decay_matches_exponential():
    """Zero thrust, linear drag only: u(t) = u0 * exp(-X_u*t/m_tot)."""
    p = SurgeParams(m_tot=24.0, X_u=8.0, X_uu=0.0)
    u0 = 0.5
    t_end = 3.0
    assert _roll(u0, 0.0, p, t_end) == pytest.approx(u0 * np.exp(-p.X_u * t_end / p.m_tot), rel=1e-6)


def test_constant_thrust_reaches_sqrt_steady_state():
    """Constant thrust, quadratic drag only: steady state is sqrt(T/X_uu)."""
    p = SurgeParams(m_tot=24.0, X_u=0.0, X_uu=30.0)
    thrust = 12.0
    assert _roll(0.0, thrust, p, 60.0) == pytest.approx(np.sqrt(thrust / p.X_uu), rel=1e-4)


def test_steady_state_speed_agrees_with_integration():
    """The algebraic steady state matches a long rollout for mixed linear+quadratic damping."""
    p = SurgeParams(m_tot=24.0, X_u=8.0, X_uu=30.0)
    for thrust in (1.3, 5.0, 17.0):
        assert _roll(0.0, thrust, p, 120.0) == pytest.approx(steady_state_speed(thrust, p), rel=1e-4)


def test_steady_state_is_odd_in_thrust():
    p = SurgeParams(m_tot=24.0, X_u=8.0, X_uu=30.0)
    assert steady_state_speed(-7.0, p) == pytest.approx(-steady_state_speed(7.0, p))


def test_surge_accel_balances_at_steady_state():
    p = SurgeParams(m_tot=24.0, X_u=8.0, X_uu=30.0)
    u_ss = steady_state_speed(10.0, p)
    assert surge_accel(u_ss, 10.0, p) == pytest.approx(0.0, abs=1e-12)


def test_m_tot_must_be_positive():
    with pytest.raises(ValueError):
        SurgeParams(m_tot=0.0, X_u=1.0, X_uu=1.0)


def test_curve_interpolates_and_converts_kgf_to_newtons(tmp_path: Path):
    csv = tmp_path / "curve.csv"
    csv.write_text("# comment line\npwm_us,force_kgf\n1500,0.0\n1600,1.0\n1700,3.0\n",
                   encoding="utf-8")
    curve = ThrusterCurve.from_csv(csv)

    assert curve.force_n(1500) == pytest.approx(0.0)
    assert curve.force_n(1600) == pytest.approx(KGF_TO_N)
    assert curve.force_n(1550) == pytest.approx(0.5 * KGF_TO_N)  # linear between points
    assert curve.force_n(1650) == pytest.approx(2.0 * KGF_TO_N)


def test_curve_clamps_outside_table(tmp_path: Path):
    csv = tmp_path / "curve.csv"
    csv.write_text("pwm_us,force_kgf\n1500,0.0\n1700,3.0\n", encoding="utf-8")
    curve = ThrusterCurve.from_csv(csv)
    assert curve.force_n(1200) == pytest.approx(0.0)
    assert curve.force_n(1900) == pytest.approx(3.0 * KGF_TO_N)


def test_curve_rejects_bad_columns(tmp_path: Path):
    csv = tmp_path / "curve.csv"
    csv.write_text("pwm,thrust\n1500,0.0\n1700,3.0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="pwm_us"):
        ThrusterCurve.from_csv(csv)


def test_dummy_bluerov2_curve_loads_and_is_symmetric():
    """Guards from_csv against the real on-disk curve file: the T200 polynomial is odd,
    so forward and reverse thrust must mirror about neutral and vanish there."""
    curve = ThrusterCurve.from_csv(
        Path(__file__).resolve().parent.parent / "thruster_curves" / "DUMMY_bluerov2_t200_poly.csv"
    )
    assert float(curve.force_n(1500)) == pytest.approx(0.0, abs=1e-9)
    for d in (100, 200, 400):
        assert float(curve.force_n(1500 + d)) == pytest.approx(-float(curve.force_n(1500 - d)))


def test_dummy_bluerov2_curve_gives_plausible_steady_states():
    """Sanity-check the dummy model end to end: a BlueROV2 at full forward PWM should sit
    somewhere around 1 m/s, not 0.01 or 100."""
    curve = ThrusterCurve.from_csv(
        Path(__file__).resolve().parent.parent / "thruster_curves" / "DUMMY_bluerov2_t200_poly.csv"
    )
    p = SurgeParams(m_tot=13.5 + 6.36, X_u=13.7, X_uu=141.0)
    v_full = steady_state_speed(float(curve.force_n(1900)), p)
    assert 0.3 < v_full < 1.5
    assert steady_state_speed(float(curve.force_n(1700)), p) < v_full
