#!/usr/bin/env python3
"""
Surge dynamics for the POLARIS identified model. Pure numpy: no ROS, no file I/O except
the thruster curve CSV loader.

Model (surge only, 1 DOF, maneuvers.md section 1):

    (m - X_udot) * du/dt = T(pwm) - X_u * u - X_uu * |u| * u

Sign convention follows Fossen: the added-mass derivative X_udot is negative, so the
total inertia m_tot = m - X_udot is larger than the dry mass. The loader rejects a
non-positive m_tot rather than silently producing unstable integration.

The thruster curve is a lookup table (PWM in microseconds -> force in kgf) converted to
newtons on load. Values outside the tabulated range are clamped to the end points; no
extrapolation.

Dependency: pip install numpy

Example:
    curve = ThrusterCurve.from_csv("thruster_curves/DUMMY_bluerov2_t200_poly.csv")
    p = SurgeParams(m_tot=19.86, X_u=13.7, X_uu=141.0)
    u = rk4_step(0.0, curve.force_n(1700), 0.01, p)
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

KGF_TO_N = 9.80665


@dataclass(frozen=True)
class SurgeParams:
    """Coefficients of the 1-DOF surge equation, in SI units."""

    m_tot: float  # m - X_udot  [kg]
    X_u: float  # linear damping     [N s/m]
    X_uu: float  # quadratic damping  [N s^2/m^2]

    def __post_init__(self) -> None:
        if not self.m_tot > 0.0:
            raise ValueError(f"m_tot = m - X_udot must be positive, got {self.m_tot}")


class ThrusterCurve:
    """PWM (us) -> thrust (N), linear interpolation between tabulated points."""

    def __init__(self, pwm_us: np.ndarray, force_table_n: np.ndarray) -> None:
        pwm_us = np.asarray(pwm_us, dtype=np.float64)
        force_table_n = np.asarray(force_table_n, dtype=np.float64)
        if pwm_us.ndim != 1 or pwm_us.shape != force_table_n.shape:
            raise ValueError("pwm_us and force_table_n must be 1-D arrays of equal length")
        if pwm_us.size < 2:
            raise ValueError("thruster curve needs at least two points")
        order = np.argsort(pwm_us)
        self.pwm_us = pwm_us[order]
        self.force_table_n = force_table_n[order]
        if np.any(np.diff(self.pwm_us) <= 0):
            raise ValueError("thruster curve has duplicate PWM values")

    @classmethod
    def from_csv(cls, path: str | Path) -> ThrusterCurve:
        """Read a CSV with columns pwm_us, force_kgf. Lines starting with '#' are ignored."""
        path = Path(path)
        pwm: list[float] = []
        kgf: list[float] = []
        with path.open(newline="", encoding="utf-8") as fh:
            rows = csv.reader(r for r in fh if not r.lstrip().startswith("#"))
            header = next(rows, None)
            if header is None:
                raise ValueError(f"{path} is empty")
            cols = [h.strip().lower() for h in header]
            try:
                i_pwm, i_f = cols.index("pwm_us"), cols.index("force_kgf")
            except ValueError:
                raise ValueError(f"{path} must have columns 'pwm_us' and 'force_kgf', got {cols}")
            for row in rows:
                if not row or not row[0].strip():
                    continue
                pwm.append(float(row[i_pwm]))
                kgf.append(float(row[i_f]))
        return cls(np.array(pwm), np.array(kgf) * KGF_TO_N)

    def force_n(self, pwm_us: float | np.ndarray) -> float | np.ndarray:
        """Thrust in newtons. Clamped to the table's end points outside its range."""
        return np.interp(pwm_us, self.pwm_us, self.force_table_n)


def surge_accel(u: float, thrust_n: float, p: SurgeParams) -> float:
    """du/dt from the 1-DOF surge equation."""
    return (thrust_n - p.X_u * u - p.X_uu * abs(u) * u) / p.m_tot


def rk4_step(u: float, thrust_n: float, dt: float, p: SurgeParams) -> float:
    """One RK4 step with thrust held constant across the interval (zero-order hold)."""
    k1 = surge_accel(u, thrust_n, p)
    k2 = surge_accel(u + 0.5 * dt * k1, thrust_n, p)
    k3 = surge_accel(u + 0.5 * dt * k2, thrust_n, p)
    k4 = surge_accel(u + dt * k3, thrust_n, p)
    return u + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def steady_state_speed(thrust_n: float, p: SurgeParams) -> float:
    """Speed where thrust balances damping: X_uu*u^2 + X_u*u - T = 0, positive root."""
    if thrust_n < 0.0:
        return -steady_state_speed(-thrust_n, p)
    if p.X_uu == 0.0:
        return thrust_n / p.X_u
    disc = p.X_u**2 + 4.0 * p.X_uu * thrust_n
    return (-p.X_u + np.sqrt(disc)) / (2.0 * p.X_uu)


def powerlaw_speed(pwm_us: float | np.ndarray, pwm_neutral: float = 1500.0) -> float | np.ndarray:
    """Deployed baseline from thruster_velocity_estimator.py: vx = 0.780*sign(c)*|c|^0.964,
    with c = (pwm - 1500) / 500. Used for side-by-side comparison, not identification."""
    c = (np.asarray(pwm_us, dtype=np.float64) - pwm_neutral) / 500.0
    return 0.780 * np.sign(c) * np.abs(c) ** 0.964
