#!/usr/bin/env python3
"""
Tests for fitting/coupling_check.py on synthetic surge data with known coefficients.

The surge force is generated from u, du/dt and, optionally, a coupling term k * v * r. The
check must recover the own-axis coefficients, flag v*r when the coupling is present, and flag
nothing when it is absent.

Run: python -m pytest scripts/sysid/tests/test_coupling_check.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "fitting"))

from coupling_check import (  # noqa: E402
    effective_n,
    own_axis_fit,
    partial_scan,
    smooth,
    surge_terms,
)

DT = 0.05
M_TOT, X_U, X_UU, OFFSET = 36.0, 8.0, 150.0, 2.0


def _smooth_random(rng, n, scale, window_s):
    return scale * smooth(rng.standard_normal(n), DT, window_s, poly=2)


def _synthetic(coupling_k: float, seed: int = 1):
    rng = np.random.default_rng(seed)
    n = 6000  # 300 s
    t = np.arange(n) * DT
    u = 0.25 + 0.15 * np.sin(2 * np.pi * t / 40.0) + _smooth_random(rng, n, 0.6, 3.0)
    v = _smooth_random(rng, n, 0.4, 4.0)
    r = _smooth_random(rng, n, 1.5, 4.0)
    w = _smooth_random(rng, n, 0.3, 4.0)
    q = _smooth_random(rng, n, 0.8, 4.0)
    du = np.gradient(u, DT)
    force = M_TOT * du + X_U * u + X_UU * np.abs(u) * u + OFFSET + coupling_k * v * r
    force = force + 0.3 * rng.standard_normal(n)  # thrust curve / sensor noise
    sig = {"u": u, "v": v, "w": w, "q": q, "r": r}
    return force, du, sig


def _candidates(s):
    return {"v*r": s["v"] * s["r"], "w*q": s["w"] * s["q"], "v": s["v"], "r": s["r"],
            "w": s["w"], "q": s["q"]}


def test_recovers_own_coefficients_without_coupling():
    force, du, s = _synthetic(coupling_k=0.0)
    fit = own_axis_fit(force, surge_terms(s["u"], du, with_offset=True))
    m, xu, xuu, off = fit.theta
    assert m == pytest.approx(M_TOT, rel=0.02)
    assert xu == pytest.approx(X_U, rel=0.1)
    assert xuu == pytest.approx(X_UU, rel=0.02)
    assert off == pytest.approx(OFFSET, abs=0.2)
    assert fit.r2 > 0.99


def test_flags_nothing_without_coupling():
    force, du, s = _synthetic(coupling_k=0.0)
    own = surge_terms(s["u"], du, with_offset=True)
    fit = own_axis_fit(force, own)
    results = partial_scan(fit.residual, own, _candidates(s), DT)
    assert not any(c.significant for c in results), [(c.name, c.r, c.threshold) for c in results]


def test_flags_v_times_r_when_coupled():
    force, du, s = _synthetic(coupling_k=-20.0)  # e.g. a sway added-mass Coriolis term
    own = surge_terms(s["u"], du, with_offset=True)
    fit = own_axis_fit(force, own)
    results = partial_scan(fit.residual, own, _candidates(s), DT)
    top = results[0]
    assert top.name == "v*r"
    assert top.significant
    assert top.r < 0  # sign of the injected coupling
    assert abs(top.lag_s) < 1e-9


def test_effective_n_shrinks_for_smooth_series():
    rng = np.random.default_rng(0)
    white = rng.standard_normal(4000)
    slow = smooth(rng.standard_normal(4000), DT, 4.0)
    assert effective_n(white, rng.standard_normal(4000)) > 3000
    assert effective_n(slow, smooth(rng.standard_normal(4000), DT, 4.0)) < 1000
