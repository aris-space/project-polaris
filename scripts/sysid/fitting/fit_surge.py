"""
Fit the surge equation  T = m_tot * du/dt + X_u * u + X_uu * |u| u  to a bag by least squares.

Usage (from the repo root):
    python scripts/sysid/fitting/fit_surge.py recordings/<bag>.mcap [--dummy]

--dummy takes the thrust from dummy_bluerov2_heavy.yaml (BlueROV2 T200 polynomial) instead of
model.yaml (T500 table).
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent)
)  # finds model_io, bag_io

from bag_io import DEFAULT_DIAG_TOPIC, DEFAULT_EKF_TOPIC, DEFAULT_SERVO_TOPIC
from coupling_check import load_surge_data, smooth
from results_log import log_fit

SYSID = Path(__file__).resolve().parent.parent
MODEL = SYSID / ("dummy_bluerov2_heavy.yaml" if "--dummy" in sys.argv else "model.yaml")
RATE_HZ = 100.0  # >= 100 Hz: at 20 Hz the grid delays the thrust steps and biases m_tot (notes 10.6)
SMOOTH_S = 1.0


def prepare(bag):
    """Return speed u, acceleration du, quadratic term uu = |u|u and thrust T: all passed through
    the same smoothing filter, with unusable samples removed."""
    t, sig, _, thrust, valid, _ = load_surge_data(
        bag,
        MODEL,
        RATE_HZ,
        None,
        None,
        DEFAULT_EKF_TOPIC,
        DEFAULT_SERVO_TOPIC,
        DEFAULT_DIAG_TOPIC,
        None,
    )
    dt = 1.0 / RATE_HZ

    # Fill stale-PWM samples so the filter has no holes; they are dropped again via keep.
    bad = ~valid
    thrust_f = thrust.copy()
    if bad.any():
        thrust_f[bad] = np.interp(t[bad], t[valid], thrust[valid])

    # The SAME filter on every term keeps T = m_tot du + X_u u + X_uu |u|u exactly true for the
    # filtered signals: |u|u is formed from the raw speed BEFORE smoothing, and du is the
    # derivative of the smoothed speed (not the separate SG derivative filter).
    u_raw = sig["u"]
    u = smooth(u_raw, dt, SMOOTH_S)
    du = np.gradient(u, dt)
    uu = smooth(np.abs(u_raw) * u_raw, dt, SMOOTH_S)
    T = smooth(thrust_f, dt, SMOOTH_S)

    # Drop samples within half a filter window of a bad one, and at both ends.
    half = int(round(SMOOTH_S / dt / 2))
    keep = np.convolve(bad.astype(float), np.ones(2 * half + 1), mode="same") == 0
    keep[:half] = False
    keep[-half:] = False
    return u[keep], du[keep], uu[keep], T[keep]


def build_regression(u, du, uu, T):
    """Return A (columns: du, u, |u|u) and y (thrust)."""

    A = np.column_stack([du, u, uu])
    return A, T


def fit(A, y):
    """Return theta = [m_tot, X_u, X_uu]."""
    theta = np.linalg.lstsq(A, y, rcond=None)[0]
    return theta


if __name__ == "__main__":
    u, du, uu, T = prepare(Path(sys.argv[1]))
    A, y = build_regression(u, du, uu, T)
    m_tot, X_u, X_uu = fit(A, y)
    print(f"m_tot = {m_tot:.3f} kg")
    print(f"X_u   = {X_u:.3f} N s/m")
    print(f"X_uu  = {X_uu:.3f} N s^2/m^2")

    fit_id = log_fit(
        __file__,
        [sys.argv[1]],
        {"m_tot": m_tot, "X_u": X_u, "X_uu": X_uu},
        units={"m_tot": "kg", "X_u": "N s/m", "X_uu": "N s^2/m^2"},
        model=MODEL,
        settings={"rate_hz": RATE_HZ, "smooth_s": SMOOTH_S, "filtering": "consistent"},
    )
    print(f"logged as {fit_id} in results/fit_log.yaml")
