"""
Fit the heave equation to a bag by least squares:

    Z_thr = m_z * dw/dt + Z_w * w + Z_ww * |w| w - (B - W)

Z_thr is the heave force of the vertical thrusters (thrust.allocation.heave in the model, PWM x
voltage interpolation), w the EKF heave velocity (body z, FLU: up is positive). B - W is the net
buoyancy, fitted as a constant (column of -1); positive means the vehicle is buoyant.
All coefficients are positive magnitudes; m_z = m - Z_wdot.

Usage (from the repo root):
    python scripts/sysid/fitting/fit_heave.py recordings/<bag>.mcap
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(
    0, str(Path(__file__).resolve().parent.parent)
)  # finds model_io, bag_io

from bag_io import DEFAULT_DIAG_TOPIC, DEFAULT_EKF_TOPIC, DEFAULT_SERVO_TOPIC
from coupling_check import load_heave_data, smooth
from results_log import log_fit

MODEL = Path(__file__).resolve().parent.parent / "model.yaml"
RATE_HZ = 100.0  # >= 100 Hz, as in fit_surge.py (notes 10.6)
SMOOTH_S = 1.0


def prepare(bag):
    """Return heave speed w, acceleration dw, quadratic term ww = |w|w and heave thrust Z: all passed
    through the same smoothing filter, with unusable samples removed."""
    t, sig, _, thrust, valid, _ = load_heave_data(
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

    # The same filter on every term, as in fit_surge.py: |w|w is formed from the raw speed BEFORE
    # smoothing, and dw is the derivative of the smoothed speed.
    w_raw = sig["w"]
    w = smooth(w_raw, dt, SMOOTH_S)
    dw = np.gradient(w, dt)
    ww = smooth(np.abs(w_raw) * w_raw, dt, SMOOTH_S)
    Z = smooth(thrust_f, dt, SMOOTH_S)

    # Drop samples within half a filter window of a bad one, and at both ends.
    half = int(round(SMOOTH_S / dt / 2))
    keep = np.convolve(bad.astype(float), np.ones(2 * half + 1), mode="same") == 0
    keep[:half] = False
    keep[-half:] = False
    return w[keep], dw[keep], ww[keep], Z[keep]


def build_regression(w, dw, ww, Z):
    """Return A (columns: dw, w, |w|w, -1) and y (heave thrust). The -1 column carries B - W."""
    A = np.column_stack([dw, w, ww, -np.ones_like(w)])
    return A, Z


def fit(A, y):
    """Return theta = [m_z, Z_w, Z_ww, B - W]."""
    theta = np.linalg.lstsq(A, y, rcond=None)[0]
    return theta


if __name__ == "__main__":
    w, dw, ww, Z = prepare(Path(sys.argv[1]))
    A, y = build_regression(w, dw, ww, Z)
    m_z, Z_w, Z_ww, B_minus_W = fit(A, y)
    print(f"m_z   = {m_z:.3f} kg")
    print(f"Z_w   = {Z_w:.3f} N s/m")
    print(f"Z_ww  = {Z_ww:.3f} N s^2/m^2")
    print(f"B - W = {B_minus_W:.3f} N  (positive = buoyant)")

    fit_id = log_fit(
        __file__,
        [sys.argv[1]],
        {"m_z": m_z, "Z_w": Z_w, "Z_ww": Z_ww, "B_minus_W": B_minus_W},
        units={"m_z": "kg", "Z_w": "N s/m", "Z_ww": "N s^2/m^2", "B_minus_W": "N"},
        model=MODEL,
        settings={"rate_hz": RATE_HZ, "smooth_s": SMOOTH_S, "filtering": "consistent"},
    )
    print(f"logged as {fit_id} in results/fit_log.yaml")
