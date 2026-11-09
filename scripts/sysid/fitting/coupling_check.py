#!/usr/bin/env python3
"""
Coupling check: fit one axis on its own, then test whether signals from the other axes still
explain what that per-axis fit leaves over.

Per-axis identification assumes the axes are decoupled (port-starboard symmetry splits the
6-DOF system into a longitudinal and a lateral group). This script checks that assumption on a
bag, before any coupling terms are added to the model:

  1. Own-axis fit. The axis equation is fitted on the bag itself by least squares, using only
     that axis's own terms. For surge, with all coefficients as positive magnitudes:
         T_surge = m_tot*du/dt + X_u*u + X_uu*|u|u (+ constant offset: tether / current)
  2. Residual. Whatever the own terms cannot explain, in newtons.
  3. Partial correlation. Each candidate coupling signal (other-axis velocities, the Fossen
     cross terms such as v*r, attitude, the other thrusters' PWM) is first cleaned of the part
     the own regressors already explain, then correlated with the residual. Without this step
     every signal would correlate with the residual through the speed alone.
  4. Significance. Neighbouring samples are strongly correlated, so a correlation is judged
     against z/sqrt(N_eff), with N_eff from the autocorrelation of both series (Bartlett).
     Many candidates are screened at once, so z = 3 at lag 0 rather than the usual 2, which
     would flag about one candidate in twenty by chance. A lag scan (default +-0.5 s) reports
     whether the best match is delayed; since scanning lags is itself many tests, a lagged
     match only counts at z = 3.5.

A flagged candidate means: this signal explains part of what the axis's own model misses. That
is coupling, or an unmodelled effect that moves with it (current, tether, a controller
reacting to both). It is a pointer for which coupling term to try, not proof of one.

Caveats: autonomy-driven bags are closed loop, so the own-axis fit is biased, and only bag
sections with a trustworthy EKF should be analysed (--t-start / --t-end crop the bag).
Only surge is implemented: it is the only axis whose thrust curve is known.

Dependencies: pip install rosbags numpy scipy pyyaml

Example:
    python scripts/sysid/fitting/coupling_check.py \\
        recordings/zermatt_rectangle_07_2026_04_29-13_38_44_0.mcap \\
        --model scripts/sysid/dummy_bluerov2_heavy_t500.yaml
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


# ============================================================
# Analysis core (numpy only, no bag access - used by the tests)
# ============================================================


def smooth(x: np.ndarray, dt: float, window_s: float, poly: int = 2, deriv: int = 0) -> np.ndarray:
    """Savitzky-Golay smoothing or differentiation on a uniform grid (zero phase)."""
    n = max(poly + 2, int(round(window_s / dt)) | 1)  # odd window, long enough for the poly
    return savgol_filter(x, n, poly, deriv=deriv, delta=dt)


def _acf(x: np.ndarray, max_lag: int) -> np.ndarray:
    x = x - x.mean()
    n = x.size
    f = np.fft.rfft(x, 2 * n)
    ac = np.fft.irfft(f * np.conj(f))[: max_lag + 1]
    return ac / ac[0] if ac[0] > 0 else np.zeros(max_lag + 1)


def effective_n(a: np.ndarray, b: np.ndarray) -> float:
    """Bartlett's effective sample size for the correlation of two autocorrelated series.

    N_eff = N / (1 + 2 * sum_k rho_a(k) rho_b(k)), summed until either autocorrelation has
    decayed below 0.05, so the noisy tail of the estimate does not dominate.
    """
    n = a.size
    max_lag = max(1, n // 4)
    ra, rb = _acf(a, max_lag), _acf(b, max_lag)
    s = 0.0
    for k in range(1, max_lag + 1):
        if ra[k] < 0.05 or rb[k] < 0.05:
            break
        s += ra[k] * rb[k]
    return float(np.clip(n / (1.0 + 2.0 * s), 3.0, n))


@dataclass
class OwnFit:
    names: list[str]
    theta: np.ndarray
    residual: np.ndarray
    r2: float


def own_axis_fit(y: np.ndarray, own: dict[str, np.ndarray]) -> OwnFit:
    """Least squares of the axis target on the axis's own regressors."""
    names = list(own)
    X = np.column_stack([own[k] for k in names])
    theta, *_ = np.linalg.lstsq(X, y, rcond=None)
    res = y - X @ theta
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - float(np.sum(res**2)) / ss_tot if ss_tot > 0 else float("nan")
    return OwnFit(names=names, theta=theta, residual=res, r2=r2)


@dataclass
class CandidateResult:
    name: str
    r: float  # partial correlation at lag 0
    r_best: float  # partial correlation at the best lag
    lag_s: float  # best lag; positive = candidate leads the residual
    n_eff: float
    threshold: float  # Z_LAG0 / sqrt(n_eff)

    @property
    def significant(self) -> bool:
        return (abs(self.r) > self.threshold
                or abs(self.r_best) > self.threshold * Z_LAGGED / Z_LAG0)


Z_LAG0 = 3.0  # screening threshold at lag 0, in standard errors
Z_LAGGED = 3.5  # stricter for the best lag of the scan (several lags tested)


def _corr(a: np.ndarray, b: np.ndarray) -> float:
    a = a - a.mean()
    b = b - b.mean()
    den = np.sqrt(np.sum(a * a) * np.sum(b * b))
    return float(np.sum(a * b) / den) if den > 0 else 0.0


def partial_scan(residual: np.ndarray, own: dict[str, np.ndarray],
                 candidates: dict[str, np.ndarray], dt: float,
                 max_lag_s: float = 0.5) -> list[CandidateResult]:
    """Partial correlation of each candidate with the residual, given the own regressors.

    Each candidate is projected off the own regressors first (Frisch-Waugh), so only the part
    the own-axis model could not already have used is tested. Sorted by |r|, largest first.
    """
    X = np.column_stack(list(own.values()))
    pinv = np.linalg.pinv(X)
    max_lag = int(round(max_lag_s / dt))
    out = []
    for name, c in candidates.items():
        c_perp = c - X @ (pinv @ c)
        if np.std(c_perp) < 1e-12:
            continue
        r0 = _corr(residual, c_perp)
        best_r, best_lag = r0, 0
        for lag in range(-max_lag, max_lag + 1):
            if lag == 0:
                continue
            if lag > 0:  # candidate leads: compare c[t] with residual[t + lag]
                r = _corr(residual[lag:], c_perp[:-lag])
            else:
                r = _corr(residual[:lag], c_perp[-lag:])
            if abs(r) > abs(best_r):
                best_r, best_lag = r, lag
        n_eff = effective_n(residual, c_perp)
        out.append(CandidateResult(name, r0, best_r, best_lag * dt, n_eff,
                                   Z_LAG0 / np.sqrt(n_eff)))
    out.sort(key=lambda c: abs(c.r), reverse=True)
    return out


def surge_terms(u: np.ndarray, du: np.ndarray, with_offset: bool) -> dict[str, np.ndarray]:
    """Own regressors of the surge equation; coefficients come out as m_tot, X_u, X_uu."""
    own = {"m_tot [kg]": du, "X_u [N s/m]": u, "X_uu [N s^2/m^2]": np.abs(u) * u}
    if with_offset:
        own["offset [N]"] = np.ones_like(u)
    return own


def surge_candidates(s: dict[str, np.ndarray], pwm: np.ndarray, neutral: float,
                     surge_channel: int) -> dict[str, np.ndarray]:
    """Coupling candidates for the surge equation.

    The Fossen cross terms that enter surge (rigid-body and added-mass Coriolis: v*r, w*q, and
    the centripetal r^2, q^2), the other axes' velocities and attitude (pitch tilts gravity and
    buoyancy into surge), and the other thrusters' PWM as a proxy for their force until their
    curves exist (a misaligned thruster pushing along x shows up here).
    """
    c = {
        "v*r": s["v"] * s["r"],
        "w*q": s["w"] * s["q"],
        "r^2": s["r"] ** 2,
        "q^2": s["q"] ** 2,
        "v": s["v"],
        "w": s["w"],
        "p": s["p"],
        "q": s["q"],
        "r": s["r"],
        "|r|": np.abs(s["r"]),
        "sin(pitch)": np.sin(s["pitch"]),
        "roll": s["roll"],
    }
    for ch in range(pwm.shape[1]):
        if ch != surge_channel:
            c[f"pwm ch{ch}"] = pwm[:, ch] - neutral
    return c


# ============================================================
# Bag loading
# ============================================================


def load_surge_data(bag: Path, model_path: Path, rate_hz: float, t_start: float | None,
                    t_end: float | None, ekf_topic: str, servo_topic: str, diag_topic: str,
                    fixed_voltage: float | None):
    """Resample everything onto a uniform grid. Returns (t, signals, pwm, thrust, info)."""
    from actuation import ThrustSource
    from bag_io import read_tracks, read_voltage
    from model_io import load_model

    model = load_model(model_path)
    channel = int(model.thrust.get("channel", 0))
    neutral = float(model.thrust.get("pwm_neutral", 1500))
    sign = int(model.thrust.get("sign", 1))
    delay = float(model.thrust.get("servo_delay_s", 0.0))
    source = ThrustSource(model, channel, neutral, sign)
    if not source.has(channel):
        raise SystemExit(f"{model_path.name} gives the surge channel ch{channel} no thrust "
                         "(thrust.maps or thrust.curve_file)")

    odom, servo = read_tracks(bag, ekf_topic, servo_topic)
    servo.log_t = servo.log_t - delay
    voltage = None if fixed_voltage is not None else read_voltage(bag, diag_topic)
    if source.needs_voltage and voltage is None and fixed_voltage is None:
        fixed_voltage = model.thrust.get("voltage_fallback_v")
        if fixed_voltage is None:
            raise SystemExit("thrust maps need a voltage: the bag has no BMS voltage and "
                             "thrust.voltage_fallback_v is null - pass --voltage")

    t0_bag = odom.t[0]
    lo = max(odom.t[0], servo.log_t[0])
    hi = min(odom.t[-1], servo.log_t[-1])
    if t_start is not None:
        lo = max(lo, t0_bag + t_start)
    if t_end is not None:
        hi = min(hi, t0_bag + t_end)
    if hi - lo < 10.0:
        raise SystemExit(f"only {hi - lo:.1f}s of overlapping EKF and servo data in the window")
    t = np.arange(lo, hi, 1.0 / rate_hz)

    yaw = np.unwrap(odom.yaw)
    sig = {
        "u": np.interp(t, odom.t, odom.vel_body[:, 0]),
        "v": np.interp(t, odom.t, odom.vel_body[:, 1]),
        "w": np.interp(t, odom.t, odom.vel_body[:, 2]),
        "p": np.interp(t, odom.t, odom.ang_vel[:, 0]),
        "q": np.interp(t, odom.t, odom.ang_vel[:, 1]),
        "r": np.interp(t, odom.t, odom.ang_vel[:, 2]),
        "roll": np.interp(t, odom.t, odom.roll),
        "pitch": np.interp(t, odom.t, odom.pitch),
        "yaw": np.interp(t, odom.t, yaw),
    }

    idx = np.searchsorted(servo.log_t, t, side="right") - 1
    stale_s = max(0.5, 3.0 * float(np.median(np.diff(servo.log_t))))
    valid = (idx >= 0) & (t - servo.log_t[np.clip(idx, 0, None)] <= stale_s)
    pwm = servo.pwm[np.clip(idx, 0, None)].astype(np.float64)

    if fixed_voltage is not None:
        volts = np.full(t.size, float(fixed_voltage))
    elif voltage is not None:
        volts = np.asarray(voltage.at(t), dtype=np.float64)
    else:
        volts = np.full(t.size, np.nan)
    thrust = np.array([source.force(channel, pwm[k, channel],
                                    None if np.isnan(volts[k]) else volts[k])
                       for k in range(t.size)])
    valid &= np.isfinite(thrust)

    info = {
        "version": model.version_tag(),
        "thrust": source.describe(),
        "channel": channel,
        "neutral": neutral,
        "window": (lo - t0_bag, hi - t0_bag),
        "stale_s": stale_s,
        "volts": (np.nanmin(volts), np.nanmax(volts)) if np.isfinite(volts).any() else None,
        "volts_series": volts,  # (N,) per grid sample, NaN = unknown; used by load_heave_data
    }
    return t, sig, pwm, thrust, valid, info


def load_heave_data(bag: Path, model_path: Path, rate_hz: float, t_start: float | None,
                    t_end: float | None, ekf_topic: str, servo_topic: str, diag_topic: str,
                    fixed_voltage: float | None):
    """Same grid and signals as load_surge_data, but `thrust` is the heave force: the sum of the
    thrust.allocation.heave entries in the model (vertical thrusters, PWM x voltage interpolation).
    Returns (t, sig, pwm, thrust, valid, info); sig["w"] is the heave velocity."""
    from actuation import ThrustSource, axis_force
    from model_io import load_model

    t, sig, pwm, _, valid, info = load_surge_data(
        bag, model_path, rate_hz, t_start, t_end, ekf_topic, servo_topic, diag_topic, fixed_voltage)
    model = load_model(model_path)
    entries = model.thrust.get("allocation", {}).get("heave")
    if not entries:
        raise SystemExit(f"{model_path.name} has no thrust.allocation.heave entries")
    source = ThrustSource(model, info["channel"], info["neutral"],
                          int(model.thrust.get("sign", 1)))
    thrust = axis_force(source, entries, pwm, info["volts_series"])
    valid = valid & np.isfinite(thrust)
    info = {**info, "thrust": source.describe(), "axis": "heave",
            "channels": [int(e["channel"]) for e in entries]}
    return t, sig, pwm, thrust, valid, info


# ============================================================
# CLI
# ============================================================


def main(argv=None) -> int:
    from bag_io import DEFAULT_DIAG_TOPIC, DEFAULT_EKF_TOPIC, DEFAULT_SERVO_TOPIC

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path, help="input .mcap (or rosbag2 directory)")
    ap.add_argument("--axis", default="surge", choices=["surge"],
                    help="axis to fit on its own (only surge has a known thrust curve so far)")
    ap.add_argument("--model", type=Path,
                    default=Path(__file__).resolve().parent.parent / "model.yaml",
                    help="model file; only its thrust section is used (curves, channel, sign)")
    ap.add_argument("--rate", type=float, default=20.0, help="analysis grid rate [Hz]")
    ap.add_argument("--smooth-s", type=float, default=1.0,
                    help="Savitzky-Golay window [s] for smoothing and the derivative du/dt")
    ap.add_argument("--max-lag-s", type=float, default=0.5, help="lag scan range [s]")
    ap.add_argument("--no-offset", action="store_true",
                    help="drop the constant force offset (tether / current) from the own fit")
    ap.add_argument("--t-start", type=float, default=None,
                    help="analyse from this many seconds after the first EKF message")
    ap.add_argument("--t-end", type=float, default=None,
                    help="analyse up to this many seconds after the first EKF message")
    ap.add_argument("--voltage", type=float, default=None,
                    help="fixed battery voltage [V] instead of the BMS reading")
    ap.add_argument("--ekf-topic", default=DEFAULT_EKF_TOPIC)
    ap.add_argument("--servo-topic", default=DEFAULT_SERVO_TOPIC)
    ap.add_argument("--diag-topic", default=DEFAULT_DIAG_TOPIC)
    ap.add_argument("--out", type=Path, default=None,
                    help="report file (default recordings/sysid/<stem>_coupling_<axis>.txt)")
    args = ap.parse_args(argv)

    t, sig, pwm, thrust, valid, info = load_surge_data(
        args.input, args.model, args.rate, args.t_start, args.t_end,
        args.ekf_topic, args.servo_topic, args.diag_topic, args.voltage)
    dt = 1.0 / args.rate

    # Same zero-phase filter on every signal, so no column is delayed against another.
    sm = {k: smooth(v, dt, args.smooth_s) for k, v in sig.items()}
    du = smooth(sig["u"], dt, args.smooth_s, deriv=1)
    # Bridge stale-PWM samples before filtering, then drop everything within half a filter
    # window of them, so no filtered value is built from bridged thrust.
    thrust_f = thrust.copy()
    bad = ~valid
    if bad.any():
        thrust_f[bad] = np.interp(t[bad], t[valid], thrust[valid])
    y = smooth(thrust_f, dt, args.smooth_s)
    pwm_s = np.column_stack([smooth(pwm[:, ch], dt, args.smooth_s) for ch in range(pwm.shape[1])])

    half = max(1, int(round(args.smooth_s / dt / 2)))
    near_bad = np.convolve(bad.astype(float), np.ones(2 * half + 1), mode="same") > 0
    keep = ~near_bad
    keep[:half] = False  # filter edge effects
    keep[-half:] = False
    own = {k: v[keep] for k, v in surge_terms(sm["u"], du, not args.no_offset).items()}
    cands = {k: v[keep] for k, v in
             surge_candidates(sm, pwm_s, info["neutral"], info["channel"]).items()}

    fit = own_axis_fit(y[keep], own)
    results = partial_scan(fit.residual, own, cands, dt, args.max_lag_s)

    lines = [
        f"coupling_check.py - {args.input.name} - axis {args.axis}",
        f"  {info['version']}",
        f"  thrust: {info['thrust']}"
        + (f" | V {info['volts'][0]:.2f}-{info['volts'][1]:.2f}" if info["volts"] else ""),
        f"  window {info['window'][0]:.1f}-{info['window'][1]:.1f}s after first EKF message, "
        f"{int(keep.sum())} samples at {args.rate:g} Hz, smoothing {args.smooth_s:g}s",
        "",
        f"  own-axis fit (R^2 = {fit.r2:.3f}, residual std {np.std(fit.residual):.2f} N, "
        f"thrust std {np.std(y[keep]):.2f} N):",
    ]
    for name, val in zip(fit.names, fit.theta):
        lines.append(f"    {name:<20} {val:+10.3f}")
    lines += [
        "",
        "  partial correlation of the residual with each candidate (own regressors removed):",
        f"    {'candidate':<12} {'r(lag 0)':>9} {'r(best)':>9} {'lag [s]':>8} "
        f"{'N_eff':>7} {'3/sqrt(N)':>9}  flag",
    ]
    for c in results:
        flag = "COUPLED?" if c.significant else ""
        lines.append(f"    {c.name:<12} {c.r:+9.3f} {c.r_best:+9.3f} {c.lag_s:+8.2f} "
                     f"{c.n_eff:7.0f} {c.threshold:9.3f}  {flag}")
    lines += [
        "",
        "  r^2 is the share of the residual variance that candidate alone would explain.",
        f"  Flagged = |r(lag 0)| above {Z_LAG0:g}/sqrt(N_eff), or |r(best)| above "
        f"{Z_LAGGED:g}/sqrt(N_eff).",
        "  A flag points at a coupling term to try, or at an unmodelled effect moving with it;",
        "  closed-loop bags can produce both.",
    ]
    print("\n".join(lines))

    out = args.out or Path("recordings/sysid") / f"{args.input.stem}_coupling_{args.axis}.txt"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\n[out] {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
