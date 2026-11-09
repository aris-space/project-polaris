#!/usr/bin/env python3
"""
Reproduce the numbers of the section "Bias study of the surge fit" in the estimation chapter.

Three questions about the equation-error surge fit of fitting/fit_surge.py:
  1. How much of its error on perfect data comes from the smoothing filter, and how much from
     the analysis grid rate?                                     -> data/bias_filter_rows.tex,
                                                                    data/bias_grid_rows.tex
  2. Is the IMU accelerometer a better source of du/dt than differentiating the EKF speed?
                                                                 -> data/bias_method_rows.tex,
                                                                    data/bias_leak_rows.tex
  3. How much does the servo_output_raw LOG rate matter, when the real thrust changes at the
     command time and the log only samples it?                   -> data/bias_servo_rows.tex
Plus, if recordings/zermatt_rectangle_08_*.mcap is present, the timing and noise of the real
/imu/data topic                                                  -> data/bias_real_imu.txt

Three fit variants (all least squares on T = m_tot du + X_u u + X_uu |u|u):
  A "current":    fit_surge.py as of 16 Oct 2026: SG smoothing of u and T, SG derivative for
                  du, |u|u formed from the smoothed u.
  B "consistent": one filter H on every term; |u|u formed from the raw u BEFORE filtering;
                  du = d/dt H(u) (central difference of the filtered speed).
  C "IMU":        as B, but du = H(f_x + g sin theta_0) from the synthetic accelerometer.

Synthetic bags come from scripts/sysid/simulation/synth_bag.py (steps programme, dummy T500
truth). Question 3 uses a small standalone simulation, because synth_bag lets the thrust follow
the LOGGED PWM (see the text). Takes about ten minutes.

Usage:
    python docs/sysid_notes/gen_biasstudy.py
"""
from __future__ import annotations

import sys
import tempfile
from dataclasses import replace
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
SYSID = REPO / "scripts" / "sysid"
for sub in ("", "simulation", "fitting"):
    sys.path.insert(0, str(SYSID / sub))

from bag_io import (DEFAULT_DIAG_TOPIC, DEFAULT_EKF_TOPIC, DEFAULT_SERVO_TOPIC,  # noqa: E402
                    read_tracks)
from coupling_check import load_surge_data, smooth  # noqa: E402
from model_io import load_model  # noqa: E402
from synth_bag import SynthConfig, build_program, generate  # noqa: E402
from thruster_map import ThrusterMap  # noqa: E402

OUT = HERE / "data"
TRUTH = SYSID / "dummy_bluerov2_heavy_t500.yaml"
FIT_MODEL = SYSID / "model.yaml"  # same T500 table as the truth; only its thrust section is used
_tm = load_model(TRUTH)
M_TOT = _tm.get("rigid_body.m") - _tm.get("added_mass.X_udot")
X_U, X_UU = _tm.get("damping_linear.X_u"), _tm.get("damping_quadratic.X_uu")
TRUE = np.array([M_TOT, X_U, X_UU])
G = 9.80665
THETA0 = SynthConfig().trim_pitch_rad


def pct(est: np.ndarray) -> np.ndarray:
    return 100.0 * (est - TRUE) / TRUE


def cell(x: float) -> str:
    return f"${x:+.1f}$"


def read_imu(bag: Path) -> tuple[np.ndarray, np.ndarray]:
    t, fx = [], []
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as r:
        for c, _, raw in r.messages(connections=[c for c in r.connections if c.topic == "/imu/data"]):
            m = r.deserialize(raw, c.msgtype)
            t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
            fx.append(m.linear_acceleration.x)
    return np.array(t), np.array(fx)


def lsq(cols, y, keep) -> np.ndarray:
    A = np.column_stack([c[keep] for c in cols])
    return np.linalg.lstsq(A, y[keep], rcond=None)[0]


def fits(bag: Path, W: float, rate: float, leak_deg: float = 0.0) -> dict[str, np.ndarray]:
    """Percentage errors of variants A, B, C on one bag, window W [s], grid rate [Hz]."""
    t, sig, _, T, valid, _ = load_surge_data(bag, FIT_MODEL, rate, None, None, DEFAULT_EKF_TOPIC,
                                            DEFAULT_SERVO_TOPIC, DEFAULT_DIAG_TOPIC, None)
    dt = 1.0 / rate
    bad = ~valid
    Tf = T.copy()
    if bad.any():
        Tf[bad] = np.interp(t[bad], t[valid], T[valid])
    u = sig["u"]
    half = int(round(W / dt / 2))
    keep = np.convolve(bad.astype(float), np.ones(2 * half + 1), mode="same") == 0
    keep[:half] = False
    keep[-half:] = False
    HT = smooth(Tf, dt, W)
    us = smooth(u, dt, W)
    out = {"A": pct(lsq([smooth(u, dt, W, deriv=1), us, np.abs(us) * us], HT, keep))}
    Huu = smooth(np.abs(u) * u, dt, W)
    out["B"] = pct(lsq([np.gradient(us, dt), us, Huu], HT, keep))
    ti, fx = read_imu(bag)
    a = np.interp(t, ti, fx + G * np.sin(THETA0))
    if leak_deg:  # pitch error proportional to thrust, leak_deg at the largest thrust
        a = a + G * np.sin(np.radians(leak_deg) * Tf / np.max(np.abs(Tf)))
    out["C"] = pct(lsq([smooth(a, dt, W), us, Huu], HT, keep))
    return out


# ------------------------------------------------------------ question 3: servo log rate
_TMAP = ThrusterMap.from_csv(SYSID / "thruster_curves" / "t500_force_vs_voltage.csv")


def servo_rate_fit(f_log: float | None, seed: int, W: float = 0.5, rate: float = 100.0,
                   volts: float = 16.0) -> np.ndarray:
    """Ideal surge run whose thrust switches at the COMMAND time; the PWM is logged at f_log
    with a random phase (None = edges known exactly). Fit variant B on a `rate` grid."""
    rng = np.random.default_rng(seed)
    h = 1e-3
    edges, levels, t = [], [], 5.0 + rng.uniform(0, 1)
    for c in (200, 300, 400, 500):
        edges += [t, t + 10.0]
        levels += [1500.0 + 0.4 * c, 1500.0]
        t += 25.0 + rng.uniform(0, 1)
    force = {p: _TMAP.force_n(p, volts) for p in set(levels) | {1500.0}}
    tt = np.arange(0.0, t, h)
    pwm = np.full(tt.size, 1500.0)
    for e, p in zip(edges, levels):
        pwm[tt >= e] = p
    thrust = np.vectorize(force.get)(pwm)
    u = np.zeros(tt.size)
    for k in range(tt.size - 1):
        f = lambda x, T=thrust[k]: (T - X_U * x - X_UU * abs(x) * x) / M_TOT  # noqa: E731
        k1 = f(u[k]); k2 = f(u[k] + h / 2 * k1); k3 = f(u[k] + h / 2 * k2); k4 = f(u[k] + h * k3)
        u[k + 1] = u[k] + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    g = np.arange(1.0, tt[-1] - 1.0, 1.0 / rate)
    t_ekf = np.arange(0.0, tt[-1], 1.0 / 30.0)
    ug = np.interp(g, t_ekf, np.interp(t_ekf, tt, u))
    if f_log:
        tl = np.arange(rng.uniform(0, 1.0 / f_log), tt[-1], 1.0 / f_log)
        pl = pwm[np.searchsorted(tt, tl)]
        pg = pl[np.clip(np.searchsorted(tl, g, side="right") - 1, 0, None)]
    else:
        pg = pwm[np.searchsorted(tt, g)]
    Tg = np.vectorize(force.get)(pg)
    dt = 1.0 / rate
    half = int(W / dt / 2)
    Hu = smooth(ug, dt, W)
    keep = np.zeros(g.size, bool)
    keep[half:-half] = True
    return pct(lsq([np.gradient(Hu, dt), Hu, smooth(np.abs(ug) * ug, dt, W)], smooth(Tg, dt, W), keep))


# ------------------------------------------------------------ real IMU
def real_imu_summary(bag: Path) -> str:
    from scipy.signal import welch
    t, fx = read_imu(bag)
    d = np.diff(t)
    span = t[-1] - t[0]
    fs = t.size / span
    f, P = welch(fx - fx.mean(), fs=fs, nperseg=2048)
    bands = []
    for lo, hi in ((0, 1), (1, 5), (5, fs / 2)):
        k = (f >= lo) & (f < hi)
        bands.append(f"{lo:g}-{hi:.0f} Hz: {np.sqrt(np.trapz(P[k], f[k])):.3f} m/s^2")
    odom, servo = read_tracks(bag)
    u = odom.vel_body[:, 0]
    k = int(round(1.0 / np.median(np.diff(odom.t))))
    du = np.gradient(np.convolve(u, np.ones(k) / k, mode="same"), odom.t)
    on = np.abs(np.interp(odom.t, servo.log_t, servo.pwm[:, 0]) - 1500) > 50
    return "\n".join([
        f"bag: {bag.name}",
        f"imu samples {t.size} over {span:.1f} s -> mean rate {fs:.1f} Hz",
        f"header-stamp spacing: median {np.median(d) * 1e3:.3f} ms, 95th pct {np.percentile(d, 95) * 1e3:.1f} ms, "
        f"max {d.max() * 1e3:.0f} ms",
        "f_x rms per band: " + "; ".join(bands),
        f"servo_output_raw rate {servo.log_t.size / (servo.log_t[-1] - servo.log_t[0]):.1f} Hz",
        f"std of EKF du/dt (1 s mean) while thrusting: {du[on].std():.3f} m/s^2",
    ]) + "\n"


def main() -> int:
    OUT.mkdir(exist_ok=True)
    steps = build_program("steps")
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        ideal, *_ = generate(TRUTH, steps, SynthConfig.ideal(), tmp / "ideal.mcap")
        noiseonly, *_ = generate(TRUTH, steps, replace(SynthConfig(), actuation_lag_s=0.0,
                                 spinup_tau_s=0.0, ekf_lag_s=0.0), tmp / "noiseonly.mcap")
        default, *_ = generate(TRUTH, steps, SynthConfig(), tmp / "default.mcap")

        # 1a. filter mechanism, fine grid so the grid-rate effect is negligible
        rows = []
        for W in (0.25, 0.5, 1.0, 2.0):
            r = fits(ideal, W, 200.0)
            rows.append(f"{W:g} & " + " & ".join(cell(r[v][i]) for v in "AB" for i in (0, 1, 2)) + r" \\")
        (OUT / "bias_filter_rows.tex").write_text("\n".join(rows) + "\n", encoding="utf-8")
        print("filter:\n" + "\n".join(rows))

        # 1b. grid rate, variants A and B, W = 1 s
        rows = []
        for rate in (20.0, 50.0, 100.0, 200.0):
            r = fits(ideal, 1.0, rate)
            rows.append(f"{rate:g} & {cell(r['A'][0])} & {cell(r['A'][1])} & {cell(r['B'][0])} & "
                        f"{cell(r['B'][1])} & {cell(r['C'][0])} \\\\")
        (OUT / "bias_grid_rows.tex").write_text("\n".join(rows) + "\n", encoding="utf-8")
        print("grid:\n" + "\n".join(rows))

        # 2a. methods on the three bags, 100 Hz grid
        rows = []
        for name, bag in (("ideal", ideal), ("noise only", noiseonly), ("all defaults", default)):
            for W in (0.25, 0.5, 1.0):
                r = fits(bag, W, 100.0)
                rows.append(f"{name} & {W:g} & " + " & ".join(
                    f"{cell(r[v][0])} & {cell(r[v][1])}" for v in "ABC") + r" \\")
        (OUT / "bias_method_rows.tex").write_text("\n".join(rows) + "\n", encoding="utf-8")
        print("methods:\n" + "\n".join(rows))

        # 2b. thrust-correlated pitch error leaking gravity into the IMU
        rows = []
        for leak in (0.0, 0.25, 0.5, 1.0):
            r = fits(ideal, 0.5, 100.0, leak_deg=leak)["C"]
            rows.append(f"{leak:g} & {G * np.sin(np.radians(leak)):.3f} & {cell(r[0])} & "
                        f"{cell(r[1])} & {cell(r[2])} \\\\")
        (OUT / "bias_leak_rows.tex").write_text("\n".join(rows) + "\n", encoding="utf-8")
        print("leak:\n" + "\n".join(rows))

    # 3. servo log rate
    rows = []
    for f_log in (2.0, 5.0, 10.0, 20.0, 50.0, 100.0, None):
        e = np.array([servo_rate_fit(f_log, s) for s in range(12)])
        lab = f"{f_log:g}" if f_log else "exact edges"
        rows.append(f"{lab} & {cell(e[:, 0].mean())} & ${e[:, 0].min():+.1f}$ to ${e[:, 0].max():+.1f}$ & "
                    f"{cell(e[:, 1].mean())} & {cell(e[:, 2].mean())} \\\\")
    (OUT / "bias_servo_rows.tex").write_text("\n".join(rows) + "\n", encoding="utf-8")
    print("servo:\n" + "\n".join(rows))

    real = sorted((REPO / "recordings").glob("zermatt_rectangle_08_*.mcap"))
    if real:
        text = real_imu_summary(real[0])
        (OUT / "bias_real_imu.txt").write_text(text, encoding="utf-8")
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
