#!/usr/bin/env python3
"""
Generate the plot data (.dat tables) for the sysid lecture notes, using the repo's own sysid
modules so every curve in the notes is what the code actually computes.

Coefficients are the DUMMY BlueROV2 Heavy values (dummy_bluerov2_heavy.yaml), never POLARIS.

Usage:
    python docs/sysid_notes/gen_figdata.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SYSID = HERE.parent.parent / "scripts" / "sysid"
sys.path.insert(0, str(SYSID))
sys.path.insert(0, str(SYSID / "fitting"))

from coupling_check import effective_n, own_axis_fit, partial_scan, smooth, surge_terms  # noqa: E402
from dynamics import SurgeParams, rk4_step, steady_state_speed, surge_accel  # noqa: E402
from thruster_map import ThrusterMap  # noqa: E402

OUT = HERE / "data"
OUT.mkdir(exist_ok=True)

P = SurgeParams(m_tot=13.5 + 6.36, X_u=13.7, X_uu=141.0)  # dummy BlueROV2 Heavy


def save(name: str, header: str, cols) -> None:
    arr = np.column_stack(cols)
    np.savetxt(OUT / name, arr, header=header, comments="", fmt="%.6g")


def roll(u0, thrust_fn, p, t_end, dt=0.01):
    t = np.arange(0.0, t_end + 1e-12, dt)
    u = np.empty_like(t)
    u[0] = u0
    for k in range(1, t.size):
        u[k] = rk4_step(u[k - 1], thrust_fn(t[k - 1]), dt, p)
    return t, u


# 1. T500 thrust table ------------------------------------------------------------------
tmap = ThrusterMap.from_csv(SYSID / "thruster_curves" / "t500_force_vs_voltage.csv")
pwm = np.array(tmap.pwm_us)
save("t500.dat", "pwm f12 f14 f16 f18",
     [pwm] + [np.array([tmap.force_n(x, v) for x in pwm]) for v in tmap.volts])
v_sweep = np.linspace(11, 19, 81)
save("t500_volt.dat", "v f1700 f1800 f1900",
     [v_sweep] + [np.array([tmap.force_n(x, v) for v in v_sweep]) for x in (1700, 1800, 1900)])

# 2. Step responses at several thrust levels -------------------------------------------
cols, hdr = [], ["t"]
for T in (5.0, 10.0, 20.0, 40.0):
    t, u = roll(0.0, lambda _t, T=T: T, P, 3.0)
    if not cols:
        cols.append(t)
    cols.append(u)
    hdr.append(f"T{int(T)}")
save("step.dat", " ".join(hdr), cols)

# 3. Drag curve and effective time constant -------------------------------------------
u = np.linspace(0, 0.8, 81)
save("drag.dat", "u lin quad tot tau",
     [u, P.X_u * u, P.X_uu * u**2, P.X_u * u + P.X_uu * u**2,
      P.m_tot / (P.X_u + 2 * P.X_uu * u)])

# 4. Coast-down: linear only, quadratic only, both --------------------------------------
u0 = 0.5
cases = {
    "lin": SurgeParams(P.m_tot, P.X_u, 0.0),
    "quad": SurgeParams(P.m_tot, 1e-12, P.X_uu),
    "both": P,
}
cols = []
for key, p in cases.items():
    t, uu = roll(u0, lambda _t: 0.0, p, 6.0)
    if not cols:
        cols.append(t)
    cols.append(uu)
cols.append(1.0 / cols[2])  # 1/u for quadratic only: linear in t
cols.append(np.log(cols[1]))  # ln u for linear only: linear in t
save("coast.dat", "t lin quad both invquad lnlin", cols)

# 5. Integrator convergence: Euler vs RK4 on quadratic decay with closed form -----------
pq = SurgeParams(P.m_tot, 0.0, P.X_uu)
t_end = 2.0
exact = u0 / (1 + pq.X_uu * u0 * t_end / pq.m_tot)
hs = np.array([0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001])
e_eu, e_rk = [], []
for h in hs:
    n = int(round(t_end / h))
    ue = ur = u0
    for _ in range(n):
        ue = ue + h * surge_accel(ue, 0.0, pq)
        ur = rk4_step(ur, 0.0, h, pq)
    e_eu.append(abs(ue - exact))
    e_rk.append(max(abs(ur - exact), 1e-16))
save("conv.dat", "h euler rk4", [hs, e_eu, e_rk])

# 6. Delay bias on the initial acceleration ---------------------------------------------
T = 20.0
t, u_true = roll(0.0, lambda _t: T, P, 2.0, dt=0.005)
d = 0.2
u_del = np.interp(t - d, t, u_true, left=0.0)
save("delay.dat", "t true delayed", [t, u_true, u_del])
windows = np.linspace(0.25, 1.0, 31)
bias, bias0 = [], []
for W in windows:
    m = t <= W
    slope_true = np.polyfit(t[m], u_true[m], 1)[0]
    slope_del = np.polyfit(t[m], u_del[m], 1)[0]
    bias0.append((T / slope_true) / P.m_tot)
    bias.append((T / slope_del) / P.m_tot)
bias = np.array(bias)
save("delay_bias.dat", "W nodelay delayed", [windows, np.array(bias0), bias])

# 7. Pitch free decay with the dummy coefficients ---------------------------------------
I_tot = 0.23 + 0.135
K, Mq, Mqq = 1.316, 0.8, 0.47
th, q = 0.35, 0.0
h = 0.002
ts, ths = [0.0], [th]


def f(x):
    th_, q_ = x
    return np.array([q_, (-K * th_ - Mq * q_ - Mqq * abs(q_) * q_) / I_tot])


x = np.array([th, q])
for k in range(int(8.0 / h)):
    k1 = f(x)
    k2 = f(x + 0.5 * h * k1)
    k3 = f(x + 0.5 * h * k2)
    k4 = f(x + h * k3)
    x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    ts.append((k + 1) * h)
    ths.append(x[0])
# Same vehicle with ten times less linear damping, to show visible oscillation.
x = np.array([th, q])
ths2 = [th]
for k in range(int(8.0 / h)):
    def g(y):
        return np.array([y[1], (-K * y[0] - 0.08 * y[1] - Mqq * abs(y[1]) * y[1]) / I_tot])
    k1 = g(x)
    k2 = g(x + 0.5 * h * k1)
    k3 = g(x + 0.5 * h * k2)
    k4 = g(x + h * k3)
    x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    ths2.append(x[0])
ts = np.array(ts)
save("decay.dat", "t dummy lowdamp", [ts[::10], np.array(ths)[::10], np.array(ths2)[::10]])

# 8. Error vs horizon for two kinds of model error --------------------------------------
rng = np.random.default_rng(3)
dt = 0.02
n = int(600 / dt)
# Piecewise-constant thrust profile (new level every 4-8 s), like a sequence of steps.
thrust = np.zeros(n)
k = 0
while k < n:
    L = int(rng.uniform(4, 8) / dt)
    thrust[k:k + L] = rng.uniform(-10, 30)
    k += L
u_true = np.zeros(n)
for k in range(1, n):
    u_true[k] = rk4_step(u_true[k - 1], thrust[k - 1], dt, P)
variants = {
    "thrust": (SurgeParams(P.m_tot, P.X_u, P.X_uu), 0.85),  # thrust curve 15 % low
    "mass": (SurgeParams(P.m_tot * 1.5, P.X_u, P.X_uu), 1.0),  # inertia 50 % high
}
H = 10.0
nh = int(H / dt)
hor = np.arange(nh) * dt
cols = [hor]
for key, (pv, scale) in variants.items():
    errs = []
    for s in range(0, n - nh, int(2.0 / dt)):
        um, x_err = u_true[s], 0.0
        e = np.zeros(nh)
        for j in range(1, nh):
            um = rk4_step(um, scale * thrust[s + j - 1], dt, pv)
            x_err += dt * (um - u_true[s + j])
            e[j] = x_err
        errs.append(e)
    cols.append(np.sqrt(np.mean(np.array(errs) ** 2, axis=0)))
save("horizon.dat", "h thrust mass", [c[::5] for c in cols])

# 9. Coupling check on the synthetic data of test_coupling_check.py ----------------------
DT = 0.05
M_TOT, X_U, X_UU, OFFSET = 36.0, 8.0, 150.0, 2.0


def synthetic(kc, seed=1):
    r_ = np.random.default_rng(seed)
    nn = 6000
    tt = np.arange(nn) * DT
    sm = lambda sc, w: sc * smooth(r_.standard_normal(nn), DT, w, poly=2)  # noqa: E731
    uu = 0.25 + 0.15 * np.sin(2 * np.pi * tt / 40.0) + sm(0.6, 3.0)
    v, r, w, qq = sm(0.4, 4.0), sm(1.5, 4.0), sm(0.3, 4.0), sm(0.8, 4.0)
    du = np.gradient(uu, DT)
    F = M_TOT * du + X_U * uu + X_UU * np.abs(uu) * uu + OFFSET + kc * v * r
    F = F + 0.3 * r_.standard_normal(nn)
    return F, du, dict(u=uu, v=v, r=r, w=w, q=qq)


lines = []
for kc in (0.0, -20.0):
    F, du, s = synthetic(kc)
    own = surge_terms(s["u"], du, with_offset=True)
    fit = own_axis_fit(F, own)
    cands = {"v*r": s["v"] * s["r"], "w*q": s["w"] * s["q"], "v": s["v"], "r": s["r"],
             "w": s["w"], "q": s["q"]}
    res = partial_scan(fit.residual, own, cands, DT)
    lines.append(f"% coupling k = {kc}")
    lines.append("% fit " + " ".join(f"{nm}={th:.4g}" for nm, th in zip(fit.names, fit.theta))
                 + f" r2={fit.r2:.5f}")
    for c in res:
        lines.append(f"% {c.name} r0={c.r:+.3f} rbest={c.r_best:+.3f} lag={c.lag_s:+.2f} "
                     f"neff={c.n_eff:.0f} thr={c.threshold:.3f} sig={c.significant}")
white = np.random.default_rng(0).standard_normal(4000)
slow = smooth(np.random.default_rng(1).standard_normal(4000), DT, 4.0)
lines.append(f"% neff white={effective_n(white, np.random.default_rng(2).standard_normal(4000)):.0f}"
             f" slow={effective_n(slow, smooth(np.random.default_rng(4).standard_normal(4000), DT, 4.0)):.0f}")
(OUT / "coupling_results.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
print("\n".join(lines))

# Key numbers quoted in the text.
print(f"tau(0)={P.m_tot / P.X_u:.3f}s")
for T in (5, 10, 20, 40):
    uss = steady_state_speed(T, P)
    print(f"T={T}N u_ss={uss:.3f} tau_eff={P.m_tot / (P.X_u + 2 * P.X_uu * uss):.3f}s")
print("delay bias", dict(zip(windows, np.round(bias, 3))))
print("decay omega_n", np.sqrt(K / I_tot), "zeta", Mq / (2 * np.sqrt(K * I_tot)))
print("conv", list(zip(hs, e_eu, e_rk)))
