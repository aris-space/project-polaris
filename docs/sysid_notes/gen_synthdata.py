#!/usr/bin/env python3
"""
Reproduce every number and plot of the "Synthetic test bench" chapter of the sysid notes.

Generates synthetic bags with scripts/sysid/simulation/synth_bag.py into a temporary directory,
runs the repository's own propagation and coupling_check fit on them, and writes:
  data/synth_step.dat       truth, EKF and propagated surge speed around one step
  data/synth_prop_rows.tex  table rows: propagation RMS for lag x servo_delay_s combinations
  data/synth_fit_rows.tex   table rows: coupling_check fit on bags with known imperfections
  data/synth_win_rows.tex   table rows: same fit against the smoothing window
  data/synth_comp_rows.tex  table rows: same fit with the delay compensated

Needs rosbags (pip install rosbags numpy scipy pyyaml). Takes a few minutes.

Usage:
    python docs/sysid_notes/gen_synthdata.py
"""
from __future__ import annotations

import re
import sys
import tempfile
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SYSID = HERE.parent.parent / "scripts" / "sysid"
for sub in ("", "simulation", "validation", "fitting"):
    sys.path.insert(0, str(SYSID / sub))

from actuation import ThrustSource  # noqa: E402
from bag_io import read_tracks, read_voltage  # noqa: E402
from coupling_check import main as coupling_main  # noqa: E402
from dynamics import SurgeParams  # noqa: E402
from model_io import load_model  # noqa: E402
from propagate_model import default_pwm_stale_s, propagate  # noqa: E402
from synth_bag import SynthConfig, build_program, generate  # noqa: E402

OUT = HERE / "data"
TRUTH = SYSID / "dummy_bluerov2_heavy_t500.yaml"
TRUTH_MODEL = load_model(TRUTH)
M_TOT = TRUTH_MODEL.get("rigid_body.m") - TRUTH_MODEL.get("added_mass.X_udot")
X_U = TRUTH_MODEL.get("damping_linear.X_u")
X_UU = TRUTH_MODEL.get("damping_quadratic.X_uu")
PARAMS = SurgeParams(M_TOT, X_U, X_UU)


def propagated(bag: Path, servo_delay_s: float):
    """Free-run propagation of the truth model; returns (odom, model surge)."""
    source = ThrustSource(TRUTH_MODEL, 0, 1500.0, 1)
    odom, servo = read_tracks(bag)
    volt = read_voltage(bag)
    servo.log_t = servo.log_t - servo_delay_s
    lo, hi = max(odom.t[0], servo.log_t[0]), min(odom.t[-1], servo.log_t[-1])
    keep = (odom.t >= lo) & (odom.t <= hi)
    for name in ("t", "log_t", "pos", "yaw", "vel_body", "yaw_rate", "ang_vel", "roll", "pitch"):
        setattr(odom, name, getattr(odom, name)[keep])
    out = propagate(odom, servo, source, lambda t: float(volt.at(t)), PARAMS, channel=0,
                    neutral=1500.0, sign=1, reinit_s=0.0, dt=0.01, with_baseline=False,
                    pwm_stale_s=default_pwm_stale_s(servo.log_t))
    return odom, out[2]


def fit(bag: Path, model: Path, tmp: Path, smooth_s: float = 1.0) -> tuple[float, float, float]:
    """coupling_check's own-axis fit, read back from its report file."""
    report = tmp / "cc.txt"
    coupling_main([str(bag), "--model", str(model), "--smooth-s", str(smooth_s),
                   "--rate", "50", "--out", str(report)])
    text = report.read_text(encoding="utf-8")
    val = {k: float(re.search(rf"{re.escape(k)}\s+([+-][\d.]+)", text).group(1))
           for k in ("m_tot [kg]", "X_u [N s/m]", "X_uu [N s^2/m^2]")}
    return val["m_tot [kg]"], val["X_u [N s/m]"], val["X_uu [N s^2/m^2]"]


def pct(x: float, ref: float) -> str:
    dev = round(100 * (x / ref - 1)) or 0  # avoid "-0"
    return f"{x:.2f} ({dev:+d}\\,\\%)"


def main() -> int:
    OUT.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        ideal = SynthConfig.ideal()
        steps = build_program("steps")
        suite = build_program("suite")

        # 1. Propagation against actuation lag and servo_delay_s.
        rows = []
        for lag in (0.0, 0.2):
            bag, *_ = generate(TRUTH, steps, replace(ideal, actuation_lag_s=lag),
                               tmp / f"prop_lag{lag}.mcap")
            for d in (0.0, -0.2, 0.2):
                odom, u = propagated(bag, d)
                rms = float(np.sqrt(np.mean((u - odom.vel_body[:, 0]) ** 2)))
                rows.append(f"{lag:.1f} & {d:+.1f} & \\num{{{rms:.1e}}} \\\\")
        (OUT / "synth_prop_rows.tex").write_text("\n".join(rows) + "\n", encoding="utf-8")

        # 2. Truth vs propagation around the surge-500 step, default imperfections.
        cfg = SynthConfig()
        bag, _, truth, _ = generate(TRUTH, suite, cfg, tmp / "default.mcap")
        on = next(t for t, p in truth.events
                  if p["event"] == "thrust_on" and p["surge"] == 500)
        odom, u0 = propagated(bag, 0.0)
        odom1, u1 = propagated(bag, -cfg.actuation_lag_s)  # trimmed to a shifted overlap
        win = (odom.t >= on - 0.5) & (odom.t <= on + 2.5)
        t = odom.t[win]
        np.savetxt(OUT / "synth_step.dat", np.column_stack([
            t - on, np.interp(t, truth.t, truth.u), odom.vel_body[win, 0],
            u0[win], np.interp(t, odom1.t, u1)]),
            header="t truth ekf delay0 delaycomp", comments="", fmt="%.6g")

        # 3. Equation-error fit on bags with one known imperfection each.
        cases = [
            ("ideal", ideal),
            (r"actuation lag \qty{0.15}{\second}", replace(ideal, actuation_lag_s=0.15)),
            (r"EKF lag \qty{0.1}{\second}", replace(ideal, ekf_lag_s=0.1)),
            ("true thrust \\qty{15}{\\percent} above table", replace(ideal, thrust_scale=1.15)),
            ("all defaults", SynthConfig()),
        ]
        bags = {}
        rows = []
        for label, c in cases:
            bag, *_ = generate(TRUTH, suite, c, tmp / f"fit_{len(bags)}.mcap")
            bags[label] = bag
            m, xu, xuu = fit(bag, TRUTH, tmp)
            rows.append(f"{label} & {pct(m, M_TOT)} & {pct(xu, X_U)} & {pct(xuu, X_UU)} \\\\")
        (OUT / "synth_fit_rows.tex").write_text("\n".join(rows) + "\n", encoding="utf-8")

        # 4. Smoothing window.
        rows = []
        for w in (1.0, 0.5, 0.25, 0.15):
            cells = []
            for label in ("ideal", r"actuation lag \qty{0.15}{\second}", "all defaults"):
                m, _, _ = fit(bags[label], TRUTH, tmp, smooth_s=w)
                cells.append(pct(m, M_TOT))
            rows.append(f"{w:g} & " + " & ".join(cells) + " \\\\")
        (OUT / "synth_win_rows.tex").write_text("\n".join(rows) + "\n", encoding="utf-8")

        # 5. Delay compensation in the fit: a copy of the truth model with servo_delay_s set.
        doc = yaml.safe_load(TRUTH.read_text(encoding="utf-8"))
        doc["thrust"]["maps"] = {0: str((SYSID / "thruster_curves" /
                                         "t500_force_vs_voltage.csv").resolve())}
        rows = []
        for d in (0.0, -0.15, 0.15):
            doc["thrust"]["servo_delay_s"] = d
            mpath = tmp / f"model_{d:+.2f}.yaml"
            mpath.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
            cells = [pct(fit(bags[r"actuation lag \qty{0.15}{\second}"], mpath, tmp, w)[0],
                         M_TOT) for w in (1.0, 0.25)]
            rows.append(f"{d:+.2f} & " + " & ".join(cells) + " \\\\")
        (OUT / "synth_comp_rows.tex").write_text("\n".join(rows) + "\n", encoding="utf-8")

    for f in sorted(OUT.glob("synth_*")):
        print(f"[out] {f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
