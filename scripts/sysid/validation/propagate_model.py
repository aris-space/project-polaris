#!/usr/bin/env python3
"""
Propagate the identified POLARIS surge model over a recorded bag and write a new bag that
contains every original message plus the model prediction on /sysid/* topics.

Offline only: no ROS installation, no `ros2 bag play`, no live node. The output bag is meant
to be opened in Foxglove and plotted against the EKF.

What it does:
  1. loads the model (model.yaml) and refuses to run while a needed coefficient is null;
  2. reads the EKF odometry and the raw servo PWM, and verifies that their two time bases
     are monotonic, overlapping and free of large gaps (reported, never silently fixed);
  3. integrates (m - X_udot) du/dt = T(pwm, V) - X_u*u - X_uu*|u|*u over the EKF grid, taking
     heading, sway and depth from the EKF, re-initialising from the EKF every --reinit-s.
     T comes from the channel's PWM x battery-voltage table (thrust.maps, voltage from the
     BMS on /diagnostics) or, failing that, from the single-voltage thrust.curve_file;
  4. writes the augmented bag plus a <stem>_summary.txt with RMS errors per horizon bin. The
     bag also carries /sysid/thruster_forces (all six channels, NaN where no curve exists yet)
     and /sysid/battery_voltage, so the thrust input itself can be checked in Foxglove.

Dependencies: pip install rosbags numpy scipy pyyaml pyproj

Example:
    python scripts/sysid/validation/propagate_model.py \\
        recordings/zermatt_rectangle_07_2026_04_29-13_38_44_0.mcap \\
        --model scripts/sysid/dummy_bluerov2_heavy.yaml --reinit-s 10
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bag_io import (  # noqa: E402
    DEFAULT_DIAG_TOPIC,
    DEFAULT_EKF_TOPIC,
    DEFAULT_SERVO_TOPIC,
    OutputSeries,
    fit_geo_anchor,
    read_tracks,
    read_voltage,
    verify_timestamps,
    write_augmented_bag,
)
from dynamics import (  # noqa: E402
    SurgeParams,
    ThrusterCurve,
    powerlaw_speed,
    rk4_step,
    steady_state_speed,
)
from model_io import load_model  # noqa: E402
from thruster_map import ThrusterMap  # noqa: E402

N_SERVO_CHANNELS = 6

REQUIRED = [
    "rigid_body.m",
    "added_mass.X_udot",
    "damping_linear.X_u",
    "damping_quadratic.X_uu",
]


def default_pwm_stale_s(servo_log_t: np.ndarray) -> float:
    """Age at which a held PWM sample counts as a dropout rather than normal zero-order hold.

    The servo topic is published at anywhere from 2 Hz (rectangle bags) to 20 Hz (grid bags),
    so a fixed threshold either flags every normal interval or misses real dropouts. Scale it
    off the observed median interval instead.
    """
    dt = np.diff(servo_log_t)
    med = float(np.median(dt)) if dt.size else 0.5
    return max(0.5, 3.0 * med)


def _mirror(pwm: float, neutral: float, sign: int) -> float:
    """Mirror the PWM about neutral when the thruster sign is flipped.

    Mirroring rather than negating the force keeps an asymmetric forward/reverse curve correct.
    """
    return neutral + sign * (pwm - neutral)


class ThrustSource:
    """Force per servo channel from the held PWM and the battery voltage.

    A channel listed in thrust.maps uses its PWM x voltage table (ThrusterMap). Otherwise the
    surge channel falls back to the single-voltage thrust.curve_file (ThrusterCurve), which is
    what the dummy model uses. Every other channel has no curve yet and returns NaN, so a
    missing curve can never pass as zero thrust. Only the surge channel's sign is configured
    (thrust.sign); other channels are evaluated as tabulated until theirs are known.
    """

    def __init__(self, model, surge_channel: int, neutral: float, sign: int) -> None:
        self.surge_channel = surge_channel
        self.neutral = neutral
        self.sign = sign
        self.map_paths = model.thrust_maps()
        self.maps = {ch: ThrusterMap.from_csv(p) for ch, p in self.map_paths.items()}
        self.curve = None
        self.curve_name = None
        if surge_channel not in self.maps and model.thrust.get("curve_file"):
            self.curve = ThrusterCurve.from_csv(model.curve_path())
            self.curve_name = model.curve_path().name

    @property
    def needs_voltage(self) -> bool:
        return bool(self.maps)

    def has(self, ch: int) -> bool:
        return ch in self.maps or (ch == self.surge_channel and self.curve is not None)

    def force(self, ch: int, pwm: float, volts: float | None) -> float:
        p = _mirror(pwm, self.neutral, self.sign) if ch == self.surge_channel else pwm
        if ch in self.maps:
            return self.maps[ch].force_n(p, volts)
        if ch == self.surge_channel and self.curve is not None:
            return float(self.curve.force_n(p))
        return float("nan")

    def describe(self) -> str:
        parts = [f"ch{ch} map {self.map_paths[ch].name} "
                 f"({m.volts[0]:g}-{m.volts[-1]:g} V)" for ch, m in sorted(self.maps.items())]
        if self.curve is not None:
            parts.append(f"ch{self.surge_channel} curve {self.curve_name}")
        unknown = [ch for ch in range(N_SERVO_CHANNELS) if not self.has(ch)]
        if unknown:
            parts.append("unknown " + ",".join(f"ch{ch}" for ch in unknown))
        return "; ".join(parts)


def forces_on_grid(t_grid, servo, source: ThrustSource, volt_at, pwm_stale_s: float):
    """Force of every servo channel at each grid time, from the held PWM. NaN where the channel
    has no curve or the PWM sample is stale."""
    idx = np.searchsorted(servo.log_t, t_grid, side="right") - 1
    out = np.full((t_grid.size, N_SERVO_CHANNELS), np.nan)
    known = [ch for ch in range(N_SERVO_CHANNELS) if source.has(ch)]
    for k, i in enumerate(idx):
        if i < 0 or t_grid[k] - servo.log_t[i] > pwm_stale_s:
            continue
        v = volt_at(t_grid[k])
        for ch in known:
            out[k, ch] = source.force(ch, float(servo.pwm[i, ch]), v)
    return out


def propagate(odom, servo, source: ThrustSource, volt_at, params, *, channel, neutral, sign,
              reinit_s, dt, with_baseline, pwm_stale_s, zero_sway=False):
    """Integrate the model over the EKF grid. Returns arrays on that grid plus a gap count."""
    t_grid = odom.t
    yaw_unwrapped = np.unwrap(odom.yaw)
    # Sway is normally taken from the EKF, which means the predicted path keeps moving even
    # when the modelled surge is zero. Zeroing it isolates the modelled axis.
    sway = np.zeros_like(odom.vel_body[:, 1]) if zero_sway else odom.vel_body[:, 1]
    pwm_ch = servo.pwm[:, channel]

    n = t_grid.size
    model_pos = np.zeros((n, 3))
    model_yaw = np.zeros(n)
    model_surge = np.zeros(n)
    horizon = np.zeros(n)
    base_pos = np.zeros((n, 3)) if with_baseline else None
    base_surge = np.zeros(n) if with_baseline else None

    def pwm_at(t: float) -> tuple[float, bool]:
        """Zero-order hold. Returns (pwm, stale) - stale means the last sample is too old."""
        i = int(np.searchsorted(servo.log_t, t, side="right")) - 1
        if i < 0:
            return neutral, True
        return float(pwm_ch[i]), (t - servo.log_t[i]) > pwm_stale_s

    def reinit(k: int):
        return (
            np.array([odom.pos[k, 0], odom.pos[k, 1], odom.pos[k, 2]]),
            float(odom.vel_body[k, 0]),
        )

    pos, u = reinit(0)
    base_p = pos.copy()
    last_reinit_t = t_grid[0]
    gaps = 0

    for k in range(n):
        t = t_grid[k]
        if k > 0:
            t_prev = t_grid[k - 1]
            span = t - t_prev
            # March from the previous grid point to this one at the internal step size.
            n_sub = max(1, int(np.ceil(span / dt)))
            h = span / n_sub
            stale_here = False
            for j in range(n_sub):
                t_sub = t_prev + j * h
                pwm, stale = pwm_at(t_sub)
                stale_here |= stale
                thrust = source.force(channel, pwm, volt_at(t_sub))
                yaw_sub = float(np.interp(t_sub, t_grid, yaw_unwrapped))
                v_sub = float(np.interp(t_sub, t_grid, sway))
                c, s = np.cos(yaw_sub), np.sin(yaw_sub)
                pos[0] += h * (c * u - s * v_sub)
                pos[1] += h * (s * u + c * v_sub)
                u = rk4_step(u, thrust, h, params)
                if with_baseline:
                    u_b = float(powerlaw_speed(neutral + sign * (pwm - neutral), neutral))
                    base_p[0] += h * (c * u_b - s * v_sub)
                    base_p[1] += h * (s * u_b + c * v_sub)

            if stale_here:
                # No fresh PWM across this span: thrust is unknown, not zero. Restart.
                gaps += 1
                pos, u = reinit(k)
                base_p = pos.copy()
                last_reinit_t = t

        if t - last_reinit_t >= reinit_s > 0.0:
            pos, u = reinit(k)
            base_p = pos.copy()
            last_reinit_t = t

        pos[2] = odom.pos[k, 2]  # heave is measured, not modelled
        model_pos[k] = pos
        model_yaw[k] = odom.yaw[k]
        model_surge[k] = u
        horizon[k] = t - last_reinit_t
        if with_baseline:
            base_p[2] = odom.pos[k, 2]
            base_pos[k] = base_p
            pwm_now, _ = pwm_at(t)
            base_surge[k] = float(powerlaw_speed(neutral + sign * (pwm_now - neutral), neutral))

    return model_pos, model_yaw, model_surge, horizon, base_pos, base_surge, gaps


def _path_length(pos: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1).sum())


def summarize(label: str, err_pos, err_surge, pred_pos, ekf_pos, ekf_surge) -> list[str]:
    """One compact block per predictor.

    Surge error is the direct test of a surge-only model; position error is its integral and
    also absorbs heading and run length. The surge figure is printed next to the EKF's own
    surge spread, because an RMS error equal to that spread means the prediction explains
    none of the measured motion.
    """
    travelled = _path_length(ekf_pos)
    rms_surge = float(np.sqrt(np.mean(err_surge**2)))
    spread = float(np.std(ekf_surge))
    drift = float(err_pos[-1])
    return [
        f"  {label}",
        f"    surge RMS error    : {rms_surge:7.3f} m/s   "
        f"({100*rms_surge/spread:5.1f} % of the EKF surge spread {spread:.3f} m/s; "
        f"100 % means the prediction is no better than guessing the mean)"
        if spread > 0 else f"    surge RMS error    : {rms_surge:7.3f} m/s",
        f"    position RMS error : {float(np.sqrt(np.mean(err_pos**2))):7.3f} m",
        f"    final drift        : {drift:7.2f} m after {travelled:.1f} m travelled "
        f"({100*drift/travelled if travelled else 0:.1f} % of distance)",
        f"    path length        : predicted {_path_length(pred_pos):.1f} m "
        f"vs EKF {travelled:.1f} m",
    ]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path, help="input .mcap (or rosbag2 directory)")
    ap.add_argument("--model", type=Path,
                    default=Path(__file__).resolve().parent.parent / "model.yaml")
    ap.add_argument("--out", type=Path, default=None,
                    help="output .mcap file (default recordings/sysid/<stem>_propagated.mcap)")
    ap.add_argument("--reinit-s", type=float, default=1800.0,
                    help="re-initialise from the EKF every N seconds; 0 = single rollout. "
                         "Default 1800 s is longer than any current bag, so the model "
                         "free-runs; lower it to get the error-vs-horizon diagnostic")
    ap.add_argument("--dt", type=float, default=0.01, help="internal integration step [s]")
    ap.add_argument("--ekf-topic", default=DEFAULT_EKF_TOPIC)
    ap.add_argument("--servo-topic", default=DEFAULT_SERVO_TOPIC)
    ap.add_argument("--diag-topic", default=DEFAULT_DIAG_TOPIC,
                    help="DiagnosticArray topic carrying the BMS 'Battery: Voltage' status")
    ap.add_argument("--voltage", type=float, default=None,
                    help="evaluate the thrust maps at this fixed battery voltage [V] instead of "
                         "the BMS reading (overrides thrust.voltage_fallback_v as well)")
    ap.add_argument("--no-baseline", action="store_true",
                    help="skip the deployed power-law comparison")
    ap.add_argument("--zero-sway", action="store_true",
                    help="drop the EKF sway term from the position integration, so the "
                         "predicted track reflects only the modelled surge axis")
    ap.add_argument("--no-navsatfix", action="store_true",
                    help="skip /sysid/model_navsatfix (the geographic projection of the "
                         "prediction, for Foxglove's Map panel)")
    ap.add_argument("--pwm-stale-s", type=float, default=None,
                    help="held-PWM age counted as a dropout (default: 3x the median "
                         "servo interval, floored at 0.5 s)")
    ap.add_argument("--verify-only", action="store_true",
                    help="run the timestamp checks and stop, writing no bag")
    args = ap.parse_args(argv)

    if not args.input.exists():
        print(f"no such bag: {args.input}", file=sys.stderr)
        return 1

    model = load_model(args.model)
    print(f"[model] {model.version_tag()}")
    print(f"[model] file: {args.model}")

    missing = model.missing_for(REQUIRED)
    if missing:
        print("\nCannot propagate - these parameters are still null in "
              f"{args.model.name}:", file=sys.stderr)
        for key in missing:
            print(f"  {key}  [{model.params[key]['unit']}]", file=sys.stderr)
        print("\nFit them first, or pass --model scripts/sysid/dummy_bluerov2_heavy.yaml to "
              "exercise the pipeline with literature values.", file=sys.stderr)
        return 1

    params = SurgeParams(
        m_tot=model.get("rigid_body.m") - model.get("added_mass.X_udot"),
        X_u=model.get("damping_linear.X_u"),
        X_uu=model.get("damping_quadratic.X_uu"),
    )
    channel = int(model.thrust.get("channel", 0))
    neutral = float(model.thrust.get("pwm_neutral", 1500))
    sign = int(model.thrust.get("sign", 1))
    delay = float(model.thrust.get("servo_delay_s", 0.0))

    source = ThrustSource(model, channel, neutral, sign)
    if not source.has(channel):
        print(f"\nCannot propagate - {args.model.name} gives the surge channel ch{channel} no "
              "thrust: list it under thrust.maps (PWM x voltage table) or set "
              "thrust.curve_file, or use --model scripts/sysid/dummy_bluerov2_heavy.yaml.",
              file=sys.stderr)
        return 1

    print(f"[model] m_tot={params.m_tot:g} kg  X_u={params.X_u:g}  X_uu={params.X_uu:g}  "
          f"channel={channel}  neutral={neutral:g}  sign={sign:+d}  delay={delay:g}s")
    print(f"[model] thrust: {source.describe()}")

    print(f"\n[bag] {args.input}")
    odom, servo = read_tracks(args.input, args.ekf_topic, args.servo_topic)
    print(f"[bag] EKF {args.ekf_topic}: {len(odom)} msgs, "
          f"frame_id={odom.frame_id!r} child_frame_id={odom.child_frame_id!r}")
    print(f"[bag] servo {args.servo_topic}: {len(servo)} msgs, "
          f"ch{channel} range {servo.pwm[:, channel].min():.0f}-"
          f"{servo.pwm[:, channel].max():.0f} us")

    # Battery voltage: read whenever available (it is also written out for inspection), but
    # only the PWM x voltage maps actually use it. A fixed voltage (--voltage, or
    # thrust.voltage_fallback_v when the bag has no BMS data) replaces it explicitly.
    voltage = read_voltage(args.input, args.diag_topic)
    fixed_v = args.voltage
    if fixed_v is None and voltage is None and source.needs_voltage:
        fixed_v = model.thrust.get("voltage_fallback_v")
        if fixed_v is None:
            print(f"\nCannot propagate - the thrust maps need a battery voltage, {args.input.name} "
                  "has no BMS voltage, and thrust.voltage_fallback_v is null. Pass --voltage V.",
                  file=sys.stderr)
            return 1
    if fixed_v is not None:
        fixed_v = float(fixed_v)
        volt_at = lambda t: fixed_v  # noqa: E731
        volt_desc = f"V=fixed {fixed_v:g} V"
        print(f"[voltage] using a fixed {fixed_v:g} V")
    elif voltage is not None:
        volt_at = lambda t: float(voltage.at(t))  # noqa: E731
        volt_desc = f"V=BMS {voltage.volts.min():.2f}-{voltage.volts.max():.2f} V"
        print(f"[voltage] BMS {args.diag_topic}: {len(voltage)} samples, "
              f"{voltage.volts.min():.2f}-{voltage.volts.max():.2f} V "
              f"(median {np.median(voltage.volts):.2f} V)")
    else:
        volt_at = lambda t: None  # noqa: E731
        volt_desc = "V=unused"
    for ch, tmap in sorted(source.maps.items()):
        vs = voltage.volts if (voltage is not None and fixed_v is None) else np.array([fixed_v])
        outside = (vs < tmap.volts[0]) | (vs > tmap.volts[-1])
        if outside.any():
            print(f"[voltage] WARN {100*outside.mean():.0f}% of samples lie outside the ch{ch} "
                  f"table ({tmap.volts[0]:g}-{tmap.volts[-1]:g} V) and are clamped to its edge")

    v_ref = volt_at(0.0) if fixed_v is not None else (
        float(np.median(voltage.volts)) if voltage is not None else None)
    for pwm in (neutral + 100, neutral + 200, neutral + 400):
        thrust = source.force(channel, pwm, v_ref)
        at_v = f" @ {v_ref:.2f} V" if (v_ref is not None and source.needs_voltage) else ""
        print(f"[model] thrust {thrust:+6.1f} N{at_v} at {pwm:.0f} us -> steady state "
              f"{steady_state_speed(thrust, params):.3f} m/s")

    servo.log_t = servo.log_t - delay
    report = verify_timestamps(odom, servo)
    report.print()
    if not report.ok:
        return 1

    # Sign check: does PWM above neutral coincide with positive measured surge?
    idx = np.clip(np.searchsorted(servo.log_t, odom.t, side="right") - 1, 0, None)
    held = servo.pwm[idx, channel] - neutral
    moving = np.abs(held) > 20
    if moving.sum() > 10:
        agree = np.mean(np.sign(held[moving]) == np.sign(odom.vel_body[moving, 0]))
        corr = np.corrcoef(held[moving], odom.vel_body[moving, 0])[0, 1]
        verdict = "consistent" if corr > 0 else "INVERTED - consider thrust.sign: -1"
        print(f"\n[sign check] sign(pwm-neutral) matches sign(EKF surge) in {100*agree:.0f}% "
              f"of {int(moving.sum())} moving samples, corr={corr:+.3f} -> {verdict}")
    else:
        print("\n[sign check] not enough off-neutral samples to judge")

    # Fit the odom -> UTM anchor on the full track, before the overlap trim, so the fit uses
    # every available reference sample.
    anchor = None if args.no_navsatfix else fit_geo_anchor(args.input, odom)
    if anchor is not None:
        print(f"\n[geo] anchor fitted from {anchor.ref_topic} ({anchor.n_samples} samples): "
              f"yaw={anchor.yaw_deg:+.4f} deg  {anchor.epsg}  "
              f"residual={anchor.residual_m*100:.2f} cm")

    if args.verify_only:
        print("\n--verify-only: stopping before propagation.")
        return 0

    lo, hi = report.overlap
    keep = (odom.t >= lo) & (odom.t <= hi)
    if keep.sum() < 2:
        print("too few EKF samples inside the overlap window", file=sys.stderr)
        return 1
    odom.t = odom.t[keep]
    odom.log_t = odom.log_t[keep]
    odom.pos = odom.pos[keep]
    odom.yaw = odom.yaw[keep]
    odom.vel_body = odom.vel_body[keep]
    odom.yaw_rate = odom.yaw_rate[keep]
    stale_s = args.pwm_stale_s or default_pwm_stale_s(servo.log_t)
    print(f"\n[propagate] {len(odom)} EKF samples in the overlap window, "
          f"dt={args.dt}s, reinit={args.reinit_s}s, pwm-stale={stale_s:.2f}s")

    with_baseline = not args.no_baseline
    mpos, myaw, msurge, horizon, bpos, bsurge, gaps = propagate(
        odom, servo, source, volt_at, params,
        channel=channel, neutral=neutral, sign=sign,
        reinit_s=args.reinit_s, dt=args.dt, with_baseline=with_baseline,
        pwm_stale_s=stale_s, zero_sway=args.zero_sway,
    )
    if gaps:
        print(f"[propagate] {gaps} PWM gap(s) > {stale_s:.2f}s - restarted from the EKF there")

    err_pos = np.linalg.norm(mpos[:, :2] - odom.pos[:, :2], axis=1)
    err_surge = msurge - odom.vel_body[:, 0]
    b_err_pos = b_err_surge = None
    if with_baseline:
        b_err_pos = np.linalg.norm(bpos[:, :2] - odom.pos[:, :2], axis=1)
        b_err_surge = bsurge - odom.vel_body[:, 0]

    thruster_forces = forces_on_grid(odom.t, servo, source, volt_at, stale_s)
    if fixed_v is not None:
        battery_voltage = np.full(odom.t.size, fixed_v)
    elif voltage is not None:
        battery_voltage = np.asarray(voltage.at(odom.t), dtype=np.float64)
    else:
        battery_voltage = None

    info = (f"{model.version_tag()} | m_tot={params.m_tot:g} kg, X_u={params.X_u:g} N s/m, "
            f"X_uu={params.X_uu:g} N s^2/m^2 | thrust: {source.describe()} | {volt_desc} | "
            f"channel={channel} neutral={neutral:g} sign={sign:+d} "
            f"servo_delay={delay:g}s | reinit={args.reinit_s:g}s dt={args.dt:g}s "
            f"pwm_stale={stale_s:.2f}s"
            + (" | sway=ZEROED" if args.zero_sway else ""))

    summary = [f"propagate_model.py - {args.input.name}", f"  {info}", ""]
    ekf_surge = odom.vel_body[:, 0]
    summary += summarize("model", err_pos, err_surge, mpos, odom.pos, ekf_surge)
    if with_baseline:
        summary += summarize("baseline (deployed power law)", b_err_pos, b_err_surge,
                             bpos, odom.pos, ekf_surge)
    print()
    for line in summary:
        print(line)

    model_lat = model_lon = model_alt = None
    if anchor is not None:
        model_lat, model_lon, model_alt = anchor.to_latlon(mpos[:, :2], mpos[:, 2])

    out_path = args.out or (Path("recordings/sysid") / f"{args.input.stem}_propagated.mcap")
    if out_path.suffix != ".mcap":
        out_path = out_path.with_suffix(".mcap")
    series = OutputSeries(
        t=odom.t, model_pos=mpos, model_yaw=myaw, model_surge=msurge,
        err_pos=err_pos, err_surge=err_surge, horizon=horizon, info_text=info,
        base_pos=bpos, base_yaw=myaw if with_baseline else None, base_surge=bsurge,
        base_err_pos=b_err_pos, base_err_surge=b_err_surge,
        model_lat=model_lat, model_lon=model_lon, model_alt=model_alt,
        thruster_forces=thruster_forces, battery_voltage=battery_voltage,
    )
    written = write_augmented_bag(args.input, out_path, series,
                                  odom.frame_id, odom.child_frame_id)
    summary_path = out_path.with_name(f"{out_path.stem}_summary.txt")
    summary_path.write_text("\n".join(summary) + "\n", encoding="utf-8")

    print(f"\n[out] bag:     {written}")
    print(f"[out] summary: {summary_path}")
    print("\nPlot in Foxglove: /sysid/model_odom vs /odometry/filtered/local (XY),\n"
          "  /sysid/error/surge_mps and /sysid/error/position_m against /sysid/horizon_s,\n"
          f"  /sysid/thruster_forces.data[{channel}] (surge thrust, N) and /sysid/battery_voltage.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
