#!/usr/bin/env python3
"""
Thruster force per servo channel, shared by the propagation and the fitting scripts.

ThrustSource maps the held PWM of each servo channel (and the battery voltage) to a force in
newtons: a channel listed under thrust.maps in the model file uses its PWM x voltage table
(thruster_map.ThrusterMap); otherwise the surge channel falls back to the single-voltage
thrust.curve_file (dynamics.ThrusterCurve, used by the dummy models). Every other channel has
no curve yet and returns NaN, so a missing curve can never pass as zero thrust.

Dependencies: pip install numpy
"""

from __future__ import annotations

import numpy as np

from dynamics import ThrusterCurve
from thruster_map import ThrusterMap

N_SERVO_CHANNELS = 6


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


def axis_force(source: ThrustSource, entries: list[dict], pwm: np.ndarray,
               volts: np.ndarray) -> np.ndarray:
    """Force (or moment) along one body axis from the allocation entries in model.yaml.

    `entries` is one axis of thrust.allocation, e.g. [{channel: 2, lever_m: 1.0}, ...]: each
    channel's force (PWM x voltage interpolation, as for surge) times its lever_m, summed.
    `pwm` is (N, 6) held PWM, `volts` is (N,) battery voltage (NaN = unknown). The result is NaN
    wherever any contributing channel has no curve, so a missing curve never counts as zero.
    """
    out = np.zeros(pwm.shape[0])
    for entry in entries:
        ch, lever = int(entry["channel"]), float(entry["lever_m"])
        out += lever * np.array([source.force(ch, float(pwm[k, ch]),
                                              None if np.isnan(volts[k]) else float(volts[k]))
                                 for k in range(pwm.shape[0])])
    return out
