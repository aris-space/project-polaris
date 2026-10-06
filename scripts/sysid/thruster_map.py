#!/usr/bin/env python3
"""
Thruster force from PWM and supply voltage, using a manufacturer table with one force column
per voltage (e.g. thruster_curves/t500_force_vs_voltage.csv).

Interpolation: linear in PWM within each voltage column, then linear in voltage between the two
bracketing columns (bilinear on a regular grid). Inputs outside the table are clamped to its
edges, never extrapolated; `clamped()` reports whether a query hit an edge.

No dependencies beyond the standard library.

Example:
    tmap = ThrusterMap.from_csv("thruster_curves/t500_force_vs_voltage.csv")
    tmap.force_n(1700, 15.2)        # thrust in newtons at 1700 us and 15.2 V

    python thruster_map.py thruster_curves/t500_force_vs_voltage.csv 1700 15.2
"""
from __future__ import annotations

import argparse
import bisect
import csv
import re
from pathlib import Path

KGF_TO_N = 9.80665
_COL = re.compile(r"force_kgf_(\d+(?:\.\d+)?)v$", re.IGNORECASE)


class ThrusterMap:
    def __init__(self, pwm_us: list[float], volts: list[float], force_kgf: list[list[float]]) -> None:
        """force_kgf[j][i] is the force at volts[j] and pwm_us[i]."""
        if len(pwm_us) < 2 or len(volts) < 1:
            raise ValueError("need at least two PWM points and one voltage column")
        if any(b <= a for a, b in zip(pwm_us, pwm_us[1:])):
            raise ValueError("pwm_us must be strictly increasing")
        if any(b <= a for a, b in zip(volts, volts[1:])):
            raise ValueError("voltage columns must be strictly increasing")
        if any(len(col) != len(pwm_us) for col in force_kgf) or len(force_kgf) != len(volts):
            raise ValueError("force table shape does not match pwm_us x volts")
        self.pwm_us = list(pwm_us)
        self.volts = list(volts)
        self.force_kgf = [list(col) for col in force_kgf]

    @classmethod
    def from_csv(cls, path: str | Path) -> ThrusterMap:
        """Columns: pwm_us, then force_kgf_<V>V for each voltage. '#' lines are ignored."""
        path = Path(path)
        with path.open(newline="", encoding="utf-8") as fh:
            rows = csv.reader(r for r in fh if r.strip() and not r.lstrip().startswith("#"))
            header = [h.strip() for h in next(rows)]
            if header[0].lower() != "pwm_us":
                raise ValueError(f"{path}: first column must be pwm_us, got {header[0]}")
            volt_cols = []
            for idx, name in enumerate(header[1:], start=1):
                m = _COL.match(name)
                if not m:
                    raise ValueError(f"{path}: column '{name}' is not of the form force_kgf_<V>V")
                volt_cols.append((float(m.group(1)), idx))
            volt_cols.sort()
            pwm: list[float] = []
            cols: list[list[float]] = [[] for _ in volt_cols]
            for row in rows:
                pwm.append(float(row[0]))
                for j, (_, idx) in enumerate(volt_cols):
                    cols[j].append(float(row[idx]))
        return cls(pwm, [v for v, _ in volt_cols], cols)

    @staticmethod
    def _interp(x: float, xs: list[float], ys: list[float]) -> float:
        if x <= xs[0]:
            return ys[0]
        if x >= xs[-1]:
            return ys[-1]
        i = bisect.bisect_right(xs, x)
        x0, x1, y0, y1 = xs[i - 1], xs[i], ys[i - 1], ys[i]
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0)

    def force_kgf_at(self, pwm_us: float, voltage_v: float) -> float:
        per_volt = [self._interp(pwm_us, self.pwm_us, col) for col in self.force_kgf]
        if len(self.volts) == 1:
            return per_volt[0]
        return self._interp(voltage_v, self.volts, per_volt)

    def force_n(self, pwm_us: float, voltage_v: float) -> float:
        """Thrust in newtons (negative = reverse)."""
        return self.force_kgf_at(pwm_us, voltage_v) * KGF_TO_N

    def clamped(self, pwm_us: float, voltage_v: float) -> bool:
        """True if either input lies outside the table and was clamped to its edge."""
        return not (self.pwm_us[0] <= pwm_us <= self.pwm_us[-1]) or not (
            self.volts[0] <= voltage_v <= self.volts[-1]
        )


def main() -> int:
    ap = argparse.ArgumentParser(description="Evaluate thruster force at a PWM and supply voltage.")
    ap.add_argument("csv", type=Path)
    ap.add_argument("pwm_us", type=float)
    ap.add_argument("voltage_v", type=float)
    args = ap.parse_args()
    tmap = ThrusterMap.from_csv(args.csv)
    f_kgf = tmap.force_kgf_at(args.pwm_us, args.voltage_v)
    note = "  (clamped to table edge)" if tmap.clamped(args.pwm_us, args.voltage_v) else ""
    print(f"{args.pwm_us:.0f} us @ {args.voltage_v:.2f} V -> {f_kgf:.3f} kgf = {f_kgf * KGF_TO_N:.2f} N{note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
