#!/usr/bin/env python3
"""Hand vs sensor ice-thickness scatter for the 29 Zermatt grid points."""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DATA = [
    (17.336, 15.750), (17.447, 17.650), (14.678, 14.150), (13.333, 13.750),
    (15.245, 17.000), (14.667, 15.350), (20.127, 20.250), (19.994, 20.550),
    (19.338, 18.250), (20.961, 19.750), (13.655, 13.450), (20.739, 20.950),
    (18.493, 19.550), (18.826, 18.950), (14.990, 16.550), (12.910, 12.750),
    (15.212, 15.250), (14.500, 15.150), (16.913, 17.000), (13.644, 13.750),
    (16.458, 16.500), (18.871, 18.000), (19.327, 19.250), (23.018, 22.250),
    (22.307, 20.250), (17.258, 16.650), (16.547, 16.550), (20.072, 20.000),
    (16.702, 16.650),
]

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "zermatt_results" / "plots" / "hand_vs_sensor.png"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", default=str(DEFAULT_OUT))
    args = p.parse_args()

    sensor = np.array([d[0] for d in DATA])
    hand   = np.array([d[1] for d in DATA])

    lo = min(hand.min(), sensor.min()) - 0.5
    hi = max(hand.max(), sensor.max()) + 0.5

    envelope_cm = 1.0  # ±1 cm tolerance band; adjust as needed

    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    diag = np.array([lo, hi])
    ax.fill_between(diag, diag - envelope_cm, diag + envelope_cm,
                    color="#ffaa33", alpha=0.20,
                    label=f"±{envelope_cm:.0f} cm")
    ax.plot([lo, hi], [lo, hi], color="#888888", ls="--", lw=1.0)
    ax.scatter(hand, sensor, s=45, color="#5577ff", edgecolor="#3355cc", linewidth=0.5)
    ax.legend(loc="upper left", frameon=False)
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_aspect("equal")
    ax.set_xlabel("Reference ice thickness (cm)")
    ax.set_ylabel("Sensor-measured ice thickness (cm)")
    ax.grid(True, color="#eeeeee", linewidth=0.6)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved {out}")


if __name__ == "__main__":
    main()
