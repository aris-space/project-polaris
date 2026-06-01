#!/usr/bin/env python3
"""Grouped bar chart: sensor vs reference ice thickness per grid point."""

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

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "zermatt_results" / "plots" / "hand_vs_sensor_bars.png"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", default=str(DEFAULT_OUT))
    args = p.parse_args()

    sensor = np.array([d[0] for d in DATA])
    hand   = np.array([d[1] for d in DATA])
    gp     = np.arange(1, len(DATA) + 1)

    width = 0.4
    fig, ax = plt.subplots(figsize=(13, 5))
    ax.bar(gp - width/2, hand,   width, color="#888888", label="Reference")
    ax.bar(gp + width/2, sensor, width, color="#5577ff", label="Sensor")

    ax.set_xticks(gp)
    ax.set_xticklabels([str(g) for g in gp], fontsize=8)
    ax.set_xlabel("Grid point")
    ax.set_ylabel("Ice thickness (cm)")
    ax.set_ylim(min(hand.min(), sensor.min()) - 1.0, max(hand.max(), sensor.max()) + 1.0)
    ax.grid(True, axis="y", color="#eeeeee", linewidth=0.6)
    ax.legend(loc="upper left", frameon=False)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved {out}")


if __name__ == "__main__":
    main()
