#!/usr/bin/env python3
"""
Plot multiple ekf_offline_diagnostic CSVs side by side for comparison.

Usage
-----
    python scripts/compare_diag_runs.py <CSV1> <CSV2> [...]
    python scripts/compare_diag_runs.py diagnosis/global_ekf_residual/

The first form takes individual diag.csv paths; the second auto-discovers
``*/diag.csv`` under the directory and treats each subdirectory name as a
run label. Output goes to ``--output`` (default: ``diag_comparison.png`` next
to the first CSV).

Pure-Python — runs on the Windows host without ROS2. Uses matplotlib + numpy
+ stdlib csv. No pandas.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _parse_float(s: str) -> float:
    if s is None or s == "":
        return float("nan")
    try:
        return float(s)
    except ValueError:
        return float("nan")


def load_diag(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    cols: dict[str, list[float]] = {k: [] for k in reader.fieldnames or []}
    for row in rows:
        for k in cols:
            cols[k].append(_parse_float(row.get(k, "")))
    arrs: dict[str, np.ndarray] = {k: np.asarray(v, dtype=float) for k, v in cols.items()}
    if "t_sim" in arrs and arrs["t_sim"].size > 0:
        arrs["t_rel"] = arrs["t_sim"] - arrs["t_sim"][0]
    return arrs


def discover_runs(input_paths: list[Path]) -> list[tuple[str, Path]]:
    """Resolve CLI args to a list of (label, csv_path) pairs."""
    runs: list[tuple[str, Path]] = []
    for p in input_paths:
        if p.is_dir():
            for csvp in sorted(p.glob("*/diag.csv")):
                label = csvp.parent.name
                runs.append((label, csvp))
            top_level = p / "diag.csv"
            if top_level.exists():
                runs.append((p.name, top_level))
        elif p.suffix == ".csv" and p.exists():
            label = p.parent.name
            runs.append((label, p))
        else:
            print(f"warning: {p} is not a directory or CSV file; skipping",
                  file=sys.stderr)
    return runs


def plot_runs(runs: list[tuple[str, dict[str, np.ndarray]]],
              output_path: Path) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    ((ax_traj, ax_mag), (ax_sbl, ax_innov_pos), (ax_innov_yaw, ax_p)) = axes

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(runs), 1)))

    for (label, d), color in zip(runs, colors):
        t = d.get("t_rel", np.array([]))
        x = d.get("x", np.array([]))
        y = d.get("y", np.array([]))
        sbl_err = d.get("sbl_err_m", np.array([]))
        sbl_x = d.get("sbl_x", np.array([]))
        sbl_y = d.get("sbl_y", np.array([]))
        innov_x = d.get("innov_x", np.array([]))
        innov_y = d.get("innov_y", np.array([]))
        innov_yaw = d.get("innov_yaw", np.array([]))
        p_xx = d.get("P_xx", np.array([]))
        p_yy = d.get("P_yy", np.array([]))

        if t.size == 0:
            continue

        ax_traj.plot(x, y, color=color, label=label, linewidth=1.0, alpha=0.85)

        with np.errstate(invalid="ignore"):
            mag = np.sqrt(x * x + y * y)
        ax_mag.plot(t, mag, color=color, label=label, linewidth=1.0, alpha=0.85)

        if np.any(np.isfinite(sbl_err)):
            ax_sbl.plot(t, sbl_err, color=color, label=label,
                        linewidth=1.0, alpha=0.85)

        if np.any(np.isfinite(innov_x)) or np.any(np.isfinite(innov_y)):
            with np.errstate(invalid="ignore"):
                innov_mag = np.sqrt(np.nan_to_num(innov_x) ** 2
                                    + np.nan_to_num(innov_y) ** 2)
                innov_mag[~(np.isfinite(innov_x) | np.isfinite(innov_y))] = np.nan
            ax_innov_pos.plot(t, innov_mag, color=color, label=label,
                              linewidth=1.0, alpha=0.85)

        if np.any(np.isfinite(innov_yaw)):
            ax_innov_yaw.plot(t, np.degrees(innov_yaw), color=color, label=label,
                              linewidth=1.0, alpha=0.85)

        if np.any(np.isfinite(p_xx)):
            with np.errstate(invalid="ignore", divide="ignore"):
                ax_p.plot(t, p_xx, color=color, label=f"{label} P_xx",
                          linewidth=1.0, alpha=0.85)
                ax_p.plot(t, p_yy, color=color, linestyle="--",
                          label=f"{label} P_yy", linewidth=1.0, alpha=0.6)

    # Add SBL ground truth on the trajectory panel using the first run that has it.
    for label, d in runs:
        sbl_x = d.get("sbl_x", np.array([]))
        sbl_y = d.get("sbl_y", np.array([]))
        if np.any(np.isfinite(sbl_x)) and np.any(np.isfinite(sbl_y)):
            ax_traj.plot(sbl_x, sbl_y, color="black", linewidth=0.8, alpha=0.5,
                         label="SBL (truth)", zorder=0)
            break

    ax_traj.set_title("Trajectory (map frame)")
    ax_traj.set_xlabel("x [m]")
    ax_traj.set_ylabel("y [m]")
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.grid(alpha=0.3)
    ax_traj.legend(fontsize=8, loc="best")

    ax_mag.set_title("|position| over time (log scale)")
    ax_mag.set_xlabel("t_sim [s]")
    ax_mag.set_ylabel("|x,y| [m]")
    ax_mag.set_yscale("symlog", linthresh=1.0)
    ax_mag.grid(alpha=0.3, which="both")
    ax_mag.legend(fontsize=8)

    ax_sbl.set_title("|global - SBL| (independent ground truth)")
    ax_sbl.set_xlabel("t_sim [s]")
    ax_sbl.set_ylabel("error [m]")
    ax_sbl.set_yscale("symlog", linthresh=1.0)
    ax_sbl.grid(alpha=0.3, which="both")
    ax_sbl.legend(fontsize=8)

    ax_innov_pos.set_title("Position innovation magnitude  |gps - global|")
    ax_innov_pos.set_xlabel("t_sim [s]")
    ax_innov_pos.set_ylabel("[m]")
    ax_innov_pos.grid(alpha=0.3)
    ax_innov_pos.legend(fontsize=8)

    ax_innov_yaw.set_title("Yaw innovation  (local_yaw - global_yaw)")
    ax_innov_yaw.set_xlabel("t_sim [s]")
    ax_innov_yaw.set_ylabel("[deg]")
    ax_innov_yaw.grid(alpha=0.3)
    ax_innov_yaw.legend(fontsize=8)

    ax_p.set_title("Filter covariance diagonals  (log scale)")
    ax_p.set_xlabel("t_sim [s]")
    ax_p.set_ylabel("variance [m²]")
    ax_p.set_yscale("log")
    ax_p.grid(alpha=0.3, which="both")
    ax_p.legend(fontsize=7, ncol=2)

    fig.suptitle("EKF offline-replay diagnostic comparison", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=110)
    print(f"wrote {output_path}")


def print_summary(runs: list[tuple[str, dict[str, np.ndarray]]]) -> None:
    def _stat(name: str, values: np.ndarray, unit: str) -> str:
        finite = values[np.isfinite(values)]
        if finite.size == 0:
            return f"  {name:>22}: (no samples)"
        return (
            f"  {name:>22}: n={finite.size:>5}  "
            f"mean={np.mean(finite):+.3f}{unit}  "
            f"med={np.median(finite):+.3f}{unit}  "
            f"|p95|={np.percentile(np.abs(finite), 95):.3f}{unit}  "
            f"|max|={np.max(np.abs(finite)):.3f}{unit}"
        )

    print("\n" + "=" * 92)
    print(f"{'COMPARISON SUMMARY':^92}")
    print("=" * 92)

    for label, d in runs:
        print(f"\n--- run: {label} ---")
        x = d.get("x", np.array([]))
        y = d.get("y", np.array([]))
        if x.size:
            mag = np.sqrt(x * x + y * y)
            mag_finite = mag[np.isfinite(mag)]
            if mag_finite.size:
                print(f"  final |x,y| = {mag_finite[-1]:>12.2f} m  "
                      f"(max in-run = {mag_finite.max():.2f} m)")
        print(_stat("innov x [m]", d.get("innov_x", np.array([])), " m"))
        print(_stat("innov y [m]", d.get("innov_y", np.array([])), " m"))
        innov_yaw = d.get("innov_yaw", np.array([]))
        if innov_yaw.size:
            print(_stat("innov yaw [deg]", np.degrees(innov_yaw), " deg"))
        print(_stat("innov vx_world [m/s]",
                    d.get("innov_vx_world", np.array([])), " m/s"))
        print(_stat("innov vy_world [m/s]",
                    d.get("innov_vy_world", np.array([])), " m/s"))
        print(_stat("|global - SBL| [m]",
                    d.get("sbl_err_m", np.array([])), " m"))

    print("\n" + "=" * 92)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("inputs", nargs="+", type=Path,
                   help="Diag CSV file(s) and/or directory(ies) containing run subdirs.")
    p.add_argument("-o", "--output", type=Path, default=None,
                   help="Output PNG path. Default: diag_comparison.png next to first input.")
    args = p.parse_args(argv)

    runs_paths = discover_runs(args.inputs)
    if not runs_paths:
        print("error: no diag.csv files found in inputs", file=sys.stderr)
        return 1

    print(f"loading {len(runs_paths)} run(s):")
    runs: list[tuple[str, dict[str, np.ndarray]]] = []
    for label, csv_path in runs_paths:
        print(f"  {label}: {csv_path}")
        runs.append((label, load_diag(csv_path)))

    output = args.output or (runs_paths[0][1].parent.parent / "diag_comparison.png")
    plot_runs(runs, output)
    print_summary(runs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
