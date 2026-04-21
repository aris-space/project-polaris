#!/usr/bin/env python3
"""
DVL topic dt comparison — velocity vs raw odometry vs odometry_cov.

The C++ dvl_a50 driver sets:
  velocity header.stamp    = time_of_validity (DVL hardware clock, µs)
  odometry header.stamp    = velocity_report.header.stamp  (identical)
  odometry_cov header.stamp = same (covariance node republishes unchanged header)

If all three share the same stamp the dt histograms should be identical.
Any difference pinpoints WHERE the bimodal cov dt originates:
  velocity == odometry != cov  →  Python odometry_covariance_node introduces it
  velocity != odometry         →  C++ driver diverges (should not happen with fix)
  all identical                →  the "bimodal" in previous plots was a rendering
                                  artefact from plotting 34 k points on a thin axis

Runs on stationary_11 by default (most data); pass a different bag dir as argument.

Dependencies: pip install rosbags matplotlib numpy
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("Install: pip install matplotlib numpy", file=sys.stderr)
    raise

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

TOPIC_VEL = "/sensors/dvl/velocity"
TOPIC_ODO = "/sensors/dvl/odometry"
TOPIC_COV = "/sensors/dvl/odometry_cov"

DEFAULT_BAG = Path(
    r"C:\Users\gleb0\Downloads\rosbags (2)\rosbags"
    r"\stationary_11_2026_04_19-16_18_49"
)


def _stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)


def collect(bag_dir: Path) -> dict[str, list[int]]:
    stamps: dict[str, list[int]] = {TOPIC_VEL: [], TOPIC_ODO: [], TOPIC_COV: []}
    wanted = set(stamps)
    with AnyReader([bag_dir]) as reader:
        conns = [c for c in reader.connections if c.topic in wanted]
        for conn, _log_ns, raw in reader.messages(connections=conns):
            try:
                msg = reader.deserialize(raw, conn.msgtype)
                st = _stamp_ns(msg)
            except Exception:
                continue
            if st != 0:
                stamps[conn.topic].append(st)
    return stamps


def dt_s(stamps: list[int]) -> np.ndarray:
    if len(stamps) < 2:
        return np.array([])
    return np.diff(np.array(stamps, dtype=np.int64)) / 1e9


def main() -> int:
    ap = argparse.ArgumentParser(description="DVL dt comparison: velocity vs odometry vs odometry_cov")
    ap.add_argument("bag_dir", type=Path, nargs="?", default=DEFAULT_BAG)
    ap.add_argument("--output-dir", type=Path, default=Path("."))
    args = ap.parse_args()

    bag_dir = args.bag_dir.resolve()
    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not bag_dir.exists():
        print(f"ERROR: {bag_dir} not found", file=sys.stderr)
        return 1

    print(f"Reading: {bag_dir.name}")
    stamps = collect(bag_dir)

    topics = [TOPIC_VEL, TOPIC_ODO, TOPIC_COV]
    labels = ["velocity\n(C++ driver, time_of_validity)", "odometry\n(C++ driver, same stamp)", "odometry_cov\n(Python node, republished)"]
    colors = ["steelblue", "seagreen", "darkorange"]

    dts = {t: dt_s(stamps[t]) for t in topics}

    # ------------------------------------------------------------------ #
    # Print summary stats
    # ------------------------------------------------------------------ #
    print(f"\n{'Topic':<35} {'n_msgs':>7} {'mean_dt_ms':>11} {'std_dt_ms':>10} {'n<0':>5} {'n>0.5':>7} {'n==0':>6}")
    print("-" * 85)
    for t, label in zip(topics, labels):
        s = stamps[t]
        d = dts[t]
        n = len(s)
        if len(d) == 0:
            print(f"{t:<35} {n:>7}  (no data)")
            continue
        mean_ms = np.mean(d) * 1000
        std_ms = np.std(d) * 1000
        n_neg = int(np.sum(d < 0))
        n_gap = int(np.sum(d > 0.5))
        n_dup = int(np.sum(d == 0))
        print(f"{t:<35} {n:>7} {mean_ms:>11.2f} {std_ms:>10.3f} {n_neg:>5} {n_gap:>7} {n_dup:>6}")

    # Check stamp identity between velocity and odometry
    vel_set = set(stamps[TOPIC_VEL])
    odo_set = set(stamps[TOPIC_ODO])
    cov_set = set(stamps[TOPIC_COV])
    print(f"\nStamp overlap:")
    print(f"  vel & odo:  {len(vel_set & odo_set)} of vel={len(vel_set)}, odo={len(odo_set)}")
    print(f"  vel & cov:  {len(vel_set & cov_set)} of vel={len(vel_set)}, cov={len(cov_set)}")
    print(f"  odo & cov:  {len(odo_set & cov_set)} of odo={len(odo_set)}, cov={len(cov_set)}")
    print(f"  stamps unique to odo (not in vel): {len(odo_set - vel_set)}")
    print(f"  stamps unique to cov (not in odo): {len(cov_set - odo_set)}")

    # ------------------------------------------------------------------ #
    # Figure 1: overlaid dt histograms (zoomed 0..0.3 s)
    # ------------------------------------------------------------------ #
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), sharey=False)
    fig.suptitle(f"{bag_dir.name}\nDVL dt histogram comparison (header.stamp, message order)", fontsize=11)

    bin_edges = np.linspace(0, 0.3, 120)

    for ax, t, label, color in zip(axes, topics, labels, colors):
        d = dts[t]
        if len(d) == 0:
            ax.text(0.5, 0.5, "No data", ha="center", va="center", transform=ax.transAxes)
            ax.set_title(label)
            continue
        # Only plot 0..0.3 s range (negative dt and gaps handled separately)
        d_zoom = d[(d >= 0) & (d <= 0.3)]
        ax.hist(d_zoom, bins=bin_edges, color=color, alpha=0.85, edgecolor="none")
        ax.axvline(1 / 9.5, color="red", lw=1.5, linestyle="--", label="1/9.5 Hz nominal")
        mean_dt = np.mean(d)
        ax.axvline(mean_dt, color="black", lw=1.2, linestyle="-", label=f"mean={mean_dt*1000:.1f} ms")
        ax.set_xlabel("dt (s)")
        ax.set_ylabel("count")
        title_extra = f"n={len(stamps[t])}, mean={np.mean(d)*1000:.1f}ms, std={np.std(d)*1000:.1f}ms"
        n_neg = int(np.sum(d < 0))
        n_gap = int(np.sum(d > 0.3))
        if n_neg or n_gap:
            title_extra += f"\nbackward={n_neg}, >0.3s={n_gap} (excluded from plot)"
        ax.set_title(label + f"\n{title_extra}", fontsize=9)
        ax.legend(fontsize=8)

    fig.tight_layout()
    out1 = out_dir / f"dvl_dt_comparison_{bag_dir.name}_histogram.png"
    fig.savefig(out1, dpi=150)
    plt.close(fig)
    print(f"\nSaved: {out1.name}")

    # ------------------------------------------------------------------ #
    # Figure 2: velocity vs cov dt — scatter to detect any offset
    # ------------------------------------------------------------------ #
    vel_stamps_sorted = sorted(stamps[TOPIC_VEL])
    cov_stamps_sorted = sorted(stamps[TOPIC_COV])

    if len(vel_stamps_sorted) >= 2 and len(cov_stamps_sorted) >= 2:
        fig2, axes2 = plt.subplots(2, 1, figsize=(16, 8))
        fig2.suptitle(f"{bag_dir.name}\nVelocity vs odometry_cov — dt overlay (stamp-sorted)", fontsize=11)

        dt_vel = np.diff(np.array(vel_stamps_sorted, dtype=np.int64)) / 1e9
        dt_cov = np.diff(np.array(cov_stamps_sorted, dtype=np.int64)) / 1e9

        t0 = vel_stamps_sorted[0] / 1e9
        t_vel = np.array(vel_stamps_sorted[1:]) / 1e9 - t0
        t_cov = np.array(cov_stamps_sorted[1:]) / 1e9 - t0

        ax = axes2[0]
        # Subsample for plotting clarity (plot at most 5000 points)
        step = max(1, len(t_vel) // 5000)
        ax.plot(t_vel[::step], dt_vel[::step], lw=0.5, color="steelblue", alpha=0.7, label="velocity dt")
        step2 = max(1, len(t_cov) // 5000)
        ax.plot(t_cov[::step2], dt_cov[::step2], lw=0.5, color="darkorange", alpha=0.7, label="odometry_cov dt")
        ax.axhline(1/9.5, color="green", lw=0.8, linestyle="--", label="nominal 1/9.5 Hz")
        ax.set_ylabel("dt (s)")
        ax.set_ylim(0, 0.35)
        ax.set_title("dt time series (subsampled for clarity)")
        ax.legend(fontsize=8)

        ax2 = axes2[1]
        # Histogram overlaid
        bins = np.linspace(0, 0.3, 120)
        ax2.hist(dt_vel[(dt_vel >= 0) & (dt_vel <= 0.3)], bins=bins,
                 color="steelblue", alpha=0.6, label="velocity", edgecolor="none")
        ax2.hist(dt_cov[(dt_cov >= 0) & (dt_cov <= 0.3)], bins=bins,
                 color="darkorange", alpha=0.5, label="odometry_cov", edgecolor="none")
        ax2.axvline(1/9.5, color="green", lw=1.2, linestyle="--", label="1/9.5 Hz")
        ax2.set_xlabel("dt (s)")
        ax2.set_ylabel("count")
        ax2.set_title("dt histograms overlaid")
        ax2.legend(fontsize=9)

        axes2[-1].set_xlabel(f"time from bag start (s)  [t0={t0:.1f} UNIX]")
        fig2.tight_layout()
        out2 = out_dir / f"dvl_dt_comparison_{bag_dir.name}_vel_vs_cov.png"
        fig2.savefig(out2, dpi=150)
        plt.close(fig2)
        print(f"Saved: {out2.name}")

    # ------------------------------------------------------------------ #
    # Figure 3: zoomed into a short window (first 60 s) for visual clarity
    # ------------------------------------------------------------------ #
    WINDOW_S = 60.0
    if vel_stamps_sorted and cov_stamps_sorted:
        t0_ns = vel_stamps_sorted[0]
        t_end_ns = t0_ns + int(WINDOW_S * 1e9)

        vel_window = [s for s in stamps[TOPIC_VEL] if t0_ns <= s <= t_end_ns]
        odo_window = [s for s in stamps[TOPIC_ODO] if t0_ns <= s <= t_end_ns]
        cov_window = [s for s in stamps[TOPIC_COV] if t0_ns <= s <= t_end_ns]

        fig3, axes3 = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
        fig3.suptitle(f"{bag_dir.name} — first {WINDOW_S:.0f} s window\ndt per topic (all individual points visible)", fontsize=11)

        for ax, window, label, color in zip(axes3,
                                             [vel_window, odo_window, cov_window],
                                             ["velocity", "odometry (raw)", "odometry_cov"],
                                             ["steelblue", "seagreen", "darkorange"]):
            if len(window) < 2:
                ax.text(0.5, 0.5, "No data in window", ha="center", va="center", transform=ax.transAxes)
                ax.set_title(label)
                continue
            arr = np.array(sorted(window), dtype=np.int64)
            d = np.diff(arr) / 1e9
            t = arr[1:] / 1e9 - t0_ns / 1e9
            ax.stem(t, d, linefmt=color, markerfmt=f".", basefmt="grey",
                    label=f"{label} (n={len(window)})")
            ax.axhline(1/9.5, color="green", lw=0.8, linestyle="--", label="1/9.5 Hz nominal")
            ax.axhline(0, color="red", lw=0.6, linestyle="--")
            ax.set_ylabel("dt (s)")
            ax.set_ylim(-0.05, 0.45)
            mean_d = np.mean(d)
            std_d = np.std(d)
            ax.set_title(f"{label}  |  mean={mean_d*1000:.1f} ms, std={std_d*1000:.2f} ms, n={len(window)}", fontsize=9)
            ax.legend(fontsize=8, loc="upper right")

        axes3[-1].set_xlabel(f"time (s) relative to bag start  [t0={t0_ns/1e9:.1f} UNIX]")
        fig3.tight_layout()
        out3 = out_dir / f"dvl_dt_comparison_{bag_dir.name}_first60s_zoom.png"
        fig3.savefig(out3, dpi=150)
        plt.close(fig3)
        print(f"Saved: {out3.name}")

    print("\n=== INTERPRETATION ===")
    for t in topics:
        d = dts[t]
        if len(d) == 0:
            continue
        # Check bimodality via histogram: compare counts in [0.05,0.15] vs [0.25,0.45]
        low = np.sum((d >= 0.05) & (d <= 0.15))
        high = np.sum((d >= 0.25) & (d <= 0.45))
        pct_high = 100.0 * high / max(len(d), 1)
        bimodal = pct_high > 2.0
        print(f"  {t}:")
        print(f"    dt in 0.05-0.15s: {low}  |  dt in 0.25-0.45s: {high} ({pct_high:.1f}%)")
        print(f"    Bimodal: {'YES -- investigate' if bimodal else 'NO -- clean'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
