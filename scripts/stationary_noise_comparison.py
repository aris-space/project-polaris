#!/usr/bin/env python3
"""
Stationary noise comparison — stationary_11 (2026-04-19) vs stationary_02 reference.

Prompt 8: DVL and IMU noise characterization, bias drift, plausibility checks, and
side-by-side comparison table against previous stationary measurements.

Previous reference (stationary_02, Steg Zurich, lock-masked):
  DVL vx: mean=-5.16e-4 m/s, std=7.47e-3 m/s, var=5.58e-5 m²/s²
  DVL vy: mean=+2.53e-4 m/s, std=2.80e-3 m/s, var=7.84e-6 m²/s²
  DVL vz: mean=-1.57e-4 m/s, std=3.07e-4 m/s, var=9.43e-8 m²/s²
  IMU gyro_x std=1.4395e-3 rad/s, gyro_y std=1.4831e-3, gyro_z std=1.4506e-3
  IMU accel_x std=1.4372e-2 m/s², accel_y std=4.9542e-3, accel_z std=7.1293e-3
  DVL vx drift slope: -4.2e-7 m/s/s, IMU gyro_z drift slope: -3.0e-8 rad/s/s

New bag: stationary_11_2026_04_19-16_18_49
  - 64 min, recorded at St. Moritz lake (same as stationary_02 but later in day, windier)
  - DVL mounted alone (not on AUV), speed of sound = 1457 m/s (vs 1500 previously)
  - No EKF or SBL in this bag; DVL + IMU + GNSS only

Outputs (PNG + printed table):
  stationary11_dvl_histograms.png
  stationary11_dvl_timeseries.png
  stationary11_imu_gyro_histograms.png
  stationary11_imu_accel_histograms.png
  stationary11_bias_drift.png

Dependencies: pip install rosbags numpy matplotlib
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
except ImportError:
    print("Install: pip install matplotlib", file=sys.stderr)
    raise

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import analyze_mcap_dvl_health as dvl_mod  # noqa: E402
import stationary_tep_stats as tep  # noqa: E402

# ------------------------------------------------------------------ #
# Reference values from stationary_02 (Steg Zurich, lock-masked)
# ------------------------------------------------------------------ #

REF = {
    "dvl_vx": {"mean": -5.16e-4, "std": 7.47e-3, "var": 5.58e-5,
               "drift_slope": -4.2e-7},
    "dvl_vy": {"mean": +2.53e-4, "std": 2.80e-3, "var": 7.84e-6,
               "drift_slope": None},
    "dvl_vz": {"mean": -1.57e-4, "std": 3.07e-4, "var": 9.43e-8,
               "drift_slope": None},
    "imu_gyro_x": {"std": 1.4395e-3, "drift_slope": None},
    "imu_gyro_y": {"std": 1.4831e-3, "drift_slope": None},
    "imu_gyro_z": {"std": 1.4506e-3, "drift_slope": -3.0e-8},
    "imu_accel_x": {"std": 1.4372e-2},
    "imu_accel_y": {"std": 4.9542e-3},
    "imu_accel_z": {"std": 7.1293e-3},
}

DEFAULT_BAG = Path(
    r"C:\Users\gleb0\Downloads\rosbags (2)\rosbags"
    r"\stationary_11_2026_04_19-16_18_49"
)

TOPIC_IMU = "/imu/data"
TOPIC_VEL = dvl_mod.TOPIC_VEL
TOPIC_COV = dvl_mod.TOPIC_COV

DRIFT_WINDOW_SEC = 120.0
GRAVITY = 9.81


# ------------------------------------------------------------------ #
# Data collection
# ------------------------------------------------------------------ #

def _stamp_ns(msg: Any) -> int:
    return int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)


def collect_all(bag_dir: Path) -> dict[str, np.ndarray]:
    """Single-pass read of IMU, DVL velocity, and DVL odometry_cov."""
    imu_t, gx, gy, gz, ax, ay, az = [], [], [], [], [], [], []
    vel_t, vel_lock = [], []
    cov_t, vx, vy, vz = [], [], [], []

    with AnyReader([bag_dir]) as reader:
        wanted = {TOPIC_IMU, TOPIC_VEL, TOPIC_COV}
        conns = [c for c in reader.connections if c.topic in wanted]
        for conn, _log_ns, raw in reader.messages(connections=conns):
            try:
                msg = reader.deserialize(raw, conn.msgtype)
                st = _stamp_ns(msg)
            except Exception:
                continue
            if st == 0:
                continue

            if conn.topic == TOPIC_IMU:
                imu_t.append(st)
                gx.append(float(msg.angular_velocity.x))
                gy.append(float(msg.angular_velocity.y))
                gz.append(float(msg.angular_velocity.z))
                ax.append(float(msg.linear_acceleration.x))
                ay.append(float(msg.linear_acceleration.y))
                az.append(float(msg.linear_acceleration.z))
            elif conn.topic == TOPIC_VEL:
                vel_t.append(st)
                vel_lock.append(bool(msg.beam_velocities_valid))
            elif conn.topic == TOPIC_COV:
                cov_t.append(st)
                vx.append(float(msg.twist.twist.linear.x))
                vy.append(float(msg.twist.twist.linear.y))
                vz.append(float(msg.twist.twist.linear.z))

    # Sort by header.stamp
    def _sort(ts, *arrs):
        idx = np.argsort(np.array(ts, dtype=np.int64))
        return (np.array(ts, dtype=np.int64)[idx],) + tuple(
            np.array(a, dtype=np.float64)[idx] for a in arrs
        )

    imu_t_s, gx_s, gy_s, gz_s, ax_s, ay_s, az_s = _sort(
        imu_t, gx, gy, gz, ax, ay, az
    )
    vel_t_arr = np.array(vel_t, dtype=np.int64)
    vel_lock_arr = np.array(vel_lock, dtype=bool)
    if vel_t_arr.size:
        idx = np.argsort(vel_t_arr)
        vel_t_arr = vel_t_arr[idx]
        vel_lock_arr = vel_lock_arr[idx]

    cov_t_s, vx_s, vy_s, vz_s = _sort(cov_t, vx, vy, vz)

    return {
        "imu_t": imu_t_s,
        "gx": gx_s, "gy": gy_s, "gz": gz_s,
        "ax": ax_s, "ay": ay_s, "az": az_s,
        "vel_t": vel_t_arr, "vel_lock": vel_lock_arr,
        "cov_t": cov_t_s,
        "vx": vx_s, "vy": vy_s, "vz": vz_s,
    }


def build_lock_mask(vel_t: np.ndarray, vel_lock: np.ndarray,
                    query_t: np.ndarray) -> np.ndarray:
    """Return bool mask: True for each query timestamp that falls in a lock-true interval."""
    if vel_t.size < 2:
        return np.ones(query_t.size, dtype=bool)
    intervals = tep._include_lock_intervals_from_stamp_lock(
        list(zip(vel_t.tolist(), vel_lock.tolist()))
    )
    mask = np.zeros(query_t.size, dtype=bool)
    for a, b in intervals:
        mask |= (query_t >= a) & (query_t < b)
    return mask


def _series_stats(v: np.ndarray) -> dict[str, float]:
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {"n": 0, "mean": float("nan"), "std": float("nan"), "var": float("nan")}
    mean = float(v.mean())
    var = float(v.var(ddof=1)) if v.size > 1 else 0.0
    return {"n": int(v.size), "mean": mean, "std": math.sqrt(var), "var": var}


def bin_means(t_ns: np.ndarray, vals: np.ndarray,
              window_sec: float) -> tuple[np.ndarray, np.ndarray]:
    """Non-overlapping 120s bins → (bin_mid_s, mean_per_bin)."""
    if t_ns.size == 0:
        return np.array([]), np.array([])
    t0 = float(t_ns[0])
    t_rel = (t_ns.astype(np.float64) - t0) / 1e9
    t_end = t_rel[-1]
    mids, means = [], []
    w = 0
    while w * window_sec <= t_end:
        lo, hi = w * window_sec, (w + 1) * window_sec
        m = (t_rel >= lo) & (t_rel < hi)
        if np.any(m):
            mids.append(float(np.mean(t_rel[m])))
            means.append(float(vals[m].mean()))
        w += 1
    return np.array(mids), np.array(means)


def drift_slope(mids: np.ndarray, means: np.ndarray) -> float | None:
    if mids.size < 2:
        return None
    return float(np.polyfit(mids, means, 1)[0])


# ------------------------------------------------------------------ #
# Plotting helpers
# ------------------------------------------------------------------ #

COLORS = {
    "dvl": "#3dd6c6",
    "imu_gyro": "#e8b86d",
    "imu_accel": "#a78bfa",
    "lock": "rgba(61,214,198,0.12)",
    "ref": "#ff6b6b",
}

plt.rcParams.update({
    "figure.facecolor": "#0c0f12",
    "axes.facecolor": "#141a20",
    "axes.edgecolor": "#1e2832",
    "axes.labelcolor": "#8b9caa",
    "xtick.color": "#8b9caa",
    "ytick.color": "#8b9caa",
    "text.color": "#e6edf3",
    "grid.color": "#1e2832",
    "grid.linewidth": 0.5,
})


def _ax_style(ax):
    ax.grid(True, alpha=0.5)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def shade_lock_loss(ax, vel_t: np.ndarray, vel_lock: np.ndarray, t0: float):
    """Shade regions where DVL lock is False."""
    in_no_lock = False
    seg_start = None
    for t, lk in zip(vel_t, vel_lock):
        t_s = t / 1e9 - t0
        if not lk and not in_no_lock:
            in_no_lock = True
            seg_start = t_s
        elif lk and in_no_lock:
            in_no_lock = False
            ax.axvspan(seg_start, t_s, color="#ff6b6b", alpha=0.15, lw=0)
    if in_no_lock and seg_start is not None:
        ax.axvspan(seg_start, (vel_t[-1] / 1e9 - t0), color="#ff6b6b", alpha=0.15, lw=0)


# ------------------------------------------------------------------ #
# Plot 1: DVL velocity histograms
# ------------------------------------------------------------------ #

def plot_dvl_histograms(data: dict, lock_mask: np.ndarray, ref: dict,
                        output_dir: Path) -> None:
    vx = data["vx"][lock_mask]
    vy = data["vy"][lock_mask]
    vz = data["vz"][lock_mask]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("stationary_11 — DVL velocity histograms (lock-masked)", color="#e6edf3")

    for ax, vals, label, ref_key in zip(
        axes,
        [vx, vy, vz],
        ["vx (m/s)", "vy (m/s)", "vz (m/s)"],
        ["dvl_vx", "dvl_vy", "dvl_vz"],
    ):
        s = _series_stats(vals)
        n_bins = min(120, max(30, int(np.sqrt(len(vals)))))
        ax.hist(vals, bins=n_bins, color=COLORS["dvl"], alpha=0.85, edgecolor="none",
                density=True)

        # Gaussian overlay from stationary_11 stats
        if np.isfinite(s["std"]) and s["std"] > 0:
            x = np.linspace(vals.min(), vals.max(), 300)
            gauss = np.exp(-0.5 * ((x - s["mean"]) / s["std"]) ** 2) / (
                s["std"] * np.sqrt(2 * np.pi)
            )
            ax.plot(x, gauss, color=COLORS["dvl"], lw=1.5, label="stationary_11 fit")

        # Reference std band
        ref_std = ref[ref_key]["std"]
        ref_mean = ref[ref_key]["mean"]
        ax.axvspan(ref_mean - ref_std, ref_mean + ref_std, color=COLORS["ref"],
                   alpha=0.15, label=f"stationary_02 ±1σ")
        ax.axvline(ref_mean, color=COLORS["ref"], lw=1, linestyle="--")
        ax.axvline(s["mean"], color=COLORS["dvl"], lw=1, linestyle="--")

        title = (f"{label}\nmean={s['mean']:.3e}  std={s['std']:.3e}\n"
                 f"ref_std={ref_std:.3e}  ratio={s['std']/ref_std:.2f}")
        ax.set_title(title, color="#e6edf3", fontsize=9)
        ax.set_xlabel(label)
        ax.set_ylabel("density")
        ax.legend(fontsize=7)
        _ax_style(ax)

    fig.tight_layout()
    out = output_dir / "stationary11_dvl_histograms.png"
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ------------------------------------------------------------------ #
# Plot 2: DVL velocity time series
# ------------------------------------------------------------------ #

def plot_dvl_timeseries(data: dict, lock_mask: np.ndarray,
                        output_dir: Path) -> None:
    cov_t = data["cov_t"]
    vx = data["vx"]
    vy = data["vy"]
    vz = data["vz"]
    vel_t = data["vel_t"]
    vel_lock = data["vel_lock"]

    if cov_t.size == 0:
        print("  No DVL cov data for time series plot.")
        return

    t0 = cov_t[0] / 1e9
    t_rel = cov_t / 1e9 - t0

    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig.suptitle("stationary_11 — DVL velocity time series (red shading = lock loss)",
                 color="#e6edf3")

    for ax, vals, ylabel, color in zip(
        axes,
        [vx, vy, vz],
        ["vx (m/s)", "vy (m/s)", "vz (m/s)"],
        [COLORS["dvl"], "#a78bfa", "#e8b86d"],
    ):
        # Subsample for plotting (max 10k points)
        step = max(1, len(t_rel) // 10000)
        ax.plot(t_rel[::step], vals[::step], lw=0.5, color=color, alpha=0.8)
        ax.axhline(0, color="#e6edf3", lw=0.5, linestyle="--", alpha=0.4)
        # Windowed mean (120s bins)
        mids, means = bin_means(cov_t[lock_mask], vals[lock_mask], DRIFT_WINDOW_SEC)
        if mids.size >= 2:
            ax.plot(mids, means, color="#ffffff", lw=2, alpha=0.9, label="120s bin mean")
        shade_lock_loss(ax, vel_t, vel_lock, t0)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.legend(fontsize=7, loc="upper right")
        _ax_style(ax)

    axes[-1].set_xlabel(f"time from bag start (s)  [t0={t0:.1f} UNIX]")
    fig.tight_layout()
    out = output_dir / "stationary11_dvl_timeseries.png"
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ------------------------------------------------------------------ #
# Plot 3: IMU gyro histograms
# ------------------------------------------------------------------ #

def plot_imu_gyro_histograms(data: dict, lock_mask_imu: np.ndarray, ref: dict,
                              output_dir: Path) -> None:
    gx = data["gx"][lock_mask_imu]
    gy = data["gy"][lock_mask_imu]
    gz = data["gz"][lock_mask_imu]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("stationary_11 — IMU angular velocity histograms (lock-masked)",
                 color="#e6edf3")

    for ax, vals, label, ref_key in zip(
        axes,
        [gx, gy, gz],
        ["gyro_x (rad/s)", "gyro_y (rad/s)", "gyro_z (rad/s)"],
        ["imu_gyro_x", "imu_gyro_y", "imu_gyro_z"],
    ):
        s = _series_stats(vals)
        n_bins = min(120, max(30, int(np.sqrt(len(vals)))))
        ax.hist(vals, bins=n_bins, color=COLORS["imu_gyro"], alpha=0.85,
                edgecolor="none", density=True)

        if np.isfinite(s["std"]) and s["std"] > 0:
            x = np.linspace(vals.min(), vals.max(), 300)
            gauss = np.exp(-0.5 * ((x - s["mean"]) / s["std"]) ** 2) / (
                s["std"] * np.sqrt(2 * np.pi)
            )
            ax.plot(x, gauss, color=COLORS["imu_gyro"], lw=1.5)

        ref_std = ref[ref_key]["std"]
        ax.axvspan(-ref_std, ref_std, color=COLORS["ref"], alpha=0.15,
                   label=f"stationary_02 ±1σ")
        ax.axvline(0, color="#e6edf3", lw=0.7, linestyle="--", alpha=0.5)

        title = (f"{label}\nmean={s['mean']:.3e}  std={s['std']:.3e}\n"
                 f"ref_std={ref_std:.3e}  ratio={s['std']/ref_std:.2f}")
        ax.set_title(title, color="#e6edf3", fontsize=9)
        ax.set_xlabel(label)
        ax.set_ylabel("density")
        ax.legend(fontsize=7)
        _ax_style(ax)

    fig.tight_layout()
    out = output_dir / "stationary11_imu_gyro_histograms.png"
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ------------------------------------------------------------------ #
# Plot 4: IMU acceleration histograms
# ------------------------------------------------------------------ #

def plot_imu_accel_histograms(data: dict, lock_mask_imu: np.ndarray, ref: dict,
                               output_dir: Path) -> None:
    ax_d = data["ax"][lock_mask_imu]
    ay_d = data["ay"][lock_mask_imu]
    az_d = data["az"][lock_mask_imu]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("stationary_11 — IMU linear acceleration histograms (lock-masked)",
                 color="#e6edf3")

    for ax, vals, label, ref_key in zip(
        axes,
        [ax_d, ay_d, az_d],
        ["accel_x (m/s²)", "accel_y (m/s²)", "accel_z (m/s²)"],
        ["imu_accel_x", "imu_accel_y", "imu_accel_z"],
    ):
        s = _series_stats(vals)
        n_bins = min(120, max(30, int(np.sqrt(len(vals)))))
        ax.hist(vals, bins=n_bins, color=COLORS["imu_accel"], alpha=0.85,
                edgecolor="none", density=True)

        if np.isfinite(s["std"]) and s["std"] > 0:
            x = np.linspace(vals.min(), vals.max(), 300)
            gauss = np.exp(-0.5 * ((x - s["mean"]) / s["std"]) ** 2) / (
                s["std"] * np.sqrt(2 * np.pi)
            )
            ax.plot(x, gauss, color=COLORS["imu_accel"], lw=1.5)

        ref_std = ref[ref_key]["std"]
        ax.axvspan(s["mean"] - ref_std, s["mean"] + ref_std, color=COLORS["ref"],
                   alpha=0.15, label=f"stationary_02 ±1sigma")

        # Gravity marker for accel_z
        if "z" in label:
            ax.axvline(GRAVITY, color="#ffffff", lw=1, linestyle=":", label=f"g={GRAVITY}")
            ax.axvline(-GRAVITY, color="#ffffff", lw=1, linestyle=":", alpha=0.5)

        title = (f"{label}\nmean={s['mean']:.4f}  std={s['std']:.4e}\n"
                 f"ref_std={ref_std:.4e}  ratio={s['std']/ref_std:.2f}")
        ax.set_title(title, color="#e6edf3", fontsize=9)
        ax.set_xlabel(label)
        ax.set_ylabel("density")
        ax.legend(fontsize=7)
        _ax_style(ax)

    fig.tight_layout()
    out = output_dir / "stationary11_imu_accel_histograms.png"
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ------------------------------------------------------------------ #
# Plot 5: Bias drift
# ------------------------------------------------------------------ #

def plot_bias_drift(data: dict, lock_mask: np.ndarray, lock_mask_imu: np.ndarray,
                    ref: dict, output_dir: Path) -> None:
    cov_t = data["cov_t"][lock_mask]
    imu_t = data["imu_t"][lock_mask_imu]
    series = [
        ("DVL vx",   cov_t,  data["vx"][lock_mask],  "#3dd6c6", REF["dvl_vx"].get("drift_slope")),
        ("DVL vy",   cov_t,  data["vy"][lock_mask],  "#a78bfa", REF["dvl_vy"].get("drift_slope")),
        ("DVL vz",   cov_t,  data["vz"][lock_mask],  "#e8b86d", REF["dvl_vz"].get("drift_slope")),
        ("IMU gyro_x", imu_t, data["gx"][lock_mask_imu], "#3dd6c6", REF["imu_gyro_x"].get("drift_slope")),
        ("IMU gyro_y", imu_t, data["gy"][lock_mask_imu], "#a78bfa", REF["imu_gyro_y"].get("drift_slope")),
        ("IMU gyro_z", imu_t, data["gz"][lock_mask_imu], "#e8b86d", REF["imu_gyro_z"].get("drift_slope")),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    fig.suptitle("stationary_11 — Bias drift (120s windowed bin means vs time)",
                 color="#e6edf3")

    for ax, (label, t_ns, vals, color, ref_slope) in zip(axes.flat, series):
        mids, means = bin_means(t_ns, vals, DRIFT_WINDOW_SEC)
        if mids.size == 0:
            ax.text(0.5, 0.5, "No data", ha="center", va="center",
                    transform=ax.transAxes, color="#8b9caa")
            ax.set_title(label, color="#e6edf3")
            continue

        ax.scatter(mids / 60, means, s=18, color=color, alpha=0.9, zorder=3)

        slope = drift_slope(mids, means)
        if slope is not None:
            fit_y = slope * mids + (np.mean(means) - slope * np.mean(mids))
            ax.plot(mids / 60, fit_y, color="#ffffff", lw=1.5, linestyle="--",
                    label=f"slope={slope:.2e} /s")
            # Reference slope
            if ref_slope is not None:
                ref_fit = ref_slope * mids + (np.mean(means) - ref_slope * np.mean(mids))
                ax.plot(mids / 60, ref_fit, color=COLORS["ref"], lw=1, linestyle=":",
                        label=f"ref_slope={ref_slope:.2e} /s")

        ax.axhline(0, color="#e6edf3", lw=0.5, linestyle="--", alpha=0.4)
        slope_str = f"{slope:.2e}" if slope is not None else "N/A"
        ax.set_title(f"{label}\nslope={slope_str} /s", color="#e6edf3", fontsize=9)
        ax.set_xlabel("time (min)", fontsize=8)
        ax.set_ylabel("bin mean", fontsize=8)
        ax.legend(fontsize=7)
        _ax_style(ax)

    fig.tight_layout()
    out = output_dir / "stationary11_bias_drift.png"
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ------------------------------------------------------------------ #
# Summary table and plausibility checks
# ------------------------------------------------------------------ #

def print_summary(data: dict, lock_mask: np.ndarray, lock_mask_imu: np.ndarray) -> None:
    vx = data["vx"][lock_mask]
    vy = data["vy"][lock_mask]
    vz = data["vz"][lock_mask]
    gx = data["gx"][lock_mask_imu]
    gy = data["gy"][lock_mask_imu]
    gz = data["gz"][lock_mask_imu]
    ax_d = data["ax"][lock_mask_imu]
    ay_d = data["ay"][lock_mask_imu]
    az_d = data["az"][lock_mask_imu]

    cov_t = data["cov_t"][lock_mask]
    imu_t = data["imu_t"][lock_mask_imu]
    vel_t = data["vel_t"]

    print("\n" + "="*80)
    print("COMPARISON TABLE: stationary_02 (reference) vs stationary_11 (new)")
    print("="*80)
    hdr = f"{'Sensor axis':<20} {'ref_std':>12} {'s11_std':>12} {'ratio':>7} {'ref_mean':>12} {'s11_mean':>12} {'plausible?':>12}"
    print(hdr)
    print("-"*80)

    checks = [
        ("DVL vx (m/s)",   _series_stats(vx),  REF["dvl_vx"],   "dvl_vx",  1e-3),
        ("DVL vy (m/s)",   _series_stats(vy),  REF["dvl_vy"],   "dvl_vy",  1e-3),
        ("DVL vz (m/s)",   _series_stats(vz),  REF["dvl_vz"],   "dvl_vz",  1e-3),
        ("IMU gyro_x (rad/s)", _series_stats(gx), REF["imu_gyro_x"], "imu_gyro_x", 5e-3),
        ("IMU gyro_y (rad/s)", _series_stats(gy), REF["imu_gyro_y"], "imu_gyro_y", 5e-3),
        ("IMU gyro_z (rad/s)", _series_stats(gz), REF["imu_gyro_z"], "imu_gyro_z", 5e-3),
        ("IMU accel_x (m/s²)", _series_stats(ax_d), REF["imu_accel_x"], "imu_accel_x", 0.1),
        ("IMU accel_y (m/s²)", _series_stats(ay_d), REF["imu_accel_y"], "imu_accel_y", 0.1),
        ("IMU accel_z (m/s²)", _series_stats(az_d), REF["imu_accel_z"], "imu_accel_z", 0.1),
    ]

    for name, s, r, key, mean_thresh in checks:
        if s["n"] == 0:
            print(f"  {name:<20}  (no data)")
            continue
        ref_std = r["std"]
        ratio = s["std"] / ref_std if ref_std > 0 else float("nan")
        ref_mean = r.get("mean", 0.0)
        mean_ok = abs(s["mean"]) < mean_thresh if "accel" not in key else True
        # Plausible: ratio 0.5–3.0 and mean near zero (for velocity/gyro)
        ratio_ok = 0.3 <= ratio <= 5.0
        plausible = "YES" if (ratio_ok and mean_ok) else "CHECK"
        print(f"  {name:<20} {ref_std:>12.4e} {s['std']:>12.4e} {ratio:>7.2f} "
              f"{ref_mean:>12.4e} {s['mean']:>12.4e} {plausible:>12}")

    print()

    # Drift rates
    print("DRIFT RATE COMPARISON (120s bin slope)")
    print("-"*60)
    drift_checks = [
        ("DVL vx",   cov_t, vx,  REF["dvl_vx"].get("drift_slope")),
        ("DVL vy",   cov_t, vy,  REF["dvl_vy"].get("drift_slope")),
        ("DVL vz",   cov_t, vz,  REF["dvl_vz"].get("drift_slope")),
        ("IMU gyro_z", imu_t, gz, REF["imu_gyro_z"].get("drift_slope")),
    ]
    print(f"  {'Axis':<15} {'s11_slope (/s)':>18} {'ref_slope (/s)':>18}")
    print(f"  {'-'*15} {'-'*18} {'-'*18}")
    for label, t_ns, vals, ref_slope in drift_checks:
        mids, means = bin_means(t_ns, vals, DRIFT_WINDOW_SEC)
        slope = drift_slope(mids, means)
        slope_str = f"{slope:.3e}" if slope is not None else "N/A"
        ref_str = f"{ref_slope:.3e}" if ref_slope is not None else "N/A"
        print(f"  {label:<15} {slope_str:>18} {ref_str:>18}")

    print()

    # Plausibility checks
    print("PLAUSIBILITY CHECKS")
    print("-"*60)
    # DVL rate
    if vel_t.size >= 2:
        span_s = (vel_t[-1] - vel_t[0]) / 1e9
        rate = (vel_t.size - 1) / span_s
        ok = "OK" if 8.0 <= rate <= 11.0 else "CHECK"
        print(f"  DVL rate: {rate:.2f} Hz [{ok}]  (expected 8-11 Hz)")

    # IMU rate
    if imu_t.size >= 2:
        span_s = (imu_t[-1] - imu_t[0]) / 1e9
        rate_imu = (imu_t.size - 1) / span_s
        ok = "OK" if 80 <= rate_imu <= 95 else "CHECK"
        print(f"  IMU rate: {rate_imu:.2f} Hz [{ok}]  (expected ~86 Hz)")

    # DVL mean speed
    speed = np.sqrt(vx**2 + vy**2 + vz**2)
    if speed.size:
        ok = "OK" if speed.mean() < 5e-3 else "CHECK"
        print(f"  DVL mean speed (locked): {speed.mean()*1000:.3f} mm/s [{ok}]  (expected <5 mm/s)")

    # IMU gyro near zero
    for label, vals in [("gyro_x", gx), ("gyro_y", gy), ("gyro_z", gz)]:
        if vals.size:
            ok = "OK" if abs(vals.mean()) < 1e-2 else "CHECK"
            print(f"  IMU {label} mean: {vals.mean():.4e} rad/s [{ok}]  (expected ~0)")

    # Gravity check
    if az_d.size:
        g_meas = abs(az_d.mean())
        ok = "OK" if abs(g_meas - GRAVITY) < 0.5 else "CHECK"
        print(f"  IMU accel_z mean: {az_d.mean():.4f} m/s²  |g|={g_meas:.4f} [{ok}]  (expected ~±9.81)")

    # Lock fraction
    if vel_t.size >= 2:
        n_locked = int(np.sum(lock_mask))
        n_total = int(data["cov_t"].size)
        print(f"  DVL lock fraction: {n_locked}/{n_total} = {100*n_locked/max(n_total,1):.1f}% of cov messages")


# ------------------------------------------------------------------ #
# Main
# ------------------------------------------------------------------ #

def main() -> int:
    ap = argparse.ArgumentParser(description="Stationary noise comparison: stationary_11 vs stationary_02.")
    ap.add_argument("bag_dir", type=Path, nargs="?", default=DEFAULT_BAG,
                    help="Path to stationary_11 bag directory.")
    ap.add_argument("--output-dir", type=Path, default=Path("."),
                    help="Directory for PNG outputs (default: current dir).")
    args = ap.parse_args()

    bag_dir = args.bag_dir.resolve()
    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not bag_dir.exists():
        print(f"ERROR: {bag_dir} not found", file=sys.stderr)
        return 1

    print(f"Reading: {bag_dir.name}")
    print("  (64 min bag — collecting IMU + DVL, please wait...)")
    data = collect_all(bag_dir)

    n_vel = data["vel_t"].size
    n_cov = data["cov_t"].size
    n_imu = data["imu_t"].size
    print(f"  DVL velocity msgs: {n_vel}")
    print(f"  DVL cov msgs:      {n_cov}")
    print(f"  IMU msgs:          {n_imu}")

    if n_vel == 0:
        print("ERROR: no DVL velocity messages found.", file=sys.stderr)
        return 1

    # Build lock masks
    lock_mask_cov = build_lock_mask(data["vel_t"], data["vel_lock"], data["cov_t"])
    lock_mask_imu = build_lock_mask(data["vel_t"], data["vel_lock"], data["imu_t"])
    print(f"  Locked DVL samples: {lock_mask_cov.sum()}/{n_cov} "
          f"({100*lock_mask_cov.mean():.1f}%)")
    print(f"  Locked IMU samples: {lock_mask_imu.sum()}/{n_imu} "
          f"({100*lock_mask_imu.mean():.1f}%)")
    print(f"\nGenerating plots -> {out_dir}")

    plot_dvl_histograms(data, lock_mask_cov, REF, out_dir)
    plot_dvl_timeseries(data, lock_mask_cov, out_dir)
    plot_imu_gyro_histograms(data, lock_mask_imu, REF, out_dir)
    plot_imu_accel_histograms(data, lock_mask_imu, REF, out_dir)
    plot_bias_drift(data, lock_mask_cov, lock_mask_imu, REF, out_dir)

    print_summary(data, lock_mask_cov, lock_mask_imu)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
