"""
Measure the effective DVL->IMU latency on a recorded bag.

Background:
    A colleague's stack had a ~0.2 s transport delay between DVL and IMU and
    solved it by buffering the IMU. We want to know if the same problem
    affects POLARIS. The local EKF aligns sensor measurements by
    `header.stamp`. If both sensors stamp at the physical sample instant the
    EKF is fine; if the DVL effectively lags IMU even in stamp-space, every
    fused velocity update is being rotated through the wrong (later)
    orientation.

What this script does (per bag):

    1. Intra-stamp cross-correlation. Read /imu/data and
       /sensors/dvl/odometry_cov, resample both to a common 50 Hz grid by
       their own header.stamp (only across DVL-locked spans), bandpass
       filter, then cross-correlate IMU body acceleration against the
       numerical derivative of DVL body velocity. The lag at peak
       correlation = the DVL->IMU latency *as the EKF sees it*.

    2. Wall-clock arrival lag context. For each topic compute
       (bag log_time − header.stamp) — this is the on-the-wire staleness
       of each message. Big difference between IMU's and DVL's mean
       suggests one stamps at sample-time and the other at receive-time.

    3. Motion-onset overlay. Find the largest IMU-accel events in the bag
       (events where |a_x| crosses a threshold from idle), and plot 3 s
       windows of IMU accel vs DVL velocity around each event so the eye
       can confirm the cross-correlation result.

Output (under <bag_dir>/dvl_imu_lag/):
    summary.txt         — per-axis lag, peak correlation, dt distributions
    crosscorr.png       — cross-correlation curves, ±0.5 s lag window
    arrival_lag.png     — log_time − stamp histograms for IMU vs DVL
    onsets.png          — 3 s overlays around the strongest motion events

Usage:
    python scripts/dvl_imu_lag.py <bag_dir> [--output-dir DIR]

Pure-Python (rosbags + numpy + scipy + matplotlib). No ROS2 install needed.
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

# Force UTF-8 output on Windows consoles that default to cp1252.
try:
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass

import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy import signal as scisig


_TOPIC_IMU = "/imu/data"
_TOPIC_DVL = "/sensors/dvl/odometry_cov"

# Resample rate for the cross-correlation grid.
_RESAMPLE_HZ = 50.0
_RESAMPLE_DT = 1.0 / _RESAMPLE_HZ

# Bandpass: cuts DC bias + slow drift + high-freq noise. AUV manoeuvre
# *onset events* (start, stop, direction reversal) live in the 0.3-5 Hz band.
# A wider low cutoff would leave in slow velocity drift across the bag,
# whose autocorrelation is broad and hides the per-onset alignment.
_BANDPASS_LO_HZ = 0.3
_BANDPASS_HI_HZ = 5.0

# Lag search window for cross-correlation (seconds).
_LAG_WINDOW_S = 1.0

# Variance threshold (m²/s²) above which DVL is considered no-lock.
# odometry_covariance_node sets var=1e6 on no-lock; normal locked is < 0.1.
_NO_LOCK_VAR = 1.0


# Dark theme to match other diagnostic scripts in this tree.
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
    "savefig.facecolor": "#0c0f12",
})

C_IMU = "#a78bfa"
C_DVL = "#3dd6c6"
C_REF = "#ff6b6b"


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass
class Bag:
    # IMU
    imu_t: np.ndarray         # header stamp, ns
    imu_t_log: np.ndarray     # bag log_time, ns
    imu_a: np.ndarray         # (N, 3) linear acceleration, m/s²

    # DVL
    dvl_t: np.ndarray
    dvl_t_log: np.ndarray
    dvl_v: np.ndarray         # (M, 3) body velocity, m/s
    dvl_var: np.ndarray       # (M, 3) velocity covariance diag

    duration_s: float


def read_bag(bag_dir: Path) -> Bag:
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    imu = []   # (t_ns, t_log, ax, ay, az)
    dvl = []   # (t_ns, t_log, vx, vy, vz, var_x, var_y, var_z)

    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        wanted = {_TOPIC_IMU, _TOPIC_DVL}
        conns = [c for c in reader.connections if c.topic in wanted]
        for conn, t_log, raw in reader.messages(connections=conns):
            ros = reader.deserialize(raw, conn.msgtype)
            try:
                t_ns = _stamp_ns(ros.header.stamp)
            except AttributeError:
                continue
            if conn.topic == _TOPIC_IMU:
                a = ros.linear_acceleration
                imu.append((t_ns, int(t_log), float(a.x), float(a.y), float(a.z)))
            elif conn.topic == _TOPIC_DVL:
                v = ros.twist.twist.linear
                cov = list(ros.twist.covariance)
                # 6x6 row-major: [0]=var(vx), [7]=var(vy), [14]=var(vz)
                dvl.append((
                    t_ns, int(t_log),
                    float(v.x), float(v.y), float(v.z),
                    float(cov[0]), float(cov[7]), float(cov[14]),
                ))

    if not imu or not dvl:
        raise RuntimeError(
            f"Missing required topics in {bag_dir.name} "
            f"(imu={len(imu)}, dvl={len(dvl)})"
        )

    imu.sort(key=lambda r: r[0])
    dvl.sort(key=lambda r: r[0])

    imu_arr = np.asarray(imu, dtype=np.float64)
    dvl_arr = np.asarray(dvl, dtype=np.float64)

    t0 = min(imu_arr[0, 0], dvl_arr[0, 0])
    t1 = max(imu_arr[-1, 0], dvl_arr[-1, 0])

    return Bag(
        imu_t=imu_arr[:, 0].astype(np.int64),
        imu_t_log=imu_arr[:, 1].astype(np.int64),
        imu_a=imu_arr[:, 2:5],
        dvl_t=dvl_arr[:, 0].astype(np.int64),
        dvl_t_log=dvl_arr[:, 1].astype(np.int64),
        dvl_v=dvl_arr[:, 2:5],
        dvl_var=dvl_arr[:, 5:8],
        duration_s=(t1 - t0) * 1e-9,
    )


def _build_lock_intervals(dvl_t: np.ndarray, dvl_var: np.ndarray) -> list[tuple[int, int]]:
    """Spans (t_start_ns, t_end_ns) where DVL was locked and steady."""
    locked = np.all(dvl_var < _NO_LOCK_VAR, axis=1)
    intervals: list[tuple[int, int]] = []
    in_lock = False
    seg_start = 0
    for i in range(len(dvl_t)):
        if locked[i] and not in_lock:
            in_lock = True
            seg_start = int(dvl_t[i])
        elif not locked[i] and in_lock:
            in_lock = False
            # End the interval at the last locked sample's stamp.
            intervals.append((seg_start, int(dvl_t[i - 1])))
    if in_lock:
        intervals.append((seg_start, int(dvl_t[-1])))
    # Filter out very short spans — need enough samples for cross-corr.
    return [(a, b) for a, b in intervals if (b - a) > 5_000_000_000]  # > 5 s


def _resample(t_ns: np.ndarray, vals: np.ndarray, t_grid_ns: np.ndarray) -> np.ndarray:
    """Linear interpolation onto a common grid. vals: (N,) or (N, 3)."""
    t_s = t_ns.astype(np.float64) * 1e-9
    g_s = t_grid_ns.astype(np.float64) * 1e-9
    if vals.ndim == 1:
        return np.interp(g_s, t_s, vals)
    out = np.empty((g_s.size, vals.shape[1]), dtype=np.float64)
    for k in range(vals.shape[1]):
        out[:, k] = np.interp(g_s, t_s, vals[:, k])
    return out


def _bandpass(x: np.ndarray, fs: float, lo: float, hi: float) -> np.ndarray:
    nyq = 0.5 * fs
    sos = scisig.butter(4, [lo / nyq, hi / nyq], btype="band", output="sos")
    return scisig.sosfiltfilt(sos, x, axis=0)


def _crosscorr_lag(a: np.ndarray, b: np.ndarray, dt: float, window_s: float
                   ) -> tuple[float, float, np.ndarray, np.ndarray]:
    """Cross-correlate a vs b. Positive lag = a leads b (i.e., b is delayed).

    Returns (best_lag_s, peak_corr, lags_s, corr_normalized).
    """
    a = a - a.mean()
    b = b - b.mean()
    n = min(a.size, b.size)
    a, b = a[:n], b[:n]

    corr = scisig.correlate(a, b, mode="full")
    # Normalise to a Pearson-style coefficient (independent of length / amplitude).
    norm = np.sqrt(np.dot(a, a) * np.dot(b, b))
    if norm <= 0:
        return float("nan"), float("nan"), np.array([]), np.array([])
    corr = corr / norm

    # lags[k] = (k - (n-1)) samples of a relative to b; positive = a delayed
    # relative to b. We want positive lag = "DVL late vs IMU", so we'll
    # interpret the result against the order we pass arguments.
    lags = scisig.correlation_lags(a.size, b.size, mode="full")
    lags_s = lags * dt

    mask = np.abs(lags_s) <= window_s
    lags_w = lags_s[mask]
    corr_w = corr[mask]
    if corr_w.size == 0:
        return float("nan"), float("nan"), lags_w, corr_w
    k = int(np.argmax(corr_w))
    return float(lags_w[k]), float(corr_w[k]), lags_w, corr_w


def _refine_peak(lags_s: np.ndarray, corr: np.ndarray) -> float:
    """Parabolic refinement around the integer-sample peak. Sub-sample lag."""
    if corr.size < 3:
        return float("nan")
    k = int(np.argmax(corr))
    if k == 0 or k == corr.size - 1:
        return float(lags_s[k])
    y0, y1, y2 = corr[k - 1], corr[k], corr[k + 1]
    denom = (y0 - 2 * y1 + y2)
    if denom == 0:
        return float(lags_s[k])
    delta = 0.5 * (y0 - y2) / denom
    dt = lags_s[1] - lags_s[0]
    return float(lags_s[k] + delta * dt)


@dataclass
class AxisResult:
    axis: str
    lag_s: float
    lag_refined_s: float
    peak_corr: float
    n_samples: int
    lags_s: np.ndarray
    corr: np.ndarray


def analyze(bag: Bag) -> tuple[list[AxisResult], dict[str, float]]:
    """Run the per-axis cross-correlation analysis.

    Convention: `_crosscorr_lag(IMU_accel, DVL_vel_derivative)` returns lag in
    "samples of IMU relative to DVL". Positive lag = IMU is later (i.e., DVL
    leads). Negative lag = IMU is earlier (DVL lags IMU). We negate so that
    positive = "DVL late vs IMU", which is the colleague's convention.
    """
    intervals = _build_lock_intervals(bag.dvl_t, bag.dvl_var)
    if not intervals:
        raise RuntimeError("No DVL-locked spans > 5 s found in this bag.")

    # Build a single concatenated series across all locked intervals.
    grids: list[np.ndarray] = []
    imu_chunks: list[np.ndarray] = []
    dvl_chunks: list[np.ndarray] = []
    for a, b in intervals:
        n = int((b - a) * 1e-9 / _RESAMPLE_DT)
        if n < int(_BANDPASS_LO_HZ * 4 / _RESAMPLE_DT):  # need a few cycles of LO
            continue
        t_grid = a + np.arange(n) * int(_RESAMPLE_DT * 1e9)
        if t_grid[-1] > min(bag.imu_t[-1], bag.dvl_t[-1]) or t_grid[0] < max(bag.imu_t[0], bag.dvl_t[0]):
            # Trim to the overlap of the IMU and DVL stamp ranges.
            lo = max(bag.imu_t[0], bag.dvl_t[0], a)
            hi = min(bag.imu_t[-1], bag.dvl_t[-1], b)
            if hi - lo < 5_000_000_000:
                continue
            n = int((hi - lo) * 1e-9 / _RESAMPLE_DT)
            t_grid = lo + np.arange(n) * int(_RESAMPLE_DT * 1e9)
        imu_g = _resample(bag.imu_t, bag.imu_a, t_grid)        # (n, 3)
        dvl_g = _resample(bag.dvl_t, bag.dvl_v, t_grid)        # (n, 3)
        grids.append(t_grid)
        imu_chunks.append(imu_g)
        dvl_chunks.append(dvl_g)

    if not imu_chunks:
        raise RuntimeError("All locked intervals were too short for the bandpass filter.")

    # We bandpass and cross-correlate each interval separately (so phase isn't
    # polluted by the gaps between intervals), then average corr curves
    # weighted by sample count.
    sum_corr: dict[str, np.ndarray] = {}
    sum_w: dict[str, float] = {}
    lags_s_template: dict[str, np.ndarray] = {}
    n_total = {axis: 0 for axis in ("x", "y", "z")}

    for imu_g, dvl_g in zip(imu_chunks, dvl_chunks):
        # Compare *velocities*, not accelerations. Integrating IMU accel and
        # bandpassing has much better SNR than differentiating DVL velocity:
        # numerical differentiation amplifies the DVL's per-sample noise
        # (which is ~mm/s at 14 Hz, becoming ~cm/s² after gradient), whereas
        # integrating IMU accel and bandpassing collapses high-freq noise
        # into a smooth velocity signal in the manoeuvre band.
        #
        # Bandpass removes both the DC gravity component (which sits in z
        # and any pitch-roll coupling) and the integration drift.
        imu_v = np.cumsum(imu_g, axis=0) * _RESAMPLE_DT
        imu_bp = _bandpass(imu_v, _RESAMPLE_HZ, _BANDPASS_LO_HZ, _BANDPASS_HI_HZ)
        dvl_bp = _bandpass(dvl_g, _RESAMPLE_HZ, _BANDPASS_LO_HZ, _BANDPASS_HI_HZ)

        for k, axis in enumerate(("x", "y", "z")):
            # _crosscorr_lag(a=IMU, b=DVL_dot): positive lag here means
            # IMU appears later than DVL. We negate to get "DVL late vs IMU".
            _, _, lags_s, corr = _crosscorr_lag(
                imu_bp[:, k], dvl_bp[:, k], _RESAMPLE_DT, _LAG_WINDOW_S,
            )
            if lags_s.size == 0:
                continue
            lags_s_neg = -lags_s
            order = np.argsort(lags_s_neg)
            lags_s_neg = lags_s_neg[order]
            corr_neg = corr[order]
            w = imu_bp.shape[0]
            if axis not in sum_corr:
                sum_corr[axis] = w * corr_neg
                lags_s_template[axis] = lags_s_neg
                sum_w[axis] = w
            else:
                # Lengths may differ by a sample at edges — clip.
                m = min(sum_corr[axis].size, corr_neg.size)
                sum_corr[axis] = sum_corr[axis][:m] + w * corr_neg[:m]
                lags_s_template[axis] = lags_s_template[axis][:m]
                sum_w[axis] += w
            n_total[axis] += w

    results: list[AxisResult] = []
    for axis in ("x", "y", "z"):
        if axis not in sum_corr:
            continue
        corr = sum_corr[axis] / sum_w[axis]
        lags_s = lags_s_template[axis]
        k = int(np.argmax(corr))
        peak = float(corr[k])
        coarse_lag = float(lags_s[k])
        refined = _refine_peak(lags_s, corr)
        results.append(AxisResult(
            axis=axis,
            lag_s=coarse_lag,
            lag_refined_s=refined,
            peak_corr=peak,
            n_samples=n_total[axis],
            lags_s=lags_s,
            corr=corr,
        ))

    # Wall-clock arrival lag stats: log_time − header.stamp, ms.
    imu_arr_lag_ms = (bag.imu_t_log - bag.imu_t).astype(np.float64) * 1e-6
    dvl_arr_lag_ms = (bag.dvl_t_log - bag.dvl_t).astype(np.float64) * 1e-6
    arrival = {
        "imu_arr_lag_ms_p50": float(np.percentile(imu_arr_lag_ms, 50)),
        "imu_arr_lag_ms_p95": float(np.percentile(imu_arr_lag_ms, 95)),
        "dvl_arr_lag_ms_p50": float(np.percentile(dvl_arr_lag_ms, 50)),
        "dvl_arr_lag_ms_p95": float(np.percentile(dvl_arr_lag_ms, 95)),
        "diff_p50_ms": float(np.percentile(dvl_arr_lag_ms, 50) - np.percentile(imu_arr_lag_ms, 50)),
    }
    return results, arrival


def find_motion_onsets(bag: Bag, n_events: int = 6) -> list[int]:
    """Return IMU stamp indices for the strongest motion-onset events.

    "Onset" = local max of |a_x| filtered to AUV manoeuvre band. We pick the
    top-N peaks separated by at least 10 s so we don't double-count.
    """
    t_s = (bag.imu_t - bag.imu_t[0]) * 1e-9
    if t_s[-1] < 5:
        return []
    fs = 1.0 / np.median(np.diff(t_s))
    if fs < 2 * _BANDPASS_HI_HZ:
        fs = max(fs, 2 * _BANDPASS_HI_HZ + 1)
    # Bandpass surge axis only (x). May be noisy — that's fine, we just want
    # the largest events.
    a_x = bag.imu_a[:, 0] - bag.imu_a[:, 0].mean()
    if a_x.size < 32:
        return []
    try:
        sos = scisig.butter(2, [_BANDPASS_LO_HZ / (fs / 2),
                                min(_BANDPASS_HI_HZ, fs / 2 * 0.9) / (fs / 2)],
                            btype="band", output="sos")
        a_bp = scisig.sosfiltfilt(sos, a_x)
    except ValueError:
        a_bp = a_x

    # Find peaks of |a_bp|. Use scipy.signal.find_peaks with a min-distance
    # in samples corresponding to 10 s.
    min_dist = max(1, int(10.0 * fs))
    peaks, props = scisig.find_peaks(np.abs(a_bp), distance=min_dist)
    if peaks.size == 0:
        return []
    # Sort by amplitude desc, take top N.
    order = np.argsort(-np.abs(a_bp[peaks]))
    return peaks[order][:n_events].tolist()


def plot_crosscorr(results: list[AxisResult], bag_name: str, out_path: Path) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2), sharey=True)
    fig.suptitle(
        f"{bag_name}  —  DVL->IMU cross-correlation by axis  "
        f"(positive lag = DVL late vs IMU)",
        color="#e6edf3", fontsize=12,
    )
    for ax, r in zip(axes, results):
        ax.plot(r.lags_s, r.corr, color=C_DVL, linewidth=1.5)
        ax.axvline(0, color="#444c55", linewidth=1, linestyle="--")
        ax.axvline(r.lag_refined_s, color=C_REF, linewidth=1, linestyle="-")
        ax.set_xlabel("lag (s)")
        ax.set_title(
            f"axis {r.axis}   lag = {r.lag_refined_s*1000:+.0f} ms   "
            f"peak = {r.peak_corr:.2f}",
            color="#e6edf3",
        )
        ax.grid(True, alpha=0.5)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.set_xlim(-_LAG_WINDOW_S, _LAG_WINDOW_S)
    axes[0].set_ylabel("normalised cross-corr")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_arrival_lag(bag: Bag, arrival: dict, bag_name: str, out_path: Path) -> None:
    imu_lag = (bag.imu_t_log - bag.imu_t).astype(np.float64) * 1e-6
    dvl_lag = (bag.dvl_t_log - bag.dvl_t).astype(np.float64) * 1e-6

    fig, ax = plt.subplots(figsize=(10, 4.5))
    fig.suptitle(
        f"{bag_name}  —  bag log_time − header.stamp  "
        f"(transport staleness on the wire)",
        color="#e6edf3", fontsize=12,
    )
    bins = np.linspace(0, max(np.percentile(np.concatenate([imu_lag, dvl_lag]), 99), 1), 80)
    ax.hist(imu_lag, bins=bins, color=C_IMU, alpha=0.55, label=f"/imu/data  (p50 {arrival['imu_arr_lag_ms_p50']:.1f} ms,  p95 {arrival['imu_arr_lag_ms_p95']:.1f} ms)")
    ax.hist(dvl_lag, bins=bins, color=C_DVL, alpha=0.55, label=f"/sensors/dvl/odometry_cov  (p50 {arrival['dvl_arr_lag_ms_p50']:.1f} ms,  p95 {arrival['dvl_arr_lag_ms_p95']:.1f} ms)")
    ax.set_xlabel("log_time − header.stamp  (ms)")
    ax.set_ylabel("count")
    ax.legend(loc="upper right", facecolor="#141a20", edgecolor="#1e2832")
    ax.grid(True, alpha=0.5)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_onsets(bag: Bag, onsets: list[int], bag_name: str, out_path: Path) -> None:
    if not onsets:
        return
    n = min(6, len(onsets))
    fig, axes = plt.subplots(n, 3, figsize=(15, 2.4 * n + 0.6), sharex=False)
    if n == 1:
        axes = axes.reshape(1, -1)
    fig.suptitle(
        f"{bag_name}  —  motion-onset overlays  "
        f"(IMU body accel  vs  DVL body velocity, ±1.5 s window)",
        color="#e6edf3", fontsize=12,
    )
    win_ns = int(1.5 * 1e9)
    axis_labels = ("x (surge)", "y (sway)", "z (heave)")
    for r, peak_idx in enumerate(onsets[:n]):
        center = int(bag.imu_t[peak_idx])
        for c, ax in enumerate(axes[r]):
            ax2 = ax.twinx()
            t_lo = center - win_ns
            t_hi = center + win_ns
            m_imu = (bag.imu_t >= t_lo) & (bag.imu_t <= t_hi)
            m_dvl = (bag.dvl_t >= t_lo) & (bag.dvl_t <= t_hi)
            t_imu = (bag.imu_t[m_imu] - center) * 1e-9
            t_dvl = (bag.dvl_t[m_dvl] - center) * 1e-9
            ax.plot(t_imu, bag.imu_a[m_imu, c], color=C_IMU, linewidth=1.3,
                    label="IMU accel" if r == 0 and c == 0 else None)
            ax2.plot(t_dvl, bag.dvl_v[m_dvl, c], color=C_DVL, linewidth=1.3,
                     label="DVL vel" if r == 0 and c == 0 else None)
            ax.axvline(0, color="#444c55", linewidth=0.8, linestyle="--")
            if r == 0:
                ax.set_title(f"axis {axis_labels[c]}", color="#e6edf3")
            if c == 0:
                ax.set_ylabel(f"event {r+1}\nIMU a (m/s²)", color=C_IMU)
            if c == 2:
                ax2.set_ylabel("DVL v (m/s)", color=C_DVL)
            if r == n - 1:
                ax.set_xlabel("t − onset (s)")
            ax.tick_params(axis="y", colors=C_IMU)
            ax2.tick_params(axis="y", colors=C_DVL)
            ax.grid(True, alpha=0.4)
            ax.spines["top"].set_visible(False)
            ax2.spines["top"].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output dir (default: <bag_dir>/dvl_imu_lag/).")
    args = ap.parse_args()

    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        return 1

    out_dir = (args.output_dir or (bag_dir / "dvl_imu_lag")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[dvl_imu_lag] reading bag {bag_dir.name}")
    bag = read_bag(bag_dir)
    print(f"[dvl_imu_lag]   IMU: {bag.imu_t.size} msgs, "
          f"DVL: {bag.dvl_t.size} msgs, duration {bag.duration_s:.1f} s")

    results, arrival = analyze(bag)

    summary_lines = [
        f"DVL->IMU latency report — {bag_dir.name}",
        "=" * 72,
        f"Bag duration                          {bag.duration_s:.1f} s",
        f"IMU msgs                              {bag.imu_t.size} ({bag.imu_t.size / max(1,bag.duration_s):.1f} Hz)",
        f"DVL msgs                              {bag.dvl_t.size} ({bag.dvl_t.size / max(1,bag.duration_s):.1f} Hz)",
        "",
        "Cross-correlation lag (positive = DVL late vs IMU)",
        "-" * 72,
        "axis         lag         peak corr   N samples",
    ]
    for r in results:
        summary_lines.append(
            f"  {r.axis}     {r.lag_refined_s*1000:+7.1f} ms     {r.peak_corr:5.2f}      {r.n_samples}"
        )
    summary_lines += [
        "",
        "Wall-clock arrival lag (bag log_time − header.stamp)",
        "-" * 72,
        f"  /imu/data            p50  {arrival['imu_arr_lag_ms_p50']:.2f} ms  /  p95 {arrival['imu_arr_lag_ms_p95']:.2f} ms",
        f"  /sensors/dvl/cov     p50  {arrival['dvl_arr_lag_ms_p50']:.2f} ms  /  p95 {arrival['dvl_arr_lag_ms_p95']:.2f} ms",
        f"  DVL − IMU diff (p50)         {arrival['diff_p50_ms']:+.2f} ms",
        "",
    ]

    # Verdict — only trust an axis if its peak correlation is strong enough
    # for the lag estimate to be reliable. With peak corr < 0.20 the maximum
    # is broad and the lag location is dominated by noise; with peak corr
    # >= 0.20 the peak is a real feature.
    if results:
        strong = [r for r in results if r.peak_corr >= 0.20
                  and abs(r.lag_refined_s) < _LAG_WINDOW_S * 0.95]
        weak = [r for r in results if r not in strong]
        summary_lines.append("")
        summary_lines.append(
            f"Trustworthy axes (peak corr >= 0.20, peak inside window):  "
            f"{[r.axis for r in strong] or 'none'}"
        )
        summary_lines.append(
            f"Inconclusive axes (low peak corr or peak at edge):          "
            f"{[r.axis for r in weak] or 'none'}"
        )
        if strong:
            med = float(np.median([r.lag_refined_s * 1000 for r in strong]))
            summary_lines.append(f"Median lag of trustworthy axes = {med:+.0f} ms")
            if abs(med) < 30:
                summary_lines.append("=> No measurable DVL latency in stamp-space "
                                     "(< 30 ms). EKF alignment by header.stamp is fine.")
            elif abs(med) < 100:
                summary_lines.append("=> Small stamp-space lag (30-100 ms). "
                                     "Minor impact on EKF; not worth IMU buffering.")
            else:
                summary_lines.append("=> Significant stamp-space lag (>= 100 ms). "
                                     "Worth investigating ekf_local.yaml imu0_queue_size "
                                     "and the IMU 'delay' parameter, or upstream "
                                     "IMU buffering.")
        else:
            summary_lines.append("=> No axis crossed the 0.20 peak-correlation threshold. "
                                 "Either the bag has insufficient dynamic content in "
                                 "the 0.3-5 Hz manoeuvre band, or DVL and IMU are "
                                 "near-perfectly time-aligned (a flat broad correlation "
                                 "is what you get when the relative lag is small "
                                 "compared to the dominant period of motion).")

    summary = "\n".join(summary_lines)
    print()
    print(summary)
    (out_dir / "summary.txt").write_text(summary, encoding="utf-8")

    print()
    print(f"[dvl_imu_lag] writing plots to {out_dir}")
    plot_crosscorr(results, bag_dir.name, out_dir / "crosscorr.png")
    plot_arrival_lag(bag, arrival, bag_dir.name, out_dir / "arrival_lag.png")

    onsets = find_motion_onsets(bag)
    print(f"[dvl_imu_lag]   {len(onsets)} motion-onset events identified")
    if onsets:
        plot_onsets(bag, onsets, bag_dir.name, out_dir / "onsets.png")

    print(f"[dvl_imu_lag] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
