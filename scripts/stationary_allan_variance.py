#!/usr/bin/env python3
"""
Allan variance (OADEV) analysis -- stationary_11 (2026-04-19, Prompt 9).

Computes overlapping Allan deviation for:
  DVL vx, vy, vz  (~9.5 Hz, bottom-lock-masked)
  IMU gyro x/y/z  (~86 Hz)
  IMU accel x/y/z (~86 Hz, informational)

Noise parameter extraction per signal:
  N  -- white-noise (velocity/angle random walk), ADEV slope -1/2
  B  -- bias instability floor (ADEV minimum)
  K  -- rate random walk, ADEV slope +1/2

EKF connection:
  R  = N^2 * f_sample   (per-measurement noise variance, goes into EKF R)
  Q  = K^2              (process noise spectral density, goes into EKF Q)

Trustworthy tau: up to T/5 = 768 s  (T = 64.2 min recording)

Outputs:
  stationary11_allan_dvl.png
  stationary11_allan_imu_gyro.png
  stationary11_allan_imu_accel.png
  stationary11_allan_params.json

Dependencies: pip install rosbags numpy matplotlib allantools
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

try:
    import allantools
except ImportError:
    print("Install: pip install allantools", file=sys.stderr)
    raise

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
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
# Constants
# ------------------------------------------------------------------ #

TOPIC_IMU = "/imu/data"
TOPIC_VEL = dvl_mod.TOPIC_VEL   # /sensors/dvl/velocity
TOPIC_COV = dvl_mod.TOPIC_COV   # /sensors/dvl/odometry_cov

DEFAULT_BAG = Path(
    r"C:\Users\gleb0\Downloads\rosbags (2)\rosbags"
    r"\stationary_11_2026_04_19-16_18_49"
)

TRUST_TAU_MAX_S = 768.0   # T/5 where T = 64.2 min

# Reference N from stationary_02: derived as sigma_02 / sqrt(f=9.5)
# sigma_02: vx=7.47e-3, vy=2.80e-3, vz=3.07e-4, gyro_z=1.4506e-3
REF_N_DVL_VX = 2.42e-3   # m/sqrt(s)
REF_N_DVL_VY = 9.09e-4
REF_N_DVL_VZ = 9.97e-5
REF_N_IMU_GZ = 1.56e-4   # rad/sqrt(s)  (sigma/sqrt(86))

# Dark-theme styling
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
    "legend.facecolor": "#141a20",
    "legend.edgecolor": "#1e2832",
})

SIGNAL_COLORS = [
    "#3dd6c6",  # teal
    "#e8b86d",  # amber
    "#a78bfa",  # violet
]


# ------------------------------------------------------------------ #
# Data collection
# ------------------------------------------------------------------ #

def _stamp_ns(msg: Any) -> int:
    return int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)


def collect_all(bag_dir: Path) -> dict[str, np.ndarray]:
    """Single-pass: IMU + DVL velocity (lock) + DVL odometry_cov."""
    imu_t, gx, gy, gz, ax, ay, az = [], [], [], [], [], [], []
    vel_t, vel_lock = [], []
    cov_t, vx, vy, vz = [], [], [], []

    wanted = {TOPIC_IMU, TOPIC_VEL, TOPIC_COV}
    with AnyReader([bag_dir]) as reader:
        conns = [c for c in reader.connections if c.topic in wanted]
        for conn, _log_ns, raw in reader.messages(connections=conns):
            try:
                msg = reader.deserialize(raw, conn.msgtype)
                st = _stamp_ns(msg)
            except Exception:
                continue
            if st == 0:
                continue
            t = conn.topic
            if t == TOPIC_IMU:
                imu_t.append(st)
                gx.append(float(msg.angular_velocity.x))
                gy.append(float(msg.angular_velocity.y))
                gz.append(float(msg.angular_velocity.z))
                ax.append(float(msg.linear_acceleration.x))
                ay.append(float(msg.linear_acceleration.y))
                az.append(float(msg.linear_acceleration.z))
            elif t == TOPIC_VEL:
                vel_t.append(st)
                vel_lock.append(bool(msg.beam_velocities_valid))
            elif t == TOPIC_COV:
                cov_t.append(st)
                vx.append(float(msg.twist.twist.linear.x))
                vy.append(float(msg.twist.twist.linear.y))
                vz.append(float(msg.twist.twist.linear.z))

    def _sort(ts: list, *arrs: list) -> tuple[np.ndarray, ...]:
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
        "cov_t": cov_t_s, "vx": vx_s, "vy": vy_s, "vz": vz_s,
    }


def build_lock_mask(
    vel_t: np.ndarray, vel_lock: np.ndarray, query_t: np.ndarray
) -> np.ndarray:
    if vel_t.size < 2:
        return np.ones(query_t.size, dtype=bool)
    intervals = tep._include_lock_intervals_from_stamp_lock(
        list(zip(vel_t.tolist(), vel_lock.tolist()))
    )
    mask = np.zeros(query_t.size, dtype=bool)
    for a, b in intervals:
        mask |= (query_t >= a) & (query_t < b)
    return mask


def _span_rate(timestamps_ns: np.ndarray) -> float:
    """Average rate over the full recording span.

    More robust than median-dt for IMU data where the Xsens driver publishes
    several messages sharing the same header.stamp (bursts), making median-dt
    estimate the inter-burst gap rather than the true sample rate.
    """
    if timestamps_ns.size < 2:
        return 1.0
    t_span = (float(timestamps_ns[-1]) - float(timestamps_ns[0])) / 1e9
    if t_span <= 0:
        return 1.0
    return (timestamps_ns.size - 1) / t_span


# ------------------------------------------------------------------ #
# Allan deviation computation
# ------------------------------------------------------------------ #

def compute_oadev(
    data: np.ndarray, rate: float, trust_max_tau: float = TRUST_TAU_MAX_S
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """OADEV truncated to the trustworthy tau range."""
    tau_a, adev_a, adev_err_a, _ = allantools.oadev(
        data, rate=rate, data_type="freq", taus="decade"
    )
    tau_a = np.asarray(tau_a, dtype=np.float64)
    adev_a = np.asarray(adev_a, dtype=np.float64)
    adev_err_a = np.asarray(adev_err_a, dtype=np.float64)
    ok = (tau_a <= trust_max_tau) & (adev_a > 0) & np.isfinite(adev_a)
    return tau_a[ok], adev_a[ok], adev_err_a[ok]


def extract_params(
    tau: np.ndarray, adev: np.ndarray, rate: float
) -> dict[str, float | None]:
    """
    N = adev(tau_0) * sqrt(tau_0)  -- white-noise intercept (shortest tau)
    B = min(adev)                  -- bias instability floor
    K = adev(tau_N) / sqrt(tau_N/3) -- rate random walk (longest tau)

    EKF:
      R = N^2 * f_sample
      Q = K^2
    """
    if len(tau) == 0:
        return {"N": None, "B": None, "K": None}
    N = float(adev[0] * math.sqrt(tau[0]))
    B = float(np.min(adev))
    tau_last = float(tau[-1])
    K = float(adev[-1] / math.sqrt(tau_last / 3.0))
    R = N ** 2 * rate
    Q = K ** 2
    # Log-log slope at each consecutive pair (for diagnosing regime)
    if len(tau) >= 2:
        log_tau = np.log10(tau)
        log_adev = np.log10(adev)
        slopes = np.diff(log_adev) / np.diff(log_tau)
        slope_at_first = float(slopes[0])
        slope_at_last = float(slopes[-1])
    else:
        slope_at_first = slope_at_last = float("nan")

    return {
        "N": N, "B": B, "K": K,
        "R": R, "Q": Q,
        "f_sample": rate,
        "tau_min": float(tau[0]),
        "tau_max": tau_last,
        "slope_at_first_tau": slope_at_first,
        "slope_at_last_tau": slope_at_last,
    }


# ------------------------------------------------------------------ #
# Plotting
# ------------------------------------------------------------------ #

def _slope_line(
    ax: plt.Axes, tau_ref: float, adev_ref: float, slope: float,
    tau_lo: float, tau_hi: float, color: str, ls: str = "--", lw: float = 1.0
) -> None:
    """Draw a reference power-law line through (tau_ref, adev_ref)."""
    t = np.array([tau_lo, tau_hi])
    a = adev_ref * (t / tau_ref) ** slope
    ax.plot(t, a, ls=ls, color=color, lw=lw, alpha=0.6)


def plot_adev_group(
    signals: list[tuple[str, np.ndarray, np.ndarray, np.ndarray, dict]],
    title: str,
    out_path: Path,
    unit_label: str,
    trust_tau: float = TRUST_TAU_MAX_S,
) -> None:
    """
    Log-log ADEV plot for a group of signals.

    signals: list of (label, tau, adev, adev_err, params)
    """
    fig, ax = plt.subplots(figsize=(11, 7))

    all_tau = np.concatenate([s[1] for s in signals if len(s[1]) > 0])
    all_adev = np.concatenate([s[2] for s in signals if len(s[2]) > 0])

    if all_tau.size == 0:
        ax.text(0.5, 0.5, "No data", transform=ax.transAxes,
                ha="center", va="center", color="#e6edf3")
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        return

    tau_lo = float(all_tau.min())
    tau_hi = float(all_tau.max())
    adev_lo = float(all_adev.min())
    adev_hi = float(all_adev.max())

    # Reference slope lines (drawn once, at median ADEV level, neutral gray)
    adev_mid = math.sqrt(adev_lo * adev_hi)
    tau_mid = math.sqrt(tau_lo * tau_hi)
    _slope_line(ax, tau_mid, adev_mid, -0.5, tau_lo, tau_hi / 2,
                color="#555e68", ls="--", lw=1.2)
    _slope_line(ax, tau_mid, adev_mid, +0.5, tau_lo * 2, tau_hi,
                color="#555e68", ls=":", lw=1.2)

    for i, (label, tau, adev, adev_err, params) in enumerate(signals):
        if len(tau) == 0:
            continue
        color = SIGNAL_COLORS[i % len(SIGNAL_COLORS)]
        ax.loglog(tau, adev, "o-", color=color, lw=1.8, ms=5,
                  label=label, zorder=3)
        # Error shading
        if adev_err is not None and len(adev_err) == len(adev):
            lo = np.maximum(adev - adev_err, 1e-20)
            hi = adev + adev_err
            ax.fill_between(tau, lo, hi, color=color, alpha=0.12)

        # Annotate N (first point) and K (last point)
        N = params.get("N")
        K = params.get("K")
        B = params.get("B")
        if N is not None and len(tau) >= 1:
            ax.annotate(
                f"N={N:.2e}",
                xy=(tau[0], adev[0]),
                xytext=(tau[0] * 1.5, adev[0] * 2.5),
                fontsize=7.5, color=color,
                arrowprops=dict(arrowstyle="->", color=color, lw=0.8,
                                connectionstyle="arc3,rad=0.1"),
            )
        if K is not None and len(tau) >= 1:
            ax.annotate(
                f"K={K:.2e}",
                xy=(tau[-1], adev[-1]),
                xytext=(tau[-1] * 0.4, adev[-1] * 2.5),
                fontsize=7.5, color=color,
                arrowprops=dict(arrowstyle="->", color=color, lw=0.8,
                                connectionstyle="arc3,rad=-0.1"),
            )
        idx_min = int(np.argmin(adev))
        if B is not None:
            ax.scatter([tau[idx_min]], [adev[idx_min]], s=60,
                       color=color, marker="D", zorder=5)

    # T/5 trustworthy boundary
    ax.axvline(trust_tau, color="#ff6b6b", lw=1.3, ls="-.",
               label=f"T/5 = {trust_tau:.0f} s (trust limit)")

    # Legend additions for reference slope lines
    legend_extras = [
        Line2D([0], [0], ls="--", color="#555e68", lw=1.2, label="slope -1/2  (white noise)"),
        Line2D([0], [0], ls=":", color="#555e68", lw=1.2, label="slope +1/2  (rate random walk)"),
    ]
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles + legend_extras, labels + [e.get_label() for e in legend_extras],
              fontsize=8, labelcolor="#e6edf3", loc="best")

    ax.set_xlabel("Averaging time tau (s)", fontsize=11)
    ax.set_ylabel(f"OADEV  [{unit_label}]", fontsize=11)
    ax.set_title(title, fontsize=12, pad=10, color="#e6edf3")
    ax.grid(True, which="both", lw=0.4, color="#1e2832")
    ax.grid(True, which="major", lw=0.8, color="#2a3540")
    for sp in ax.spines.values():
        sp.set_edgecolor("#1e2832")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, facecolor="#0c0f12")
    plt.close(fig)
    print(f"Saved: {out_path.name}")


# ------------------------------------------------------------------ #
# Main
# ------------------------------------------------------------------ #

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Overlapping Allan deviation for stationary_11 (DVL + IMU)"
    )
    ap.add_argument("bag_dir", nargs="?", type=Path, default=DEFAULT_BAG)
    ap.add_argument("--output-dir", type=Path, default=Path("."))
    ap.add_argument("--trust-tau", type=float, default=TRUST_TAU_MAX_S,
                    help="Max trustworthy tau in seconds (default: 768 = T/5)")
    args = ap.parse_args()

    bag_dir = args.bag_dir.resolve()
    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    trust_tau = args.trust_tau

    if not bag_dir.exists():
        print(f"ERROR: bag not found: {bag_dir}", file=sys.stderr)
        return 1

    print(f"Bag:  {bag_dir.name}")
    print(f"Trust tau max: {trust_tau:.0f} s  (T/5 = {trust_tau/60:.1f} min)")
    print()

    print("Reading bag (single pass)...")
    d = collect_all(bag_dir)

    # Lock mask for DVL cov
    lock_mask = build_lock_mask(d["vel_t"], d["vel_lock"], d["cov_t"])
    lock_frac = float(lock_mask.sum()) / max(lock_mask.size, 1)
    print(f"DVL lock fraction: {lock_frac:.1%}  "
          f"({lock_mask.sum()} / {lock_mask.size} cov samples)")

    cov_t = d["cov_t"][lock_mask]
    vx = d["vx"][lock_mask]
    vy = d["vy"][lock_mask]
    vz = d["vz"][lock_mask]

    # Sample rates (span-based: robust against Xsens burst timestamps)
    rate_dvl = _span_rate(cov_t)
    rate_imu = _span_rate(d["imu_t"])

    print(f"DVL  sample rate: {rate_dvl:.3f} Hz  (n={cov_t.size})")
    print(f"IMU  sample rate: {rate_imu:.3f} Hz  (n={d['imu_t'].size})")
    print()

    # ------------------------------------------------------------------ #
    # Compute OADEV for each signal
    # ------------------------------------------------------------------ #

    results: dict[str, dict[str, Any]] = {}

    def _run(name: str, data: np.ndarray, rate: float) -> dict[str, Any] | None:
        if data.size < 64:
            print(f"  SKIP {name}: too few samples ({data.size})")
            return None
        tau, adev, adev_err = compute_oadev(data, rate, trust_tau)
        if len(tau) == 0:
            print(f"  SKIP {name}: OADEV returned no valid points")
            return None
        params = extract_params(tau, adev, rate)
        rec = {"tau": tau.tolist(), "adev": adev.tolist(),
               "adev_err": adev_err.tolist(), "params": params}
        results[name] = rec
        N = params.get("N"); B = params.get("B"); K = params.get("K")
        R = params.get("R"); Q = params.get("Q")
        s0 = params.get("slope_at_first_tau", float("nan"))
        sN = params.get("slope_at_last_tau", float("nan"))
        print(f"  {name:<12}  N={N:.3e}  B={B:.3e}  K={K:.3e}  "
              f"R={R:.3e}  Q={Q:.3e}  "
              f"slopes: first={s0:+.2f} last={sN:+.2f}")
        return rec

    print("Computing OADEV (DVL)...")
    _run("dvl_vx", vx, rate_dvl)
    _run("dvl_vy", vy, rate_dvl)
    _run("dvl_vz", vz, rate_dvl)

    print("Computing OADEV (IMU gyro)...")
    _run("imu_gx", d["gx"], rate_imu)
    _run("imu_gy", d["gy"], rate_imu)
    _run("imu_gz", d["gz"], rate_imu)

    print("Computing OADEV (IMU accel)...")
    _run("imu_ax", d["ax"], rate_imu)
    _run("imu_ay", d["ay"], rate_imu)
    _run("imu_az", d["az"], rate_imu)
    print()

    # ------------------------------------------------------------------ #
    # Plots
    # ------------------------------------------------------------------ #

    def _sig(name: str, label: str) -> tuple | None:
        r = results.get(name)
        if not r:
            return None
        return (label,
                np.array(r["tau"]), np.array(r["adev"]), np.array(r["adev_err"]),
                r["params"])

    dvl_sigs = [s for s in [
        _sig("dvl_vx", "vx"), _sig("dvl_vy", "vy"), _sig("dvl_vz", "vz"),
    ] if s is not None]
    if dvl_sigs:
        plot_adev_group(
            dvl_sigs,
            f"DVL A50 Velocity  --  Overlapping Allan Deviation\n{bag_dir.name}",
            out_dir / "stationary11_allan_dvl.png",
            "m/s", trust_tau,
        )

    gyro_sigs = [s for s in [
        _sig("imu_gx", "gyro_x"), _sig("imu_gy", "gyro_y"), _sig("imu_gz", "gyro_z"),
    ] if s is not None]
    if gyro_sigs:
        plot_adev_group(
            gyro_sigs,
            f"Xsens IMU Gyroscope  --  Overlapping Allan Deviation\n{bag_dir.name}",
            out_dir / "stationary11_allan_imu_gyro.png",
            "rad/s", trust_tau,
        )

    accel_sigs = [s for s in [
        _sig("imu_ax", "accel_x"), _sig("imu_ay", "accel_y"), _sig("imu_az", "accel_z"),
    ] if s is not None]
    if accel_sigs:
        plot_adev_group(
            accel_sigs,
            f"Xsens IMU Accelerometer  --  Overlapping Allan Deviation\n{bag_dir.name}  [informational]",
            out_dir / "stationary11_allan_imu_accel.png",
            "m/s^2", trust_tau,
        )

    # ------------------------------------------------------------------ #
    # Summary table
    # ------------------------------------------------------------------ #

    print("=" * 88)
    print("Allan Variance Parameters  (stationary_11 lock-masked)")
    print("=" * 88)
    hdr = f"{'Signal':<12} {'N':>11} {'B':>11} {'K':>11} {'R=N^2*f':>13} {'Q=K^2':>11}  unit"
    print(hdr)
    print("-" * 88)

    groups = [
        ("dvl_vx", "DVL vx", "m/s"),
        ("dvl_vy", "DVL vy", "m/s"),
        ("dvl_vz", "DVL vz", "m/s"),
        ("imu_gz", "IMU gz", "rad/s"),
        ("imu_gx", "IMU gx", "rad/s"),
        ("imu_gy", "IMU gy", "rad/s"),
        ("imu_ax", "IMU ax", "m/s^2"),
        ("imu_ay", "IMU ay", "m/s^2"),
        ("imu_az", "IMU az", "m/s^2"),
    ]
    for key, label, unit in groups:
        r = results.get(key)
        if not r:
            continue
        p = r["params"]
        def _f(v: Any) -> str:
            return f"{v:.3e}" if isinstance(v, float) else "n/a"
        print(f"{label:<12} {_f(p.get('N')):>11} {_f(p.get('B')):>11} "
              f"{_f(p.get('K')):>11} {_f(p.get('R')):>13} "
              f"{_f(p.get('Q')):>11}  {unit}")

    print()
    print("=" * 72)
    print("EKF-relevant comparison: N (stationary_11) vs stationary_02 reference")
    print("=" * 72)
    print(f"{'Signal':<12} {'N_11':>11} {'N_02 ref':>12} {'ratio':>8}  note")
    print("-" * 72)

    refs = [
        ("dvl_vx", "DVL vx", REF_N_DVL_VX, "m/sqrt(s)"),
        ("dvl_vy", "DVL vy", REF_N_DVL_VY, "m/sqrt(s)"),
        ("dvl_vz", "DVL vz", REF_N_DVL_VZ, "m/sqrt(s)"),
        ("imu_gz", "IMU gz", REF_N_IMU_GZ, "rad/sqrt(s)"),
    ]
    for key, label, ref_N, unit in refs:
        r = results.get(key)
        if not r:
            continue
        N = r["params"].get("N")
        if N is None:
            continue
        ratio = N / ref_N if ref_N else float("nan")
        print(f"{label:<12} {N:>11.3e} {ref_N:>12.3e} {ratio:>8.2f}  {unit}")

    print()
    print("R values (measurement noise variance for EKF):")
    for key, label, _ in groups[:3]:
        r = results.get(key)
        if not r:
            continue
        R = r["params"].get("R")
        if R:
            print(f"  {label}: R = {R:.4e} m^2/s^2")

    print()
    print("Q values (process noise PSD for EKF, goes into Q matrix):")
    for key, label, _ in groups[:3]:
        r = results.get(key)
        if not r:
            continue
        K = r["params"].get("K")
        Q = r["params"].get("Q")
        slope = r["params"].get("slope_at_last_tau", float("nan"))
        flag = "" if abs(slope - 0.5) < 0.3 else "  [slope not +0.5, K unreliable]"
        if Q:
            print(f"  {label}: K = {K:.3e}  Q = K^2 = {Q:.4e} m^2/s^3{flag}")

    # ------------------------------------------------------------------ #
    # Save JSON
    # ------------------------------------------------------------------ #

    json_out: dict[str, Any] = {}
    for name, r in results.items():
        p = {k: v for k, v in r["params"].items()}
        # Include tau/adev arrays for reproducibility
        p["tau_s"] = r["tau"]
        p["adev"] = r["adev"]
        json_out[name] = p

    json_path = out_dir / "stationary11_allan_params.json"
    with open(json_path, "w", encoding="utf-8") as fh:
        json.dump(json_out, fh, indent=2)
    print(f"\nSaved: {json_path.name}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
