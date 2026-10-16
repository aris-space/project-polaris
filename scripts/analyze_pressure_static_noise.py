"""
Stationary pressure-noise characterization for the EKF z measurement.

Reads a static rosbag, extracts /sensors/pressure/pose_enu (pose.position.z) and
/pixhawk/scaled_pressure (fluid_pressure, Pa), and reports raw / linearly-detrended /
high-frequency variance estimates. Compares the detrended std against the z_variance
currently set in the pressure_pose_pkg launch file (default R = 4e-6 m^2, sigma = 2 mm)
and prints a verdict on whether R is too low / realistic / too high.

Caveat: if the vehicle was not rigidly fixed (e.g. floating in water), the residual
variance is an upper bound on true sensor noise — it includes water-column dynamics
(micro-waves, currents, thermal stratification) and tether motion.

Usage:
    python scripts/analyze_pressure_static_noise.py /path/to/bag_dir [--current-r 4e-6]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Force UTF-8 stdout on Windows so the unicode symbols below don't crash cp1252.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mcap_ros2.reader import read_ros2_messages

# ── constants ─────────────────────────────────────────────────────────────────

TOPIC_POSE     = "/sensors/pressure/pose_enu"
TOPIC_PRESSURE = "/pixhawk/scaled_pressure"

WATER_DENSITY_KG_M3 = 1000.0
GRAVITY_M_S2        = 9.80665   # matches pressure_z_ned_to_pose_node.py default

DEFAULT_CURRENT_R = 4.0e-6      # m^2, per pressure_z_ned_to_pose.launch.py

# Verdict thresholds on sigma ratio (sigma_measured / sigma_R).
SIGMA_LOW_RATIO  = 0.5
SIGMA_HIGH_RATIO = 1.5


# ── dark theme ────────────────────────────────────────────────────────────────

plt.rcParams.update({
    "figure.facecolor": "#0c0f12",
    "axes.facecolor":   "#141a20",
    "axes.edgecolor":   "#1e2832",
    "axes.labelcolor":  "#8b9caa",
    "xtick.color":      "#8b9caa",
    "ytick.color":      "#8b9caa",
    "text.color":       "#e6edf3",
    "grid.color":       "#1e2832",
    "grid.linewidth":   0.5,
    "legend.facecolor": "#141a20",
    "legend.edgecolor": "#1e2832",
})

C_Z          = "#3dd6c6"   # teal — z time series
C_TREND      = "#e8b86d"   # amber — linear trend
C_RESID      = "#a78bfa"   # purple — detrended residual
C_SIGMA_R    = "#ff6b6b"   # red — current R band
C_SIGMA_MEAS = "#3dd6c6"   # teal — measured-sigma band
C_HIST       = "#a78bfa"   # purple — histogram bars


# ── bag reader ────────────────────────────────────────────────────────────────

def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def read_bag(bag_dir: Path) -> dict[str, np.ndarray]:
    """Single-pass extract of pose_enu z and scaled_pressure fluid_pressure."""
    mcap_path = bag_dir / f"{bag_dir.name}_0.mcap"
    if not mcap_path.exists():
        candidates = list(bag_dir.glob("*_0.mcap"))
        if not candidates:
            print(f"ERROR: no *_0.mcap found in {bag_dir}", file=sys.stderr)
            sys.exit(1)
        mcap_path = candidates[0]

    pose_t, pose_z = [], []
    press_t, press_pa = [], []

    wanted = {TOPIC_POSE, TOPIC_PRESSURE}
    for msg in read_ros2_messages(str(mcap_path)):
        topic = msg.channel.topic
        if topic not in wanted:
            continue
        ros = msg.ros_msg
        # Header stamp if non-zero, else fall back to bag log_time. The mavlink
        # bridge does not stamp /pixhawk/scaled_pressure, so we'd lose all of it
        # if we required header.stamp.
        try:
            t = _stamp_ns(ros.header.stamp)
        except AttributeError:
            t = 0
        if t == 0:
            t = int(msg.log_time_ns)
        if topic == TOPIC_POSE:
            pose_t.append(t)
            pose_z.append(float(ros.pose.pose.position.z))
        else:
            press_t.append(t)
            press_pa.append(float(ros.fluid_pressure))

    if not pose_z:
        print(f"ERROR: no messages on {TOPIC_POSE}", file=sys.stderr)
        sys.exit(1)
    if not press_pa:
        print(f"ERROR: no messages on {TOPIC_PRESSURE}", file=sys.stderr)
        sys.exit(1)

    return {
        "pose_t_ns":  np.array(pose_t,  dtype=np.int64),
        "pose_z":     np.array(pose_z,  dtype=np.float64),
        "press_t_ns": np.array(press_t, dtype=np.int64),
        "press_pa":   np.array(press_pa, dtype=np.float64),
    }


# ── stats ─────────────────────────────────────────────────────────────────────

def compute_stats(t_ns: np.ndarray, x: np.ndarray) -> dict:
    """Mean, std, var; linear-detrended std, var; first-difference (high-freq) var; trend slope."""
    t_s = (t_ns - t_ns[0]) / 1e9
    duration_s = float(t_s[-1])
    n = int(len(x))

    raw_mean = float(np.mean(x))
    raw_std  = float(np.std(x, ddof=1))
    raw_var  = float(np.var(x, ddof=1))

    # Linear trend via least squares: x ≈ slope * t_s + intercept.
    slope, intercept = np.polyfit(t_s, x, 1)
    trend = slope * t_s + intercept
    detrended = x - trend
    det_std = float(np.std(detrended, ddof=1))
    det_var = float(np.var(detrended, ddof=1))

    # First-difference variance: var(diff(x)) / 2 ≈ white-noise variance.
    # Robust to any monotonic drift; captures hi-freq content only.
    diffs = np.diff(x)
    hi_freq_var = float(np.var(diffs, ddof=1) / 2.0)
    hi_freq_std = float(np.sqrt(hi_freq_var))

    return {
        "n":            n,
        "duration_s":   duration_s,
        "rate_hz":      n / duration_s if duration_s > 0 else float("nan"),
        "raw_mean":     raw_mean,
        "raw_std":      raw_std,
        "raw_var":      raw_var,
        "trend_slope":  float(slope),
        "trend_intercept": float(intercept),
        "det_std":      det_std,
        "det_var":      det_var,
        "hi_freq_std":  hi_freq_std,
        "hi_freq_var":  hi_freq_var,
        "trend":        trend,
        "detrended":    detrended,
        "t_s":          t_s,
    }


def verdict(sigma_meas: float, sigma_r: float) -> tuple[str, str]:
    """Return (verdict_short, verdict_long) based on sigma ratio."""
    ratio = sigma_meas / sigma_r if sigma_r > 0 else float("inf")
    if ratio <= SIGMA_LOW_RATIO:
        return ("R_TOO_HIGH",
                f"σ_measured ({sigma_meas*1000:.2f} mm) << σ_R ({sigma_r*1000:.2f} mm). "
                f"Current R is {1.0/(ratio*ratio):.1f}× too pessimistic. Lower R toward σ²_meas.")
    if ratio <= SIGMA_HIGH_RATIO:
        return ("R_REALISTIC",
                f"σ_measured ({sigma_meas*1000:.2f} mm) ≈ σ_R ({sigma_r*1000:.2f} mm). "
                f"Current R is realistic for this condition (ratio {ratio:.2f}).")
    return ("R_TOO_LOW",
            f"σ_measured ({sigma_meas*1000:.2f} mm) > σ_R ({sigma_r*1000:.2f} mm). "
            f"Current R is {ratio*ratio:.1f}× too optimistic. Raise R to ≥ σ²_meas = {sigma_meas**2:.2e} m².")


# ── plot ──────────────────────────────────────────────────────────────────────

def _ax_style(ax):
    ax.grid(True, alpha=0.5)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def plot_three_panel(z_stats: dict, sigma_r: float, bag_name: str, out_path: Path) -> None:
    """Time series + detrended residual + histogram."""
    t_s        = z_stats["t_s"]
    z          = t_s * z_stats["trend_slope"] + z_stats["trend_intercept"] + z_stats["detrended"]
    trend      = z_stats["trend"]
    detrended  = z_stats["detrended"]
    sigma_meas = z_stats["det_std"]

    fig, axes = plt.subplots(3, 1, figsize=(11, 11),
                             gridspec_kw={"height_ratios": [1.0, 1.0, 0.8],
                                          "hspace": 0.35})

    # Panel 1: raw z + trend + ±σ_R band centered on raw mean.
    ax = axes[0]
    ax.plot(t_s, z, color=C_Z, lw=0.6, alpha=0.85, label="z (raw)")
    ax.plot(t_s, trend, color=C_TREND, lw=1.4, ls="--",
            label=f"linear trend ({z_stats['trend_slope']*1000*60:+.2f} mm/min)")
    ax.fill_between(t_s, trend - sigma_r, trend + sigma_r,
                    color=C_SIGMA_R, alpha=0.15,
                    label=f"±σ_R band (current R, σ={sigma_r*1000:.2f} mm)")
    ax.set_ylabel("z (m, ENU)")
    ax.set_title(f"Pressure z time series — {bag_name}", color="#e6edf3", fontsize=10)
    ax.legend(fontsize=8, loc="best")
    _ax_style(ax)

    # Panel 2: detrended residual + ±σ_R and ±σ_meas bands.
    ax = axes[1]
    ax.plot(t_s, detrended * 1000.0, color=C_RESID, lw=0.6, alpha=0.85,
            label="z − linear trend (mm)")
    ax.axhline(0.0, color="#8b9caa", lw=0.5, ls=":")
    ax.axhline( sigma_r * 1000.0,   color=C_SIGMA_R, lw=1.0, ls="--",
                label=f"±σ_R = {sigma_r*1000:.2f} mm")
    ax.axhline(-sigma_r * 1000.0,   color=C_SIGMA_R, lw=1.0, ls="--")
    ax.axhline( sigma_meas * 1000.0, color=C_SIGMA_MEAS, lw=1.0,
                label=f"±σ_measured = {sigma_meas*1000:.2f} mm")
    ax.axhline(-sigma_meas * 1000.0, color=C_SIGMA_MEAS, lw=1.0)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("residual (mm)")
    ax.set_title("Detrended residual (linear trend removed)",
                 color="#e6edf3", fontsize=10)
    ax.legend(fontsize=8, loc="best")
    _ax_style(ax)

    # Panel 3: histogram + Gaussian overlays.
    ax = axes[2]
    bins = 50
    counts, edges, _ = ax.hist(detrended * 1000.0, bins=bins,
                               color=C_HIST, alpha=0.6, edgecolor="#1e2832",
                               label="residual (mm)")
    centers = 0.5 * (edges[:-1] + edges[1:])
    bin_width = float(edges[1] - edges[0])
    n = len(detrended)

    def gauss(x_mm, sigma_m):
        s_mm = sigma_m * 1000.0
        return n * bin_width / (s_mm * np.sqrt(2 * np.pi)) * np.exp(-0.5 * (x_mm / s_mm) ** 2)

    x_fine = np.linspace(centers[0], centers[-1], 200)
    ax.plot(x_fine, gauss(x_fine, sigma_meas), color=C_SIGMA_MEAS, lw=1.6,
            label=f"N(0, σ²_meas)  σ={sigma_meas*1000:.2f} mm")
    ax.plot(x_fine, gauss(x_fine, sigma_r), color=C_SIGMA_R, lw=1.6, ls="--",
            label=f"N(0, σ²_R)    σ={sigma_r*1000:.2f} mm")
    ax.set_xlabel("residual (mm)")
    ax.set_ylabel("count")
    ax.set_title("Histogram of detrended residual",
                 color="#e6edf3", fontsize=10)
    ax.legend(fontsize=8, loc="best")
    _ax_style(ax)

    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_path.name}")


# ── report ────────────────────────────────────────────────────────────────────

def print_report(z_stats: dict, p_stats: dict, sigma_r: float,
                 sigma_r_var: float, bag_name: str) -> dict:
    """Print human-readable report; return the verdict dict for JSON."""
    sigma_meas = z_stats["det_std"]
    v_short, v_long = verdict(sigma_meas, sigma_r)

    # Press_abs std → equivalent depth std for cross-check.
    press_det_std_pa = p_stats["det_std"]
    eq_z_std_from_p  = press_det_std_pa / (WATER_DENSITY_KG_M3 * GRAVITY_M_S2)
    cross_check_pct  = abs(eq_z_std_from_p - sigma_meas) / sigma_meas * 100.0 if sigma_meas > 0 else float("nan")

    print(f"\n{'=' * 64}")
    print(f"Pressure z stationary-noise analysis  —  {bag_name}")
    print(f"{'=' * 64}")
    print(f"\nCAVEAT: vehicle was floating in water, not rigidly fixed.")
    print(f"        Result is an UPPER BOUND on true sensor R.")
    print(f"        Includes water-column dynamics (waves, currents, thermal).")
    print(f"\n--- Recording ---")
    print(f"  duration       : {z_stats['duration_s']:.1f} s")
    print(f"  pose_enu rate  : {z_stats['rate_hz']:.1f} Hz  (n = {z_stats['n']})")
    print(f"  pressure rate  : {p_stats['rate_hz']:.1f} Hz  (n = {p_stats['n']})")
    print(f"\n--- /sensors/pressure/pose_enu z (m, ENU) ---")
    print(f"  raw mean       : {z_stats['raw_mean']*1000:+8.3f} mm")
    print(f"  raw std        : {z_stats['raw_std']*1000:8.3f} mm   var = {z_stats['raw_var']:.3e} m²")
    print(f"  trend slope    : {z_stats['trend_slope']*1000*60:+8.3f} mm/min")
    print(f"  detrended std  : {z_stats['det_std']*1000:8.3f} mm   var = {z_stats['det_var']:.3e} m²")
    print(f"  hi-freq std    : {z_stats['hi_freq_std']*1000:8.3f} mm   var = {z_stats['hi_freq_var']:.3e} m²  (first-diff/2)")
    print(f"\n--- /pixhawk/scaled_pressure fluid_pressure (Pa) ---")
    print(f"  raw mean       : {p_stats['raw_mean']:9.1f} Pa")
    print(f"  raw std        : {p_stats['raw_std']:9.3f} Pa")
    print(f"  detrended std  : {p_stats['det_std']:9.3f} Pa")
    print(f"  → equiv z std  : {eq_z_std_from_p*1000:8.3f} mm   "
          f"(σ_P / (ρ·g), ρ=1000, g=9.80665)")
    print(f"  cross-check    : Δ vs pose_enu = {cross_check_pct:+.1f}%   "
          f"({'OK' if cross_check_pct < 5.0 else 'MISMATCH — investigate conversion node'})")
    print(f"\n--- Verdict on current R ---")
    print(f"  current R      : {sigma_r_var:.3e} m²   (σ_R = {sigma_r*1000:.2f} mm)")
    print(f"  σ_measured     : {sigma_meas*1000:.3f} mm")
    print(f"  ratio σ/σ_R    : {sigma_meas/sigma_r:.2f}")
    print(f"  → {v_short}: {v_long}")
    print(f"\n--- Hi-freq vs lo-freq decomposition ---")
    lf_var = max(0.0, z_stats["det_var"] - z_stats["hi_freq_var"])
    lf_std = float(np.sqrt(lf_var))
    print(f"  hi-freq σ      : {z_stats['hi_freq_std']*1000:.3f} mm  (sensor noise + fast water motion)")
    print(f"  lo-freq σ      : {lf_std*1000:.3f} mm  (slow water motion / drift residual after linear)")
    if z_stats["det_var"] > 0:
        print(f"  hi-freq fraction: {z_stats['hi_freq_var']/z_stats['det_var']*100:.0f}% of detrended variance")
    print(f"\n  Reading: if hi-freq << detrended, the residual is dominated by water motion")
    print(f"  (waves), not sensor noise. True sensor R ≤ hi-freq variance.")
    print(f"{'=' * 64}\n")

    return {
        "short": v_short,
        "long":  v_long,
        "ratio_sigma_meas_over_sigma_r": float(sigma_meas / sigma_r) if sigma_r > 0 else None,
        "cross_check_pct": float(cross_check_pct),
        "lo_freq_std_m":   float(lf_std),
        "hi_freq_fraction_of_detrended": float(z_stats["hi_freq_var"] / z_stats["det_var"])
                                          if z_stats["det_var"] > 0 else None,
    }


def save_metadata(z_stats: dict, p_stats: dict, sigma_r: float, sigma_r_var: float,
                  verdict_dict: dict, bag_name: str, out_path: Path) -> None:
    def clean(d: dict) -> dict:
        return {k: v for k, v in d.items() if not isinstance(v, np.ndarray)}

    meta = {
        "bag":        bag_name,
        "script":     "analyze_pressure_static_noise.py",
        "topics": {
            "pose":     TOPIC_POSE,
            "pressure": TOPIC_PRESSURE,
        },
        "constants": {
            "water_density_kg_m3": WATER_DENSITY_KG_M3,
            "gravity_m_s2":        GRAVITY_M_S2,
            "current_R_m2":        sigma_r_var,
            "current_sigma_R_m":   sigma_r,
        },
        "caveat": ("vehicle floating in water; measured variance is upper bound on "
                   "sensor noise, contains water-column dynamics (waves, currents, "
                   "thermal stratification, tether tension)"),
        "z_stats_m":    clean(z_stats),
        "press_stats_pa": clean(p_stats),
        "verdict":      verdict_dict,
    }
    out_path.write_text(json.dumps(meta, indent=2))
    print(f"Saved: {out_path.name}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def _parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path, help="Bag directory containing <name>_0.mcap")
    ap.add_argument("--current-r", type=float, default=DEFAULT_CURRENT_R,
                    help=f"Current R (z_variance) in m², for verdict comparison "
                         f"(default: {DEFAULT_CURRENT_R})")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output directory (default: bag_dir/pressure_static_noise)")
    return ap.parse_args(argv)


def main():
    args = _parse_args()
    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)
    out_dir = args.output_dir or (bag_dir / "pressure_static_noise")
    out_dir.mkdir(parents=True, exist_ok=True)

    sigma_r_var = float(args.current_r)
    sigma_r     = float(np.sqrt(sigma_r_var))

    print(f"{'=' * 60}")
    print(f"Pressure stationary-noise analysis")
    print(f"Bag       : {bag_dir.name}")
    print(f"Current R : {sigma_r_var:.3e} m²  (σ_R = {sigma_r*1000:.2f} mm)")
    print(f"Output    : {out_dir}")
    print(f"{'=' * 60}")

    data = read_bag(bag_dir)
    print(f"\nMessages: pose_enu={len(data['pose_z'])}  "
          f"scaled_pressure={len(data['press_pa'])}")

    z_stats = compute_stats(data["pose_t_ns"], data["pose_z"])
    p_stats = compute_stats(data["press_t_ns"], data["press_pa"])

    verdict_dict = print_report(z_stats, p_stats, sigma_r, sigma_r_var, bag_dir.name)

    plot_three_panel(z_stats, sigma_r, bag_dir.name,
                     out_dir / "pressure_static_noise.png")
    save_metadata(z_stats, p_stats, sigma_r, sigma_r_var, verdict_dict,
                  bag_dir.name, out_dir / "pressure_static_noise_metadata.json")


if __name__ == "__main__":
    main()
