#!/usr/bin/env python3
"""
Slide 4 — characteristic failure modes from the Zermatt Schwarzsee survey.

Reads both Zermatt bags, auto-detects the most dramatic example of each of
the two failure modes that drove the post-processing filters, and writes a
single two-row PNG suitable for the presentation slide:

  Top row    — high-pitch excursion (|pitch| > 25 deg gate)
  Bottom row — Pixhawk depth-hold oscillation (depth-stability gate)

Each panel annotates the physical cause and shades the samples that the
production extraction pipeline (extract_zermatt_measurements.py) drops.

Default behaviour:
  - reads both bags
  - picks the touching session with the largest |pitch| excursion
  - picks the touching session with the largest fraction of samples that
    would be rejected by the depth-stability filter (max_depth_dev_m = 0.01)
  - writes failure_modes_zermatt.png next to this script

Usage:
    /usr/bin/python3 plot_failure_modes.py [--out FILE]
                                            [--pitch-session GP --pitch-bag BAG]
                                            [--osc-session GP   --osc-bag   BAG]

Inside the devcontainer the bags live at /ros2_ws/recordings. On the WSL
host they live at /mnt/c/Users/ridhc/Downloads. Auto-resolved below.
"""

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    sys.exit(1)

# ─── Constants (mirror config.yaml) ────────────────────────────────────────────
MAX_PITCH_DEG     = 25.0
MAX_ROLL_DEG      = 10.0
MAX_DEPTH_DEV_M   = 0.01     # depth-stability gate
DEPTH_WINDOW_S    = 30.0     # rolling local-minimum window
PRESSURE_TO_CONTACT_Z_M = 0.210
PRESSURE_TO_CONTACT_X_M = 0.515
RHO_WATER = 999.4
RHO_ICE   = 887.5
G         = 9.802
MIN_SESSION_DURATION_S = 20.0  # ignore very short touch blips when scoring

TOPIC_TOUCH = "/ice_touch_detection/touching"
TOPIC_ODOM  = "/odometry/filtered/local"
TOPIC_PRESS = "/pixhawk/scaled_pressure"
TOPIC_PSURF = "/sensors/pressure/p_surface_pa"
TOPICS = {TOPIC_TOUCH, TOPIC_ODOM, TOPIC_PRESS, TOPIC_PSURF}

# Bag candidates — pick whichever exists at runtime.
BAG_CANDIDATES = [
    [
        "/ros2_ws/recordings/zermatt_grid_01_2026_04_30-12_04_16",
        "/ros2_ws/recordings/zermatt_grid_02_2026_04_30-13_00_44",
    ],
    [
        "/mnt/c/Users/ridhc/Downloads/zermatt_grid_01_2026_04_30-12_04_16_0.mcap",
        "/mnt/c/Users/ridhc/Downloads/zermatt_grid_02_2026_04_30-13_00_44_0.mcap",
    ],
]


# ─── Geometry / physics ────────────────────────────────────────────────────────

def quat_to_rp(qx, qy, qz, qw):
    """Quaternion → (roll_deg, pitch_deg). Matches tf_transformations sxyz."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


def thickness_archimedes(p_pa, p_surf_pa, pitch_deg, roll_deg, depth_pa_override=None):
    """Identical formula to extract_zermatt_measurements.calc_thickness.
    When depth_pa_override is given, use it (in metres) as the sensor depth
    instead of recomputing from pressure."""
    if depth_pa_override is not None:
        depth_sensor = depth_pa_override
    else:
        depth_sensor = (p_pa - p_surf_pa) / (RHO_WATER * G)
    pr = math.radians(pitch_deg)
    rr = math.radians(roll_deg)
    omega = (PRESSURE_TO_CONTACT_X_M * math.sin(pr)
             + PRESSURE_TO_CONTACT_Z_M * math.cos(pr) * math.cos(rr))
    return (depth_sensor - omega) * RHO_WATER / RHO_ICE


# ─── Bag reading ───────────────────────────────────────────────────────────────

def resolve_bags(cli_bags):
    if cli_bags:
        return [Path(b) for b in cli_bags]
    for candidate in BAG_CANDIDATES:
        if all(Path(c).exists() for c in candidate):
            return [Path(c) for c in candidate]
    print("error: could not locate Zermatt bags in any known location",
          file=sys.stderr)
    sys.exit(2)


def read_bag(bag_path):
    """Return per-signal arrays + touch sessions for one bag."""
    ts_odom, depth, pitch, roll = [], [], [], []
    ts_press, press = [], []
    ts_psurf, psurf = [], []
    touch_changes = []  # [(ts, bool)]

    with AnyReader([bag_path]) as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS]
        for conn, ts_ns, data in reader.messages(connections=conns):
            ts = ts_ns * 1e-9
            msg = reader.deserialize(data, conn.msgtype)
            if conn.topic == TOPIC_ODOM:
                p = msg.pose.pose
                ts_odom.append(ts)
                depth.append(-p.position.z)
                r, pt = quat_to_rp(p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w)
                roll.append(r)
                pitch.append(pt)
            elif conn.topic == TOPIC_PRESS:
                ts_press.append(ts)
                press.append(float(msg.fluid_pressure))
            elif conn.topic == TOPIC_PSURF:
                ts_psurf.append(ts)
                psurf.append(float(msg.data))
            elif conn.topic == TOPIC_TOUCH:
                touch_changes.append((ts, bool(msg.data)))

    return {
        "bag": bag_path.name,
        "ts_odom":  np.asarray(ts_odom),
        "depth":    np.asarray(depth),
        "pitch":    np.asarray(pitch),
        "roll":     np.asarray(roll),
        "ts_press": np.asarray(ts_press),
        "press":    np.asarray(press),
        "ts_psurf": np.asarray(ts_psurf),
        "psurf":    np.asarray(psurf),
        "touch_changes": touch_changes,
    }


def find_sessions(touch_changes):
    sessions = []
    start = None
    for ts, val in touch_changes:
        if val and start is None:
            start = ts
        elif not val and start is not None:
            sessions.append((start, ts))
            start = None
    return sessions


# ─── Session scoring ───────────────────────────────────────────────────────────

def slice_session(sig, t0, t1):
    """Return arrays for the touching session [t0, t1]."""
    m = (sig["ts_odom"] >= t0) & (sig["ts_odom"] <= t1)
    t = sig["ts_odom"][m]
    return {
        "t":     t,
        "depth": sig["depth"][m],
        "pitch": sig["pitch"][m],
        "roll":  sig["roll"][m],
    }


def interp_pressure(sig, ts):
    """Linear-interpolate pressure and surface pressure onto odom timestamps."""
    if sig["ts_press"].size == 0 or sig["ts_psurf"].size == 0:
        return None, None
    p = np.interp(ts, sig["ts_press"], sig["press"])
    ps = np.interp(ts, sig["ts_psurf"], sig["psurf"])
    return p, ps


def score_high_pitch(sig, sessions):
    """Return (max_abs_pitch_deg, (t0, t1)) for the session with the worst pitch."""
    best = None
    for t0, t1 in sessions:
        if t1 - t0 < MIN_SESSION_DURATION_S:
            continue
        s = slice_session(sig, t0, t1)
        if s["pitch"].size < 10:
            continue
        score = float(np.max(np.abs(s["pitch"])))
        if best is None or score > best[0]:
            best = (score, (t0, t1))
    return best


def score_oscillation(sig, sessions):
    """Return (rejection_fraction, (t0, t1)) for the session with the largest
    share of samples beyond the depth-stability gate."""
    best = None
    for t0, t1 in sessions:
        if t1 - t0 < MIN_SESSION_DURATION_S:
            continue
        s = slice_session(sig, t0, t1)
        n = s["depth"].size
        if n < 100:
            continue
        rolling_min = rolling_local_min(s["t"], s["depth"], DEPTH_WINDOW_S)
        rejected = s["depth"] > (rolling_min + MAX_DEPTH_DEV_M)
        score = rejected.sum() / n
        if best is None or score > best[0]:
            best = (score, (t0, t1))
    return best


def rolling_local_min(t, d, window_s):
    """Symmetric rolling min over a ±window_s/2 window. Same logic as
    apply_depth_stability_filter in extract_zermatt_measurements.py."""
    half = window_s / 2.0
    out = np.empty_like(d)
    for i in range(d.size):
        lo = np.searchsorted(t, t[i] - half)
        hi = np.searchsorted(t, t[i] + half, side="right")
        out[i] = d[lo:hi].min()
    return out


# ─── Plotting ──────────────────────────────────────────────────────────────────

def plot_high_pitch(ax, sig, t0, t1):
    s = slice_session(sig, t0, t1)
    t_rel = s["t"] - s["t"][0]

    bad = np.abs(s["pitch"]) > MAX_PITCH_DEG

    ax.axhspan(MAX_PITCH_DEG, max(35, np.max(np.abs(s["pitch"])) + 5),
               color="red", alpha=0.08, zorder=0)
    ax.axhspan(-max(35, np.max(np.abs(s["pitch"])) + 5), -MAX_PITCH_DEG,
               color="red", alpha=0.08, zorder=0)
    ax.axhline( MAX_PITCH_DEG, color="red", lw=1.0, ls="--", alpha=0.7)
    ax.axhline(-MAX_PITCH_DEG, color="red", lw=1.0, ls="--", alpha=0.7,
               label=f"±{MAX_PITCH_DEG:.0f}° validity gate")

    ax.plot(t_rel, s["pitch"], color="#1f4e79", lw=1.2, label="pitch")
    if bad.any():
        ax.plot(t_rel[bad], s["pitch"][bad], "o", color="red", ms=3,
                label=f"rejected ({bad.sum()} samples)")

    ax.set_xlabel("time within touch session (s)")
    ax.set_ylabel("pitch (deg)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower right", fontsize=8, framealpha=0.9)

    # Annotation pointing at the worst pitch excursion
    i_peak = int(np.argmax(np.abs(s["pitch"])))
    peak_t = t_rel[i_peak]
    peak_p = s["pitch"][i_peak]
    ax.annotate(
        f"peak pitch {peak_p:+.1f}°\n"
        f"→ lever-arm rotated by\n"
        f"  {PRESSURE_TO_CONTACT_X_M*math.sin(math.radians(peak_p))*100:+.0f} cm vertically",
        xy=(peak_t, peak_p),
        xytext=(peak_t + (t_rel[-1] - peak_t) * 0.25,
                peak_p + (10 if peak_p < 0 else -10)),
        fontsize=8,
        ha="left", va="center",
        arrowprops=dict(arrowstyle="->", color="black", lw=0.8),
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.95),
    )

    title = ("Failure mode 1: high-pitch excursion\n"
             "Physical cause: AUV nose-down trim while in contact → tower no "
             "longer perpendicular to ice → spurious Archimedes thickness")
    ax.set_title(title, fontsize=10, loc="left", pad=8)


def plot_oscillation(ax, sig, t0, t1):
    s = slice_session(sig, t0, t1)
    t_rel = s["t"] - s["t"][0]
    rolling_min = rolling_local_min(s["t"], s["depth"], DEPTH_WINDOW_S)
    band_hi = rolling_min + MAX_DEPTH_DEV_M
    bad = s["depth"] > band_hi

    ax.fill_between(t_rel, rolling_min, band_hi,
                    color="#2e7d32", alpha=0.18,
                    label=f"valid contact band (rolling min, +{MAX_DEPTH_DEV_M*100:.0f} cm)")
    ax.plot(t_rel, rolling_min, color="#2e7d32", lw=0.8, ls=":")
    ax.plot(t_rel, s["depth"], color="#1f4e79", lw=1.0, label="depth")
    if bad.any():
        ax.plot(t_rel[bad], s["depth"][bad], "o", color="red", ms=2.5,
                label=f"rejected ({bad.sum()}/{s['depth'].size} samples, "
                      f"{100*bad.sum()/s['depth'].size:.0f}%)")

    ax.invert_yaxis()  # depth increases downward
    ax.set_xlabel("time within touch session (s)")
    ax.set_ylabel("depth (m, downward positive)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower right", fontsize=8, framealpha=0.9)

    # Annotation pointing at the largest deviation
    i_peak = int(np.argmax(s["depth"] - rolling_min))
    peak_t = t_rel[i_peak]
    peak_d = s["depth"][i_peak]
    peak_dev_cm = (s["depth"][i_peak] - rolling_min[i_peak]) * 100
    ax.annotate(
        f"AUV drifted {peak_dev_cm:.0f} cm\n"
        f"below ice contact",
        xy=(peak_t, peak_d),
        xytext=(peak_t + (t_rel[-1] - peak_t) * 0.15,
                peak_d + 0.02),
        fontsize=8,
        ha="left", va="center",
        arrowprops=dict(arrowstyle="->", color="black", lw=0.8),
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.95),
    )

    title = ("Failure mode 2: Pixhawk depth-hold oscillation\n"
             "Physical cause: depth-hold setpoint deeper than the ice → controller "
             "keeps pulling the AUV down, losing firm contact between cycles")
    ax.set_title(title, fontsize=10, loc="left", pad=8)


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bags", nargs="*", metavar="BAG",
                   help="Override the auto-resolved bag paths.")
    p.add_argument("--out", default=str(Path(__file__).with_name("failure_modes_zermatt.png")),
                   help="Output PNG path (default: alongside this script).")
    args = p.parse_args()

    bag_paths = resolve_bags(args.bags)
    print("Reading bags:")
    for b in bag_paths:
        print(f"  - {b}")

    all_signals = []
    for bp in bag_paths:
        sig = read_bag(bp)
        sig["sessions"] = find_sessions(sig["touch_changes"])
        print(f"  {sig['bag']}: {len(sig['sessions'])} touch sessions, "
              f"{sig['ts_odom'].size} odom samples")
        all_signals.append(sig)

    # Score across all bags
    best_pitch = None    # (score, sig, (t0, t1))
    best_osc   = None
    for sig in all_signals:
        cand = score_high_pitch(sig, sig["sessions"])
        if cand and (best_pitch is None or cand[0] > best_pitch[0]):
            best_pitch = (cand[0], sig, cand[1])
        cand = score_oscillation(sig, sig["sessions"])
        if cand and (best_osc is None or cand[0] > best_osc[0]):
            best_osc = (cand[0], sig, cand[1])

    if best_pitch is None or best_osc is None:
        print("error: could not find a qualifying session for both failure modes",
              file=sys.stderr)
        sys.exit(3)

    p_score, p_sig, p_window = best_pitch
    o_score, o_sig, o_window = best_osc
    print(f"\nHigh-pitch session : {p_sig['bag']} "
          f"@ {p_window[0]:.1f}-{p_window[1]:.1f}s, peak |pitch| = {p_score:.1f}°")
    print(f"Oscillation session: {o_sig['bag']} "
          f"@ {o_window[0]:.1f}-{o_window[1]:.1f}s, rejected = {o_score*100:.0f}%")

    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(11, 7.5),
                                          constrained_layout=True)
    plot_high_pitch(ax_top, p_sig, *p_window)
    plot_oscillation(ax_bot, o_sig, *o_window)
    fig.suptitle("Two failure modes that motivated the post-processing filters",
                 fontsize=12, fontweight="bold")

    out = Path(args.out)
    fig.savefig(out, dpi=160)
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
