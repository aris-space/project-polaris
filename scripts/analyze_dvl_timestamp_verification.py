#!/usr/bin/env python3
"""
DVL Timestamp Verification — POLARIS lake test 2026-04-19.

Verifies the DVL timestamp fix (publish_odometry_on_dead_reckoning=false).
Prior to the fix, dead-reckoning messages with stale timestamps leaked onto
/sensors/dvl/velocity, causing backward header.stamp jumps when the driver
published dead-reckoning odometry on the same topic as velocity estimates.

Checks per bag:
  - Inter-message header.stamp dt on /sensors/dvl/velocity and /sensors/dvl/odometry_cov:
      backward jumps (dt < 0)  — dead-reckoning contamination signature
      large gaps    (dt > 0.5 s) — DVL lock loss or driver stall
      duplicates    (dt == 0)    — duplicate stamp issue
  - header.stamp vs log_time skew on /sensors/dvl/velocity (stale timestamp detection)
  - /sensors/dvl/dead_reckoning timestamps kept on their own topic
  - Effective DVL rate (Hz)

Bags with DVL data (lake test 2026-04-19):
  - stationary_11_2026_04_19-16_18_49   (64 min, ~34k msgs — primary analysis)
  - gnss_reference_11_2026_04_19-14_54_16  (~40 s)
  - vertical_04_2026_04_19-12_50_14        (DVL present, no local EKF)

Plots saved as PNG to --output-dir (default: current working directory).

Dependencies: pip install rosbags matplotlib numpy
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
except ImportError:
    print("Install: pip install matplotlib numpy", file=sys.stderr)
    raise

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #

TOPIC_VEL = "/sensors/dvl/velocity"
TOPIC_COV = "/sensors/dvl/odometry_cov"
TOPIC_DR = "/sensors/dvl/dead_reckoning"

DVL_BAGS = {
    "stationary_11_2026_04_19-16_18_49",
    "gnss_reference_11_2026_04_19-14_54_16",
    "vertical_04_2026_04_19-12_50_14",
}

# Expected DVL rate bounds (Hz)
RATE_EXPECTED_MIN = 8.0
RATE_EXPECTED_MAX = 11.0

# Thresholds for anomaly classification
DT_BACKWARD_THRESH = 0.0        # dt < 0 → backward jump
DT_GAP_THRESH = 0.5             # dt > 0.5 s → large gap
DT_NOMINAL_LOW = 0.08           # below this is a burst/duplicate concern
DT_STAMP_LOGTIME_SKEW_WARN = 1.0  # |header.stamp - log_time| > 1 s → stale stamp

# --------------------------------------------------------------------------- #
# Bag discovery
# --------------------------------------------------------------------------- #


def find_dvl_bag_dirs(root: Path) -> list[Path]:
    """Return sorted list of bag dirs under root whose name is in DVL_BAGS."""
    out: list[Path] = []
    for meta in root.rglob("metadata.yaml"):
        d = meta.parent.resolve()
        if d.name in DVL_BAGS and any(d.glob("*.mcap")):
            out.append(d)
    # Also accept direct bag dirs passed as root
    if root.name in DVL_BAGS and any(root.glob("*.mcap")):
        if root not in out:
            out.append(root)
    return sorted(set(out))


# --------------------------------------------------------------------------- #
# Data extraction
# --------------------------------------------------------------------------- #


def _stamp_ns(msg: Any) -> int:
    return int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)


def extract_dvl_timestamps(bag_dir: Path) -> dict[str, Any]:
    """
    Returns dict with keys:
      vel_stamps_ns  — header.stamp (ns) for TOPIC_VEL, in message order
      vel_logtimes_ns — log_time (ns) for TOPIC_VEL, in message order
      cov_stamps_ns  — header.stamp (ns) for TOPIC_COV, in message order
      dr_stamps_ns   — header.stamp (ns) for TOPIC_DR, in message order
      topics_present — set of topics found
    """
    vel_stamps: list[int] = []
    vel_logtimes: list[int] = []
    cov_stamps: list[int] = []
    dr_stamps: list[int] = []
    topics_present: set[str] = set()

    with AnyReader([bag_dir]) as reader:
        available = {c.topic for c in reader.connections}
        wanted = {TOPIC_VEL, TOPIC_COV, TOPIC_DR} & available
        topics_present = wanted
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            return {
                "vel_stamps_ns": vel_stamps,
                "vel_logtimes_ns": vel_logtimes,
                "cov_stamps_ns": cov_stamps,
                "dr_stamps_ns": dr_stamps,
                "topics_present": topics_present,
            }

        for conn, log_ns, raw in reader.messages(connections=conns):
            topic = conn.topic
            try:
                msg = reader.deserialize(raw, conn.msgtype)
                st = _stamp_ns(msg)
            except Exception:
                continue
            if st == 0:
                continue
            if topic == TOPIC_VEL:
                vel_stamps.append(st)
                vel_logtimes.append(log_ns)
            elif topic == TOPIC_COV:
                cov_stamps.append(st)
            elif topic == TOPIC_DR:
                dr_stamps.append(st)

    return {
        "vel_stamps_ns": vel_stamps,
        "vel_logtimes_ns": vel_logtimes,
        "cov_stamps_ns": cov_stamps,
        "dr_stamps_ns": dr_stamps,
        "topics_present": topics_present,
    }


# --------------------------------------------------------------------------- #
# Analysis helpers
# --------------------------------------------------------------------------- #


def compute_dt_stats(stamps_ns: list[int]) -> dict[str, Any]:
    """Compute dt sequence and anomaly statistics from a header.stamp list (message order)."""
    if len(stamps_ns) < 2:
        return {
            "n": len(stamps_ns),
            "n_backward": 0,
            "n_gap": 0,
            "n_duplicate": 0,
            "dt_s": np.array([], dtype=float),
            "mean_hz": None,
            "mean_dt_s": None,
            "median_dt_s": None,
            "backward_indices": [],
            "gap_indices": [],
        }
    arr = np.array(stamps_ns, dtype=np.int64)
    dt_ns = np.diff(arr)
    dt_s = dt_ns / 1e9

    backward = np.where(dt_s < DT_BACKWARD_THRESH)[0]
    gaps = np.where(dt_s > DT_GAP_THRESH)[0]
    duplicates = np.where(dt_s == 0.0)[0]

    # Rate: total span / (n-1 intervals)
    span_s = (arr[-1] - arr[0]) / 1e9
    mean_hz = (len(arr) - 1) / span_s if span_s > 0 else None
    mean_dt = float(np.mean(dt_s)) if len(dt_s) > 0 else None
    median_dt = float(np.median(dt_s)) if len(dt_s) > 0 else None

    return {
        "n": len(stamps_ns),
        "n_backward": int(len(backward)),
        "n_gap": int(len(gaps)),
        "n_duplicate": int(len(duplicates)),
        "dt_s": dt_s,
        "mean_hz": mean_hz,
        "mean_dt_s": mean_dt,
        "median_dt_s": median_dt,
        "backward_indices": backward.tolist(),
        "gap_indices": gaps.tolist(),
        "span_s": span_s,
    }


def compute_stamp_logtime_skew(stamps_ns: list[int], logtimes_ns: list[int]) -> dict[str, Any]:
    """Compare header.stamp vs log_time; large differences indicate stale stamps."""
    if not stamps_ns or not logtimes_ns:
        return {"n": 0, "n_stale": 0}
    skew_s = (np.array(stamps_ns, dtype=np.float64) - np.array(logtimes_ns, dtype=np.float64)) / 1e9
    n_stale = int(np.sum(np.abs(skew_s) > DT_STAMP_LOGTIME_SKEW_WARN))
    return {
        "n": len(skew_s),
        "n_stale": n_stale,
        "mean_skew_s": float(np.mean(skew_s)),
        "std_skew_s": float(np.std(skew_s)),
        "min_skew_s": float(np.min(skew_s)),
        "max_skew_s": float(np.max(skew_s)),
        "skew_s": skew_s,
    }


# --------------------------------------------------------------------------- #
# Plotting
# --------------------------------------------------------------------------- #


def _t0_from_stamps(stamps_ns: list[int]) -> float:
    return stamps_ns[0] / 1e9 if stamps_ns else 0.0


def plot_bag(
    bag_name: str,
    data: dict[str, Any],
    vel_stats: dict[str, Any],
    cov_stats: dict[str, Any],
    skew: dict[str, Any],
    output_dir: Path,
) -> None:
    vel_stamps = data["vel_stamps_ns"]
    cov_stamps = data["cov_stamps_ns"]
    dr_stamps = data["dr_stamps_ns"]

    # ------------------------------------------------------------------ #
    # Figure 1: dt histogram for /sensors/dvl/velocity
    # ------------------------------------------------------------------ #
    fig1, axes1 = plt.subplots(1, 2, figsize=(14, 5))
    fig1.suptitle(f"{bag_name}\n/sensors/dvl/velocity — dt histogram", fontsize=11)

    dt_vel = vel_stats["dt_s"]
    ax = axes1[0]
    if len(dt_vel) > 0:
        # Full range
        ax.hist(dt_vel, bins=100, color="steelblue", edgecolor="none", alpha=0.85)
        ax.axvline(0.0, color="red", lw=1.5, label="dt=0 (duplicate)")
        ax.axvline(DT_GAP_THRESH, color="orange", lw=1.5, label=f"dt={DT_GAP_THRESH}s (gap)")
        ax.axvline(1 / 9.5, color="green", lw=1.5, linestyle="--", label="dt=1/9.5 Hz (nominal)")
        ax.set_xlabel("dt (s)")
        ax.set_ylabel("count")
        ax.set_title("Full dt range")
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, "No data", ha="center", va="center", transform=ax.transAxes)

    ax2 = axes1[1]
    if len(dt_vel) > 0:
        # Zoomed: 0..0.3 s to see nominal rate and backward jumps
        mask = (dt_vel >= -0.05) & (dt_vel <= 0.3)
        zoom_dt = dt_vel[mask]
        if len(zoom_dt) > 0:
            ax2.hist(zoom_dt, bins=100, color="steelblue", edgecolor="none", alpha=0.85)
        ax2.axvline(0.0, color="red", lw=1.5, label="dt=0")
        ax2.axvline(1 / 9.5, color="green", lw=1.5, linestyle="--", label="dt=1/9.5 Hz")
        ax2.set_xlabel("dt (s)")
        ax2.set_title("Zoomed: -0.05 to 0.3 s")
        ax2.legend(fontsize=8)
    ax2.set_ylabel("count")

    fig1.tight_layout()
    out1 = output_dir / f"dvl_ts_{bag_name}_vel_dt_histogram.png"
    fig1.savefig(out1, dpi=150)
    plt.close(fig1)
    print(f"  Saved: {out1.name}")

    # ------------------------------------------------------------------ #
    # Figure 2: dt time series for /sensors/dvl/velocity
    # ------------------------------------------------------------------ #
    fig2, axes2 = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig2.suptitle(f"{bag_name}\n/sensors/dvl/velocity — dt time series", fontsize=11)

    if len(vel_stamps) >= 2:
        t0 = vel_stamps[0] / 1e9
        t_mid = np.array(vel_stamps[1:]) / 1e9 - t0  # midpoint of each interval

        # Panel 1: raw dt
        ax = axes2[0]
        ax.plot(t_mid, dt_vel, lw=0.6, color="steelblue", label="dt (s)")
        if len(vel_stats["backward_indices"]) > 0:
            bi = np.array(vel_stats["backward_indices"])
            ax.scatter(t_mid[bi], dt_vel[bi], color="red", s=25, zorder=5, label=f"backward ({len(bi)})")
        if len(vel_stats["gap_indices"]) > 0:
            gi = np.array(vel_stats["gap_indices"])
            ax.scatter(t_mid[gi], dt_vel[gi], color="orange", s=25, zorder=5, label=f"gap>{DT_GAP_THRESH}s ({len(gi)})")
        ax.axhline(0.0, color="red", lw=0.8, linestyle="--")
        ax.axhline(DT_GAP_THRESH, color="orange", lw=0.8, linestyle="--")
        ax.set_ylabel("dt (s)")
        ax.legend(fontsize=8, loc="upper right")
        ax.set_title("Inter-message dt")

        # Panel 2: clipped dt (0..0.5 s) to show nominal behaviour
        ax = axes2[1]
        dt_clipped = np.clip(dt_vel, 0, 0.5)
        ax.plot(t_mid, dt_clipped, lw=0.6, color="steelblue")
        ax.axhline(1 / 9.5, color="green", lw=0.8, linestyle="--", label="nominal 1/9.5 Hz")
        ax.axhline(DT_GAP_THRESH, color="orange", lw=0.8, linestyle="--", label=f"{DT_GAP_THRESH}s gap threshold")
        ax.set_ylabel("dt (s) clipped 0..0.5")
        ax.legend(fontsize=8, loc="upper right")
        ax.set_title("dt (clipped, gaps and backward excluded from view)")

        # Panel 3: header.stamp vs log_time skew
        ax = axes2[2]
        if skew["n"] > 0:
            t_skew = np.array(vel_stamps) / 1e9 - t0
            ax.plot(t_skew, skew["skew_s"], lw=0.6, color="purple", label="header.stamp − log_time (s)")
            ax.axhline(0.0, color="black", lw=0.8, linestyle="--")
            ax.axhline(DT_STAMP_LOGTIME_SKEW_WARN, color="red", lw=0.8, linestyle="--",
                       label=f"±{DT_STAMP_LOGTIME_SKEW_WARN}s stale threshold")
            ax.axhline(-DT_STAMP_LOGTIME_SKEW_WARN, color="red", lw=0.8, linestyle="--")
            ax.set_ylabel("skew (s)")
            ax.legend(fontsize=8, loc="upper right")
            ax.set_title("header.stamp − log_time (positive = stamp ahead of recorder)")
        else:
            ax.text(0.5, 0.5, "No skew data", ha="center", va="center", transform=ax.transAxes)

        axes2[-1].set_xlabel(f"time from bag start (s)  [t0={t0:.3f} UNIX]")
    else:
        for ax in axes2:
            ax.text(0.5, 0.5, "Insufficient data", ha="center", va="center", transform=ax.transAxes)

    fig2.tight_layout()
    out2 = output_dir / f"dvl_ts_{bag_name}_vel_dt_timeseries.png"
    fig2.savefig(out2, dpi=150)
    plt.close(fig2)
    print(f"  Saved: {out2.name}")

    # ------------------------------------------------------------------ #
    # Figure 3: odometry_cov dt + dead_reckoning stamp overlay
    # ------------------------------------------------------------------ #
    fig3, axes3 = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig3.suptitle(f"{bag_name}\n/sensors/dvl/odometry_cov dt + dead_reckoning arrival", fontsize=11)

    t0_cov = (cov_stamps[0] / 1e9) if cov_stamps else 0.0
    ref_t0 = t0_cov

    if len(cov_stamps) >= 2:
        cov_arr = np.array(cov_stamps)
        dt_cov = np.diff(cov_arr) / 1e9
        t_cov_mid = cov_arr[1:] / 1e9 - ref_t0

        ax = axes3[0]
        ax.plot(t_cov_mid, dt_cov, lw=0.6, color="teal", label="odometry_cov dt (s)")
        n_back_cov = int(np.sum(dt_cov < 0))
        n_gap_cov = int(np.sum(dt_cov > DT_GAP_THRESH))
        if n_back_cov > 0:
            bi2 = np.where(dt_cov < 0)[0]
            ax.scatter(t_cov_mid[bi2], dt_cov[bi2], color="red", s=25, zorder=5,
                       label=f"backward ({n_back_cov})")
        ax.axhline(0.0, color="red", lw=0.8, linestyle="--")
        ax.axhline(DT_GAP_THRESH, color="orange", lw=0.8, linestyle="--")
        ax.set_ylabel("dt (s)")
        ax.legend(fontsize=8, loc="upper right")
        ax.set_title(f"/sensors/dvl/odometry_cov — dt  (backward={n_back_cov}, gap={n_gap_cov})")
    else:
        axes3[0].text(0.5, 0.5, "No odometry_cov data", ha="center", va="center",
                      transform=axes3[0].transAxes)

    ax = axes3[1]
    if dr_stamps:
        dr_t = np.array(dr_stamps) / 1e9 - ref_t0
        ax.eventplot(dr_t, orientation="horizontal", lineoffsets=0.5, linelengths=0.8,
                     color="darkorange", label=f"dead_reckoning stamps (n={len(dr_stamps)})")
    if vel_stamps:
        vel_t_rel = np.array(vel_stamps) / 1e9 - ref_t0
        ax.eventplot(vel_t_rel, orientation="horizontal", lineoffsets=1.5, linelengths=0.8,
                     color="steelblue", label=f"velocity stamps (n={len(vel_stamps)})", alpha=0.5)
    ax.set_ylabel("topic")
    ax.set_yticks([0.5, 1.5])
    ax.set_yticklabels(["dead_reckoning", "velocity"])
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("Stamp arrival: dead_reckoning vs velocity (should be separate)")
    axes3[-1].set_xlabel(f"time from bag start (s)  [t0={ref_t0:.3f} UNIX]")

    fig3.tight_layout()
    out3 = output_dir / f"dvl_ts_{bag_name}_cov_and_dr.png"
    fig3.savefig(out3, dpi=150)
    plt.close(fig3)
    print(f"  Saved: {out3.name}")


# --------------------------------------------------------------------------- #
# Per-bag analysis
# --------------------------------------------------------------------------- #


def analyze_bag(bag_dir: Path, output_dir: Path) -> dict[str, Any]:
    name = bag_dir.name
    print(f"\n{'='*70}")
    print(f"Bag: {name}")
    print(f"  Path: {bag_dir}")

    data = extract_dvl_timestamps(bag_dir)
    vel_stamps = data["vel_stamps_ns"]
    cov_stamps = data["cov_stamps_ns"]
    dr_stamps = data["dr_stamps_ns"]
    topics_present = data["topics_present"]

    print(f"  Topics found: {sorted(topics_present) or '(none)'}")
    print(f"  /sensors/dvl/velocity msgs:      {len(vel_stamps)}")
    print(f"  /sensors/dvl/odometry_cov msgs:  {len(cov_stamps)}")
    print(f"  /sensors/dvl/dead_reckoning msgs:{len(dr_stamps)}")

    if not vel_stamps:
        print("  => No DVL velocity data in this bag. Skipping plots.")
        return {"bag_name": name, "error": "no_vel_stamps"}

    vel_stats = compute_dt_stats(vel_stamps)
    cov_stats = compute_dt_stats(cov_stamps)
    skew = compute_stamp_logtime_skew(data["vel_stamps_ns"], data["vel_logtimes_ns"])

    # Print dt statistics
    print(f"\n  /sensors/dvl/velocity dt stats (message order):")
    print(f"    n messages       : {vel_stats['n']}")
    print(f"    span             : {vel_stats.get('span_s', 0):.2f} s")
    print(f"    mean rate        : {vel_stats['mean_hz']:.4f} Hz" if vel_stats["mean_hz"] else "    mean rate        : N/A")
    print(f"    mean dt          : {vel_stats['mean_dt_s']*1000:.2f} ms" if vel_stats["mean_dt_s"] else "    mean dt          : N/A")
    print(f"    median dt        : {vel_stats['median_dt_s']*1000:.2f} ms" if vel_stats["median_dt_s"] else "    median dt        : N/A")
    print(f"    backward jumps   : {vel_stats['n_backward']}"
          + ("  <-- CRITICAL: dead-reckoning contamination?" if vel_stats["n_backward"] > 0 else "  OK"))
    print(f"    large gaps >0.5s : {vel_stats['n_gap']}"
          + ("  (DVL lock loss or driver stall)" if vel_stats["n_gap"] > 0 else ""))
    print(f"    duplicate stamps : {vel_stats['n_duplicate']}"
          + ("  (zero-dt pairs)" if vel_stats["n_duplicate"] > 0 else ""))

    # Rate check
    rate = vel_stats["mean_hz"]
    if rate is not None:
        if RATE_EXPECTED_MIN <= rate <= RATE_EXPECTED_MAX:
            rate_verdict = "OK (within 8–11 Hz)"
        elif rate < RATE_EXPECTED_MIN:
            rate_verdict = f"LOW — expected {RATE_EXPECTED_MIN}–{RATE_EXPECTED_MAX} Hz"
        else:
            rate_verdict = f"HIGH — expected {RATE_EXPECTED_MIN}–{RATE_EXPECTED_MAX} Hz"
        print(f"    rate verdict     : {rate_verdict}")

    # header.stamp vs log_time
    print(f"\n  header.stamp vs log_time skew:")
    if skew["n"] > 0:
        print(f"    mean skew        : {skew['mean_skew_s']*1000:.1f} ms")
        print(f"    std skew         : {skew['std_skew_s']*1000:.1f} ms")
        print(f"    min/max skew     : {skew['min_skew_s']:.3f} s / {skew['max_skew_s']:.3f} s")
        print(f"    n stale (>1s)    : {skew['n_stale']}"
              + ("  <-- stale timestamp events" if skew["n_stale"] > 0 else "  OK"))
    else:
        print("    No skew data available.")

    # dead_reckoning separation
    print(f"\n  Dead reckoning separation:")
    if dr_stamps:
        print(f"    /sensors/dvl/dead_reckoning has {len(dr_stamps)} messages — topic is populated.")
        # Check if any dr stamps appear identically in vel stamps (contamination)
        dr_set = set(dr_stamps)
        vel_set = set(vel_stamps)
        overlap = dr_set & vel_set
        if overlap:
            print(f"    WARNING: {len(overlap)} dead_reckoning stamps found in velocity topic — contamination!")
        else:
            print(f"    No stamp overlap between dead_reckoning and velocity topics — fix appears working.")
    else:
        print(f"    /sensors/dvl/dead_reckoning: 0 messages (topic absent or empty).")
        if vel_stats["n_backward"] == 0:
            print(f"    Velocity topic has no backward jumps — timestamp fix confirmed working.")

    # Overall verdict
    print(f"\n  === VERDICT ===")
    issues = []
    if vel_stats["n_backward"] > 0:
        issues.append(f"{vel_stats['n_backward']} backward timestamp jumps on /velocity")
    if skew.get("n_stale", 0) > 0:
        issues.append(f"{skew['n_stale']} stale stamp events (|skew|>1s)")
    if rate is not None and not (RATE_EXPECTED_MIN <= rate <= RATE_EXPECTED_MAX):
        issues.append(f"DVL rate {rate:.2f} Hz outside expected {RATE_EXPECTED_MIN}–{RATE_EXPECTED_MAX} Hz")
    if issues:
        print(f"  ISSUES: {'; '.join(issues)}")
    else:
        print(f"  PASS — no backward jumps, no stale stamps, rate nominal.")

    # Generate plots
    plot_bag(name, data, vel_stats, cov_stats, skew, output_dir)

    return {
        "bag_name": name,
        "n_vel": vel_stats["n"],
        "n_cov": len(cov_stamps),
        "n_dr": len(dr_stamps),
        "mean_hz": vel_stats["mean_hz"],
        "n_backward": vel_stats["n_backward"],
        "n_gap": vel_stats["n_gap"],
        "n_duplicate": vel_stats["n_duplicate"],
        "n_stale_stamp": skew.get("n_stale", 0),
        "mean_skew_ms": skew.get("mean_skew_s", 0) * 1000,
        "issues": issues,
    }


# --------------------------------------------------------------------------- #
# Summary table
# --------------------------------------------------------------------------- #


def print_summary(results: list[dict[str, Any]]) -> None:
    print("\n" + "=" * 90)
    print("SUMMARY TABLE")
    print("=" * 90)
    hdr = f"{'Bag':<50} {'n_vel':>6} {'Hz':>6} {'back':>5} {'gap':>5} {'dup':>5} {'stale':>6} {'OK?':>6}"
    print(hdr)
    print("-" * 90)
    for r in results:
        if "error" in r:
            print(f"{r['bag_name']:<50}  (no DVL data)")
            continue
        hz = f"{r['mean_hz']:.2f}" if r["mean_hz"] else "N/A"
        ok = "PASS" if not r["issues"] else "FAIL"
        print(
            f"{r['bag_name']:<50} {r['n_vel']:>6} {hz:>6} {r['n_backward']:>5} "
            f"{r['n_gap']:>5} {r['n_duplicate']:>5} {r['n_stale_stamp']:>6} {ok:>6}"
        )
        if r["issues"]:
            for iss in r["issues"]:
                print(f"  {'':50} ^ {iss}")
    print("-" * 90)
    print("Columns: n_vel=velocity msgs, Hz=mean rate, back=backward dt, gap=dt>0.5s,")
    print("         dup=duplicate stamps, stale=|header.stamp-log_time|>1s")

    n_pass = sum(1 for r in results if not r.get("issues") and "error" not in r)
    n_fail = sum(1 for r in results if r.get("issues"))
    print(f"\nBags: {len(results)} total, {n_pass} pass, {n_fail} fail")

    # Interpretation
    total_backward = sum(r.get("n_backward", 0) for r in results)
    if total_backward == 0:
        print("\nCONCLUSION: No backward timestamp jumps found across all DVL bags.")
        print("  => publish_odometry_on_dead_reckoning=false fix is working correctly.")
        print("  => Dead-reckoning messages are confined to /sensors/dvl/dead_reckoning.")
    else:
        print(f"\nWARNING: {total_backward} backward timestamp jumps found -- investigate.")
        print("  => Check publish_odometry_on_dead_reckoning parameter in dvl_a50_pkg.")


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #


def main() -> int:
    ap = argparse.ArgumentParser(
        description="DVL timestamp verification for POLARIS lake test 2026-04-19."
    )
    ap.add_argument(
        "root",
        type=Path,
        nargs="?",
        default=Path(r"C:\Users\gleb0\Downloads\rosbags (2)\rosbags"),
        help="Root directory containing rosbag2 bag folders (default: Downloads/rosbags).",
    )
    ap.add_argument(
        "--output-dir",
        type=Path,
        default=Path("."),
        help="Directory to save PNG plots (default: current working directory).",
    )
    args = ap.parse_args()

    root = args.root.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not root.exists():
        print(f"ERROR: Root directory not found: {root}", file=sys.stderr)
        print("Pass the correct path as the first argument.", file=sys.stderr)
        return 1

    bags = find_dvl_bag_dirs(root)
    if not bags:
        print(f"No DVL bag directories found under {root}", file=sys.stderr)
        print(f"Expected bag names: {sorted(DVL_BAGS)}", file=sys.stderr)
        return 1

    print(f"Found {len(bags)} DVL bag(s) under {root}")
    print(f"Plots will be saved to: {output_dir}\n")

    results = []
    for bag_dir in bags:
        try:
            results.append(analyze_bag(bag_dir, output_dir))
        except Exception as exc:
            print(f"  ERROR processing {bag_dir.name}: {exc}", file=sys.stderr)
            import traceback
            traceback.print_exc()
            results.append({"bag_name": bag_dir.name, "error": str(exc)})

    print_summary(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
