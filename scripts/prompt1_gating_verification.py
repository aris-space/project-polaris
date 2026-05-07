#!/usr/bin/env python3
"""
Prompt 1: GNSS/SBL Gating Verification — Lake Test 2026-04-19.

Verifies the uwgpsg2_translator/selector.py gating logic:
- GNSS accepted when h_acc <= 4.0 m; otherwise SBL published after 2 s stale window.
- h_acc sourced from UBXNavHPPosLLH (0.1 mm units, matched by stamp) or NavSatFix covariance.
- /gps/selected frame_id classifies each publish as GNSS (gnss_link) or SBL (sbl_link).

Dependencies: pip install rosbags matplotlib numpy
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

# -- paths --------------------------------------------------------------------
BAG_ROOT = Path(r"C:\Users\gleb0\Downloads\rosbags (2)\rosbags")
OUTPUT_DIR = Path(__file__).resolve().parent.parent / "recordings" / "gating_analysis"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# -- constants matching selector.py ------------------------------------------
TOPIC_FIX      = "/fix"
TOPIC_SELECTED = "/gps/selected"
TOPIC_NAVSATFIX = "/waterlinked_ugps/navsatfix"
TOPIC_UBX_HP   = "/ubx_nav_hp_pos_llh"

MAX_H_ACC_M    = 4.0       # selector default
GPS_STALE_S    = 2.0       # selector default
UBX_MATCH_NS   = 50_000_000  # 50 ms

# NavSatFix covariance type constants
COV_UNKNOWN     = 0
COV_APPROX      = 1
COV_DIAGONAL    = 2
COV_KNOWN       = 3


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _h_acc_from_navsatfix(msg) -> float | None:
    """Largest horizontal 1-sigma from NavSatFix.position_covariance (ENU)."""
    cov_type = int(msg.position_covariance_type)
    cov = list(msg.position_covariance)
    if cov_type == COV_UNKNOWN:
        return None
    if cov_type == COV_DIAGONAL:
        if len(cov) < 5:
            return None
        return math.sqrt(max(0.0, float(cov[0]), float(cov[4])))
    if cov_type in (COV_KNOWN, COV_APPROX):
        if len(cov) < 9:
            return None
        a, b, d = float(cov[0]), float(cov[1]), float(cov[4])
        tr = a + d
        det = a * d - b * b
        disc = max(0.0, tr * tr - 4.0 * det)
        return math.sqrt(max(0.0, 0.5 * (tr + math.sqrt(disc))))
    return None


def _source_from_frame(frame_id: str) -> str:
    if frame_id == "gnss_link":
        return "GNSS"
    if frame_id == "sbl_link":
        return "SBL"
    return f"unknown({frame_id})"


# -- bag reader ---------------------------------------------------------------

def collect_bag(bag_dir: Path) -> dict:
    fix_stamps: list[int] = []
    fix_h_cov: list[float | None] = []

    ubx_stamps: list[int] = []
    ubx_h_acc: list[float | None] = []

    sel_stamps: list[int] = []
    sel_source: list[str] = []

    nav_stamps: list[int] = []

    wanted = {TOPIC_FIX, TOPIC_SELECTED, TOPIC_NAVSATFIX, TOPIC_UBX_HP}

    with AnyReader([bag_dir]) as reader:
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            return {}
        for conn, _log_ts, raw in reader.messages(connections=conns):
            try:
                msg = reader.deserialize(raw, conn.msgtype)
            except Exception:
                continue

            t = _stamp_ns(msg.header.stamp)
            if t == 0:
                continue

            if conn.topic == TOPIC_FIX:
                fix_stamps.append(t)
                fix_h_cov.append(_h_acc_from_navsatfix(msg))

            elif conn.topic == TOPIC_UBX_HP:
                h_m = float(msg.h_acc) * 1e-4
                # 0xFFFFFFFF is the u-blox "invalid" sentinel (invalid_lat/lon flags set)
                if msg.h_acc == 0xFFFFFFFF:
                    h_m = None  # treat as invalid — fall back to covariance
                ubx_stamps.append(t)
                ubx_h_acc.append(h_m)

            elif conn.topic == TOPIC_SELECTED:
                sel_stamps.append(t)
                sel_source.append(_source_from_frame(str(msg.header.frame_id)))

            elif conn.topic == TOPIC_NAVSATFIX:
                nav_stamps.append(t)

    if not fix_stamps and not sel_stamps:
        return {}

    return dict(
        fix_stamps=fix_stamps, fix_h_cov=fix_h_cov,
        ubx_stamps=ubx_stamps, ubx_h_acc=ubx_h_acc,
        sel_stamps=sel_stamps, sel_source=sel_source,
        nav_stamps=nav_stamps,
    )


def merge_h_acc(
    fix_stamps: list[int],
    fix_h_cov: list[float | None],
    ubx_stamps: list[int],
    ubx_h_acc: list[float | None],
) -> tuple[list[float | None], list[str]]:
    """
    For each /fix, use nearest UBX h_acc within 50 ms (if valid and not sentinel),
    else fall back to NavSatFix covariance. Returns (merged_h_acc, source_per_fix).
    """
    ubx_arr = np.array(ubx_stamps, dtype=np.int64) if ubx_stamps else None
    merged: list[float | None] = []
    source: list[str] = []  # 'ubx_hp', 'covariance', 'sentinel_fallback', 'none'
    for i, t in enumerate(fix_stamps):
        if ubx_arr is not None and len(ubx_arr) > 0:
            idx = int(np.argmin(np.abs(ubx_arr - t)))
            if abs(int(ubx_arr[idx]) - t) <= UBX_MATCH_NS:
                val = ubx_h_acc[idx]
                if val is not None:
                    merged.append(val)
                    source.append("ubx_hp")
                else:
                    # Sentinel detected — fall back to covariance
                    merged.append(fix_h_cov[i])
                    source.append("sentinel_fallback")
                continue
        merged.append(fix_h_cov[i])
        source.append("covariance" if fix_h_cov[i] is not None else "none")
    return merged, source


# -- gating verification ------------------------------------------------------

def verify_gating(
    fix_stamps: list[int],
    fix_h_acc: list[float | None],
    sel_stamps: list[int],
    sel_source: list[str],
) -> dict:
    """
    For each /gps/selected message, check whether the source is consistent with
    the expected gate decision. Returns per-type counts and anomaly list.
    """
    if not fix_stamps:
        return {}

    fa = np.array(fix_stamps, dtype=np.int64)
    ha = np.array([x if x is not None else float("nan") for x in fix_h_acc])

    last_accepted_ns: int | None = None
    anomalies: list[str] = []
    n_gnss = n_sbl = 0
    n_gnss_correct = n_sbl_correct = 0
    transitions: list[tuple[float, str, str]] = []  # (time_s, from, to)
    prev_source = None

    for t_sel, src in zip(sel_stamps, sel_source):
        # h_acc at (or just before) this moment — use most recent /fix
        mask = fa <= t_sel
        if not mask.any():
            h = float("nan")
        else:
            h = float(ha[mask][-1])

        expected_on_fix = not math.isnan(h) and h <= MAX_H_ACC_M

        # Check stale: if we're on SBL, last accepted should be > 2 s ago
        stale = (
            last_accepted_ns is None
            or (t_sel - last_accepted_ns) / 1e9 > GPS_STALE_S
        )

        if src == "GNSS":
            n_gnss += 1
            last_accepted_ns = t_sel
            if expected_on_fix:
                n_gnss_correct += 1
            elif not math.isnan(h):
                # Only flag if we actually have a h_acc value (skip pre-fix startup)
                anomalies.append(
                    f"  t={t_sel/1e9:.1f}s: GNSS published but h_acc={h:.2f} m > {MAX_H_ACC_M} m"
                )
        elif src == "SBL":
            n_sbl += 1
            if not expected_on_fix and stale:
                n_sbl_correct += 1
            elif expected_on_fix:
                anomalies.append(
                    f"  t={t_sel/1e9:.1f}s: SBL published but h_acc={h:.2f} m <= {MAX_H_ACC_M} m"
                )
            # if not stale yet — could still be within the 2 s window

        if src != prev_source and prev_source is not None:
            transitions.append((t_sel / 1e9, prev_source, src))
        prev_source = src

    t0 = min(fix_stamps[0], sel_stamps[0]) if sel_stamps else fix_stamps[0]
    t1 = max(fix_stamps[-1], sel_stamps[-1]) if sel_stamps else fix_stamps[-1]
    total_s = (t1 - t0) / 1e9

    # Fraction of time on each source (rough: count messages)
    return dict(
        n_fix=len(fix_stamps),
        n_selected=len(sel_stamps),
        n_gnss=n_gnss, n_sbl=n_sbl,
        n_gnss_correct=n_gnss_correct, n_sbl_correct=n_sbl_correct,
        n_transitions=len(transitions),
        transitions=transitions,
        anomalies=anomalies,
        total_s=total_s,
    )


# -- plotting -----------------------------------------------------------------

GNSS_COLOR = "#2196F3"
SBL_COLOR  = "#FF9800"
GRAY       = "#999999"

def plot_bag(bag_name: str, data: dict, stats: dict, out_path: Path) -> None:
    fix_stamps = data["fix_stamps"]
    fix_h_acc  = data["fix_h_acc_merged"]
    sel_stamps = data["sel_stamps"]
    sel_source = data["sel_source"]
    nav_stamps = data["nav_stamps"]

    t0 = fix_stamps[0] if fix_stamps else (sel_stamps[0] if sel_stamps else 0)

    def rel(ts: list[int]) -> np.ndarray:
        return (np.array(ts, dtype=np.int64) - t0) / 1e9

    fig, axes = plt.subplots(
        2, 1, figsize=(14, 6),
        gridspec_kw={"height_ratios": [3, 1]},
        sharex=True,
    )
    fig.suptitle(f"GNSS/SBL Gating — {bag_name}", fontsize=11, fontweight="bold")

    # -- panel 1: h_acc + gate threshold --------------------------------------
    ax1 = axes[0]
    t_fix = rel(fix_stamps)
    h_vals = np.array([x if x is not None else float("nan") for x in fix_h_acc])

    # Color each segment by source at that fix time
    sel_t = rel(sel_stamps)
    sel_src_arr = sel_source

    def source_at(t_s: float) -> str:
        """Most recent /gps/selected source before time t_s."""
        idx = np.searchsorted(sel_t, t_s, side="right") - 1
        if idx < 0:
            return "none"
        return sel_src_arr[idx]

    for i in range(len(t_fix)):
        src = source_at(t_fix[i])
        color = GNSS_COLOR if src == "GNSS" else SBL_COLOR if src == "SBL" else GRAY
        ax1.plot(t_fix[i], h_vals[i], "o", color=color, markersize=4, alpha=0.8)

    # Step line for h_acc
    ax1.step(t_fix, h_vals, where="post", color="black", linewidth=0.8, alpha=0.5, label="h_acc (merged)")

    ax1.axhline(MAX_H_ACC_M, color="red", linestyle="--", linewidth=1.2, label=f"Gate = {MAX_H_ACC_M} m")
    ax1.set_ylabel("Horizontal accuracy (m)")
    finite_vals = h_vals[np.isfinite(h_vals)]
    y_top = float(np.percentile(finite_vals, 98) * 1.5) if len(finite_vals) else MAX_H_ACC_M * 5
    y_top = max(y_top, MAX_H_ACC_M * 2.5)
    ax1.set_ylim(bottom=0, top=y_top)
    if len(finite_vals) and np.nanmax(h_vals) > y_top:
        ax1.text(0.99, 0.97, f"(some points clipped — max={np.nanmax(h_vals):.0f} m)",
                 transform=ax1.transAxes, ha="right", va="top", fontsize=7, color="gray")
    ax1.legend(fontsize=8, loc="upper right")
    ax1.grid(axis="y", alpha=0.3)

    # Source annotation patches for legend
    gnss_patch = mpatches.Patch(color=GNSS_COLOR, label="GNSS selected")
    sbl_patch  = mpatches.Patch(color=SBL_COLOR,  label="SBL selected")
    ax1.legend(handles=[gnss_patch, sbl_patch,
               plt.Line2D([0], [0], color="black", lw=0.8, alpha=0.5, label="h_acc"),
               plt.Line2D([0], [0], color="red", lw=1.2, ls="--", label=f"gate={MAX_H_ACC_M}m")],
               fontsize=8, loc="upper right")

    # -- panel 2: /gps/selected source as step + navsatfix ticks --------------
    ax2 = axes[1]
    if sel_stamps:
        src_int = np.array([1 if s == "GNSS" else 0 for s in sel_source])
        ax2.step(sel_t, src_int, where="post", color="black", linewidth=1.0)
        ax2.fill_between(sel_t, src_int, step="post",
                         where=src_int == 1, color=GNSS_COLOR, alpha=0.4, label="GNSS")
        ax2.fill_between(sel_t, src_int, step="post",
                         where=src_int == 0, color=SBL_COLOR, alpha=0.4, label="SBL")

    if nav_stamps:
        t_nav = rel(nav_stamps)
        ax2.scatter(t_nav, np.full_like(t_nav, 0.5), marker="|", color="purple",
                    s=60, linewidths=1.0, label="navsatfix arrives", alpha=0.7)

    ax2.set_yticks([0, 1])
    ax2.set_yticklabels(["SBL", "GNSS"])
    ax2.set_xlabel("Time (s, relative to bag start)")
    ax2.set_ylabel("/gps/selected")
    ax2.legend(fontsize=8, loc="upper right")
    ax2.set_ylim(-0.1, 1.1)
    ax2.grid(axis="x", alpha=0.3)

    # -- stats annotation ------------------------------------------------------
    if stats:
        ann = (
            f"GNSS: {stats['n_gnss']}  SBL: {stats['n_sbl']}  "
            f"Transitions: {stats['n_transitions']}  "
            f"Anomalies: {len(stats['anomalies'])}"
        )
        fig.text(0.01, 0.01, ann, fontsize=7, color="gray", va="bottom")

    plt.tight_layout(rect=[0, 0.03, 1, 1])
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {out_path.name}")


# -- per-bag analysis ----------------------------------------------------------

def analyze_bag(bag_dir: Path) -> tuple[dict, dict] | None:
    name = bag_dir.name
    print(f"\n-- {name}")

    raw = collect_bag(bag_dir)
    if not raw:
        print("  No relevant topics found — skipping.")
        return None

    required = {TOPIC_FIX, TOPIC_SELECTED}
    present = set()
    if raw.get("fix_stamps"):
        present.add(TOPIC_FIX)
    if raw.get("sel_stamps"):
        present.add(TOPIC_SELECTED)
    missing = required - present
    if missing:
        print(f"  Missing topics: {missing} — skipping.")
        return None

    fix_h_acc, h_src = merge_h_acc(
        raw["fix_stamps"], raw["fix_h_cov"],
        raw["ubx_stamps"], raw["ubx_h_acc"],
    )
    n_ubx_hp    = h_src.count("ubx_hp")
    n_sentinel  = h_src.count("sentinel_fallback")
    n_cov       = h_src.count("covariance")
    n_ubx_total = sum(1 for s in h_src if "ubx" in s or "sentinel" in s)

    data = dict(raw, fix_h_acc_merged=fix_h_acc, fix_h_acc_source=h_src)

    stats = verify_gating(
        raw["fix_stamps"], fix_h_acc,
        raw["sel_stamps"], raw["sel_source"],
    )

    # -- print summary ------------------------------------------------------
    print(f"  /fix messages        : {len(raw['fix_stamps'])}")
    if n_sentinel:
        print(f"  h_acc source         : UBX HP (valid)={n_ubx_hp}, sentinel(0xFFFF)={n_sentinel} [fallback to cov], covariance={n_cov}")
    else:
        print(f"  h_acc source         : UBX HP={n_ubx_total}, covariance={n_cov}")
    h_vals = [x for x in fix_h_acc if x is not None]
    if h_vals:
        print(f"  h_acc (m)            : min={min(h_vals):.3f}  mean={sum(h_vals)/len(h_vals):.3f}  max={max(h_vals):.3f}")
        below = sum(1 for x in h_vals if x <= MAX_H_ACC_M)
        print(f"  h_acc <= {MAX_H_ACC_M} m        : {below}/{len(h_vals)} = {100*below/len(h_vals):.1f}%")
    else:
        print("  h_acc               : all None (UNKNOWN covariance)")
    print(f"  /gps/selected msgs   : {len(raw['sel_stamps'])}  (GNSS={stats.get('n_gnss',0)}, SBL={stats.get('n_sbl',0)})")
    if raw["sel_source"]:
        gnss_frac = stats["n_gnss"] / max(len(raw["sel_stamps"]), 1)
        print(f"  Time on GNSS (msgs%) : {100*gnss_frac:.1f}%")
    print(f"  Transitions          : {stats.get('n_transitions', 0)}")
    if stats.get("transitions"):
        for t_s, frm, to in stats["transitions"][:5]:
            print(f"    {t_s:.1f}s: {frm} -> {to}")
        if len(stats["transitions"]) > 5:
            print(f"    ... ({len(stats['transitions'])} total)")
    if stats.get("anomalies"):
        print(f"  Anomalies ({len(stats['anomalies'])}):")
        for a in stats["anomalies"][:10]:
            print(a)
        if len(stats["anomalies"]) > 10:
            print(f"  ... ({len(stats['anomalies'])} total)")
    else:
        print("  Anomalies            : none")

    return data, stats


# -- main ---------------------------------------------------------------------

def main() -> int:
    bag_dirs = sorted(BAG_ROOT.iterdir()) if BAG_ROOT.exists() else []
    if not bag_dirs:
        print(f"No bags found at {BAG_ROOT}", file=sys.stderr)
        return 1

    # Only bags with both /gps/selected and /fix and navsatfix (from prior analysis)
    TARGET_PREFIXES = (
        "gnss_reference_11",
        "reference_surface_01",
        "yaw_turns_01",
        "yaw_turns_02",
        "yaw_turns_03",
        "vertical_01",
    )
    bags = [d for d in bag_dirs if d.is_dir() and any(d.name.startswith(p) for p in TARGET_PREFIXES)]
    if not bags:
        bags = [d for d in bag_dirs if d.is_dir() and d.name != "vertical_11_2026_04_19-15_04_54"]

    print(f"Analyzing {len(bags)} bag(s) from {BAG_ROOT}\n")

    summary_rows: list[dict] = []

    for bag_dir in bags:
        result = analyze_bag(bag_dir)
        if result is None:
            continue
        data, stats = result

        out_png = OUTPUT_DIR / f"{bag_dir.name}_gating.png"
        plot_bag(bag_dir.name, data, stats, out_png)

        h_vals = [x for x in data["fix_h_acc_merged"] if x is not None]
        h_src = data.get("fix_h_acc_source", [])
        sentinel_count = h_src.count("sentinel_fallback")
        summary_rows.append(dict(
            bag=bag_dir.name,
            n_fix=len(data["fix_stamps"]),
            h_acc_mean=f"{sum(h_vals)/len(h_vals):.3f}" if h_vals else "n/a",
            h_acc_min=f"{min(h_vals):.3f}" if h_vals else "n/a",
            h_acc_max=f"{max(h_vals):.3f}" if h_vals else "n/a",
            pct_below_gate=f"{100*sum(1 for x in h_vals if x<=MAX_H_ACC_M)/max(len(h_vals),1):.0f}%" if h_vals else "n/a",
            sentinel=sentinel_count,
            n_gnss=stats.get("n_gnss", 0),
            n_sbl=stats.get("n_sbl", 0),
            transitions=stats.get("n_transitions", 0),
            anomalies=len(stats.get("anomalies", [])),
        ))

    # -- summary table -----------------------------------------------------
    W = 120
    print("\n" + "=" * W)
    print("SUMMARY TABLE")
    print("=" * W)
    hdr = (f"{'Bag':<45} {'n_fix':>5} {'h_acc_mean':>10} {'h_acc_min':>9} {'h_acc_max':>9} "
           f"{'<4m%':>5} {'sent':>5} {'GNSS':>5} {'SBL':>5} {'Trans':>5} {'Anom':>5}")
    print(hdr)
    print("-" * W)
    for r in summary_rows:
        short = r["bag"][:44]
        print(f"{short:<45} {r['n_fix']:>5} {r['h_acc_mean']:>10} {r['h_acc_min']:>9} "
              f"{r['h_acc_max']:>9} {r['pct_below_gate']:>5} {r['sentinel']:>5} {r['n_gnss']:>5} "
              f"{r['n_sbl']:>5} {r['transitions']:>5} {r['anomalies']:>5}")
    print("=" * W)
    print("  sent = UBX HP msgs with h_acc=0xFFFFFFFF (invalid; fell back to NavSatFix covariance)")
    print(f"\nPlots saved to: {OUTPUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
