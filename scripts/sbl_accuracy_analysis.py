"""
SBL (Water Linked UGPS G2) accuracy characterisation: position error vs horizontal
distance to topside receivers and vs vehicle depth.

Auto-discovers all bags under bag_root (handles both flat and one-level-nested MCAP
layouts).  Bags with simultaneous /fix data are used for GNSS ground-truth comparison;
all bags contribute to covariance / acoustic-quality trend plots.

Usage:
    python scripts/sbl_accuracy_analysis.py [bag_root] [--output-dir PATH] [--max-gnss-hacc 5.0]

Examples:
    # 2026-04-23 St. Moritz (flat layout)
    python scripts/sbl_accuracy_analysis.py "C:/Users/gleb0/Downloads/rosbags (2)/rosbags"
    # 2026-04-25 Zurichsee (nested layout)
    python scripts/sbl_accuracy_analysis.py recordings/rosbags/2026-04-25
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mcap_ros2.reader import read_ros2_messages
import utm

# ── bag inventory ─────────────────────────────────────────────────────────────

DEFAULT_BAG_ROOT = Path("C:/Users/gleb0/Downloads/rosbags (2)/rosbags")

# ── topics ────────────────────────────────────────────────────────────────────

T_NSF    = "/waterlinked_ugps/navsatfix"
T_REL    = "/waterlinked_ugps/locator_position_relative_wrt_topside"
T_QUAL   = "/waterlinked_ugps/locator_acoustic_quality"
T_FIX    = "/fix"
T_PRES   = "/sensors/pressure/pose_enu"

WANTED = {T_NSF, T_REL, T_QUAL, T_FIX, T_PRES}

# ── constants ─────────────────────────────────────────────────────────────────

MATCH_SBL_GNSS_S = 1.5   # max SBL↔GNSS pairing gap
MATCH_REL_S      = 0.5   # max SBL↔relative-position pairing gap
BIN_WIDTH_M      = 5.0   # distance bin width

# ── data structures ───────────────────────────────────────────────────────────

@dataclass
class BagRecord:
    name: str
    category: str
    mcap: Path
    # navsatfix arrays (nanoseconds, float)
    sbl_t_ns:       np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int64))
    sbl_lat:        np.ndarray = field(default_factory=lambda: np.array([]))
    sbl_lon:        np.ndarray = field(default_factory=lambda: np.array([]))
    sbl_hstd:       np.ndarray = field(default_factory=lambda: np.array([]))  # reported
    # relative-position arrays
    rel_t_ns:       np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int64))
    horiz_dist:     np.ndarray = field(default_factory=lambda: np.array([]))  # m
    depth_rel:      np.ndarray = field(default_factory=lambda: np.array([]))  # m (pos = down)
    # acoustic quality arrays
    qual_t_ns:      np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int64))
    acoustic_std:   np.ndarray = field(default_factory=lambda: np.array([]))  # m
    # gnss /fix arrays
    fix_t_ns:       np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int64))
    fix_lat:        np.ndarray = field(default_factory=lambda: np.array([]))
    fix_lon:        np.ndarray = field(default_factory=lambda: np.array([]))
    fix_hstd:       np.ndarray = field(default_factory=lambda: np.array([]))  # m
    # pressure depth arrays
    pres_t_ns:      np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int64))
    pres_depth:     np.ndarray = field(default_factory=lambda: np.array([]))  # m (pos = down)


@dataclass
class PairedPoint:
    """One SBL↔GNSS matched pair with associated context."""
    bag_name:      str
    category:      str
    error_m:       float
    delta_east:    float   # SBL_east - GNSS_east [m]
    delta_north:   float   # SBL_north - GNSS_north [m]
    horiz_dist_m:  float
    depth_m:       float
    reported_hstd: float
    acoustic_std:  float


# ── helpers ───────────────────────────────────────────────────────────────────

def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def nsf_h_std(cov) -> float:
    """Max horizontal 1-sigma from NavSatFix covariance (ENU row-major 3×3)."""
    a, b, d = float(cov[0]), float(cov[1]), float(cov[4])
    trace = a + d
    det = a * d - b * b
    disc = max(0.0, (trace / 2) ** 2 - det)
    lam_max = trace / 2 + math.sqrt(disc)
    return math.sqrt(max(0.0, lam_max))


def nearest_idx(arr_ns: np.ndarray, t_ns: int) -> int:
    return int(np.argmin(np.abs(arr_ns - t_ns)))


# ── bag discovery ─────────────────────────────────────────────────────────────

def _categorize_name(name: str) -> str:
    n = name.lower()
    if "stationary" in n:
        return "stationary"
    if "vertical" in n:
        return "vertical"
    if "gnss" in n and "sbl" not in n:
        return "gnss_surfaced"
    if "depthhold" in n or "depth_hold" in n or "sbl" in n:
        return "sbl_underwater"
    if "surge" in n or "straight" in n:
        return "straight_surge"
    return "unknown"


def discover_bags(bag_root: Path) -> list[BagRecord]:
    """Find all bags under bag_root via metadata.yaml.  Handles flat and nested layouts."""
    records: list[BagRecord] = []
    seen: set[Path] = set()
    for meta in sorted(bag_root.rglob("metadata.yaml")):
        bag_dir = meta.parent
        mcaps = sorted(bag_dir.glob("*.mcap"))
        if not mcaps:
            continue
        mcap = mcaps[0]
        if mcap in seen:
            continue
        seen.add(mcap)
        # For nested layout (<outer>/<inner>/metadata.yaml), use outer dir as display name.
        outer = bag_dir.parent
        name = outer.name if outer != bag_root else bag_dir.name
        records.append(BagRecord(name=name, category=_categorize_name(name), mcap=mcap))
    return records


# ── bag loading ───────────────────────────────────────────────────────────────

def load_bag(rec: BagRecord) -> None:
    raw: dict[str, list] = {t: [] for t in WANTED}

    for msg in read_ros2_messages(str(rec.mcap)):
        topic = msg.channel.topic
        if topic not in WANTED:
            continue
        ros = msg.ros_msg
        t = stamp_ns(ros.header.stamp)
        if t == 0:
            continue
        raw[topic].append((t, ros))

    # navsatfix
    if raw[T_NSF]:
        ts, msgs = zip(*raw[T_NSF])
        rec.sbl_t_ns = np.array(ts, dtype=np.int64)
        rec.sbl_lat  = np.array([m.latitude  for m in msgs])
        rec.sbl_lon  = np.array([m.longitude for m in msgs])
        rec.sbl_hstd = np.array([nsf_h_std(list(m.position_covariance)) for m in msgs])

    # relative position
    if raw[T_REL]:
        ts, msgs = zip(*raw[T_REL])
        rec.rel_t_ns   = np.array(ts, dtype=np.int64)
        rec.horiz_dist = np.array([math.hypot(m.vector.x, m.vector.y) for m in msgs])
        rec.depth_rel  = np.array([m.vector.z for m in msgs])

    # acoustic quality (vector.x = std_m from WL API)
    if raw[T_QUAL]:
        ts, msgs = zip(*raw[T_QUAL])
        rec.qual_t_ns    = np.array(ts, dtype=np.int64)
        rec.acoustic_std = np.array([m.vector.x for m in msgs])

    # GNSS fix
    if raw[T_FIX]:
        ts, msgs = zip(*raw[T_FIX])
        rec.fix_t_ns = np.array(ts, dtype=np.int64)
        rec.fix_lat  = np.array([m.latitude  for m in msgs])
        rec.fix_lon  = np.array([m.longitude for m in msgs])
        rec.fix_hstd = np.array([nsf_h_std(list(m.position_covariance)) for m in msgs])

    # pressure depth (ENU: z up → depth = -z)
    if raw[T_PRES]:
        ts, msgs = zip(*raw[T_PRES])
        rec.pres_t_ns  = np.array(ts, dtype=np.int64)
        rec.pres_depth = np.array([-m.pose.pose.position.z for m in msgs])


def depth_at(rec: BagRecord, t_ns: int, tol_ns: int) -> float:
    """Return best depth estimate at time t_ns. Prefer pressure; fall back to relative z."""
    if len(rec.pres_t_ns) > 0:
        idx = nearest_idx(rec.pres_t_ns, t_ns)
        if abs(int(rec.pres_t_ns[idx]) - t_ns) < tol_ns:
            return float(rec.pres_depth[idx])
    if len(rec.rel_t_ns) > 0:
        idx = nearest_idx(rec.rel_t_ns, t_ns)
        if abs(int(rec.rel_t_ns[idx]) - t_ns) < tol_ns:
            return float(rec.depth_rel[idx])
    return math.nan


# ── SBL-GNSS pairing ──────────────────────────────────────────────────────────

def pair_sbl_gnss(rec: BagRecord, max_gnss_hacc: float) -> list[PairedPoint]:
    if len(rec.sbl_t_ns) == 0 or len(rec.fix_t_ns) == 0:
        return []
    tol_sbl_ns  = int(MATCH_SBL_GNSS_S * 1e9)
    tol_rel_ns  = int(MATCH_REL_S * 1e9)

    points: list[PairedPoint] = []
    for i, t_sbl in enumerate(rec.sbl_t_ns):
        # find nearest /fix
        j = nearest_idx(rec.fix_t_ns, t_sbl)
        if abs(int(rec.fix_t_ns[j]) - t_sbl) > tol_sbl_ns:
            continue
        if rec.fix_hstd[j] > max_gnss_hacc or rec.fix_hstd[j] <= 0:
            continue
        if rec.fix_lat[j] == 0.0 and rec.fix_lon[j] == 0.0:
            continue
        if rec.sbl_lat[i] == 0.0 and rec.sbl_lon[i] == 0.0:
            continue

        # SBL error in UTM
        e_sbl, n_sbl, _, _ = utm.from_latlon(rec.sbl_lat[i], rec.sbl_lon[i])
        e_fix, n_fix, _, _ = utm.from_latlon(rec.fix_lat[j], rec.fix_lon[j])
        de = e_sbl - e_fix
        dn = n_sbl - n_fix
        error = math.hypot(de, dn)

        # horiz distance at same time
        horiz = math.nan
        if len(rec.rel_t_ns) > 0:
            k = nearest_idx(rec.rel_t_ns, t_sbl)
            if abs(int(rec.rel_t_ns[k]) - t_sbl) < tol_rel_ns:
                horiz = float(rec.horiz_dist[k])

        # depth
        depth = depth_at(rec, t_sbl, tol_rel_ns)

        # acoustic std
        acoustic = math.nan
        if len(rec.qual_t_ns) > 0:
            k = nearest_idx(rec.qual_t_ns, t_sbl)
            if abs(int(rec.qual_t_ns[k]) - t_sbl) < tol_rel_ns:
                acoustic = float(rec.acoustic_std[k])

        points.append(PairedPoint(
            bag_name=rec.name,
            category=rec.category,
            error_m=error,
            delta_east=de,
            delta_north=dn,
            horiz_dist_m=horiz,
            depth_m=depth,
            reported_hstd=float(rec.sbl_hstd[i]),
            acoustic_std=acoustic,
        ))
    return points


# ── all-bag aggregation ───────────────────────────────────────────────────────

@dataclass
class AllBagRow:
    bag_name:      str
    category:      str
    reported_hstd: float
    horiz_dist:    float
    depth:         float
    acoustic_std:  float


def aggregate_all_bags(records: list[BagRecord]) -> list[AllBagRow]:
    rows: list[AllBagRow] = []
    tol_ns = int(MATCH_REL_S * 1e9)
    for rec in records:
        if len(rec.sbl_t_ns) == 0:
            continue
        for i, t_sbl in enumerate(rec.sbl_t_ns):
            horiz = math.nan
            if len(rec.rel_t_ns) > 0:
                k = nearest_idx(rec.rel_t_ns, t_sbl)
                if abs(int(rec.rel_t_ns[k]) - t_sbl) < tol_ns:
                    horiz = float(rec.horiz_dist[k])

            acoustic = math.nan
            if len(rec.qual_t_ns) > 0:
                k = nearest_idx(rec.qual_t_ns, t_sbl)
                if abs(int(rec.qual_t_ns[k]) - t_sbl) < tol_ns:
                    acoustic = float(rec.acoustic_std[k])

            depth = depth_at(rec, t_sbl, tol_ns)

            rows.append(AllBagRow(
                bag_name=rec.name,
                category=rec.category,
                reported_hstd=float(rec.sbl_hstd[i]),
                horiz_dist=horiz,
                depth=depth,
                acoustic_std=acoustic,
            ))
    return rows


# ── distance binning ──────────────────────────────────────────────────────────

def bin_by_distance(errors: np.ndarray, dists: np.ndarray, bin_w: float = BIN_WIDTH_M):
    valid = np.isfinite(dists) & np.isfinite(errors)
    d, e = dists[valid], errors[valid]
    if len(d) == 0:
        return [], [], [], []
    max_d = math.ceil(d.max() / bin_w) * bin_w
    edges = np.arange(0, max_d + bin_w, bin_w)
    centers, means, stds, counts = [], [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        mask = (d >= lo) & (d < hi)
        n = int(mask.sum())
        if n == 0:
            continue
        centers.append((lo + hi) / 2)
        means.append(float(e[mask].mean()))
        stds.append(float(e[mask].std()))
        counts.append(n)
    return centers, means, stds, counts


# ── colour palette ────────────────────────────────────────────────────────────

CATEGORY_COLORS = {
    "gnss_surfaced":  "#2196F3",
    "sbl_underwater": "#FF5722",
    "stationary":     "#4CAF50",
    "straight_surge": "#9C27B0",
    "vertical":       "#FF9800",
}

BAG_COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
]


# ── figures ───────────────────────────────────────────────────────────────────

def plot_error_vs_distance(pairs: list[PairedPoint], out_dir: Path) -> None:
    bag_names = sorted({p.bag_name for p in pairs})
    color_map = {b: BAG_COLORS[i % len(BAG_COLORS)] for i, b in enumerate(bag_names)}

    errors = np.array([p.error_m      for p in pairs])
    dists  = np.array([p.horiz_dist_m for p in pairs])
    centers, means, stds, counts = bin_by_distance(errors, dists)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("SBL Position Error vs Horizontal Distance to Topside (GNSS surfaced bags)", fontsize=11)

    # scatter
    for bag in bag_names:
        mask = [p.bag_name == bag for p in pairs]
        ax1.scatter(dists[mask], errors[mask], s=22, alpha=0.7,
                    color=color_map[bag], label=bag[:26], zorder=3)
    ax1.set_xlabel("Horizontal distance to topside [m]")
    ax1.set_ylabel("SBL error vs GNSS [m]")
    ax1.set_title("Scatter: error vs distance")
    ax1.legend(fontsize=7, loc="upper left")
    ax1.grid(True, lw=0.3)
    ax1.set_xlim(left=0)
    ax1.set_ylim(bottom=0)

    # binned bar
    if centers:
        x = np.array(centers)
        y = np.array(means)
        e = np.array(stds)
        ax2.bar(x, y, width=BIN_WIDTH_M * 0.8, yerr=e, capsize=4,
                color="#2196F3", edgecolor="white", alpha=0.85, label="mean ± std")
        for xi, yi, ni in zip(x, y, counts):
            ax2.text(xi, yi + e[list(x).index(xi)] + 0.05, f"n={ni}",
                     ha="center", va="bottom", fontsize=7)
    ax2.set_xlabel("Horizontal distance to topside [m]")
    ax2.set_ylabel("Mean SBL error [m]")
    ax2.set_title(f"Binned mean ± std (bin={BIN_WIDTH_M:.0f} m)")
    ax2.grid(True, lw=0.3, axis="y")
    ax2.set_xlim(left=0)
    ax2.set_ylim(bottom=0)

    fig.tight_layout()
    out = out_dir / "sbl_error_vs_distance.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {out.name}")


def plot_error_vs_depth(pairs: list[PairedPoint], out_dir: Path) -> None:
    bag_names = sorted({p.bag_name for p in pairs})
    color_map = {b: BAG_COLORS[i % len(BAG_COLORS)] for i, b in enumerate(bag_names)}

    fig, ax = plt.subplots(figsize=(7, 5))
    fig.suptitle("SBL Position Error vs Depth (GNSS surfaced bags)", fontsize=11)

    for bag in bag_names:
        pts = [p for p in pairs if p.bag_name == bag]
        depths = np.array([p.depth_m for p in pts])
        errors = np.array([p.error_m for p in pts])
        valid  = np.isfinite(depths) & np.isfinite(errors)
        ax.scatter(depths[valid], errors[valid], s=22, alpha=0.7,
                   color=color_map[bag], label=bag[:26], zorder=3)

    ax.set_xlabel("Depth [m]  (positive = below surface)")
    ax.set_ylabel("SBL error vs GNSS [m]")
    ax.set_title("Scatter: error vs depth")
    ax.legend(fontsize=7)
    ax.grid(True, lw=0.3)
    ax.set_ylim(bottom=0)

    fig.tight_layout()
    out = out_dir / "sbl_error_vs_depth.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {out.name}")


def plot_covariance_trends(rows: list[AllBagRow], out_dir: Path) -> None:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Reported SBL Covariance (h_std) vs Distance and Depth (all bags)", fontsize=11)

    categories = sorted({r.category for r in rows})
    for cat in categories:
        cat_rows = [r for r in rows if r.category == cat]
        dists  = np.array([r.horiz_dist for r in cat_rows])
        depths = np.array([r.depth      for r in cat_rows])
        hstds  = np.array([r.reported_hstd for r in cat_rows])
        color  = CATEGORY_COLORS.get(cat, "gray")
        valid_d = np.isfinite(dists) & np.isfinite(hstds) & (hstds > 0)
        valid_z = np.isfinite(depths) & np.isfinite(hstds) & (hstds > 0)
        ax1.scatter(dists[valid_d], hstds[valid_d], s=14, alpha=0.5,
                    color=color, label=cat, zorder=3)
        ax2.scatter(depths[valid_z], hstds[valid_z], s=14, alpha=0.5,
                    color=color, label=cat, zorder=3)

    for ax, xlabel, title in [
        (ax1, "Horizontal distance to topside [m]", "Covariance h_std vs distance"),
        (ax2, "Depth [m]", "Covariance h_std vs depth"),
    ]:
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Reported h_std [m]")
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.grid(True, lw=0.3)
        ax.set_ylim(bottom=0)

    fig.tight_layout()
    out = out_dir / "sbl_covariance_trends.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {out.name}")


def plot_quality_trends(rows: list[AllBagRow], out_dir: Path) -> None:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Acoustic Quality (std_m from WL API) vs Distance and Depth (all bags)", fontsize=11)

    categories = sorted({r.category for r in rows})
    for cat in categories:
        cat_rows = [r for r in rows if r.category == cat]
        dists  = np.array([r.horiz_dist    for r in cat_rows])
        depths = np.array([r.depth         for r in cat_rows])
        quals  = np.array([r.acoustic_std  for r in cat_rows])
        color  = CATEGORY_COLORS.get(cat, "gray")
        valid_d = np.isfinite(dists) & np.isfinite(quals) & (quals >= 0)
        valid_z = np.isfinite(depths) & np.isfinite(quals) & (quals >= 0)
        ax1.scatter(dists[valid_d], quals[valid_d], s=14, alpha=0.5,
                    color=color, label=cat, zorder=3)
        ax2.scatter(depths[valid_z], quals[valid_z], s=14, alpha=0.5,
                    color=color, label=cat, zorder=3)

    for ax, xlabel, title in [
        (ax1, "Horizontal distance to topside [m]", "Acoustic std_m vs distance"),
        (ax2, "Depth [m]", "Acoustic std_m vs depth"),
    ]:
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Acoustic std_m [m]  (lower = better)")
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.grid(True, lw=0.3)
        ax.set_ylim(bottom=0)

    fig.tight_layout()
    out = out_dir / "sbl_quality_trends.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {out.name}")


def plot_error_vectors(pairs: list[PairedPoint], out_dir: Path) -> None:
    """Scatter of (Δeast, Δnorth) = SBL − GNSS per pair. Tight cluster → constant offset."""
    bag_names = sorted({p.bag_name for p in pairs})
    color_map = {b: BAG_COLORS[i % len(BAG_COLORS)] for i, b in enumerate(bag_names)}

    fig, ax = plt.subplots(figsize=(7, 7))
    fig.suptitle("SBL error vectors: SBL − GNSS in UTM  (tight cluster = constant offset)",
                 fontsize=10)

    all_de = np.array([p.delta_east  for p in pairs])
    all_dn = np.array([p.delta_north for p in pairs])

    for bag in bag_names:
        pts = [p for p in pairs if p.bag_name == bag]
        de = np.array([p.delta_east  for p in pts])
        dn = np.array([p.delta_north for p in pts])
        ax.scatter(de, dn, s=18, alpha=0.55, color=color_map[bag], label=bag[:26], zorder=3)
        # per-bag mean cross
        ax.plot(de.mean(), dn.mean(), marker="+", ms=16, mew=2,
                color=color_map[bag], zorder=5)

    # overall mean and reference circle
    mean_de = float(all_de.mean())
    mean_dn = float(all_dn.mean())
    mean_r  = float(np.hypot(all_de, all_dn).mean())
    theta = np.linspace(0, 2 * math.pi, 300)
    ax.plot(mean_de + mean_r * np.cos(theta),
            mean_dn + mean_r * np.sin(theta),
            color="black", lw=0.8, ls="--", label=f"mean |err| = {mean_r:.1f} m")
    ax.plot(mean_de, mean_dn, marker="x", ms=14, mew=2.5, color="black",
            zorder=6, label=f"overall mean ({mean_de:+.1f}, {mean_dn:+.1f}) m")

    ax.axhline(0, color="gray", lw=0.5)
    ax.axvline(0, color="gray", lw=0.5)
    ax.set_xlabel("ΔEast = SBL_E − GNSS_E [m]")
    ax.set_ylabel("ΔNorth = SBL_N − GNSS_N [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=7, loc="upper left")
    ax.grid(True, lw=0.3)

    fig.tight_layout()
    out = out_dir / "sbl_error_vectors.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {out.name}")


# ── JSON + console summary ────────────────────────────────────────────────────

def _safe_mean(arr) -> float | None:
    a = np.array(arr, dtype=float)
    valid = a[np.isfinite(a)]
    return float(valid.mean()) if len(valid) > 0 else None


def _safe_median(arr) -> float | None:
    a = np.array(arr, dtype=float)
    valid = a[np.isfinite(a)]
    return float(np.median(valid)) if len(valid) > 0 else None


def _safe_p95(arr) -> float | None:
    a = np.array(arr, dtype=float)
    valid = a[np.isfinite(a)]
    return float(np.percentile(valid, 95)) if len(valid) > 0 else None


def build_stats(records: list[BagRecord],
                pairs: list[PairedPoint],
                out_dir: Path) -> None:
    per_bag = []
    for rec in records:
        bag_pairs = [p for p in pairs if p.bag_name == rec.name]
        row = {
            "bag":             rec.name,
            "category":        rec.category,
            "n_sbl_msgs":      int(len(rec.sbl_t_ns)),
            "n_matched_gnss":  len(bag_pairs),
            "mean_error_m":    _safe_mean([p.error_m for p in bag_pairs]),
            "median_error_m":  _safe_median([p.error_m for p in bag_pairs]),
            "p95_error_m":     _safe_p95([p.error_m for p in bag_pairs]),
            "mean_delta_east_m":    _safe_mean([p.delta_east  for p in bag_pairs]),
            "mean_delta_north_m":   _safe_mean([p.delta_north for p in bag_pairs]),
            "mean_reported_hstd_m": _safe_mean(rec.sbl_hstd),
            "mean_acoustic_std_m":  _safe_mean(rec.acoustic_std),
            "mean_horiz_dist_m":    _safe_mean(rec.horiz_dist),
            "mean_depth_m":         _safe_mean(rec.pres_depth if len(rec.pres_depth) > 0
                                               else rec.depth_rel),
        }
        per_bag.append(row)

    # aggregated error bins
    all_errors = np.array([p.error_m      for p in pairs])
    all_dists  = np.array([p.horiz_dist_m for p in pairs])
    centers, means, stds, counts = bin_by_distance(all_errors, all_dists)
    bins = [{"bin_center_m": c, "mean_m": m, "std_m": s, "n": n}
            for c, m, s, n in zip(centers, means, stds, counts)]

    stats = {
        "per_bag": per_bag,
        "aggregated": {
            "n_error_pairs": len(pairs),
            "mean_error_m":  _safe_mean(all_errors),
            "p95_error_m":   _safe_p95(all_errors),
            "error_by_distance_bin": bins,
        },
    }

    out = out_dir / "sbl_accuracy_stats.json"
    out.write_text(json.dumps(stats, indent=2))
    print(f"  Saved {out.name}")

    # console table
    header = (f"{'Bag':<38} {'Cat':<16} {'SBL':>5} {'Matched':>7} "
              f"{'ErrMean':>8} {'ErrP95':>7} {'Cov':>6} {'QStd':>6} "
              f"{'Dist':>6} {'Depth':>6}")
    print("\n" + header)
    print("-" * len(header))
    for r in per_bag:
        def _f(v, fmt=".2f"):
            return f"{v:{fmt}}" if v is not None else "  n/a"
        print(f"{r['bag']:<38} {r['category']:<16} {r['n_sbl_msgs']:>5} "
              f"{r['n_matched_gnss']:>7} "
              f"{_f(r['mean_error_m']):>8} {_f(r['p95_error_m']):>7} "
              f"{_f(r['mean_reported_hstd_m']):>6} "
              f"{_f(r['mean_acoustic_std_m']):>6} "
              f"{_f(r['mean_horiz_dist_m']):>6} "
              f"{_f(r['mean_depth_m']):>6}")


# ── main ──────────────────────────────────────────────────────────────────────

def _parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_root", nargs="?", type=Path, default=DEFAULT_BAG_ROOT,
                    help="Root directory containing bag folders")
    ap.add_argument("--output-dir", type=Path, default=None,
                    help="Output directory (default: <bag_root>/sbl_accuracy_output)")
    ap.add_argument("--max-gnss-hacc", type=float, default=5.0,
                    help="Max /fix horizontal accuracy for GNSS ground truth [m] (default 5.0)")
    return ap.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    bag_root = args.bag_root.resolve()
    if not bag_root.is_dir():
        sys.exit(f"ERROR: bag_root does not exist: {bag_root}")
    date_tag = bag_root.name  # e.g. "2026-04-25" or "rosbags"
    out_dir = (args.output_dir or
               (Path(__file__).parent.parent / f"sbl_accuracy_output_{date_tag}")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Bag root  : {bag_root}")
    print(f"Output dir: {out_dir}")
    print(f"Max GNSS h_acc filter: {args.max_gnss_hacc} m\n")

    # discover
    records = discover_bags(bag_root)
    if not records:
        sys.exit("ERROR: no bags found.")
    print(f"\nDiscovered {len(records)} bags:")
    for r in records:
        print(f"  [{r.category:<16}] {r.name}")

    # load
    print("\nLoading bags …")
    for r in records:
        print(f"  {r.name} …", end=" ", flush=True)
        try:
            load_bag(r)
            print(f"SBL={len(r.sbl_t_ns)} REL={len(r.rel_t_ns)} "
                  f"QUAL={len(r.qual_t_ns)} FIX={len(r.fix_t_ns)} PRES={len(r.pres_t_ns)}")
        except Exception as exc:
            print(f"ERROR: {exc}")

    # drop bags with no SBL data
    records_with_sbl = [r for r in records if len(r.sbl_t_ns) > 0]
    if not records_with_sbl:
        sys.exit("ERROR: no SBL messages found in any bag.")
    if len(records_with_sbl) < len(records):
        dropped = [r.name for r in records if len(r.sbl_t_ns) == 0]
        print(f"\n  [INFO] Bags with no SBL messages (skipped from trend plots): "
              f"{', '.join(dropped)}")

    # SBL-GNSS pairing
    print("\nPairing SBL<->GNSS ...")
    all_pairs: list[PairedPoint] = []
    for r in records_with_sbl:
        if len(r.fix_t_ns) == 0:
            continue
        pts = pair_sbl_gnss(r, args.max_gnss_hacc)
        print(f"  {r.name[:36]}: {len(pts)} matched pairs")
        all_pairs.extend(pts)
    print(f"  Total paired points: {len(all_pairs)}")

    # all-bag aggregation
    all_rows = aggregate_all_bags(records_with_sbl)

    # figures
    print("\nGenerating figures …")
    if all_pairs:
        plot_error_vs_distance(all_pairs, out_dir)
        plot_error_vs_depth(all_pairs, out_dir)
        plot_error_vectors(all_pairs, out_dir)
    else:
        print("  [SKIP] No SBL-GNSS pairs — skipping error figures.")
    plot_covariance_trends(all_rows, out_dir)
    plot_quality_trends(all_rows, out_dir)

    # stats
    print("\nWriting stats …")
    build_stats(records, all_pairs, out_dir)

    print("\nDone.")


if __name__ == "__main__":
    main()
