#!/usr/bin/env python3
"""Publication-quality satellite overlay (Moore & Stouch 2014 paper style).

Reuses the data layer of odom_to_gnss_overlay.py (read_bag, merge_h_acc,
gate_fixes, build_dr_track) and replaces the plotting with bold solid lines on
satellite imagery (contextily Esri.WorldImagery), sparse direction arrows, a
hand-drawn scale bar and a clean legend.

CLI:
    python report_overlay.py <bag_dir>
        [--output <path>]
        [--style satellite|plain]
        [--max-h-acc 2.0]
        [--local-coords]
        [--multi <bag2> <bag3> ...]
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
import numpy as np
import utm

from odom_to_gnss_overlay import (
    BagData, DRTrack, FixMsg,
    read_bag, merge_h_acc, gate_fixes, build_dr_track,
)

COLOR_PAIRS = [
    ("crimson",       "royalblue"),
    ("darkorange",    "deepskyblue"),
    ("forestgreen",   "mediumorchid"),
]


@dataclass
class TrackData:
    bag_name:     str
    bag_short:    str
    gt_E:         np.ndarray
    gt_N:         np.ndarray
    dr_E:         np.ndarray
    dr_N:         np.ndarray
    zone_num:     int
    zone_letter:  str
    drift_rate_m_per_100m: float
    n_pairs:      int
    n_fixes:      int
    h_acc_mean:   float | None


def _bag_short(name: str) -> str:
    tokens = name.split("_")
    for i, t in enumerate(tokens):
        if re.fullmatch(r"\d{4}", t):
            if i >= 2:
                return "_".join(tokens[i - 2:i])
            return "_".join(tokens[:i]) or name
    return name


def _utm_epsg(zone_num: int, zone_letter: str) -> str:
    north = zone_letter.upper() >= "N"
    return f"EPSG:{(32600 if north else 32700) + zone_num}"


def _compute_drift_rate(dr_track: DRTrack, gated_fixes: list) -> tuple[float, int]:
    if len(gated_fixes) < 2:
        return 0.0, 0
    first_idx = int(np.argmin(np.abs(dr_track.t_ns - gated_fixes[0][0].t_ns)))
    e0, n0, _, _ = utm.from_latlon(gated_fixes[0][0].lat, gated_fixes[0][0].lon)
    shift_E = e0 - dr_track.E[first_idx]
    shift_N = n0 - dr_track.N_utm[first_idx]

    errors, distances = [], []
    for fix, _ in gated_fixes:
        idx = int(np.argmin(np.abs(dr_track.t_ns - fix.t_ns)))
        if abs(int(dr_track.t_ns[idx]) - fix.t_ns) / 1e9 > 0.5:
            continue
        eg, ng, _, _ = utm.from_latlon(fix.lat, fix.lon)
        errors.append(np.hypot(
            (dr_track.E[idx]     + shift_E) - eg,
            (dr_track.N_utm[idx] + shift_N) - ng,
        ))
        distances.append(float(dr_track.dist[idx]))
    if len(distances) < 2 or max(distances) == min(distances):
        return 0.0, len(errors)
    slope = float(np.polyfit(distances, errors, 1)[0])
    return slope * 100.0, len(errors)


def extract_tracks(bag_dir: Path, max_h_acc: float) -> TrackData | None:
    if not bag_dir.is_dir():
        print(f"WARN: {bag_dir} is not a directory, skipping", file=sys.stderr)
        return None
    bag_data = read_bag(bag_dir)
    h_accs = merge_h_acc(bag_data.fix_msgs, bag_data.ubx_hp_msgs)
    gated = gate_fixes(bag_data.fix_msgs, h_accs, max_h_acc)
    dr_track = build_dr_track(bag_data)

    if not gated:
        gt_E = np.array([])
        gt_N = np.array([])
        dr_E = dr_track.E.copy()
        dr_N = dr_track.N_utm.copy()
        h_acc_mean = None
    else:
        gt_E = np.array([utm.from_latlon(f.lat, f.lon)[0] for f, _ in gated])
        gt_N = np.array([utm.from_latlon(f.lat, f.lon)[1] for f, _ in gated])
        first_idx = int(np.argmin(np.abs(dr_track.t_ns - gated[0][0].t_ns)))
        shift_E = gt_E[0] - dr_track.E[first_idx]
        shift_N = gt_N[0] - dr_track.N_utm[first_idx]
        dr_E = dr_track.E      + shift_E
        dr_N = dr_track.N_utm  + shift_N
        h_acc_mean = float(np.mean([h for _, h in gated]))

    rate, n_pairs = _compute_drift_rate(dr_track, gated)
    return TrackData(
        bag_name=bag_dir.name,
        bag_short=_bag_short(bag_dir.name),
        gt_E=gt_E, gt_N=gt_N,
        dr_E=dr_E, dr_N=dr_N,
        zone_num=dr_track.zone_num,
        zone_letter=dr_track.zone_letter,
        drift_rate_m_per_100m=rate,
        n_pairs=n_pairs,
        n_fixes=len(gated),
        h_acc_mean=h_acc_mean,
    )


def draw_track(ax, E, N, color, label, lw, zorder, alpha=1.0):
    if len(E) == 0:
        return
    ax.plot(E, N, color=color, lw=lw, zorder=zorder, alpha=alpha, label=label,
            solid_capstyle="round", solid_joinstyle="round")


def add_direction_arrows(ax, E, N, color, zorder, n=12):
    if len(E) < 4:
        return
    idxs = np.linspace(2, len(E) - 2, n, dtype=int)
    for i in idxs:
        arr = ax.annotate(
            "",
            xy=(E[i + 1], N[i + 1]),
            xytext=(E[i - 1], N[i - 1]),
            arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8,
                            mutation_scale=14, shrinkA=0, shrinkB=0),
            zorder=zorder,
        )
        arr.arrow_patch.set_path_effects(
            [pe.Stroke(linewidth=2.6, foreground="white"), pe.Normal()]
        )


def add_start_marker(ax, x, y, zorder):
    ax.scatter([x], [y], marker="*", s=170, c="gold",
               edgecolors="black", linewidths=0.6, zorder=zorder)


def add_scale_bar(ax, length_m: float = 10.0):
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    span_x = x1 - x0
    span_y = y1 - y0
    bx = x1 - 0.06 * span_x - length_m
    by = y0 + 0.06 * span_y
    line, = ax.plot([bx, bx + length_m], [by, by], color="black", lw=2.5, zorder=10,
                    solid_capstyle="butt")
    line.set_path_effects([pe.Stroke(linewidth=4.5, foreground="white"), pe.Normal()])
    txt = ax.text(bx + length_m / 2, by + 0.015 * span_y, f"{length_m:g} m",
                  ha="center", va="bottom", fontsize=9, fontweight="bold",
                  color="black", zorder=10)
    txt.set_path_effects([pe.Stroke(linewidth=2.5, foreground="white"), pe.Normal()])


_SWISSTOPO_URL = (
    "https://wmts.geo.admin.ch/1.0.0/ch.swisstopo.swissimage-product/default/"
    "current/3857/{z}/{x}/{y}.jpeg"
)


def add_satellite_basemap(ax, epsg: str, alpha: float = 1.0,
                          zoom: int = 19) -> bool:
    try:
        import contextily as ctx
    except ImportError as exc:
        print(f"WARN: contextily not available ({exc}); falling back to gray",
              file=sys.stderr)
        return False

    for label, source in (("Swisstopo SwissImage", _SWISSTOPO_URL),
                          ("Esri WorldImagery",   ctx.providers.Esri.WorldImagery)):
        try:
            ctx.add_basemap(ax, crs=epsg, source=source, alpha=alpha,
                            zoom=zoom, attribution_size=4)
            return True
        except Exception as exc:
            print(f"WARN: {label} basemap failed ({exc})", file=sys.stderr)
    print("WARN: all basemap providers failed; falling back to gray",
          file=sys.stderr)
    return False


def _combined_extent(tracks: list[TrackData]) -> tuple[float, float, float, float]:
    xs, ys = [], []
    for t in tracks:
        for arr in (t.gt_E, t.dr_E):
            if len(arr):
                xs.append(arr)
        for arr in (t.gt_N, t.dr_N):
            if len(arr):
                ys.append(arr)
    if not xs or not ys:
        raise ValueError("no track data to plot")
    x_all = np.concatenate(xs)
    y_all = np.concatenate(ys)
    return float(x_all.min()), float(x_all.max()), float(y_all.min()), float(y_all.max())


def build_figure(tracks: list[TrackData], style: str, local_coords: bool,
                 output: Path) -> Path:
    fig, ax = plt.subplots(figsize=(8, 8))
    x_min, x_max, y_min, y_max = _combined_extent(tracks)
    span = max(x_max - x_min, y_max - y_min)
    pad = 5.0
    cx = 0.5 * (x_min + x_max)
    cy = 0.5 * (y_min + y_max)
    half = 0.5 * span + pad
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal")

    on_satellite = False
    if style == "satellite":
        epsg = _utm_epsg(tracks[0].zone_num, tracks[0].zone_letter)
        on_satellite = add_satellite_basemap(ax, epsg)
    if not on_satellite:
        ax.set_facecolor("#dddddd")

    multi = len(tracks) > 1
    for i, t in enumerate(tracks):
        gt_color, dr_color = COLOR_PAIRS[i % len(COLOR_PAIRS)]
        if multi:
            gt_label = f"{t.bag_short} - GNSS"
            dr_label = f"{t.bag_short} - DR"
        else:
            gt_label = "GNSS ground truth"
            dr_label = "Dead reckoning"

        draw_track(ax, t.gt_E, t.gt_N, gt_color, gt_label, lw=3.0, zorder=5)
        draw_track(ax, t.dr_E, t.dr_N, dr_color, dr_label, lw=2.5, zorder=4, alpha=0.95)
        add_direction_arrows(ax, t.gt_E, t.gt_N, gt_color, zorder=6, n=12)
        add_direction_arrows(ax, t.dr_E, t.dr_N, dr_color, zorder=5, n=12)
        if len(t.gt_E):
            add_start_marker(ax, t.gt_E[0], t.gt_N[0], zorder=7)

    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_color("black")
        spine.set_linewidth(0.8)

    if multi:
        info_lines = [f"{t.bag_short}: {t.drift_rate_m_per_100m:.1f} m/100m, "
                      f"{t.n_fixes} fixes" for t in tracks]
    else:
        t = tracks[0]
        info_lines = [f"DR drift: {t.drift_rate_m_per_100m:.1f} m/100m",
                      f"GNSS fixes: {t.n_fixes}"]
    ax.text(0.02, 0.02, "\n".join(info_lines), transform=ax.transAxes,
            fontsize=8, va="bottom", ha="left", zorder=11,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="white",
                      edgecolor="lightgray", alpha=0.7, linewidth=0.5))

    if local_coords:
        ref_text = (f"Reference: {tracks[0].zone_num}{tracks[0].zone_letter}  "
                    f"{cx:.0f} E, {cy:.0f} N")
        rt = ax.text(0.98, 0.02, ref_text, transform=ax.transAxes,
                     fontsize=6, va="bottom", ha="right",
                     color="white" if on_satellite else "black", zorder=11)
        if on_satellite:
            rt.set_path_effects([pe.Stroke(linewidth=1.6, foreground="black"),
                                 pe.Normal()])

    add_scale_bar(ax, length_m=10.0)

    leg = ax.legend(loc="upper right", frameon=True, fontsize=9)
    if leg is not None:
        leg.get_frame().set_alpha(0.85)
        leg.get_frame().set_edgecolor("lightgray")
        leg.get_frame().set_linewidth(0.5)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return output


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("--output", type=Path, default=None)
    ap.add_argument("--style", choices=("satellite", "plain"), default="satellite")
    ap.add_argument("--max-h-acc", type=float, default=2.0)
    ap.add_argument("--local-coords", action="store_true")
    ap.add_argument("--multi", type=Path, nargs="*", default=[])
    args = ap.parse_args()

    bag_dirs = [args.bag_dir] + list(args.multi)
    tracks: list[TrackData] = []
    for bd in bag_dirs:
        td = extract_tracks(bd, args.max_h_acc)
        if td is not None:
            tracks.append(td)

    if not tracks:
        print("ERROR: no tracks could be extracted from any bag", file=sys.stderr)
        return 1

    output = args.output or (args.bag_dir / "report_overlay.png")
    build_figure(tracks, args.style, args.local_coords, output)
    print(f"Saved: {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
