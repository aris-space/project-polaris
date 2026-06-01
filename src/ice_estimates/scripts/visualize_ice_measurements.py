#!/usr/bin/env python3
"""
Visualize ice thickness measurements from a grid-survey rosbag extraction.

Usage:
    python3 visualize_ice_measurements.py [--raw RAW] [--av AV] [--out DIR] [--config CFG]

    --raw FILE     Raw measurements CSV (default: /ros2_ws/measurements/measurements_raw.csv)
    --av  FILE     Averaged measurements CSV (default: /ros2_ws/measurements/measurements_av.csv)
    --out DIR      Output directory for PNGs and PDF (default: /ros2_ws/measurements/plots)
    --config FILE  config.yaml used for extraction (for metadata display)

Outputs:
    map_overview.png      Satellite map with planned grid and measured points
    map_heatmap.png       Satellite map with interpolated thickness heatmap
    distributions.png     Per-grid-point raw thickness distributions
    time_series.png       Thickness time series for each touch session
    summary.pdf           All figures + full statistical table in one PDF
"""

import argparse
import csv
import math
import os
import warnings
from collections import defaultdict
from datetime import datetime, timezone

import contextily as cx
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import yaml
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle
from pyproj import Transformer
from scipy.interpolate import griddata
from scipy.stats import sem

warnings.filterwarnings("ignore", category=UserWarning, module="contextily")

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
DEFAULT_RAW = "/ros2_ws/measurements/measurements_raw.csv"
DEFAULT_AV = "/ros2_ws/measurements/measurements_av.csv"
DEFAULT_OUT = "/ros2_ws/measurements/plots"

SATELLITE_SOURCE = cx.providers.Esri.WorldImagery
MAP_CRS = "EPSG:3857"  # Web Mercator used by contextily
DATA_CRS = "EPSG:4326"  # WGS-84 from GNSS

# Grid layout: 4 columns × 4 rows, column-first id assignment
# col →   0    1    2    3   (east →)
# row ↓
#  0       0    4    8   12
#  1       1    5    9   13
#  2       2    6   10   14
#  3       3    7   11   15
GRID_ROWS = 4
GRID_COLS = 4

CMAP_THICKNESS = "plasma"
ALPHA_HEATMAP = 0.55
MAP_PAD_M = 12  # metres of padding around the grid extent on maps
INTERP_RES = 200  # grid resolution for heatmap interpolation

to_mercator = Transformer.from_crs(DATA_CRS, MAP_CRS, always_xy=True)


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------


def load_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def load_config(path):
    if not path:
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def wgs_to_mercator(lats, lons):
    xs, ys = to_mercator.transform(lons, lats)
    return np.array(xs), np.array(ys)


# ---------------------------------------------------------------------------
# Data helpers
# ---------------------------------------------------------------------------


def unique_point_averages(av_rows):
    """
    Return one dict per unique grid_point_id, averaging across sessions
    that visited the same point (e.g. gp8 measured twice).
    """
    grouped = defaultdict(list)
    for r in av_rows:
        grouped[int(r["grid_point_id"])].append(r)

    result = {}
    for gp_id, rows in grouped.items():

        def avg(key):
            return np.mean([float(r[key]) for r in rows])

        result[gp_id] = {
            "gp_id": gp_id,
            "latitude": avg("latitude"),
            "longitude": avg("longitude"),
            "ice_thickness_m": avg("ice_thickness_m"),
            "grid_point_lat": float(rows[0]["grid_point_lat"]),
            "grid_point_lon": float(rows[0]["grid_point_lon"]),
            "distance_to_target_m": avg("distance_to_target_m"),
            "n_sessions": len(rows),
            "n_samples": sum(int(r["n_samples"]) for r in rows),
            "rho_ice": float(rows[0]["rho_ice"]),
            "duration_s": sum(float(r["duration_s"]) for r in rows),
        }
    return result


def raw_per_point(raw_rows):
    """Return {gp_id: [thickness values]} from raw CSV."""
    d = defaultdict(list)
    for r in raw_rows:
        d[int(r["grid_point_id"])].append(float(r["ice_thickness_m"]))
    return dict(d)


def gp_row_col(gp_id):
    """Return (row, col) for a grid point id in the 4×4 column-first grid."""
    return gp_id % GRID_ROWS, gp_id // GRID_ROWS


def add_satellite(ax, pad_m=MAP_PAD_M):
    """Fetch and draw satellite tiles for the current axes extent + padding."""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ax.set_xlim(x0 - pad_m, x1 + pad_m)
    ax.set_ylim(y0 - pad_m, y1 + pad_m)
    try:
        cx.add_basemap(
            ax, source=SATELLITE_SOURCE, crs=MAP_CRS, zoom="auto", attribution=False
        )
    except Exception as e:
        ax.set_facecolor("#1a3a5c")
        ax.text(
            0.5,
            0.5,
            f"Satellite tiles unavailable\n({e})",
            ha="center",
            va="center",
            transform=ax.transAxes,
            color="white",
        )


def latlon_label(ax):
    """Replace Mercator tick labels with approximate lat/lon values."""
    # Just remove ticks for cleanliness on small-area maps
    ax.set_xticks([])
    ax.set_yticks([])


def estimate_entry_hole(av_rows):
    """
    Extrapolate gp0's planned position from the col-0 row spacing.
    gp0 is the entry hole — filtered from CSVs because the AUV was at
    surface depth there, so its coordinates are not in the data.
    Returns (lat, lon) or None if insufficient data.
    """
    col0 = {}
    for r in av_rows:
        gid = int(r["grid_point_id"])
        if gid // GRID_ROWS == 0 and gid != 0:  # col=0, skip gp0 itself
            row = gid % GRID_ROWS
            col0[row] = (float(r["grid_point_lat"]), float(r["grid_point_lon"]))
    if len(col0) < 2:
        return None
    rows_sorted = sorted(col0)
    dlat = np.mean(
        [
            col0[rows_sorted[i + 1]][0] - col0[rows_sorted[i]][0]
            for i in range(len(rows_sorted) - 1)
        ]
    )
    dlon = np.mean(
        [
            col0[rows_sorted[i + 1]][1] - col0[rows_sorted[i]][1]
            for i in range(len(rows_sorted) - 1)
        ]
    )
    lat1, lon1 = col0[1]  # gp1 (row=1, col=0)
    return (lat1 - dlat, lon1 - dlon)


def draw_entry_hole(ax, entry_latlon, fontsize=8, label_offset_m=4, show_label=True):
    """Draw the 2m×1m entry hole at gp0 on a Web Mercator axes."""
    if entry_latlon is None:
        return
    lat0, lon0 = entry_latlon
    # Half-extents in degrees: 0.5m E-W, 1m N-S (long side toward measurements)
    half_w_deg = 0.5 / (111_320 * math.cos(math.radians(lat0)))
    half_h_deg = 1.0 / 111_320
    corners_lat = [
        lat0 - half_h_deg,
        lat0 + half_h_deg,
        lat0 + half_h_deg,
        lat0 - half_h_deg,
        lat0 - half_h_deg,
    ]
    corners_lon = [
        lon0 - half_w_deg,
        lon0 - half_w_deg,
        lon0 + half_w_deg,
        lon0 + half_w_deg,
        lon0 - half_w_deg,
    ]
    hx, hy = wgs_to_mercator(corners_lat, corners_lon)
    ax.fill(hx, hy, color="cyan", alpha=0.35, zorder=8)
    ax.plot(hx, hy, "-", color="cyan", lw=1.6, zorder=9)
    if show_label:
        cx_m, cy_m = float(hx.mean()), float(hy.max())
        ax.annotate(
            "Entry hole\n(~2×1 m)",
            xy=(cx_m, cy_m),
            xytext=(cx_m, cy_m + label_offset_m),
            color="cyan",
            fontsize=fontsize,
            ha="center",
            va="bottom",
            arrowprops=dict(arrowstyle="->", color="cyan", lw=1.0),
            zorder=10,
            bbox=dict(
                boxstyle="round,pad=0.2",
                facecolor="#0e1117",
                edgecolor="none",
                alpha=0.65,
            ),
        )


# ---------------------------------------------------------------------------
# Figure 1 – Satellite overview map
# ---------------------------------------------------------------------------


def fig_map_overview(all_grid_pts, measured_pts, av_rows, entry_hole=None):
    fig, ax = plt.subplots(figsize=(9, 8))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # All 16 planned grid points
    plan_lats = [p["lat"] for p in all_grid_pts]
    plan_lons = [p["lon"] for p in all_grid_pts]
    px, py = wgs_to_mercator(plan_lats, plan_lons)

    # Measured points
    meas_ids = set(measured_pts.keys())
    meas_lats = [measured_pts[i]["latitude"] for i in sorted(meas_ids)]
    meas_lons = [measured_pts[i]["longitude"] for i in sorted(meas_ids)]
    mx, my = wgs_to_mercator(meas_lats, meas_lons)

    ax.set_xlim(px.min(), px.max())
    ax.set_ylim(py.min(), py.max())
    add_satellite(ax)

    # Draw planned grid lines (only between known points)
    gp_lookup = {p["id"]: p for p in all_grid_pts}
    for row in range(GRID_ROWS):
        ids_in_row = [row + col * GRID_ROWS for col in range(GRID_COLS)]
        row_pts = [gp_lookup[i] for i in ids_in_row if i in gp_lookup]
        if len(row_pts) >= 2:
            rxs, rys = wgs_to_mercator(
                [p["lat"] for p in row_pts], [p["lon"] for p in row_pts]
            )
            ax.plot(rxs, rys, "--", color="white", lw=0.6, alpha=0.4, zorder=2)
    for col in range(GRID_COLS):
        ids_in_col = [row + col * GRID_ROWS for row in range(GRID_ROWS)]
        col_pts = [gp_lookup[i] for i in ids_in_col if i in gp_lookup]
        if len(col_pts) >= 2:
            cxs, cys = wgs_to_mercator(
                [p["lat"] for p in col_pts], [p["lon"] for p in col_pts]
            )
            ax.plot(cxs, cys, "--", color="white", lw=0.6, alpha=0.4, zorder=2)

    # All planned points
    ax.scatter(
        px,
        py,
        s=60,
        marker="s",
        facecolors="none",
        edgecolors="white",
        linewidths=1.2,
        alpha=0.7,
        zorder=3,
        label="Planned grid point",
    )

    # Measured actual positions (coloured by thickness)
    norm = Normalize(
        vmin=min(d["ice_thickness_m"] for d in measured_pts.values()),
        vmax=max(d["ice_thickness_m"] for d in measured_pts.values()),
    )
    cmap = plt.get_cmap(CMAP_THICKNESS)
    for gp_id in sorted(meas_ids):
        d = measured_pts[gp_id]
        gx, gy = wgs_to_mercator([d["latitude"]], [d["longitude"]])
        c = cmap(norm(d["ice_thickness_m"]))
        ax.scatter(gx, gy, s=120, color=c, edgecolors="white", linewidths=1.0, zorder=5)
        ax.text(
            gx[0],
            gy[0] - 2.5,
            f"{d['ice_thickness_m']:.3f} m",
            color="white",
            fontsize=6.5,
            ha="center",
            va="top",
            zorder=6,
            bbox=dict(boxstyle="round,pad=0.15", fc=(0, 0, 0, 0.5), ec="none"),
        )

    # Draw lines from planned → actual position
    for gp_id in sorted(meas_ids):
        d = measured_pts[gp_id]
        gx, gy = wgs_to_mercator([d["latitude"]], [d["longitude"]])
        tx, ty = wgs_to_mercator([d["grid_point_lat"]], [d["grid_point_lon"]])
        ax.plot(
            [tx[0], gx[0]],
            [ty[0], gy[0]],
            "-",
            color="yellow",
            lw=0.8,
            alpha=0.6,
            zorder=4,
        )

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.03, pad=0.02)
    cbar.set_label("Ice thickness (m)", color="black", fontsize=9)
    cbar.ax.yaxis.set_tick_params(color="black")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="black")

    draw_entry_hole(ax, entry_hole, fontsize=7, label_offset_m=3)

    legend_elements = [
        Line2D(
            [0],
            [0],
            marker="s",
            color="black",
            markerfacecolor="none",
            markersize=8,
            label="Planned point",
            linestyle="none",
        ),
        Line2D(
            [0],
            [0],
            marker="o",
            color="black",
            markerfacecolor=cmap(0.6),
            markersize=8,
            label="Measured position",
            linestyle="none",
        ),
        Line2D([0], [0], color="#cc8800", lw=1, alpha=0.9, label="Position offset"),
        Line2D([0], [0], color="#0099cc", lw=2, label="Entry hole (gp0)"),
    ]
    ax.legend(
        handles=legend_elements,
        loc="upper right",
        facecolor="white",
        edgecolor="#cccccc",
        labelcolor="black",
        fontsize=8,
        framealpha=0.92,
    )

    latlon_label(ax)
    ax.set_title(
        "Zermatt Schwarzsee — Ice Survey Grid Overview",
        color="black",
        fontsize=12,
        pad=10,
    )
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Figure 2 – Heatmap on satellite
# ---------------------------------------------------------------------------


def fig_map_heatmap(all_grid_pts, measured_pts, entry_hole=None):
    fig, ax = plt.subplots(figsize=(9, 8))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    all_lats = [p["lat"] for p in all_grid_pts]
    all_lons = [p["lon"] for p in all_grid_pts]
    px_all, py_all = wgs_to_mercator(all_lats, all_lons)
    ax.set_xlim(px_all.min(), px_all.max())
    ax.set_ylim(py_all.min(), py_all.max())
    add_satellite(ax)

    # Build interpolation from measured + planned target positions
    meas_ids = sorted(measured_pts.keys())
    meas_lats = np.array([measured_pts[i]["latitude"] for i in meas_ids])
    meas_lons = np.array([measured_pts[i]["longitude"] for i in meas_ids])
    meas_vals = np.array([measured_pts[i]["ice_thickness_m"] for i in meas_ids])
    mx, my = wgs_to_mercator(meas_lats, meas_lons)

    # Fine grid for interpolation
    xi = np.linspace(ax.get_xlim()[0], ax.get_xlim()[1], INTERP_RES)
    yi = np.linspace(ax.get_ylim()[0], ax.get_ylim()[1], INTERP_RES)
    Xi, Yi = np.meshgrid(xi, yi)
    Zi = griddata((mx, my), meas_vals, (Xi, Yi), method="linear")

    # Mask to convex hull of measured points only
    from scipy.spatial import ConvexHull, Delaunay

    hull_pts = np.column_stack([mx, my])
    if len(hull_pts) >= 3:
        tri = Delaunay(hull_pts)
        grid_flat = np.column_stack([Xi.ravel(), Yi.ravel()])
        inside = tri.find_simplex(grid_flat) >= 0
        mask = ~inside.reshape(Xi.shape)
        Zi = np.ma.array(Zi, mask=mask)

    norm = Normalize(vmin=np.nanmin(meas_vals), vmax=np.nanmax(meas_vals))
    cmap = plt.get_cmap(CMAP_THICKNESS)
    ax.pcolormesh(
        Xi, Yi, Zi, cmap=cmap, norm=norm, alpha=ALPHA_HEATMAP, zorder=3, shading="auto"
    )

    # Measured point markers
    ax.scatter(
        mx,
        my,
        s=100,
        c=meas_vals,
        cmap=cmap,
        norm=norm,
        edgecolors="white",
        linewidths=1.2,
        zorder=5,
    )
    for i, gp_id in enumerate(meas_ids):
        ax.text(
            mx[i],
            my[i] - 2.2,
            f"gp{gp_id}\n{meas_vals[i]:.3f} m",
            color="white",
            fontsize=6,
            ha="center",
            va="top",
            zorder=6,
            bbox=dict(boxstyle="round,pad=0.15", fc=(0, 0, 0, 0.55), ec="none"),
        )

    # Unmeasured planned points
    unmeas_ids = [p["id"] for p in all_grid_pts if p["id"] not in meas_ids]
    u_lats = [p["lat"] for p in all_grid_pts if p["id"] in unmeas_ids]
    u_lons = [p["lon"] for p in all_grid_pts if p["id"] in unmeas_ids]
    ux, uy = wgs_to_mercator(u_lats, u_lons)
    ax.scatter(
        ux, uy, s=40, marker="x", color="lightgray", linewidths=1.0, alpha=0.7, zorder=4
    )

    draw_entry_hole(ax, entry_hole, show_label=False)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.03, pad=0.02)
    cbar.set_label("Ice thickness (m)", color="black", fontsize=9)
    cbar.ax.yaxis.set_tick_params(color="black")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="black")
    latlon_label(ax)
    ax.set_title(
        "Zermatt Schwarzsee — Ice Thickness Heatmap\n"
        "(linear interpolation within measured convex hull)",
        color="black",
        fontsize=11,
        pad=10,
    )
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Figure 3 – Per-point distributions
# ---------------------------------------------------------------------------


def fig_distributions(measured_pts, raw_per_gp, av_rows):
    gp_ids = sorted(measured_pts.keys())
    n = len(gp_ids)
    ncols = min(4, n)
    nrows = math.ceil(n / ncols)

    fig, axes = plt.subplots(
        nrows, ncols, figsize=(4.5 * ncols, 4 * nrows), squeeze=False
    )
    fig.patch.set_facecolor("white")

    for idx, gp_id in enumerate(gp_ids):
        ax = axes[idx // ncols][idx % ncols]
        ax.set_facecolor("white")
        ax.spines[:].set_color("#cccccc")
        ax.tick_params(colors="black")

        vals = np.array(raw_per_gp.get(gp_id, []))
        mean = vals.mean()
        std = vals.std()
        se = sem(vals)
        n_s = len(vals)
        row, col = gp_row_col(gp_id)

        # Histogram
        bins = min(40, max(10, n_s // 30))
        ax.hist(
            vals,
            bins=bins,
            color="#5577ff",
            alpha=0.75,
            edgecolor="#3355cc",
            linewidth=0.4,
            density=True,
            zorder=2,
        )

        # Mean and ±1σ
        ax.axvline(mean, color="#ff9900", lw=1.8, zorder=3, label=f"mean {mean:.4f} m")
        ax.axvspan(
            mean - std,
            mean + std,
            alpha=0.18,
            color="#ffaa33",
            zorder=1,
            label=f"±1σ  {std*100:.2f} cm",
        )
        ax.axvline(mean - std, color="#ffaa33", lw=0.8, ls="--", zorder=3)
        ax.axvline(mean + std, color="#ffaa33", lw=0.8, ls="--", zorder=3)

        ax.set_title(
            f"Grid point {gp_id}  (row {row}, col {col})",
            color="black",
            fontsize=9,
            pad=4,
        )
        ax.set_xlabel("Ice thickness (m)", color="#333333", fontsize=8)
        ax.set_ylabel("Density", color="#333333", fontsize=8)
        ax.xaxis.label.set_color("#333333")
        ax.yaxis.label.set_color("#333333")
        ax.tick_params(labelsize=7)

        info = (
            f"n = {n_s:,}\n"
            f"mean = {mean:.4f} m\n"
            f"σ    = {std*100:.2f} cm\n"
            f"SE   = {se*100:.2f} cm\n"
            f"min  = {vals.min():.4f} m\n"
            f"max  = {vals.max():.4f} m"
        )
        ax.text(
            0.97,
            0.03,
            info,
            transform=ax.transAxes,
            va="bottom",
            ha="right",
            fontsize=6.5,
            color="black",
            bbox=dict(boxstyle="round,pad=0.3", fc=(1, 1, 1, 0.85), ec="#cccccc"),
        )

        ax.legend(
            fontsize=7,
            loc="upper left",
            facecolor="white",
            edgecolor="#cccccc",
            labelcolor="black",
        )

    # Hide unused subplots
    for idx in range(n, nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.suptitle(
        "Per-Grid-Point Ice Thickness Distributions", color="black", fontsize=13, y=1.01
    )
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Figure 4 – Session time series
# ---------------------------------------------------------------------------


def fig_time_series(raw_rows, av_rows):
    sessions = []
    by_session = defaultdict(list)
    for r in raw_rows:
        key = (r["bag"], r["grid_point_id"])
        by_session[key].append(r)

    # Sort by session start time
    session_list = sorted(
        by_session.items(), key=lambda kv: float(kv[1][0]["timestamp_s"])
    )

    n = len(session_list)
    ncols = 2
    nrows = math.ceil(n / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(10, 3.5 * nrows), squeeze=False)
    fig.patch.set_facecolor("white")

    cmap = plt.get_cmap("tab10")

    for idx, ((bag, gp_id), rows) in enumerate(session_list):
        ax = axes[idx // ncols][idx % ncols]
        ax.set_facecolor("white")
        ax.spines[:].set_color("#cccccc")
        ax.tick_params(colors="black", labelsize=7)

        ts = np.array([float(r["timestamp_s"]) for r in rows])
        th = np.array([float(r["ice_thickness_m"]) for r in rows])
        t0 = ts[0]
        ts -= t0

        color = cmap(idx % 10)
        ax.plot(ts, th, lw=0.8, color=color, alpha=0.85)
        ax.axhline(
            th.mean(),
            color="black",
            lw=1.2,
            ls="--",
            alpha=0.8,
            label=f"mean {th.mean():.4f} m",
        )
        ax.fill_between(
            ts, th.mean() - th.std(), th.mean() + th.std(), alpha=0.15, color="black"
        )

        ax.set_xlabel("Time since session start (s)", color="#333333", fontsize=8)
        ax.set_ylabel("Thickness (m)", color="#333333", fontsize=8)
        row, col = gp_row_col(int(gp_id))
        ax.set_title(
            f"gp{gp_id} (row {row} col {col}) — {bag[-20:]}",
            color="black",
            fontsize=8,
            pad=3,
        )
        ax.legend(
            fontsize=7, facecolor="white", edgecolor="#cccccc", labelcolor="black"
        )

    for idx in range(n, nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.suptitle(
        "Ice Thickness Time Series — Per Touch Session", color="black", fontsize=13
    )
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Figure 5 – Whole-lake context map
# ---------------------------------------------------------------------------

CHAPEL_LAT = 45.99123300690695
CHAPEL_LON = 7.70656230473879
LAKE_PAD_M = 230  # metres from measurement centroid to show the full lake


def fig_lake_context(all_grid_pts, measured_pts, entry_hole=None):
    """Satellite overview of the whole lake with the measurement area highlighted."""
    fig, ax = plt.subplots(figsize=(9, 9))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # Centre the view on the measurement centroid
    all_lats = [d["latitude"] for d in measured_pts.values()]
    all_lons = [d["longitude"] for d in measured_pts.values()]
    c_lat, c_lon = np.mean(all_lats), np.mean(all_lons)
    cx_m, cy_m = wgs_to_mercator([c_lat], [c_lon])
    cx_m, cy_m = float(cx_m[0]), float(cy_m[0])

    ax.set_xlim(cx_m - LAKE_PAD_M, cx_m + LAKE_PAD_M)
    ax.set_ylim(cy_m - LAKE_PAD_M, cy_m + LAKE_PAD_M)
    add_satellite(ax, pad_m=0)

    # Chapel landmark
    chx, chy = wgs_to_mercator([CHAPEL_LAT], [CHAPEL_LON])
    ax.plot(
        float(chx[0]),
        float(chy[0]),
        marker="*",
        ms=12,
        color="#FFD700",
        zorder=8,
        markeredgecolor="white",
        markeredgewidth=0.5,
    )
    ax.text(
        float(chx[0]) + 8,
        float(chy[0]) + 8,
        "Kapelle\nSchwarzsee",
        color="#FFD700",
        fontsize=8,
        zorder=9,
        va="bottom",
        fontweight="bold",
        bbox=dict(
            boxstyle="round,pad=0.2", facecolor="#0e1117", edgecolor="none", alpha=0.6
        ),
    )

    # Bounding box of measurement + planned-grid area
    area_lats = all_lats + [p["lat"] for p in all_grid_pts]
    area_lons = all_lons + [p["lon"] for p in all_grid_pts]
    area_xs, area_ys = wgs_to_mercator(area_lats, area_lons)
    box_pad = 10
    bx0, bx1 = float(area_xs.min()) - box_pad, float(area_xs.max()) + box_pad
    by0, by1 = float(area_ys.min()) - box_pad, float(area_ys.max()) + box_pad

    rect = Rectangle(
        (bx0, by0),
        bx1 - bx0,
        by1 - by0,
        linewidth=2,
        edgecolor="white",
        facecolor="white",
        alpha=0.12,
        linestyle="--",
        zorder=4,
    )
    ax.add_patch(rect)
    ax.plot(
        [bx0, bx1, bx1, bx0, bx0],
        [by0, by0, by1, by1, by0],
        "--",
        color="white",
        lw=1.8,
        alpha=0.85,
        zorder=5,
    )

    # Arrow annotation pointing at the measurement box
    box_cx = (bx0 + bx1) / 2
    ax.annotate(
        "Measurement\narea",
        xy=(box_cx, by1),
        xytext=(box_cx, by1 + 80),
        color="white",
        fontsize=9,
        ha="center",
        va="bottom",
        arrowprops=dict(arrowstyle="->", color="white", lw=1.3),
        zorder=9,
        bbox=dict(
            boxstyle="round,pad=0.25", facecolor="#0e1117", edgecolor="none", alpha=0.65
        ),
    )

    # Measurement points coloured by thickness
    norm = Normalize(
        vmin=min(d["ice_thickness_m"] for d in measured_pts.values()),
        vmax=max(d["ice_thickness_m"] for d in measured_pts.values()),
    )
    cmap = plt.get_cmap(CMAP_THICKNESS)
    for d in measured_pts.values():
        gx, gy = wgs_to_mercator([d["latitude"]], [d["longitude"]])
        c = cmap(norm(d["ice_thickness_m"]))
        ax.scatter(
            float(gx[0]),
            float(gy[0]),
            s=55,
            color=c,
            edgecolors="white",
            linewidths=0.7,
            zorder=6,
        )

    draw_entry_hole(ax, entry_hole, fontsize=8, label_offset_m=20)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.03, pad=0.01, shrink=0.45)
    cbar.set_label("Ice thickness (m)", color="black", fontsize=9)
    cbar.ax.yaxis.set_tick_params(color="black")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="black")

    latlon_label(ax)
    ax.set_title(
        "Schwarzsee — Lake Overview & Measurement Location",
        color="black",
        fontsize=13,
        pad=8,
    )
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Figure 6 – Statistics table
# ---------------------------------------------------------------------------


def fig_stats_table(measured_pts, raw_per_gp, av_rows, cfg):
    fig, ax = plt.subplots(figsize=(13, 0.55 * (len(measured_pts) + 3) + 1.5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    ax.axis("off")

    headers = [
        "GP",
        "Row",
        "Col",
        "Lat",
        "Lon",
        "Mean T (m)",
        "σ (cm)",
        "SE (cm)",
        "Min (m)",
        "Max (m)",
        "n raw",
        "Sessions",
        "Duration (s)",
        "Dist to target (m)",
    ]

    rows_data = []
    for gp_id in sorted(measured_pts.keys()):
        d = measured_pts[gp_id]
        vals = np.array(raw_per_gp.get(gp_id, []))
        row, col = gp_row_col(gp_id)
        rows_data.append(
            [
                str(gp_id),
                str(row),
                str(col),
                f"{d['latitude']:.6f}",
                f"{d['longitude']:.6f}",
                f"{d['ice_thickness_m']:.4f}",
                f"{vals.std()*100:.2f}",
                f"{sem(vals)*100:.2f}",
                f"{vals.min():.4f}",
                f"{vals.max():.4f}",
                f"{len(vals):,}",
                str(d["n_sessions"]),
                f"{d['duration_s']:.1f}",
                f"{d['distance_to_target_m']:.2f}",
            ]
        )

    tbl = ax.table(
        cellText=rows_data,
        colLabels=headers,
        cellLoc="center",
        loc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7.8)
    tbl.scale(1, 1.6)

    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#cccccc")
        if r == 0:
            cell.set_facecolor("#dddddd")
            cell.set_text_props(color="black", fontweight="bold")
        elif r % 2 == 1:
            cell.set_facecolor("#f4f4f4")
            cell.set_text_props(color="black")
        else:
            cell.set_facecolor("white")
            cell.set_text_props(color="black")

    # Overall stats footer
    all_vals = np.concatenate([raw_per_gp[g] for g in measured_pts if g in raw_per_gp])
    rho_ice = list(measured_pts.values())[0]["rho_ice"]

    footer = (
        f"Overall   mean = {all_vals.mean():.4f} m    σ = {all_vals.std()*100:.2f} cm    "
        f"range [{all_vals.min():.4f}, {all_vals.max():.4f}] m    "
        f"ρ_ice = {rho_ice} kg/m³    "
        f"n_total = {len(all_vals):,}"
    )
    ax.set_title(
        "Per-Grid-Point Statistical Summary", color="black", fontsize=11, pad=14
    )
    ax.text(
        0.5,
        -0.06,
        footer,
        transform=ax.transAxes,
        ha="center",
        va="top",
        color="#333333",
        fontsize=8,
    )

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Title / metadata page
# ---------------------------------------------------------------------------


def fig_title_page(av_rows, raw_rows, cfg):
    fig = plt.figure(figsize=(11, 8.5))
    fig.patch.set_facecolor("white")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor("white")
    ax.axis("off")

    all_vals = [float(r["ice_thickness_m"]) for r in raw_rows]
    unique_gps = len({r["grid_point_id"] for r in av_rows})
    total_sessions = len(av_rows)
    bags = list({r["bag"] for r in av_rows})
    rho_ice = float(av_rows[0]["rho_ice"])
    t_start = min(float(r["session_start_s"]) for r in av_rows)
    date_str = datetime.fromtimestamp(t_start, tz=timezone.utc).strftime("%Y-%m-%d")

    ax.text(
        0.5,
        0.88,
        "Ice Thickness Survey Summary",
        ha="center",
        va="center",
        transform=ax.transAxes,
        fontsize=26,
        color="black",
        fontweight="bold",
    )
    ax.text(
        0.5,
        0.80,
        "Project POLARIS - UUV Ice Survey",
        ha="center",
        va="center",
        transform=ax.transAxes,
        fontsize=16,
        color="#3355bb",
    )
    ax.text(
        0.5,
        0.74,
        f"Survey date: {date_str}",
        ha="center",
        va="center",
        transform=ax.transAxes,
        fontsize=13,
        color="#666666",
    )

    ax.axhline(0.68, xmin=0.15, xmax=0.85, color="#cccccc", lw=1)

    meta = [
        ("Bags processed", ", ".join(b[-30:] for b in bags)),
        ("Grid", f"4 × 4 = 16 planned points"),
        ("Points measured", f"{unique_gps} of 16"),
        ("Touch sessions", str(total_sessions)),
        ("Total raw samples", f"{len(raw_rows):,}"),
        ("", ""),
        ("Mean ice thickness", f"{np.mean(all_vals):.4f} m"),
        ("Std deviation", f"{np.std(all_vals)*100:.2f} cm"),
        ("Min / Max", f"{min(all_vals):.4f} m / {max(all_vals):.4f} m"),
        ("", ""),
        ("Ice density (ρ_ice)", f"{rho_ice} kg/m³"),
        ("Water density (ρ_w)", f"{cfg.get('rho_water', 1000.0)} kg/m³"),
        (
            "Pressure→contact Z",
            f"{cfg.get('pressure_to_contact_z_m', 0.210)} m (vertical)",
        ),
        (
            "Pressure→contact X",
            f"{cfg.get('pressure_to_contact_x_m', 0.515)} m (horizontal/forward)",
        ),
        ("GNSS accuracy gate", f"< {cfg.get('gnss_max_accuracy_m', 4.0)} m"),
        ("Min session duration", f"{cfg.get('min_duration_s', 10.0)} s"),
        (
            "Depth stability filter",
            f"±{cfg.get('max_depth_dev_m', 0.01)*100:.0f} cm (window {cfg.get('depth_window_s', 30)} s)",
        ),
    ]

    y = 0.63
    for key, val in meta:
        if not key:
            y -= 0.015
            continue
        ax.text(
            0.28,
            y,
            key + ":",
            ha="right",
            va="center",
            transform=ax.transAxes,
            fontsize=10,
            color="#555577",
        )
        ax.text(
            0.30,
            y,
            val,
            ha="left",
            va="center",
            transform=ax.transAxes,
            fontsize=10,
            color="black",
        )
        y -= 0.038

    ax.text(
        0.5,
        0.04,
        f"Generated {datetime.now().strftime('%Y-%m-%d %H:%M')} — "
        "Project POLARIS - UUV Ice Survey",
        ha="center",
        va="center",
        transform=ax.transAxes,
        fontsize=8,
        color="#888888",
    )
    return fig


# ---------------------------------------------------------------------------
# Figure 7 – Ice density rationale
# ---------------------------------------------------------------------------


def fig_ice_density(cfg):
    """Full-page explanation of the ice density estimate used in the analysis."""
    BG = "white"
    TXT = "black"
    SUB = "#444466"
    DIM = "#888888"

    rho_ice_used = cfg.get("rho_ice", 887.0)

    # Layer data
    h_white, rho_white = 45, 870  # cm, kg/m³
    h_black, rho_black = 25, 917
    h_total = h_white + h_black
    rho_calc = (h_white * rho_white + h_black * rho_black) / h_total

    fig = plt.figure(figsize=(11, 15))
    fig.patch.set_facecolor(BG)

    # ── 2-row layout: top=text+diagram, bottom=photo ──
    gs = fig.add_gridspec(
        2,
        2,
        left=0.06,
        right=0.97,
        top=0.91,
        bottom=0.06,
        height_ratios=[2.2, 1.0],
        hspace=0.35,
        wspace=0.08,
        width_ratios=[2.6, 1],
    )
    ax_txt = fig.add_subplot(gs[0, 0])
    ax_diag = fig.add_subplot(gs[0, 1])
    ax_photo = fig.add_subplot(gs[1, :])
    ax_txt.axis("off")
    ax_diag.axis("off")
    ax_photo.axis("off")

    # ── title ──
    fig.text(
        0.5,
        0.955,
        "Ice Density Estimation",
        ha="center",
        va="center",
        fontsize=18,
        color=TXT,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.935,
        "Methodology and rationale for the effective ice density "
        "used in the Archimedes thickness formula",
        ha="center",
        va="center",
        fontsize=10.5,
        color=SUB,
    )

    # Draw horizontal line separator
    line = plt.Line2D(
        [0.06, 0.94],
        [0.925, 0.925],
        color="#cccccc",
        lw=1,
        transform=fig.transFigure,
        clip_on=False,
    )
    fig.add_artist(line)

    def T(ax, x, y, s, **kw):
        ax.text(
            x,
            y,
            s,
            transform=ax.transAxes,
            color=kw.pop("color", TXT),
            fontsize=kw.pop("fontsize", 10),
            va=kw.pop("va", "top"),
            ha=kw.pop("ha", "left"),
            **kw,
        )

    # ── Section 1: Ice types ──────────────────────────────────────────────
    T(
        ax_txt,
        0,
        1.00,
        "1.  Physical properties of lake ice",
        fontsize=12,
        fontweight="bold",
        color="#003a66",
    )

    body1 = (
        "Freshwater lake ice typically consists of two structurally distinct layers "
        "(Leppäranta, 2015):\n\n"
        "  Black ice  (congelation ice)\n"
        "      Forms by downward freezing of lake water.  Transparent, very few air\n"
        "      bubbles.  Density close to pure ice:\n"
        "              ρₙᵇˡᵃᵏᵏ  ≈  917 kg m⁻³\n\n"
        "  White ice  (snow-ice)\n"
        "      Forms when snow on the ice surface becomes saturated with melt-water\n"
        "      (slush) and re-freezes.  Opaque, high air-bubble content:\n"
        "              ρᵂʰᵉᵗᵉ  ≈  840 – 880 kg m⁻³  (870 kg m⁻³ used here)"
    )
    T(
        ax_txt,
        0.02,
        0.93,
        body1,
        fontsize=9.5,
        color=TXT,
        family="monospace" if False else "DejaVu Sans",
    )

    # ── Section 2: Data for Schwarzsee ───────────────────────────────────
    T(
        ax_txt,
        0,
        0.635,
        "2.  Schwarzsee layer thicknesses  (30 Apr 2026)",
        fontsize=12,
        fontweight="bold",
        color="#003a66",
    )

    tbl_data = [
        ("Layer", "Thickness", "Density used"),
        ("White ice", "45 cm", "870 kg m⁻³"),
        ("Black ice", "25 cm", "917 kg m⁻³"),
        ("Total", "70 cm", "—"),
    ]
    col_x = [0.02, 0.35, 0.65]
    row_y = [0.595, 0.558, 0.521, 0.484]
    for ci, header in enumerate(tbl_data[0]):
        ax_txt.text(
            col_x[ci],
            row_y[0],
            header,
            transform=ax_txt.transAxes,
            fontsize=9.5,
            color="#003a66",
            fontweight="bold",
            va="top",
        )
    # Draw line separator without transform parameter
    line_y = row_y[0] - 0.012
    fig.add_artist(
        plt.Line2D(
            [0.08, 0.88],
            [line_y, line_y],
            color="#cccccc",
            lw=0.8,
            transform=ax_txt.transAxes,
            clip_on=False,
        )
    )
    for ri, row in enumerate(tbl_data[1:], 1):
        for ci, cell in enumerate(row):
            ax_txt.text(
                col_x[ci],
                row_y[ri],
                cell,
                transform=ax_txt.transAxes,
                fontsize=9.5,
                color=TXT,
                va="top",
            )

    # ── Section 3: Formula ───────────────────────────────────────────────
    T(
        ax_txt,
        0,
        0.435,
        "3.  Weighted-average effective density",
        fontsize=12,
        fontweight="bold",
        color="#003a66",
    )

    formula_lines = [
        r"$\rho_{\mathrm{eff}} \;=\; "
        r"\dfrac{h_{\mathrm{white}}\,\rho_{\mathrm{white}} "
        r"+ h_{\mathrm{black}}\,\rho_{\mathrm{black}}}"
        r"{h_{\mathrm{total}}}$",
    ]
    ax_txt.text(
        0.18,
        0.385,
        formula_lines[0],
        transform=ax_txt.transAxes,
        fontsize=13,
        color="#664400",
        va="top",
        ha="left",
    )

    calc_lines = [
        r"$= \;\dfrac{(45 \times 870) + (25 \times 917)}{70}$",
        r"$= \;\dfrac{39{,}150 \;+\; 22{,}925}{70}$",
        r"$= \;\dfrac{62{,}075}{70} \;\approx\; \mathbf{886.8 \; \mathrm{kg\,m^{-3}}}$",
    ]
    step_y = [0.310, 0.250, 0.190]
    for line, y in zip(calc_lines, step_y):
        ax_txt.text(
            0.22,
            y,
            line,
            transform=ax_txt.transAxes,
            fontsize=11.5,
            color=TXT,
            va="top",
            ha="left",
        )

    # ── Section 4: Value used in analysis ───────────────────────────────
    T(
        ax_txt,
        0,
        0.115,
        "4.  Value used in this analysis",
        fontsize=12,
        fontweight="bold",
        color="#003a66",
    )

    used_line = (
        f"ρ_ice  =  {rho_calc:.1f} kg m⁻³  →  rounded to  {rho_ice_used:.0f} kg m⁻³\n\n"
        f"This effective density is applied uniformly across all measurements via\n"
        f"the Archimedes thickness formula:  T = depth_contact × ρ_water / ρ_ice"
    )
    T(ax_txt, 0.02, 0.075, used_line, fontsize=10, color=TXT)

    # ── Reference ───────────────────────────────────────────────────────
    fig.text(
        0.06,
        0.022,
        "Reference:  Leppäranta, M. (2015). Freezing of Lakes and the Evolution "
        "of their Ice Cover.\n"
        "            Springer-Praxis, Berlin.  [standard reference for lake-ice "
        "physical properties]",
        ha="left",
        va="bottom",
        fontsize=8,
        color=DIM,
        style="italic",
    )

    # ── Ice-layer diagram ────────────────────────────────────────────────
    ax_diag.set_xlim(0, 1)
    ax_diag.set_ylim(-0.05, 1.05)

    # Normalised heights
    f_w = h_white / h_total  # white fraction
    f_b = h_black / h_total  # black fraction
    bar_x, bar_w = 0.2, 0.55
    y_b_bot = 0.08
    y_b_top = y_b_bot + f_b * 0.72
    y_w_top = y_b_top + f_w * 0.72

    # Black ice block
    ax_diag.add_patch(
        plt.Rectangle(
            (bar_x, y_b_bot),
            bar_w,
            y_b_top - y_b_bot,
            facecolor="#1a6688",
            edgecolor="black",
            linewidth=1.0,
            zorder=2,
        )
    )
    # White ice block
    ax_diag.add_patch(
        plt.Rectangle(
            (bar_x, y_b_top),
            bar_w,
            y_w_top - y_b_top,
            facecolor="#ccddee",
            edgecolor="black",
            linewidth=1.0,
            zorder=2,
        )
    )

    mid_b = (y_b_bot + y_b_top) / 2
    mid_w = (y_b_top + y_w_top) / 2

    # Labels inside blocks
    ax_diag.text(
        bar_x + bar_w / 2,
        mid_b,
        f"Black ice\n{h_black} cm\n917 kg m⁻³",
        ha="center",
        va="center",
        fontsize=8.5,
        color="white",
        fontweight="bold",
        zorder=3,
    )
    ax_diag.text(
        bar_x + bar_w / 2,
        mid_w,
        f"White ice\n{h_white} cm\n870 kg m⁻³",
        ha="center",
        va="center",
        fontsize=8.5,
        color="black",
        fontweight="bold",
        zorder=3,
    )

    # Total brace
    bx = bar_x + bar_w + 0.07
    ax_diag.annotate(
        "",
        xy=(bx, y_b_bot),
        xytext=(bx, y_w_top),
        arrowprops=dict(arrowstyle="<->", color="black", lw=1.2),
    )
    ax_diag.text(
        bx + 0.06,
        (y_b_bot + y_w_top) / 2,
        f"{h_total} cm\ntotal",
        ha="left",
        va="center",
        fontsize=8,
        color=TXT,
    )

    # Ice surface label
    ax_diag.text(
        bar_x + bar_w / 2,
        y_w_top + 0.04,
        "Ice surface\n(contact point)",
        ha="center",
        va="bottom",
        fontsize=7.5,
        color=SUB,
    )
    ax_diag.axhline(
        y_w_top, xmin=bar_x, xmax=bar_x + bar_w + 0.01, color="black", lw=1.2, ls="-"
    )

    # Lake-water label
    ax_diag.text(
        bar_x + bar_w / 2,
        y_b_bot - 0.035,
        "Lake water  (below)",
        ha="center",
        va="top",
        fontsize=7.5,
        color=SUB,
    )

    ax_diag.set_title("Ice column\nschematic", color=TXT, fontsize=9, pad=6)

    # ── Load and display photo of Schwarzsee ice layers ──────────────────────
    photo_path = os.path.join(
        os.path.dirname(__file__), "..", "resource", "eisdicke.jpeg"
    )
    if os.path.exists(photo_path):
        try:
            from PIL import Image

            img = Image.open(photo_path)
            ax_photo.imshow(img, aspect="equal")
            ax_photo.set_title(
                "Schwarzsee ice core sample  (30 April 2026)\n"
                "Visible white ice layer (top, frosted) and black ice layer (bottom, clear)",
                color=TXT,
                fontsize=10,
                pad=10,
                fontweight="bold",
            )
        except Exception as e:
            # Fallback if image loading fails
            ax_photo.text(
                0.5,
                0.5,
                f"[Photo not available]",
                ha="center",
                va="center",
                fontsize=9,
                color=DIM,
                style="italic",
                transform=ax_photo.transAxes,
            )
    else:
        ax_photo.text(
            0.5,
            0.5,
            f"[Photo not found]",
            ha="center",
            va="center",
            fontsize=9,
            color=DIM,
            style="italic",
            transform=ax_photo.transAxes,
        )

    return fig


# ---------------------------------------------------------------------------
# Figure 8 – Error analysis
# ---------------------------------------------------------------------------


def fig_error_analysis(raw_rows, av_rows):
    """
    Three-panel figure investigating the dominant measurement error sources:
      Panel A – gp2: pitch instability driving thickness variation
      Panel B – gp8: two sessions at slightly different ice positions
      Panel C – gp1: slow depth drift over a long session
    """
    BG = "white"
    AX = "white"
    GRID = "#cccccc"
    TXT = "black"
    SUB = "#333333"

    fig = plt.figure(figsize=(14, 13))
    fig.patch.set_facecolor(BG)

    # ── layout: 2 rows × 2 cols, bottom-left spans full width ──
    gs = fig.add_gridspec(
        2, 2, hspace=0.42, wspace=0.32, left=0.07, right=0.97, top=0.91, bottom=0.06
    )
    ax_pitch = fig.add_subplot(gs[0, 0])  # A: gp2 pitch scatter
    ax_gp8 = fig.add_subplot(gs[0, 1])  # B: gp8 two sessions
    ax_drift = fig.add_subplot(gs[1, :])  # C: gp1 drift (full width)

    def style(ax):
        ax.set_facecolor(AX)
        ax.spines[:].set_color(GRID)
        ax.tick_params(colors=TXT, labelsize=8)
        ax.xaxis.label.set_color(SUB)
        ax.yaxis.label.set_color(SUB)

    for ax in [ax_pitch, ax_gp8, ax_drift]:
        style(ax)

    by_gp = defaultdict(list)
    for r in raw_rows:
        by_gp[int(r["grid_point_id"])].append(r)

    # ── Panel A: gp2 – pitch vs thickness ──────────────────────────────────
    rows2 = sorted(by_gp[2], key=lambda r: float(r["timestamp_s"]))
    T2 = np.array([float(r["ice_thickness_m"]) for r in rows2])
    p2 = np.array([float(r["pitch_deg"]) for r in rows2])
    m, b = np.polyfit(p2, T2, 1)
    p_fit = np.linspace(p2.min(), p2.max(), 200)
    r2 = np.corrcoef(T2, p2)[0, 1] ** 2

    sc = ax_pitch.scatter(
        p2, T2 * 100, c=T2 * 100, cmap="plasma", s=10, alpha=0.6, zorder=3
    )
    ax_pitch.plot(
        p_fit,
        (m * p_fit + b) * 100,
        color="#ff5555",
        lw=2,
        zorder=4,
        label=f"Linear fit  R²={r2:.3f}",
    )
    ax_pitch.set_xlabel("Pitch (°)", fontsize=9)
    ax_pitch.set_ylabel("Ice thickness (cm)", fontsize=9)
    ax_pitch.set_title(
        f"A — gp2: Pitch instability  (σ = {T2.std()*100:.2f} cm)",
        color=TXT,
        fontsize=10,
        pad=5,
    )
    ax_pitch.legend(fontsize=8, facecolor=BG, edgecolor=GRID, labelcolor=TXT)
    ax_pitch.text(
        0.97,
        0.07,
        f"dT/dθ = {m*100:.3f} cm/°\n"
        f"pitch range: {p2.min():.1f}° → {p2.max():.1f}°\n"
        f"n = {len(T2)}",
        transform=ax_pitch.transAxes,
        ha="right",
        va="bottom",
        fontsize=7.5,
        color=TXT,
        bbox=dict(boxstyle="round,pad=0.3", fc=(1, 1, 1, 0.85), ec="#cccccc"),
    )
    cbar_a = fig.colorbar(sc, ax=ax_pitch, pad=0.01, fraction=0.035)
    cbar_a.set_label("thickness (cm)", color=TXT, fontsize=7)
    cbar_a.ax.yaxis.set_tick_params(color=TXT, labelsize=7)
    plt.setp(cbar_a.ax.yaxis.get_ticklabels(), color=TXT)

    # ── Panel B: gp8 – two sessions ────────────────────────────────────────
    rows8 = sorted(by_gp[8], key=lambda r: float(r["timestamp_s"]))
    ts8 = np.array([float(r["timestamp_s"]) for r in rows8])
    T8 = np.array([float(r["ice_thickness_m"]) for r in rows8])
    split = int(np.argmax(np.diff(ts8))) + 1
    ts8 -= ts8[0]

    s1_t, s1_T = ts8[:split], T8[:split]
    s2_t, s2_T = ts8[split:], T8[split:]

    ax_gp8.plot(
        s1_t,
        s1_T * 100,
        color="#5588ff",
        lw=0.9,
        alpha=0.8,
        label=f"Session 1  (mean {s1_T.mean()*100:.2f} cm)",
    )
    ax_gp8.plot(
        s2_t,
        s2_T * 100,
        color="#ff9933",
        lw=0.9,
        alpha=0.8,
        label=f"Session 2  (mean {s2_T.mean()*100:.2f} cm)",
    )
    ax_gp8.axhline(
        s1_T.mean() * 100, color="#5588ff", lw=1.5, ls="--", alpha=0.7, zorder=3
    )
    ax_gp8.axhline(
        s2_T.mean() * 100, color="#ff9933", lw=1.5, ls="--", alpha=0.7, zorder=3
    )

    diff_cm = abs(s1_T.mean() - s2_T.mean()) * 100
    mid_t = (s1_t[-1] + s2_t[0]) / 2
    # Gap band
    ax_gp8.axvspan(
        s1_t[-1],
        s2_t[0],
        color="#888888",
        alpha=0.10,
        label=f"Gap {s2_t[0]-s1_t[-1]:.0f} s",
    )
    # Annotate mean difference
    y_lo = min(s1_T.mean(), s2_T.mean()) * 100
    y_hi = max(s1_T.mean(), s2_T.mean()) * 100
    ax_gp8.annotate(
        "",
        xy=(mid_t, y_hi),
        xytext=(mid_t, y_lo),
        arrowprops=dict(arrowstyle="<->", color="black", lw=1.3),
    )
    ax_gp8.text(
        mid_t + 3,
        (y_lo + y_hi) / 2,
        f"Δ = {diff_cm:.2f} cm",
        color=TXT,
        fontsize=8,
        va="center",
    )
    ax_gp8.set_xlabel("Time since first contact (s)", fontsize=9)
    ax_gp8.set_ylabel("Ice thickness (cm)", fontsize=9)
    ax_gp8.set_title(
        f"B — gp8: Two separate contact positions  (combined σ = {T8.std()*100:.2f} cm)",
        color=TXT,
        fontsize=10,
        pad=5,
    )
    ax_gp8.legend(fontsize=7.5, facecolor=BG, edgecolor=GRID, labelcolor=TXT)

    # ── Panel C: gp1 – long-session drift ──────────────────────────────────
    rows1 = sorted(by_gp[1], key=lambda r: float(r["timestamp_s"]))
    ts1 = np.array([float(r["timestamp_s"]) for r in rows1])
    D1 = np.array([float(r["depth_m"]) for r in rows1])
    ts1 -= ts1[0]

    ax_drift.plot(
        ts1, D1, color="#5577ff", lw=0.7, alpha=0.8, label="depth"
    )
    ax_drift.invert_yaxis()

    ax_drift.set_xlabel("Time since session start (s)", fontsize=9)
    ax_drift.set_ylabel("Depth (m, downward positive)", fontsize=9)
    ax_drift.set_title(
        f"C — gp1: Slow depth drift over long session  (σ = {D1.std()*100:.2f} cm)",
        color=TXT,
        fontsize=10,
        pad=5,
    )
    ax_drift.legend(
        fontsize=8.5, facecolor=BG, edgecolor=GRID, labelcolor=TXT, loc="lower left"
    )

    fig.suptitle(
        "Dominant Measurement Error Sources",
        color=TXT,
        fontsize=14,
        fontweight="bold",
        y=0.97,
    )

    # Footer note
    fig.text(
        0.5,
        0.01,
        "A: 98.2% of gp2 variance explained by pitch alone. "
        "B: gp8 sessions at different contact spots; each is internally stable. "
        "C: gp1 depth creep is likely slow AUV–ice relative motion over 3+ minutes.",
        ha="center",
        va="bottom",
        fontsize=8,
        color="#444444",
        wrap=True,
    )
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--raw",
        default=DEFAULT_RAW,
        metavar="FILE",
        help=f"Raw CSV (default: {DEFAULT_RAW})",
    )
    parser.add_argument(
        "--av",
        default=DEFAULT_AV,
        metavar="FILE",
        help=f"Averaged CSV (default: {DEFAULT_AV})",
    )
    parser.add_argument(
        "--out",
        default=DEFAULT_OUT,
        metavar="DIR",
        help=f"Output directory (default: {DEFAULT_OUT})",
    )
    parser.add_argument(
        "--config",
        default=None,
        metavar="FILE",
        help="config.yaml used during extraction (for metadata)",
    )
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    mpl.rcParams["font.family"] = "DejaVu Sans"

    print("Loading data...")
    raw_rows = load_csv(args.raw)
    av_rows = load_csv(args.av)
    cfg = load_config(args.config)

    # Derive all 16 grid points from the averaged CSV target positions
    gp_coords = {}
    for r in av_rows:
        gid = int(r["grid_point_id"])
        if gid not in gp_coords:
            gp_coords[gid] = {
                "lat": float(r["grid_point_lat"]),
                "lon": float(r["grid_point_lon"]),
            }
    # Reconstruct full 16-point grid by linear interpolation of the 4 columns
    # (the script already has the coords in the av CSV; any missing planned points
    # we approximate from the known spacing)
    all_grid_pts = [{"id": gid, **coords} for gid, coords in gp_coords.items()]
    # If some planned points are missing from the av CSV, we can still show them
    # if we had their coords — here we only know measured ones.

    measured_pts = unique_point_averages(av_rows)
    rpg = raw_per_point(raw_rows)
    entry_hole = estimate_entry_hole(av_rows)

    # Figures
    print("Rendering figure 1: lake context map...")
    f1 = fig_lake_context(all_grid_pts, measured_pts, entry_hole=entry_hole)

    print("Rendering figure 2: grid overview map...")
    f2 = fig_map_overview(all_grid_pts, measured_pts, av_rows, entry_hole=entry_hole)

    print("Rendering figure 3: heatmap...")
    f3 = fig_map_heatmap(all_grid_pts, measured_pts, entry_hole=entry_hole)

    print("Rendering figure 4: distributions...")
    f4 = fig_distributions(measured_pts, rpg, av_rows)

    print("Rendering figure 5: time series...")
    f5 = fig_time_series(raw_rows, av_rows)

    print("Rendering figure 6: statistics table...")
    f6 = fig_stats_table(measured_pts, rpg, av_rows, cfg)

    print("Rendering figure 7: ice density rationale...")
    f7 = fig_ice_density(cfg)

    print("Rendering figure 8: error analysis...")
    f8 = fig_error_analysis(raw_rows, av_rows)

    print("Rendering title page...")
    f0 = fig_title_page(av_rows, raw_rows, cfg)

    # Save individual PNGs
    for fname, fig in [
        ("map_lake_context.png", f1),
        ("map_overview.png", f2),
        ("map_heatmap.png", f3),
        ("distributions.png", f4),
        ("time_series.png", f5),
        ("stats_table.png", f6),
        ("ice_density.png", f7),
        ("error_analysis.png", f8),
    ]:
        out = os.path.join(args.out, fname)
        fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
        print(f"  Saved {out}")

    # Compile PDF
    # title → lake overview → zoomed map → heatmap → distributions →
    # time series → stats → error analysis → ice density
    pdf_path = os.path.join(args.out, "summary.pdf")
    print(f"Compiling PDF → {pdf_path}")
    with PdfPages(pdf_path) as pdf:
        for fig in [f0, f1, f2, f3, f4, f5, f6, f8, f7]:
            pdf.savefig(fig, bbox_inches="tight", facecolor=fig.get_facecolor())
        d = pdf.infodict()
        d["Title"] = "Ice Thickness Survey Summary"
        d["Author"] = "ARIS Space AUV"
        d["Subject"] = "Ice thickness grid survey report"

    print(f"\nDone. All outputs in {args.out}/")
    for fn in [
        "map_lake_context.png",
        "map_overview.png",
        "map_heatmap.png",
        "distributions.png",
        "time_series.png",
        "stats_table.png",
        "ice_density.png",
        "error_analysis.png",
        "summary.pdf",
    ]:
        print(f"  {fn}")


if __name__ == "__main__":
    main()
