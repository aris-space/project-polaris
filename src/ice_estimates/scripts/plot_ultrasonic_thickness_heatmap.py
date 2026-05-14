#!/usr/bin/env python3
"""
Continuous ice-thickness heatmap from the top-mounted ultrasonic sensor.

Per ultrasonic sample, compute Archimedes thickness from the pressure depth,
the body-z offset between the pressure and ultrasonic sensors, and the
attitude-corrected ultrasonic distance:

    c                = cos(pitch) * cos(roll)
    depth_ultrasonic = depth_pressure - pressure_to_ultrasonic_z_m * c
    ice_draft        = depth_ultrasonic - d_ultrasonic * c
    T_ice            = ice_draft * rho_water / rho_ice

Samples are then placed on the map using the interpolated /waterlinked_ugps
position. The trajectory is dense and the per-sample thickness is noisy, so
samples are binned into square cells (median per cell) before the heatmap
interpolation — matching the spirit of the grid-point heatmap in
visualize_ice_measurements.py, but with cells anywhere the AUV went.

Usage:
    python3 plot_ultrasonic_thickness_heatmap.py [BAG ...] [--out FILE]
                                                  [--bin-m M] [--config FILE]

Reads:
    /top/ultrasonic/distance     (std_msgs/Float32)
    /sensors/pressure/pose_enu   (geometry_msgs/PoseWithCovarianceStamped)
    /odometry/filtered/local     (nav_msgs/Odometry)
    /waterlinked_ugps/navsatfix  (sensor_msgs/NavSatFix)
    /measurement_grid            (foxglove_msgs/GeoJSON) — optional, for planned-grid markers
"""

import argparse
import io
import math
import json
import sys
import time
import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import requests
import yaml
from PIL import Image

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    sys.exit(1)

from matplotlib.colors import Normalize
from pyproj import Transformer
from scipy.interpolate import griddata
from scipy.spatial import Delaunay

warnings.filterwarnings("ignore", category=UserWarning)

# ─── Topics ────────────────────────────────────────────────────────────────────
TOPIC_ULTRA = "/top/ultrasonic/distance"
TOPIC_DEPTH = "/sensors/pressure/pose_enu"
TOPIC_ODOM  = "/odometry/filtered/local"
TOPIC_GNSS  = "/waterlinked_ugps/navsatfix"
TOPIC_GRID  = "/measurement_grid"

# ─── Sample filters ────────────────────────────────────────────────────────────
MIN_DEPTH_M         = 0.5    # ignore pressure depth shallower than this
MAX_ULTRA_M         = 5.0    # ultrasonic outlier cap
MAX_STALENESS_S     = 1.0    # require GNSS / depth / odom within this window
MAX_PITCH_DEG       = 25.0   # drop tilted samples (mirrors extract_zermatt_measurements.py)
MAX_ROLL_DEG        = 10.0
GNSS_MAX_ACCURACY_M = 4.0    # sqrt(position_covariance[0])
MIN_THICKNESS_M     = 0.05   # physically plausible bounds for Schwarzsee
MAX_THICKNESS_M     = 2.0

# ─── Archimedes constants (mirror config.yaml) ────────────────────────────────
PRESSURE_TO_ULTRASONIC_Z_M = 0.062
RHO_WATER                  = 999.4
RHO_ICE                    = 887.5

# ─── Map style (mirror visualize_ice_measurements.py fig_map_heatmap) ──────────
MAP_CRS          = "EPSG:3857"
DATA_CRS         = "EPSG:4326"
CMAP_THICKNESS   = "plasma"
ALPHA_HEATMAP    = 0.55
MAP_PAD_M        = 12
INTERP_RES       = 200
DEFAULT_BIN_M    = 2.0   # spatial bin size for median-per-cell averaging
BASEMAP_ZOOM     = 19    # max supported by SwissFederalGeoportal.SWISSIMAGE

# Swisstopo WMTS — direct tile URL pattern (no contextily wrapper).
# Avoids contextily's joblib-batched fetcher, which is too brittle against
# transient TCP resets from this endpoint.
BASEMAP_TILE_URL = (
    "https://wmts.geo.admin.ch/1.0.0/ch.swisstopo.swissimage/"
    "default/current/3857/{z}/{x}/{y}.jpeg"
)
BASEMAP_USER_AGENT = "Mozilla/5.0 (compatible; project-polaris ice-thickness-heatmap)"

# Persistent disk cache for downloaded tiles. First run with network populates
# it; subsequent runs (including offline ones) serve from cache. Lives next to
# this script so it is self-contained and shareable.
BASEMAP_CACHE_DIR = Path(__file__).parent / ".basemap_cache"
BASEMAP_CACHE_DIR.mkdir(exist_ok=True)

DEFAULT_BAGS = [
    "/ros2_ws/recordings/zermatt_grid_01_2026_04_30-12_04_16",
    "/ros2_ws/recordings/zermatt_grid_02_2026_04_30-13_00_44",
]

to_mercator = Transformer.from_crs(DATA_CRS, MAP_CRS, always_xy=True)


# ─── Helpers ───────────────────────────────────────────────────────────────────

def roll_pitch_from_quat(qx, qy, qz, qw):
    """Tait-Bryan roll, pitch (radians) from a unit quaternion (body -> world, ZYX)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def wgs_to_mercator(lats, lons):
    xs, ys = to_mercator.transform(lons, lats)
    return np.asarray(xs), np.asarray(ys)


def load_config(path):
    if not path:
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def coactive_mask(ts_query, ts_other, max_dt=MAX_STALENESS_S):
    if len(ts_other) == 0:
        return np.zeros(len(ts_query), dtype=bool)
    idx = np.searchsorted(ts_other, ts_query).clip(0, len(ts_other) - 1)
    idx_prev = (idx - 1).clip(0, len(ts_other) - 1)
    nearest = np.minimum(
        np.abs(ts_query - ts_other[idx]),
        np.abs(ts_query - ts_other[idx_prev]),
    )
    return nearest <= max_dt


# ─── Bag extraction ────────────────────────────────────────────────────────────

def extract_data(bag_paths):
    """Read all topics across all bags. Returns numpy arrays indexed by event type."""
    topics = [TOPIC_ULTRA, TOPIC_DEPTH, TOPIC_ODOM, TOPIC_GNSS, TOPIC_GRID]

    ultra_ts, ultra_dist = [], []
    depth_ts, depth_m = [], []
    odom_ts, odom_roll, odom_pitch = [], [], []
    gnss_ts, gnss_lat, gnss_lon, gnss_acc = [], [], [], []
    grid_points = []  # [{id, lat, lon}, ...]

    t_offset = None
    latest_depth = None

    for bag_path in bag_paths:
        bag_path = Path(bag_path)
        print(f"Reading {bag_path.name}...")
        with AnyReader([bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic in topics]
            available = {c.topic for c in connections}
            for t in topics:
                if t not in available:
                    print(f"  WARNING: {t} not found in {bag_path.name}", file=sys.stderr)

            for conn, t_ns, raw in reader.messages(connections=connections):
                t_s = t_ns * 1e-9
                if t_offset is None:
                    t_offset = t_s
                t_rel = t_s - t_offset
                msg = reader.deserialize(raw, conn.msgtype)

                if conn.topic == TOPIC_DEPTH:
                    d = -float(msg.pose.pose.position.z)
                    latest_depth = d
                    if d >= MIN_DEPTH_M:
                        depth_ts.append(t_rel)
                        depth_m.append(d)

                elif conn.topic == TOPIC_ULTRA:
                    d_m = float(msg.data)
                    if d_m == 0.0 or d_m > MAX_ULTRA_M:
                        continue
                    if latest_depth is None or latest_depth < MIN_DEPTH_M:
                        continue
                    ultra_ts.append(t_rel)
                    ultra_dist.append(d_m)

                elif conn.topic == TOPIC_ODOM:
                    q = msg.pose.pose.orientation
                    norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
                    if norm_sq < 0.5:
                        continue
                    r, p = roll_pitch_from_quat(q.x, q.y, q.z, q.w)
                    odom_ts.append(t_rel)
                    odom_roll.append(r)
                    odom_pitch.append(p)

                elif conn.topic == TOPIC_GNSS:
                    if msg.status.status < 0:
                        continue
                    cov0 = float(msg.position_covariance[0])
                    acc = math.sqrt(cov0) if cov0 > 0 else float("inf")
                    gnss_ts.append(t_rel)
                    gnss_lat.append(float(msg.latitude))
                    gnss_lon.append(float(msg.longitude))
                    gnss_acc.append(acc)

                elif conn.topic == TOPIC_GRID and not grid_points:
                    try:
                        feats = json.loads(msg.geojson)["features"]
                        for f in feats:
                            pid = f["properties"].get("id")
                            if not isinstance(pid, int):
                                continue
                            lon, lat = f["geometry"]["coordinates"]
                            grid_points.append({"id": pid, "lat": lat, "lon": lon})
                    except Exception as e:
                        print(f"  WARNING: failed to parse {TOPIC_GRID}: {e}", file=sys.stderr)

    return {
        "ultra_ts":  np.array(ultra_ts),
        "ultra":    np.array(ultra_dist),
        "depth_ts":  np.array(depth_ts),
        "depth":    np.array(depth_m),
        "odom_ts":   np.array(odom_ts),
        "roll":     np.array(odom_roll),
        "pitch":    np.array(odom_pitch),
        "gnss_ts":   np.array(gnss_ts),
        "gnss_lat":  np.array(gnss_lat),
        "gnss_lon":  np.array(gnss_lon),
        "gnss_acc":  np.array(gnss_acc),
        "grid_points": grid_points,
    }


# ─── Per-sample thickness computation ──────────────────────────────────────────

def compute_thickness_samples(data, p2u_offset, rho_water, rho_ice):
    """Return (lat, lon, T_m) arrays for all valid ultrasonic samples."""
    ts = data["ultra_ts"]
    u  = data["ultra"]
    if len(ts) == 0:
        sys.exit("No ultrasonic samples found.")
    if len(data["depth_ts"]) == 0:
        sys.exit("No pressure depth samples found.")
    if len(data["odom_ts"]) == 0:
        sys.exit("No odometry samples found — pitch/roll correction requires them.")
    if len(data["gnss_ts"]) == 0:
        sys.exit("No GNSS samples found — heatmap requires lat/lon per sample.")

    # Keep ultrasonic samples that have a fresh value of every required sensor.
    mask = (
        coactive_mask(ts, data["depth_ts"])
        & coactive_mask(ts, data["odom_ts"])
        & coactive_mask(ts, data["gnss_ts"])
    )
    ts = ts[mask]
    u  = u[mask]

    depth = np.interp(ts, data["depth_ts"], data["depth"])
    roll  = np.interp(ts, data["odom_ts"],  data["roll"])
    pitch = np.interp(ts, data["odom_ts"],  data["pitch"])
    lat   = np.interp(ts, data["gnss_ts"],  data["gnss_lat"])
    lon   = np.interp(ts, data["gnss_ts"],  data["gnss_lon"])
    acc   = np.interp(ts, data["gnss_ts"],  data["gnss_acc"])

    c = np.cos(pitch) * np.cos(roll)
    draft = depth - (p2u_offset + u) * c
    T     = draft * (rho_water / rho_ice)

    valid = (
        (np.abs(np.degrees(pitch)) <= MAX_PITCH_DEG)
        & (np.abs(np.degrees(roll))  <= MAX_ROLL_DEG)
        & (acc < GNSS_MAX_ACCURACY_M)
        & np.isfinite(T)
        & (T >= MIN_THICKNESS_M)
        & (T <= MAX_THICKNESS_M)
    )

    print(f"\nSamples after gating:  {valid.sum():,} / {len(T):,}")
    if valid.sum() == 0:
        sys.exit("All thickness samples filtered out — check thresholds.")

    return lat[valid], lon[valid], T[valid]


# ─── Spatial binning ───────────────────────────────────────────────────────────

def bin_median(mx, my, vals, bin_m):
    """Bin (mx, my) onto a regular grid of `bin_m` metres; median value per cell."""
    ix = np.floor(mx / bin_m).astype(int)
    iy = np.floor(my / bin_m).astype(int)
    keys = ix.astype(np.int64) * (1 << 32) + iy.astype(np.int64)
    order = np.argsort(keys)
    keys_sorted = keys[order]
    vals_sorted = vals[order]
    mx_sorted   = mx[order]
    my_sorted   = my[order]

    boundaries = np.r_[0, np.flatnonzero(np.diff(keys_sorted)) + 1, len(keys_sorted)]

    bx, by, bv, bn = [], [], [], []
    for a, b in zip(boundaries[:-1], boundaries[1:]):
        bx.append(mx_sorted[a:b].mean())
        by.append(my_sorted[a:b].mean())
        bv.append(np.median(vals_sorted[a:b]))
        bn.append(b - a)
    return np.array(bx), np.array(by), np.array(bv), np.array(bn)


# ─── Plot ──────────────────────────────────────────────────────────────────────

def _merc_to_tile(x_m, y_m, z):
    """Slippy-map tile indices (x, y) for a Web Mercator metric coordinate."""
    R = 6378137.0
    n = 2 ** z
    lon = math.degrees(x_m / R)
    lat = math.atan(math.sinh(y_m / R))
    xt = (lon + 180.0) / 360.0 * n
    yt = (1.0 - math.log(math.tan(lat) + 1.0 / math.cos(lat)) / math.pi) / 2.0 * n
    return xt, yt


def _tile_to_merc(xt, yt, z):
    """Inverse of _merc_to_tile (returns the NW corner of the tile in EPSG:3857)."""
    R = 6378137.0
    n = 2 ** z
    lon = xt / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * yt / n))))
    return math.radians(lon) * R, math.log(math.tan(math.pi / 4 + math.radians(lat) / 2)) * R


def _fetch_tile(session, x, y, z, max_tries=4):
    """Fetch a single tile (with retries + delay) and cache to BASEMAP_CACHE_DIR."""
    cache_path = BASEMAP_CACHE_DIR / f"{z}/{x}/{y}.jpeg"
    if cache_path.exists():
        return Image.open(cache_path).convert("RGB")
    cache_path.parent.mkdir(parents=True, exist_ok=True)

    url = BASEMAP_TILE_URL.format(z=z, x=x, y=y)
    last_err = None
    for attempt in range(max_tries):
        try:
            r = session.get(url, timeout=20)
            if r.status_code == 200:
                cache_path.write_bytes(r.content)
                return Image.open(io.BytesIO(r.content)).convert("RGB")
            last_err = f"status={r.status_code}"
        except Exception as e:
            last_err = f"{type(e).__name__}: {e}"
        time.sleep(1.0 + 0.5 * attempt)
    raise RuntimeError(f"Failed to fetch {url}: {last_err}")


def fetch_basemap_image(west, east, south, north, zoom=BASEMAP_ZOOM):
    """
    Download tiles covering the (web-mercator) rectangle and stitch them into
    a single RGB image. Returns (np.uint8 array, extent=(W, E, S, N)).
    """
    xt0, yt1 = _merc_to_tile(west,  south, zoom)
    xt1, yt0 = _merc_to_tile(east,  north, zoom)
    xt_lo, xt_hi = int(math.floor(min(xt0, xt1))), int(math.floor(max(xt0, xt1)))
    yt_lo, yt_hi = int(math.floor(min(yt0, yt1))), int(math.floor(max(yt0, yt1)))
    n_tiles = (xt_hi - xt_lo + 1) * (yt_hi - yt_lo + 1)
    print(f"Basemap: {n_tiles} tiles at zoom {zoom}  (x={xt_lo}..{xt_hi}, y={yt_lo}..{yt_hi})")

    session = requests.Session()
    session.headers["User-Agent"] = BASEMAP_USER_AGENT

    tile_w = tile_h = 256
    stitched = Image.new("RGB",
                         ((xt_hi - xt_lo + 1) * tile_w,
                          (yt_hi - yt_lo + 1) * tile_h),
                         color=(20, 28, 36))

    for x in range(xt_lo, xt_hi + 1):
        for y in range(yt_lo, yt_hi + 1):
            tile = _fetch_tile(session, x, y, zoom)
            stitched.paste(tile, ((x - xt_lo) * tile_w, (y - yt_lo) * tile_h))
            time.sleep(0.4)  # be polite — keeps Swisstopo from rate-limiting

    # Extent of the stitched image in EPSG:3857
    w_m, n_m = _tile_to_merc(xt_lo,     yt_lo,     zoom)
    e_m, s_m = _tile_to_merc(xt_hi + 1, yt_hi + 1, zoom)
    return np.array(stitched), (w_m, e_m, s_m, n_m)


def add_satellite(ax, pad_m=MAP_PAD_M):
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ax.set_xlim(x0 - pad_m, x1 + pad_m)
    ax.set_ylim(y0 - pad_m, y1 + pad_m)
    west, east = ax.get_xlim()
    south, north = ax.get_ylim()
    try:
        img, extent = fetch_basemap_image(west, east, south, north)
        ax.imshow(img, extent=extent, origin="upper", zorder=2,
                  interpolation="bilinear")
    except Exception as e:
        ax.set_facecolor("#1a3a5c")
        ax.text(0.5, 0.5, f"Satellite tiles unavailable\n({e})",
                ha="center", va="center", transform=ax.transAxes, color="white")


def prefetch_basemap(grid_points):
    """Warm BASEMAP_CACHE_DIR for the planned-grid extent, then exit."""
    gx, gy = wgs_to_mercator([p["lat"] for p in grid_points],
                             [p["lon"] for p in grid_points])
    west, east   = float(gx.min() - MAP_PAD_M), float(gx.max() + MAP_PAD_M)
    south, north = float(gy.min() - MAP_PAD_M), float(gy.max() + MAP_PAD_M)
    print(f"Prefetching SWISSIMAGE for x=[{west:.1f}, {east:.1f}]  y=[{south:.1f}, {north:.1f}]")
    try:
        fetch_basemap_image(west, east, south, north)
    except Exception as e:
        print(f"FAILED: {type(e).__name__}: {e}", file=sys.stderr)
        sys.exit(1)
    n_cached = sum(1 for _ in BASEMAP_CACHE_DIR.rglob("*.jpeg"))
    print(f"Done. Tiles cached: {n_cached}  ({BASEMAP_CACHE_DIR})")


def make_heatmap(lat, lon, T, bin_m, grid_points, out_path):
    mx_all, my_all = wgs_to_mercator(lat, lon)
    bx, by, bv, bn = bin_median(mx_all, my_all, T, bin_m)
    print(f"Spatial bins ({bin_m:.1f} m):     {len(bx):,}  "
          f"(median samples/bin = {int(np.median(bn))}, max = {int(bn.max())})")
    print(f"Per-bin thickness:        median={np.median(bv):.3f} m  "
          f"std={bv.std():.3f} m  range=[{bv.min():.3f}, {bv.max():.3f}] m")

    fig, ax = plt.subplots(figsize=(9, 8))
    fig.patch.set_facecolor("#0e1117")
    ax.set_facecolor("#0e1117")

    if grid_points:
        gx, gy = wgs_to_mercator([p["lat"] for p in grid_points],
                                 [p["lon"] for p in grid_points])
        ax.set_xlim(min(gx.min(), bx.min()), max(gx.max(), bx.max()))
        ax.set_ylim(min(gy.min(), by.min()), max(gy.max(), by.max()))
    else:
        ax.set_xlim(bx.min(), bx.max())
        ax.set_ylim(by.min(), by.max())
    add_satellite(ax)

    xi = np.linspace(ax.get_xlim()[0], ax.get_xlim()[1], INTERP_RES)
    yi = np.linspace(ax.get_ylim()[0], ax.get_ylim()[1], INTERP_RES)
    Xi, Yi = np.meshgrid(xi, yi)
    Zi = griddata((bx, by), bv, (Xi, Yi), method="linear")

    if len(bx) >= 3:
        tri = Delaunay(np.column_stack([bx, by]))
        inside = tri.find_simplex(np.column_stack([Xi.ravel(), Yi.ravel()])) >= 0
        Zi = np.ma.array(Zi, mask=~inside.reshape(Xi.shape))

    norm = Normalize(vmin=np.nanmin(bv), vmax=np.nanmax(bv))
    cmap = plt.get_cmap(CMAP_THICKNESS)
    ax.pcolormesh(Xi, Yi, Zi, cmap=cmap, norm=norm,
                  alpha=ALPHA_HEATMAP, zorder=3, shading="auto")

    # Bin-centre scatter
    ax.scatter(bx, by, s=18, c=bv, cmap=cmap, norm=norm,
               edgecolors="white", linewidths=0.4, zorder=5)

    # Planned grid markers (if available)
    if grid_points:
        ax.scatter(gx, gy, s=40, marker="x", color="lightgray",
                   linewidths=1.0, alpha=0.7, zorder=4)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, fraction=0.03, pad=0.02)
    cbar.set_label("Ice thickness (m)", color="white", fontsize=9)
    cbar.ax.yaxis.set_tick_params(color="white")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="white")

    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(
        f"Zermatt Schwarzsee — Ultrasonic Ice Thickness Heatmap\n"
        f"(Archimedes, attitude-corrected; {bin_m:.1f} m bins, "
        f"linear interpolation within measured hull)",
        color="white", fontsize=11, pad=10,
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"\nSaved: {out_path}")


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bags", nargs="*", metavar="BAG",
                        help="rosbag2 directories or MCAP files. "
                             "Defaults to the two zermatt_grid bags.")
    parser.add_argument("--out", default=None, metavar="FILE",
                        help="Output PNG (default: ultrasonic_thickness_heatmap_zermatt.png)")
    parser.add_argument("--bin-m", type=float, default=DEFAULT_BIN_M, metavar="M",
                        help=f"Spatial bin size in metres (default: {DEFAULT_BIN_M})")
    parser.add_argument("--config", default=None, metavar="FILE",
                        help="config.yaml to source rho_water / rho_ice / "
                             "pressure_to_ultrasonic_z_m")
    parser.add_argument("--pressure-to-ultrasonic-z-m", type=float, default=None,
                        metavar="M",
                        help="Body-z offset (m), ultrasonic above pressure. "
                             "Overrides config and script default.")
    parser.add_argument("--prefetch-basemap", action="store_true",
                        help="Fetch SWISSIMAGE tiles for the survey extent into the "
                             "on-disk cache and exit (no heatmap rendering). "
                             "Run once with internet access; afterwards the script "
                             "works fully offline.")
    args = parser.parse_args()

    cfg = load_config(args.config)
    p2u_offset = (
        args.pressure_to_ultrasonic_z_m
        if args.pressure_to_ultrasonic_z_m is not None
        else float(cfg.get("pressure_to_ultrasonic_z_m", PRESSURE_TO_ULTRASONIC_Z_M))
    )
    rho_water  = float(cfg.get("rho_water", RHO_WATER))
    rho_ice    = float(cfg.get("rho_ice",   RHO_ICE))

    bag_paths = args.bags or DEFAULT_BAGS
    out_path = (
        Path(args.out) if args.out
        else Path(__file__).parent / "ultrasonic_thickness_heatmap_zermatt.png"
    )

    print(f"pressure_to_ultrasonic_z_m = {p2u_offset:.3f} m")
    print(f"rho_water = {rho_water:.1f}  rho_ice = {rho_ice:.1f}")
    print(f"basemap cache dir = {BASEMAP_CACHE_DIR}")

    data = extract_data(bag_paths)

    if args.prefetch_basemap:
        if not data["grid_points"]:
            sys.exit("No /measurement_grid in the bags — cannot infer extent for prefetch.")
        prefetch_basemap(data["grid_points"])
        return

    lat, lon, T = compute_thickness_samples(data, p2u_offset, rho_water, rho_ice)
    print(f"Per-sample thickness:     median={np.median(T):.3f} m  "
          f"std={T.std():.3f} m  range=[{T.min():.3f}, {T.max():.3f}] m")

    make_heatmap(lat, lon, T, args.bin_m, data["grid_points"], out_path)


if __name__ == "__main__":
    main()
