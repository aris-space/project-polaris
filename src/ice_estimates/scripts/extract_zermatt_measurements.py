#!/usr/bin/env python3
"""
Extract ice thickness measurements from grid-survey rosbags.

Usage:
  python3 extract_zermatt_measurements.py [--out DIR]
                                           [--min-duration-s S]
                                           [--max-gap-s S]
                                           [BAG ...]

  BAG               Path to a rosbag2 directory (MCAP storage).
                    Defaults to the two Zermatt bags.
  --out DIR         Output directory for CSVs (default: /ros2_ws/measurements).
  --min-duration-s      Drop touch sessions shorter than this many seconds (default: 10).
  --max-gap-s           Merge consecutive sessions at the same grid point whose
                        gap is shorter than this many seconds (default: 60).
  --depth-window-s      Rolling window size in seconds for depth stability filter (default: 30).
  --max-depth-dev-m     Drop samples where depth exceeds the local rolling minimum by more
                        than this many meters (default: 0.01 = 1 cm).

Produces two CSVs in the output directory:
  measurements_raw.csv   – every valid sample while touching ice
  measurements_av.csv    – one averaged row per (merged) touch session

Ice thickness formula (identical to archimedes_touch.py):
  gauge_pressure        = pressure_pa - surface_pressure_pa
  depth_hydrostatic     = gauge_pressure / (rho_water * g)
  T                     = (1 / rho_ice) * ((depth_hydrostatic - omega) * rho_water)
  Skipped if abs(pitch_deg) > 25 or abs(roll_deg) > 10.

GNSS validity gate (/waterlinked_ugps/navsatfix):
  status.status >= 0  AND  sqrt(position_covariance[0]) < 4.0 m

Grid point assignment:
  Each session is assigned to the nearest integer grid point from
  /measurement_grid GeoJSON, computed from the session-averaged lat/lon.
  Samples in the raw CSV inherit the session-level grid point.
"""

import argparse
import csv
import json
import math
import os

import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf_transformations import euler_from_quaternion

# ---------------------------------------------------------------------------
# Defaults — all overridable via config.yaml or CLI flags
# ---------------------------------------------------------------------------

# Physics constants (Archimedes thickness formula)
DEFAULT_PRESSURE_TO_CONTACT_Z_M = 0.210  # Vertical: BlueRobotics pressure sensor is this far below ice contact point (m)
DEFAULT_PRESSURE_TO_CONTACT_X_M = 0.515  # Horizontal: BlueRobotics pressure sensor is this far forward of ice contact point (m)
DEFAULT_RHO_WATER         = 1000.0  # Fresh water density (kg/m³)
DEFAULT_RHO_ICE           = 917.0   # Ice density (kg/m³)
DEFAULT_G                 = 9.81    # Gravitational acceleration (m/s²)

# Attitude validity filter (samples outside these limits are skipped)
DEFAULT_MAX_PITCH_DEG = 25.0     # Maximum absolute pitch (deg)
DEFAULT_MAX_ROLL_DEG  = 10.0     # Maximum absolute roll (deg)

# GNSS validity gate
DEFAULT_GNSS_MAX_ACCURACY_M = 4.0   # Max horizontal 1-sigma from waterlinked navsatfix (m)

# Input / output
DEFAULT_BAGS = [
    "/ros2_ws/recordings/zermatt_grid_01_2026_04_30-12_04_16",
    "/ros2_ws/recordings/zermatt_grid_02_2026_04_30-13_00_44",
]
DEFAULT_OUT_DIR = "/ros2_ws/measurements"

# Session filtering
DEFAULT_MIN_DURATION_S  = 10.0   # Drop sessions shorter than this (s)
DEFAULT_MAX_GAP_S       = 60.0   # Merge same-point sessions with gap ≤ this (s)
DEFAULT_MIN_DEPTH_M     = 0.5    # Drop sessions with mean depth < this (m)

# Depth stability filter
DEFAULT_DEPTH_WINDOW_S  = 30.0   # Rolling window for local depth minimum (s)
DEFAULT_MAX_DEPTH_DEV_M = 0.01   # Max depth above local min to keep a sample (m)

TOPICS = [
    "/ice_touch_detection/touching",
    "/waterlinked_ugps/navsatfix",
    "/pixhawk/scaled_pressure",
    "/sensors/pressure/p_surface_pa",
    "/odometry/filtered/local",
    "/measurement_grid",
]

RAW_FIELDS = [
    "timestamp_s", "bag", "grid_point_id",
    "latitude", "longitude", "gnss_accuracy_m",
    "pressure_pa", "surface_pressure_pa",
    "depth_m", "roll_deg", "pitch_deg",
    "ice_thickness_m", "rho_ice",
]
AV_FIELDS = [
    "bag", "grid_point_id", "grid_point_lat", "grid_point_lon",
    "distance_to_target_m", "session_start_s", "session_end_s", "duration_s",
    "latitude", "longitude", "gnss_accuracy_m",
    "pressure_pa", "surface_pressure_pa",
    "depth_m", "roll_deg", "pitch_deg",
    "ice_thickness_m", "rho_ice", "n_samples",
]

# rho_ice is a constant stamped per row, not a measurement to average
NUMERIC_RAW = [f for f in RAW_FIELDS if f not in ("bag", "grid_point_id", "timestamp_s", "rho_ice")]


# ---------------------------------------------------------------------------
# Geometry / physics helpers
# ---------------------------------------------------------------------------

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def nearest_grid_point(lat, lon, grid_points):
    best_id, best_dist = None, float("inf")
    for pt in grid_points:
        d = haversine_m(lat, lon, pt["lat"], pt["lon"])
        if d < best_dist:
            best_dist = d
            best_id = pt["id"]
    return best_id, best_dist


def calc_thickness(pressure_pa, surface_pressure_pa, pitch_deg, roll_deg,
                   pressure_to_contact_z_m, pressure_to_contact_x_m,
                   rho_water, rho_ice, g, max_pitch_deg, max_roll_deg):
    if abs(pitch_deg) > max_pitch_deg or abs(roll_deg) > max_roll_deg:
        return float("nan")
    gauge = pressure_pa - surface_pressure_pa
    depth_sensor = gauge / (rho_water * g)
    pitch_rad = math.radians(pitch_deg)
    roll_rad  = math.radians(roll_deg)
    # Effective vertical distance from sensor to ice contact point.
    # Derived from the 3-D body-frame offset rotated into world frame:
    #   omega_corr = x_offset * sin(pitch) + z_offset * cos(pitch) * cos(roll)
    # At pitch=0, roll=0 this reduces to z_offset (0.210 m).
    omega_corr = (pressure_to_contact_x_m * math.sin(pitch_rad)
                  + pressure_to_contact_z_m * math.cos(pitch_rad) * math.cos(roll_rad))
    depth_contact = depth_sensor - omega_corr
    return depth_contact * rho_water / rho_ice


# ---------------------------------------------------------------------------
# Bag helpers
# ---------------------------------------------------------------------------

def open_reader(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    return reader, type_map


def read_grid_points(bag_path):
    """Return [{id, lat, lon}] for integer-id grid points from /measurement_grid."""
    reader, type_map = open_reader(bag_path)
    points = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/measurement_grid":
            continue
        msg = deserialize_message(data, get_message(type_map[topic]))
        for feat in json.loads(msg.geojson)["features"]:
            pid = feat["properties"]["id"]
            if not isinstance(pid, int):
                continue
            lon, lat = feat["geometry"]["coordinates"]
            points.append({"id": pid, "lat": lat, "lon": lon})
        break
    return points


# ---------------------------------------------------------------------------
# Session extraction
# ---------------------------------------------------------------------------

def extract_sessions(bag_path, grid_points,
                     gnss_max_accuracy_m,
                     pressure_to_contact_z_m, pressure_to_contact_x_m,
                     rho_water, rho_ice, g,
                     max_pitch_deg, max_roll_deg):
    """
    Stream bag_path and return a list of session dicts, one per touch window.

    Each session dict:
      raw_rows   : list of row dicts (numeric fields, no bag/grid_point_id yet)
      start_ts   : float (s)
      end_ts     : float (s)
      duration_s : float (s)
      bag        : str (basename of bag_path)
      grid_point_id, grid_point_lat, grid_point_lon, distance_to_target_m:
                   assigned after session ends from the averaged lat/lon
    """
    bag_name = os.path.basename(bag_path)
    reader, type_map = open_reader(bag_path)

    touching = False
    gnss_lat = gnss_lon = gnss_accuracy_m = None
    gnss_valid = False
    pressure_pa = surface_pressure_pa = None
    depth_m = roll_deg = pitch_deg = None

    in_session = False
    session_raw = []      # buffered raw rows for current session
    session_start = None
    session_end = None

    completed_sessions = []

    def close_session():
        if not session_raw:
            return
        dur = session_end - session_start
        avg_lat = float(np.mean([r["latitude"] for r in session_raw]))
        avg_lon = float(np.mean([r["longitude"] for r in session_raw]))
        gp_id, dist = nearest_grid_point(avg_lat, avg_lon, grid_points)
        gp = next(p for p in grid_points if p["id"] == gp_id)
        # stamp every raw row with the session-level grid point
        for r in session_raw:
            r["grid_point_id"] = gp_id
        completed_sessions.append({
            "bag":                  bag_name,
            "raw_rows":             list(session_raw),
            "start_ts":             session_start,
            "end_ts":               session_end,
            "duration_s":           dur,
            "grid_point_id":        gp_id,
            "grid_point_lat":       gp["lat"],
            "grid_point_lon":       gp["lon"],
            "distance_to_target_m": round(dist, 3),
        })
        session_raw.clear()

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        ts_s = ts_ns * 1e-9
        msg = deserialize_message(data, get_message(type_map[topic]))

        if topic == "/measurement_grid":
            continue

        elif topic == "/ice_touch_detection/touching":
            new_touching = msg.data
            if not new_touching and in_session:
                nonlocal_end = ts_s  # noqa – captured below
                close_session()
                in_session = False
            elif new_touching and not in_session:
                in_session = True
                session_start = ts_s
            touching = new_touching

        elif topic == "/waterlinked_ugps/navsatfix":
            status_ok = msg.status.status >= 0
            cov0 = msg.position_covariance[0]
            accuracy = math.sqrt(cov0) if cov0 > 0 else float("inf")
            gnss_valid = status_ok and accuracy < gnss_max_accuracy_m
            if gnss_valid:
                gnss_lat = msg.latitude
                gnss_lon = msg.longitude
                gnss_accuracy_m = accuracy

        elif topic == "/pixhawk/scaled_pressure":
            pressure_pa = msg.fluid_pressure

        elif topic == "/sensors/pressure/p_surface_pa":
            surface_pressure_pa = msg.data

        elif topic == "/odometry/filtered/local":
            depth_m = -msg.pose.pose.position.z
            q = msg.pose.pose.orientation
            r_rad, p_rad, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
            roll_deg = math.degrees(r_rad)
            pitch_deg = math.degrees(p_rad)

        if not (touching and in_session):
            continue
        if not gnss_valid:
            continue
        if any(v is None for v in [gnss_lat, gnss_lon, pressure_pa,
                                    surface_pressure_pa, depth_m, roll_deg, pitch_deg]):
            continue

        thickness = calc_thickness(
            pressure_pa, surface_pressure_pa, pitch_deg, roll_deg,
            pressure_to_contact_z_m, pressure_to_contact_x_m,
            rho_water, rho_ice, g, max_pitch_deg, max_roll_deg,
        )
        if math.isnan(thickness) or thickness < 0:
            continue

        session_end = ts_s
        session_raw.append({
            "timestamp_s":         round(ts_s, 3),
            "bag":                 bag_name,
            "grid_point_id":       None,          # filled at session close
            "latitude":            round(gnss_lat, 8),
            "longitude":           round(gnss_lon, 8),
            "gnss_accuracy_m":     round(gnss_accuracy_m, 3),
            "pressure_pa":         round(pressure_pa, 4),
            "surface_pressure_pa": round(surface_pressure_pa, 4),
            "depth_m":             round(depth_m, 4),
            "roll_deg":            round(roll_deg, 4),
            "pitch_deg":           round(pitch_deg, 4),
            "ice_thickness_m":     round(thickness, 4),
            "rho_ice":             rho_ice,
        })

    # Bag ended while still touching
    if in_session and session_raw:
        close_session()

    return completed_sessions


# ---------------------------------------------------------------------------
# Post-processing: filter and merge
# ---------------------------------------------------------------------------

def filter_and_merge(sessions, min_duration_s, max_gap_s, min_depth_m):
    """
    1. Drop sessions shorter than min_duration_s.
    2. Merge consecutive sessions (within a bag) at the same grid point
       whose gap is <= max_gap_s.
    Returns the processed session list.
    """
    # Per-bag processing to avoid cross-bag merges
    by_bag = {}
    for s in sessions:
        by_bag.setdefault(s["bag"], []).append(s)

    result = []
    for bag_sessions in by_bag.values():
        bag_sessions.sort(key=lambda s: s["start_ts"])

        # Step 1: filter short sessions and surface-contact tests
        def _keep(s):
            if s["duration_s"] < min_duration_s:
                return False
            avg_depth = float(np.mean([r["depth_m"] for r in s["raw_rows"]]))
            return avg_depth >= min_depth_m

        bag_sessions = [s for s in bag_sessions if _keep(s)]

        # Step 2: merge consecutive sessions at the same grid point
        merged = []
        for s in bag_sessions:
            if (merged
                    and merged[-1]["grid_point_id"] == s["grid_point_id"]
                    and s["start_ts"] - merged[-1]["end_ts"] <= max_gap_s):
                # Merge into the last session
                prev = merged[-1]
                prev["raw_rows"].extend(s["raw_rows"])
                prev["end_ts"] = s["end_ts"]
                prev["duration_s"] = prev["end_ts"] - prev["start_ts"]
                # Re-average distance from merged average position
                all_lat = [r["latitude"] for r in prev["raw_rows"]]
                all_lon = [r["longitude"] for r in prev["raw_rows"]]
                avg_lat = float(np.mean(all_lat))
                avg_lon = float(np.mean(all_lon))
                _, dist = nearest_grid_point(avg_lat, avg_lon,
                                             [{"id": prev["grid_point_id"],
                                               "lat": prev["grid_point_lat"],
                                               "lon": prev["grid_point_lon"]}])
                prev["distance_to_target_m"] = round(dist, 3)
            else:
                merged.append(s)

        result.extend(merged)

    result.sort(key=lambda s: s["start_ts"])
    return result


# ---------------------------------------------------------------------------
# Depth stability filter
# ---------------------------------------------------------------------------

def apply_depth_stability_filter(sessions, threshold_m, window_s):
    """
    Within each session, keep only samples whose depth is within `threshold_m`
    of the rolling minimum depth computed over a centred ±(window_s/2) window.

    The rolling minimum captures the local "true contact" depth — when the AUV
    is genuinely pressed against the ice. Samples where depth rises above that
    reference (AUV oscillated away from the ice) are discarded.
    """
    half = window_s / 2.0
    filtered = []

    for s in sessions:
        rows = s["raw_rows"]
        if not rows:
            filtered.append(s)
            continue

        timestamps = np.array([r["timestamp_s"] for r in rows])
        depths = np.array([r["depth_m"] for r in rows])

        keep_mask = np.zeros(len(rows), dtype=bool)
        for i in range(len(rows)):
            lo = np.searchsorted(timestamps, timestamps[i] - half)
            hi = np.searchsorted(timestamps, timestamps[i] + half, side="right")
            local_min = depths[lo:hi].min()
            keep_mask[i] = depths[i] <= local_min + threshold_m

        kept = [r for r, k in zip(rows, keep_mask) if k]
        dropped = len(rows) - len(kept)

        if kept:
            s = dict(s)          # shallow copy so we don't mutate the original
            s["raw_rows"] = kept
            filtered.append(s)
            if dropped:
                print(f"    gp{s['grid_point_id']} ({s['bag'][-16:]}): "
                      f"dropped {dropped}/{len(rows)} samples outside {threshold_m*100:.0f} cm depth band")
        else:
            print(f"    gp{s['grid_point_id']} ({s['bag'][-16:]}): "
                  f"session entirely removed by depth stability filter")

    return filtered


# ---------------------------------------------------------------------------
# CSV output
# ---------------------------------------------------------------------------

def build_av_row(session):
    rows = session["raw_rows"]
    avg = {k: float(np.mean([r[k] for r in rows]))
           for k in NUMERIC_RAW}
    return {
        "bag":                  session["bag"],
        "grid_point_id":        session["grid_point_id"],
        "grid_point_lat":       session["grid_point_lat"],
        "grid_point_lon":       session["grid_point_lon"],
        "distance_to_target_m": session["distance_to_target_m"],
        "session_start_s":      round(session["start_ts"], 3),
        "session_end_s":        round(session["end_ts"], 3),
        "duration_s":           round(session["duration_s"], 1),
        "n_samples":            len(rows),
        "rho_ice":              rows[0]["rho_ice"],
        **{k: round(avg[k], 6) for k in NUMERIC_RAW},
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _load_config(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("bags", nargs="*", metavar="BAG",
                        help="Rosbag2 directory paths. Overrides 'bags' in config file.")
    parser.add_argument("--config", metavar="FILE",
                        help="YAML config file. CLI flags take precedence over config values.")
    parser.add_argument("--out", default=None, metavar="DIR",
                        help=f"Output directory (default: {DEFAULT_OUT_DIR})")
    parser.add_argument("--min-duration-s", type=float, default=None, metavar="S",
                        help=f"Drop sessions shorter than S seconds (default: {DEFAULT_MIN_DURATION_S})")
    parser.add_argument("--max-gap-s", type=float, default=None, metavar="S",
                        help=f"Merge same-point sessions with gap ≤ S seconds (default: {DEFAULT_MAX_GAP_S})")
    parser.add_argument("--min-depth-m", type=float, default=None, metavar="M",
                        help=f"Drop sessions with average depth < M meters (default: {DEFAULT_MIN_DEPTH_M})")
    parser.add_argument("--depth-window-s", type=float, default=None, metavar="S",
                        help=f"Rolling window (seconds) for depth stability filter (default: {DEFAULT_DEPTH_WINDOW_S})")
    parser.add_argument("--max-depth-dev-m", type=float, default=None, metavar="M",
                        help=f"Max depth above local rolling min to keep a sample (default: {DEFAULT_MAX_DEPTH_DEV_M})")
    # Physics constants
    parser.add_argument("--pressure-to-contact-z-m", type=float, default=None, metavar="M",
                        help=f"Vertical: BlueRobotics pressure sensor below ice contact point (default: {DEFAULT_PRESSURE_TO_CONTACT_Z_M})")
    parser.add_argument("--pressure-to-contact-x-m", type=float, default=None, metavar="M",
                        help=f"Horizontal: BlueRobotics pressure sensor forward of ice contact point (default: {DEFAULT_PRESSURE_TO_CONTACT_X_M})")
    parser.add_argument("--rho-water", type=float, default=None, metavar="K",
                        help=f"Water density kg/m³ (default: {DEFAULT_RHO_WATER})")
    parser.add_argument("--rho-ice", type=float, default=None, metavar="K",
                        help=f"Ice density kg/m³ (default: {DEFAULT_RHO_ICE})")
    parser.add_argument("--g", type=float, default=None, metavar="A",
                        help=f"Gravitational acceleration m/s² (default: {DEFAULT_G})")
    # Attitude and GNSS validity filters
    parser.add_argument("--max-pitch-deg", type=float, default=None, metavar="D",
                        help=f"Max absolute pitch to accept a sample (default: {DEFAULT_MAX_PITCH_DEG})")
    parser.add_argument("--max-roll-deg", type=float, default=None, metavar="D",
                        help=f"Max absolute roll to accept a sample (default: {DEFAULT_MAX_ROLL_DEG})")
    parser.add_argument("--gnss-max-accuracy-m", type=float, default=None, metavar="M",
                        help=f"Max waterlinked GNSS horizontal accuracy (default: {DEFAULT_GNSS_MAX_ACCURACY_M})")
    args = parser.parse_args()

    # Layer: script defaults → YAML config → CLI flags
    cfg = {}
    if args.config:
        cfg = _load_config(args.config)

    def get(cli_val, key, default):
        return cli_val if cli_val is not None else cfg.get(key, default)

    bag_paths   = args.bags or cfg.get("bags", DEFAULT_BAGS)
    out_dir     = get(args.out,               "out",                DEFAULT_OUT_DIR)
    min_dur     = get(args.min_duration_s,    "min_duration_s",     DEFAULT_MIN_DURATION_S)
    max_gap     = get(args.max_gap_s,         "max_gap_s",          DEFAULT_MAX_GAP_S)
    min_depth   = get(args.min_depth_m,       "min_depth_m",        DEFAULT_MIN_DEPTH_M)
    depth_win   = get(args.depth_window_s,    "depth_window_s",     DEFAULT_DEPTH_WINDOW_S)
    max_dev     = get(args.max_depth_dev_m,   "max_depth_dev_m",    DEFAULT_MAX_DEPTH_DEV_M)
    sensor_z    = get(args.pressure_to_contact_z_m, "pressure_to_contact_z_m", DEFAULT_PRESSURE_TO_CONTACT_Z_M)
    sensor_x    = get(args.pressure_to_contact_x_m, "pressure_to_contact_x_m", DEFAULT_PRESSURE_TO_CONTACT_X_M)
    rho_water   = get(args.rho_water,          "rho_water",          DEFAULT_RHO_WATER)
    rho_ice     = get(args.rho_ice,           "rho_ice",            DEFAULT_RHO_ICE)
    g           = get(args.g,                 "g",                  DEFAULT_G)
    max_pitch   = get(args.max_pitch_deg,     "max_pitch_deg",      DEFAULT_MAX_PITCH_DEG)
    max_roll    = get(args.max_roll_deg,      "max_roll_deg",       DEFAULT_MAX_ROLL_DEG)
    gnss_acc    = get(args.gnss_max_accuracy_m,"gnss_max_accuracy_m",DEFAULT_GNSS_MAX_ACCURACY_M)

    os.makedirs(out_dir, exist_ok=True)

    # Read grid points from first bag
    print("Reading grid points...")
    grid_points = read_grid_points(bag_paths[0])
    print(f"  {len(grid_points)} points loaded: {sorted(p['id'] for p in grid_points)}")

    # Extract all sessions from all bags
    all_sessions = []
    for bag_path in bag_paths:
        print(f"Extracting sessions from {os.path.basename(bag_path)}...")
        sessions = extract_sessions(
            bag_path, grid_points,
            gnss_max_accuracy_m=gnss_acc,
            pressure_to_contact_z_m=sensor_z, pressure_to_contact_x_m=sensor_x,
            rho_water=rho_water, rho_ice=rho_ice, g=g,
            max_pitch_deg=max_pitch, max_roll_deg=max_roll,
        )
        print(f"  {len(sessions)} raw sessions found")
        all_sessions.extend(sessions)

    # Filter and merge
    print(f"\nFiltering (min {min_dur}s, min depth {min_depth}m) "
          f"and merging (gap ≤ {max_gap}s)...")
    sessions = filter_and_merge(all_sessions, min_dur, max_gap, min_depth)
    print(f"  {len(sessions)} sessions remaining")

    print(f"\nApplying depth stability filter "
          f"(window={depth_win}s, max_dev={max_dev*100:.0f} cm)...")
    sessions = apply_depth_stability_filter(sessions, max_dev, depth_win)

    # Write output
    raw_path = os.path.join(out_dir, "measurements_raw.csv")
    av_path = os.path.join(out_dir, "measurements_av.csv")

    with open(raw_path, "w", newline="") as rf, open(av_path, "w", newline="") as af:
        raw_writer = csv.DictWriter(rf, fieldnames=RAW_FIELDS)
        av_writer = csv.DictWriter(af, fieldnames=AV_FIELDS)
        raw_writer.writeheader()
        av_writer.writeheader()

        for s in sessions:
            for row in s["raw_rows"]:
                raw_writer.writerow(row)
            av_writer.writerow(build_av_row(s))

    total_raw = sum(len(s["raw_rows"]) for s in sessions)
    print(f"\nDone.")
    print(f"  Sessions : {len(sessions)}")
    print(f"  Raw rows : {total_raw}")
    print(f"  Raw CSV  : {raw_path}")
    print(f"  Avg CSV  : {av_path}")
    print()
    print(f"  {'bag':<16} {'gp':>3}  {'duration_s':>10}  {'thickness_m':>12}  {'dist_m':>8}  {'n_samples':>9}")
    print("  " + "-" * 70)
    for s in sessions:
        av = build_av_row(s)
        print(f"  {s['bag'][-16:]:<16} {s['grid_point_id']:>3}  "
              f"{av['duration_s']:>10.1f}  {av['ice_thickness_m']:>12.4f}  "
              f"{av['distance_to_target_m']:>8.2f}  {av['n_samples']:>9}")


if __name__ == "__main__":
    main()
