#!/usr/bin/env python3
"""WGS84 (lat, lon, alt) to ``map`` ENU ``PoseStamped`` list; origin defines local tangent plane."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header

# Defaults when optional columns are missing or empty (Phase A — vertical bump metadata).
DEFAULT_DELTA_Z_M = 0.3
DEFAULT_HOLD_S = 1.0


@dataclass(frozen=True)
class WaypointMeta:
    """Per-row mission metadata parallel to each ``PoseStamped`` from the same CSV row."""

    up_down: bool = False
    """If True, run up → hold → down after Nav2 reports the waypoint reached."""

    delta_z_m: float = DEFAULT_DELTA_Z_M
    """Heave distance in map ENU Up (meters); used when ``up_down`` is True."""

    hold_s: float = DEFAULT_HOLD_S
    """Seconds to hold at the top before descending."""


def _row_lower(row: dict) -> dict:
    return {(k or '').strip().lower(): (v or '').strip() for k, v in row.items() if k}


def _parse_up_down_flag(raw: str) -> bool:
    s = (raw or '').strip().lower()
    if not s:
        return False
    return s in ('1', 'true', 'yes', 'y', 'on')


def _parse_optional_float(raw: str, default: float) -> float:
    s = (raw or '').strip()
    if not s:
        return default
    return float(s)


def _meta_from_row(row_l: dict) -> WaypointMeta:
    # CSV: prefer column ``up_down``; ``vertical_bump`` still accepted as an alias.
    bump_raw = row_l.get('up_down', '') or row_l.get('vertical_bump', '')
    bump = _parse_up_down_flag(bump_raw)
    dz = _parse_optional_float(row_l.get('delta_z_m', ''), DEFAULT_DELTA_Z_M)
    hold = _parse_optional_float(row_l.get('hold_s', ''), DEFAULT_HOLD_S)
    if dz < 0.0:
        raise ValueError(f'delta_z_m must be non-negative, got {dz!r}')
    if hold < 0.0:
        raise ValueError(f'hold_s must be non-negative, got {hold!r}')
    return WaypointMeta(up_down=bump, delta_z_m=dz, hold_s=hold)

# sources: https://gssc.esa.int/navipedia/index.php?title=Ellipsoidal_and_Cartesian_Coordinates_Conversion&action=edit
# sources: https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
# sources: https://en.wikipedia.org/wiki/World_Geodetic_System


def degrees_to_radians(lat: float, lon: float) -> tuple:
    return math.radians(lat), math.radians(lon)


def wgs84_to_ecef(lat_rad: float, lon_rad: float, alt_m: float) -> tuple:
    """Geodetic (radians, meters) to ECEF (m)."""
    a = 6378137.0
    f = 1.0 / 298.257223563
    e_sq = 2.0 * f - f**2
    n = a / math.sqrt(1.0 - e_sq * math.sin(lat_rad) ** 2)
    x = (n + alt_m) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (n + alt_m) * math.cos(lat_rad) * math.sin(lon_rad)
    z = ((1.0 - e_sq) * n + alt_m) * math.sin(lat_rad)
    return x, y, z


def diff_ecef_to_enu(lat0_rad: float, lon0_rad: float, dx: float, dy: float, dz: float) -> tuple:
    """ECEF delta (dx,dy,dz) to ENU at origin (lat0, lon0 in radians)."""
    sin_lat = math.sin(lat0_rad)
    cos_lat = math.cos(lat0_rad)
    sin_lon = math.sin(lon0_rad)
    cos_lon = math.cos(lon0_rad)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def make_pose(x: float, y: float, z: float) -> PoseStamped:
    return PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=x, y=y, z=z)))


def process_mission(
    csv_filepath: str | Path, lat0: float, lon0: float, alt0: float
) -> Tuple[List[PoseStamped], List[WaypointMeta]]:
    """
    Read a mission CSV into map poses and parallel ``WaypointMeta`` rows.

    Required columns: lat, lon, alt (degrees, degrees, meters; same alt datum as alt0).

    Optional columns (any omitted → defaults on every row):

    - up_down — 1 / true / yes / y / on = run up → hold → down after arrival; empty or 0 = skip
      (optional alias column name: vertical_bump)
    - delta_z_m — bump size in map ENU Up (m); empty → DEFAULT_DELTA_Z_M
    - hold_s — hold time at top (s); empty → DEFAULT_HOLD_S
    """
    path = Path(csv_filepath)
    waypoints_enu: List[PoseStamped] = []
    meta_list: List[WaypointMeta] = []

    lat0_rad, lon0_rad = degrees_to_radians(lat0, lon0)
    x0, y0, z0 = wgs84_to_ecef(lat0_rad, lon0_rad, alt0)

    with path.open(newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f'Missing CSV header in {path}')
        fields = {h.strip().lower() for h in reader.fieldnames if h}
        if not {'lat', 'lon', 'alt'}.issubset(fields):
            raise ValueError(
                f'CSV must have columns lat,lon,alt (header row). Got: {reader.fieldnames!r}'
            )

        for row in reader:
            if not row or not any((v or '').strip() for v in row.values()):
                continue
            row_l = _row_lower(row)
            lat_raw = row_l.get('lat', '')
            if lat_raw.startswith('#'):
                continue
            lat = float(lat_raw)
            lon = float(row_l['lon'])
            alt = float(row_l['alt'])
            lat_rad, lon_rad = degrees_to_radians(lat, lon)
            x, y, z = wgs84_to_ecef(lat_rad, lon_rad, alt)
            dx, dy, dz = x - x0, y - y0, z - z0
            e, n, u = diff_ecef_to_enu(lat0_rad, lon0_rad, dx, dy, dz)
            waypoints_enu.append(make_pose(e, n, u))
            meta_list.append(_meta_from_row(row_l))

    if not waypoints_enu:
        raise ValueError(f'No waypoints read from {path}')
    if len(meta_list) != len(waypoints_enu):
        raise RuntimeError('internal error: meta/poses length mismatch')
    return waypoints_enu, meta_list


def process_coordinates(csv_filepath: str | Path, lat0: float, lon0: float, alt0: float) -> List[PoseStamped]:
    """Same as ``process_mission`` but returns only the pose list (backward compatible)."""
    poses, _ = process_mission(csv_filepath, lat0, lon0, alt0)
    return poses
