#!/usr/bin/env python3
"""
Subdivide each leg of a WGS84 mission CSV so that consecutive waypoints
are at most ``--step`` metres apart horizontally (default 10 m).

Input CSV columns:  lat, lon, alt, up_down  (minimum)
Optional columns:   delta_z_m, hold_s

The first waypoint is kept as-is.  From waypoint 1 onwards every leg
(segment between two original waypoints) is subdivided: intermediate
points are inserted every ``--step`` horizontal metres, all with
``up_down = 1``.  The original endpoint of each leg keeps its original
``up_down`` value.

Usage
-----
    python3 leg_partitioning.py input.csv -o output.csv
    python3 leg_partitioning.py input.csv --step 5 -o output.csv

If ``-o`` is omitted the result is written to ``<input>_partitioned.csv``.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import List

# ── WGS-84 maths (same as load_wsg84_points_to_waypoints.py) ─────────

_A = 6_378_137.0
_F = 1.0 / 298.257223563
_E_SQ = 2.0 * _F - _F ** 2


def _deg2rad(lat: float, lon: float):
    return math.radians(lat), math.radians(lon)


def _wgs84_to_ecef(lat_rad: float, lon_rad: float, alt_m: float):
    n = _A / math.sqrt(1.0 - _E_SQ * math.sin(lat_rad) ** 2)
    x = (n + alt_m) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (n + alt_m) * math.cos(lat_rad) * math.sin(lon_rad)
    z = ((1.0 - _E_SQ) * n + alt_m) * math.sin(lat_rad)
    return x, y, z


def _diff_ecef_to_enu(lat0_rad: float, lon0_rad: float,
                       dx: float, dy: float, dz: float):
    sin_lat = math.sin(lat0_rad)
    cos_lat = math.cos(lat0_rad)
    sin_lon = math.sin(lon0_rad)
    cos_lon = math.cos(lon0_rad)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def _horizontal_distance_m(lat1: float, lon1: float, alt1: float,
                            lat2: float, lon2: float, alt2: float) -> float:
    """Horizontal (East-North) distance between two WGS84 points [m]."""
    lat1r, lon1r = _deg2rad(lat1, lon1)
    lat2r, lon2r = _deg2rad(lat2, lon2)
    x1, y1, z1 = _wgs84_to_ecef(lat1r, lon1r, alt1)
    x2, y2, z2 = _wgs84_to_ecef(lat2r, lon2r, alt2)
    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
    e, n, _u = _diff_ecef_to_enu(lat1r, lon1r, dx, dy, dz)
    return math.hypot(e, n)


# ── Leg partitioning ─────────────────────────────────────────────────

def _interpolate(lat1: float, lon1: float, alt1: float,
                 lat2: float, lon2: float, alt2: float,
                 frac: float):
    """Linear interpolation in lat/lon/alt (accurate for short legs)."""
    return (lat1 + frac * (lat2 - lat1),
            lon1 + frac * (lon2 - lon1),
            alt1 + frac * (alt2 - alt1))


def partition_leg(wp_a: dict, wp_b: dict, step_m: float,
                  extra_fields: List[str]) -> List[dict]:
    """Return intermediate + endpoint rows for leg A->B.

    Intermediate points get up_down=1; the endpoint keeps its original
    up_down value.  wp_a itself is NOT included (already emitted).
    """
    lat1, lon1, alt1 = float(wp_a['lat']), float(wp_a['lon']), float(wp_a['alt'])
    lat2, lon2, alt2 = float(wp_b['lat']), float(wp_b['lon']), float(wp_b['alt'])

    dist = _horizontal_distance_m(lat1, lon1, alt1, lat2, lon2, alt2)

    if dist <= step_m:
        return [dict(wp_b)]

    n_segments = math.ceil(dist / step_m)
    rows: List[dict] = []

    for i in range(1, n_segments):
        frac = i / n_segments
        lat_i, lon_i, alt_i = _interpolate(lat1, lon1, alt1, lat2, lon2, alt2, frac)
        row = {
            'lat': f'{lat_i:.6f}',
            'lon': f'{lon_i:.6f}',
            'alt': f'{alt_i:.1f}',
            'up_down': '1',
        }
        for col in extra_fields:
            row[col] = ''
        rows.append(row)

    rows.append(dict(wp_b))
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('input', type=Path, help='Input WGS84 mission CSV')
    parser.add_argument('-o', '--output', type=Path, default=None,
                        help='Output CSV path (default: <input>_partitioned.csv)')
    parser.add_argument('--step', type=float, default=10.0,
                        help='Max horizontal distance between waypoints [m] (default: 10)')
    args = parser.parse_args()

    if args.output is None:
        args.output = args.input.with_stem(args.input.stem + '_partitioned')

    # ── Read ──────────────────────────────────────────────────────────
    with open(args.input, newline='') as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        waypoints = list(reader)

    if len(waypoints) < 2:
        print('Need at least 2 waypoints.', file=sys.stderr)
        sys.exit(1)

    # Columns beyond the core four that we carry through (empty for intermediates).
    extra_fields = [c for c in fieldnames if c not in ('lat', 'lon', 'alt', 'up_down')]

    # ── Partition ─────────────────────────────────────────────────────
    result: List[dict] = [waypoints[0]]

    for prev, cur in zip(waypoints, waypoints[1:]):
        result.extend(partition_leg(prev, cur, args.step, extra_fields))

    # ── Write ─────────────────────────────────────────────────────────
    with open(args.output, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(result)

    print(f'Wrote {len(result)} waypoints to {args.output}  '
          f'(was {len(waypoints)} original)')


if __name__ == '__main__':
    main()
