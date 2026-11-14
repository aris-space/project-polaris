#!/usr/bin/env python3
"""Generate the SBL range-sweep waypoint line (thesis) as a WGS84 mission CSV.

A straight line of N waypoints spaced `--spacing` m apart, starting `--first` m from the antenna
(topside) and going `--bearing-enu` degrees ENU ("lakeward") for the Goldbach SBL mission. Run it on the
field day with the antenna's measured RTK position so the line is anchored exactly where the topside sits
(placement needn't be precise - the per-waypoint go-to corrects small offsets).

Defaults reproduce missions/sbl_line_goldbach.csv (191 deg ENU, first point at 10 m, 10 m spacing, 5 points
= 10..50 m, alt 453 HAE). `--first` moves the whole line out, e.g. `--first 20` gives 20..60 m.

After the line, two OFF-AXIS waypoints are appended (2026-11-14): both `--off-axis-range` m (default 30) from the
antenna, `--off-axis-angle` degrees (default 45) to either side of the line. With the default 5 line points they
are waypoint 6 (bearing - angle) and waypoint 7 (bearing + angle). Use: the off-axis station hold (6), and the
across dive / across surface mission 6 -> 3 -> 7 (two legs across the line of sight with a turn of about 45 deg
at station 3). `--off-axis-range 0` writes the line only.

Conventions:
  * bearing is ENU degrees: 0 = East, 90 = North, CCW. Steg lakeward = 191 deg (= 259 deg compass).
  * alt is WGS-84 ELLIPSOIDAL height (HAE), NOT MSL. On a surface "no-z" run it is not used for control,
    but keep it HAE for consistency.
  * CSV columns: lat,lon,alt,up_down,delta_z_m,hold_s  (last three parsed but NOT executed - left blank).

Examples:
  python scripts/make_sbl_line.py --antenna 47.328672 8.572691
  python scripts/make_sbl_line.py --antenna 47.3287 8.5727 --bearing-enu 191 --first 10 --spacing 10 --count 5 \
      --out src/autonomy/autonomy_bringup_pkg/missions/sbl_line_goldbach.csv
"""

import argparse
import math
import sys

_WGS84_M_PER_DEG_LAT = 111320.0


def generate(lat0, lon0, bearing_enu_deg, spacing_m, count, alt_hae, first_m=None):
    """Return a list of (lat, lon, range_m) along the line (range = distance from the antenna).

    first_m: distance of the first waypoint from the antenna (default: spacing_m, the original behaviour)."""
    if first_m is None:
        first_m = spacing_m
    m_per_deg_lon = _WGS84_M_PER_DEG_LAT * math.cos(math.radians(lat0))
    uE = math.cos(math.radians(bearing_enu_deg))
    uN = math.sin(math.radians(bearing_enu_deg))
    out = []
    for i in range(count):
        d = first_m + i * spacing_m
        lat = lat0 + (uN * d) / _WGS84_M_PER_DEG_LAT
        lon = lon0 + (uE * d) / m_per_deg_lon
        out.append((lat, lon, d))
    return out


def off_axis(lat0, lon0, bearing_enu_deg, range_m, angle_deg):
    """Two points `range_m` from the antenna, `angle_deg` to either side of the line: [(lat, lon, bearing), ...]
    in the order (bearing - angle), (bearing + angle)."""
    m_per_deg_lon = _WGS84_M_PER_DEG_LAT * math.cos(math.radians(lat0))
    out = []
    for b in (bearing_enu_deg - angle_deg, bearing_enu_deg + angle_deg):
        lat = lat0 + range_m * math.sin(math.radians(b)) / _WGS84_M_PER_DEG_LAT
        lon = lon0 + range_m * math.cos(math.radians(b)) / m_per_deg_lon
        out.append((lat, lon, b % 360.0))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--antenna', nargs=2, type=float, metavar=('LAT', 'LON'), required=True,
                   help='Antenna / pier-front position in degrees (from RTK).')
    p.add_argument('--bearing-enu', type=float, default=191.0,
                   help='Lakeward bearing in ENU degrees (default 191 = steg axis).')
    p.add_argument('--first', type=float, default=10.0,
                   help='Distance of the first waypoint from the antenna [m] (default 10).')
    p.add_argument('--spacing', type=float, default=10.0, help='Spacing between waypoints [m] (default 10).')
    p.add_argument('--count', type=int, default=5, help='Number of waypoints (default 5 -> 10..50 m).')
    p.add_argument('--off-axis-range', type=float, default=30.0,
                   help='Distance of the two off-axis waypoints from the antenna [m] (default 30; 0 = none).')
    p.add_argument('--off-axis-angle', type=float, default=45.0,
                   help='Angle of the off-axis waypoints to either side of the line [deg] (default 45).')
    p.add_argument('--alt', type=float, default=453.0, help='Ellipsoidal altitude HAE [m] (default 453).')
    p.add_argument('--out', type=str, default=None,
                   help='Output CSV path (default: print to stdout).')
    args = p.parse_args(argv)

    lat0, lon0 = args.antenna
    pts = generate(lat0, lon0, args.bearing_enu, args.spacing, args.count, args.alt, args.first)

    lines = ['lat,lon,alt,up_down,delta_z_m,hold_s',
             f'# SBL range-sweep line: {args.count} waypoints at '
             f'{" / ".join(f"{d:.0f}" for _, _, d in pts)} m from the antenna, '
             f'bearing {args.bearing_enu:.0f} deg ENU (= {(90 - args.bearing_enu) % 360:.0f} deg compass),',
             f'# anchored at antenna {lat0:.7f},{lon0:.7f}. alt is HAE. up_down/delta_z_m/hold_s unused.']
    side = []
    if args.off_axis_range > 0:
        side = off_axis(lat0, lon0, args.bearing_enu, args.off_axis_range, args.off_axis_angle)
        n = len(pts)
        lines.append(f'# then 2 off-axis waypoints, {args.off_axis_range:.0f} m from the antenna, '
                     f'{args.off_axis_angle:.0f} deg to either side of the line: '
                     f'waypoint {n + 1} at {side[0][2]:.0f} deg ENU, waypoint {n + 2} at {side[1][2]:.0f} deg ENU.')
    for lat, lon, d in pts:
        # no inline comment after the data: the mission loader parses hold_s and fails on it
        lines.append(f'{lat:.7f},{lon:.7f},{args.alt:.1f},0,,')
    for lat, lon, _ in side:
        lines.append(f'{lat:.7f},{lon:.7f},{args.alt:.1f},0,,')
    text = '\n'.join(lines) + '\n'

    if args.out:
        with open(args.out, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        print(f'Wrote {len(pts) + len(side)} waypoints to {args.out}', file=sys.stderr)
    else:
        sys.stdout.write(text)


if __name__ == '__main__':
    main()
