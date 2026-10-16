"""Pure (ROS-free) geodesy helpers for the SBL position tool. Unit-testable off-robot."""

import math

_EARTH_R_M = 6371000.0  # mean Earth radius [m]


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance between two WGS84 points [degrees] -> metres."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2.0 * _EARTH_R_M * math.asin(min(1.0, math.sqrt(a)))


def mean_latlon(points):
    """Arithmetic mean of (lat, lon) pairs [degrees]. Fine for metre-scale clusters."""
    if not points:
        raise ValueError("no points to average")
    n = len(points)
    return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)


def max_spread_m(points, center):
    """Max great-circle distance [m] from `center` (lat,lon) to any point in `points`."""
    if not points:
        return 0.0
    return max(haversine_m(center[0], center[1], p[0], p[1]) for p in points)
