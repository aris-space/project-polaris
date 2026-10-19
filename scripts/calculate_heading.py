"""Compass bearing from waypoint 0 to waypoint 1.

Accepts lat/lon in either convention:
  - decimal degrees  (e.g. /fix.latitude = 47.328908)
  - UBX deg * 1e7    (e.g. /ubx_nav_hp_pos_llh.lat = 473289080)

Set UBX_E7_INPUT = True when copy-pasting ints straight from
`ros2 topic echo /ubx_nav_hp_pos_llh`. Leave False for dotted decimal degrees.
"""

import math

UBX_E7_INPUT = False


def to_degrees(lat, lon):
    if UBX_E7_INPUT:
        return lat * 1e-7, lon * 1e-7
    return float(lat), float(lon)


def get_bearing(lat1, lon1, lat2, lon2):
    """Initial bearing from (lat1, lon1) to (lat2, lon2), compass degrees 0-360."""
    phi1, lambda1 = math.radians(lat1), math.radians(lon1)
    phi2, lambda2 = math.radians(lat2), math.radians(lon2)
    delta_lambda = lambda2 - lambda1

    x = math.sin(delta_lambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)

    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


# --- Input Data ---
# Waypoint 0 — set from ros2 topic echo /fix  OR  /ubx_nav_hp_pos_llh (flip UBX_E7_INPUT above).
# /fix example:                 lat0, lon0 = 47.328908, 8.572728
# /ubx_nav_hp_pos_llh example:  lat0, lon0 = 473289080, 85727280
lat0, lon0 = 47.404937, 8.631709

# Waypoint 1
lat1, lon1 = 47.404968, 8.631630


lat0_deg, lon0_deg = to_degrees(lat0, lon0)
lat1_deg, lon1_deg = to_degrees(lat1, lon1)

heading = get_bearing(lat0_deg, lon0_deg, lat1_deg, lon1_deg)

print(f"Waypoint 0: {lat0_deg:.7f}, {lon0_deg:.7f}")
print(f"Waypoint 1: {lat1_deg:.7f}, {lon1_deg:.7f}")
print("-" * 30)
print(f"Required Compass Heading: {heading:.2f}°")