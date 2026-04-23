import numpy as np
import pyproj
import csv
import math

ORIGIN_LAT, ORIGIN_LON = (
    45.97073358962203,
    7.7136995051485115,
)  # bottom-left corner of field
HEADING_DEG = 0.0  # 0 = grid aligned with north, rotate if your field is angled
GRID_SIZE_M = 100.0
SPACING_M = 10.0
OUTPUT_CSV = "grid_points.csv"

utm_zone = int((ORIGIN_LON + 180) / 6) + 1
proj = pyproj.Proj(proj="utm", zone=utm_zone, ellps="WGS84")

origin = np.array(proj(ORIGIN_LON, ORIGIN_LAT))

# Unit vectors for grid axes, rotated by heading
h = math.radians(HEADING_DEG)
x_hat = np.array([math.cos(h), math.sin(h)])
y_hat = np.array([-math.sin(h), math.cos(h)])

steps = np.arange(0, GRID_SIZE_M + SPACING_M / 2, SPACING_M)  # 0,10,20,...,100

with open(OUTPUT_CSV, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["id", "lat", "lon"])
    i = 0
    for ix, dx in enumerate(steps):
        for iy, dy in enumerate(steps):
            pt = origin + dx * x_hat + dy * y_hat
            lon, lat = proj(pt[0], pt[1], inverse=True)
            writer.writerow([i, round(lat, 8), round(lon, 8)])
            i += 1

print(f"Wrote {i} points ({len(steps)}x{len(steps)}) to {OUTPUT_CSV}")
