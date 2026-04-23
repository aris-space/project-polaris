import numpy as np
import pyproj
import csv
import math

# --- Parameters ---
ENTRY_LAT    = 45.97073358962203   # entry hole latitude
ENTRY_LON    = 7.7136995051485115  # entry hole longitude
HEADING_DEG  = 0.0                 # drive-out direction from entry hole (degrees clockwise from north)
DRIVE_OUT_M  = 50.0                # distance from entry hole to the middle of the lower grid edge
ROTATION_DEG = 0.0                 # extra rotation of the grid around the anchor (0 = aligned with heading)
GRID_SIZE_M  = 100.0               # side length of the square grid
SPACING_M    = 10.0                # distance between grid points
OUTPUT_CSV   = "grid_points.csv"

# --- Projection ---
utm_zone = int((ENTRY_LON + 180) / 6) + 1
proj = pyproj.Proj(proj="utm", zone=utm_zone, ellps="WGS84")

# --- Drive from entry hole to grid anchor (middle of lower edge) ---
h_drive = math.radians(HEADING_DEG)
drive   = np.array([math.sin(h_drive), math.cos(h_drive)])

entry_utm  = np.array(proj(ENTRY_LON, ENTRY_LAT))
anchor_utm = entry_utm + DRIVE_OUT_M * drive

# --- Grid axes rotated by heading + optional extra rotation ---
h_grid  = math.radians(HEADING_DEG + ROTATION_DEG)
forward = np.array([math.sin(h_grid),  math.cos(h_grid)])
right   = np.array([math.cos(h_grid), -math.sin(h_grid)])

# --- Lower-left corner of the grid ---
lower_left = anchor_utm - (GRID_SIZE_M / 2.0) * right

# --- Generate grid points ---
steps = np.arange(0, GRID_SIZE_M + SPACING_M / 2, SPACING_M)

with open(OUTPUT_CSV, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["id", "lat", "lon"])
    i = 0
    for dx in steps:
        for dy in steps:
            pt = lower_left + dx * right + dy * forward
            lon, lat = proj(pt[0], pt[1], inverse=True)
            writer.writerow([i, round(lat, 8), round(lon, 8)])
            i += 1

print(f"Wrote {i} points ({len(steps)}x{len(steps)}) to {OUTPUT_CSV}")
