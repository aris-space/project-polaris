# Ice Thickness Measurement Extraction

Extracts, filters, and exports ice thickness measurements from ROS2 MCAP rosbags recorded during grid-survey missions on frozen lakes.

Companion to `ice_estimates/archimedes_touch.py`, which runs the same thickness calculation live on the robot.

---

## Quick start

```bash
# With defaults (Zermatt bags, /ros2_ws/measurements output)
python3 extract_zermatt_measurements.py

# With a config file
python3 extract_zermatt_measurements.py --config config.yaml

# Point at different bags; CLI flags override config
python3 extract_zermatt_measurements.py \
    --config config.yaml \
    --max-depth-dev-m 0.02 \
    /path/to/bag_a /path/to/bag_b
```

---

## Output files

Both CSVs are written to the directory set by `out` (default `/ros2_ws/measurements`).

### `measurements_raw.csv`
One row per valid sensor sample collected while `touching = True`, after all filters.

| Column | Unit | Description |
|---|---|---|
| `timestamp_s` | s | ROS bag timestamp of the sample |
| `bag` | — | Source bag directory name |
| `grid_point_id` | — | Integer ID of the nearest grid point (session-level assignment) |
| `latitude` | deg | From `/waterlinked_ugps/navsatfix` |
| `longitude` | deg | From `/waterlinked_ugps/navsatfix` |
| `gnss_accuracy_m` | m | Horizontal 1-sigma = `sqrt(position_covariance[0])` |
| `pressure_pa` | Pa | Raw pressure from `/pixhawk/scaled_pressure` |
| `surface_pressure_pa` | Pa | Atmospheric reference from `/sensors/pressure/p_surface_pa` |
| `depth_m` | m | AUV depth from `/odometry/filtered/local` (downward positive) |
| `roll_deg` | deg | Roll from `/odometry/filtered/local` |
| `pitch_deg` | deg | Pitch from `/odometry/filtered/local` |
| `ice_thickness_m` | m | Calculated ice thickness (see formula below) |

### `measurements_av.csv`
One row per touch session (after merging and filtering). All sensor columns are the mean of the retained raw samples.

Extra columns beyond the raw fields:

| Column | Unit | Description |
|---|---|---|
| `grid_point_lat / _lon` | deg | Target grid point coordinates |
| `distance_to_target_m` | m | Haversine distance from session mean position to grid point |
| `session_start_s` | s | Timestamp of first valid sample in session |
| `session_end_s` | s | Timestamp of last valid sample in session |
| `duration_s` | s | `session_end_s − session_start_s` |
| `n_samples` | — | Number of retained raw samples in this session |

---

## Ice thickness formula

Based on the Archimedes principle. The pressure sensor is **not** co-located with the ice contact point — it sits 210 mm below and 515 mm forward of the contact point in the AUV body frame. When the AUV pitches, this horizontal offset changes the effective vertical distance to the ice, so the correction is pitch-dependent:

```
gauge_pressure = pressure_pa − surface_pressure_pa
depth_sensor   = gauge_pressure / (ρ_water × g)

omega_corr     = x_offset × sin(pitch) + z_offset × cos(pitch) × cos(roll)

depth_contact  = depth_sensor − omega_corr
T              = depth_contact × ρ_water / ρ_ice
```

| Symbol | Value | Description |
|---|---|---|
| `z_offset` | 0.210 m | **BlueRobotics pressure sensor** is this far **below** the ice contact point (body z). Not the ultrasonic touch-detection sensor. |
| `x_offset` | 0.515 m | **BlueRobotics pressure sensor** is this far **forward** of the ice contact point (body x). |
| `ρ_water` | 1000 kg/m³ | Fresh water density |
| `ρ_ice` | 887 kg/m³ | **Effective ice density** — weighted average of black ice (917 kg/m³) and white ice (870 kg/m³) layers based on Schwarzsee measurements. See the **Ice Density Estimation** page in the summary report for detailed methodology. |
| `g` | 9.81 m/s² | Gravitational acceleration |

At pitch = 0, roll = 0 the formula reduces to `omega_corr = 0.210 m` (the old single-constant form). At pitch = 10° it becomes `0.515 × sin(10°) + 0.210 × cos(10°) ≈ 0.296 m` — an 8.6 cm shift on the offset.

Samples are skipped (not written to any CSV) if:
- `abs(pitch_deg) > 25°`
- `abs(roll_deg) > 10°`
- Calculated thickness ≤ 0

---

## Ice Density Methodology

The ice density value (ρ_ice = 887 kg/m³) is derived from the physical composition of **Schwarzsee** on the survey date (30 April 2026). Freshwater lake ice typically consists of two distinct layers with different densities:

- **Black ice** (congelation ice): Forms by direct freezing of water. Clear, few bubbles. ρ = 917 kg/m³ (pure ice).
- **White ice** (snow-ice): Forms when surface snow becomes saturated with melt-water and re-freezes. Opaque, high bubble content. ρ ≈ 870 kg/m³.

At Schwarzsee, the observed column (30 April 2026) contained:
- **45 cm** of white ice (top layer)
- **25 cm** of black ice (bottom layer)

The effective density is computed as a thickness-weighted average:

$$\rho_{\mathrm{eff}} = \frac{45 \times 870 + 25 \times 917}{70} = \frac{62{,}075}{70} \approx 886.8 \text{ kg m}^{-3}$$

This is rounded to **887 kg/m³** for the Archimedes formula. A photograph of the actual ice core is shown in the **summary.pdf** report.

**Reference:** Leppäranta, M. (2015). *Freezing of Lakes and the Evolution of their Ice Cover*. Springer-Praxis, Berlin.

---

## Topics used

| Topic | Type | Used for |
|---|---|---|
| `/ice_touch_detection/touching` | `std_msgs/Bool` | Session boundary detection |
| `/waterlinked_ugps/navsatfix` | `sensor_msgs/NavSatFix` | GNSS coordinates + accuracy gate |
| `/pixhawk/scaled_pressure` | `sensor_msgs/FluidPressure` | Raw pressure for thickness formula |
| `/sensors/pressure/p_surface_pa` | `std_msgs/Float64` | Surface pressure reference |
| `/odometry/filtered/local` | `nav_msgs/Odometry` | Depth, roll, pitch |
| `/measurement_grid` | `foxglove_msgs/GeoJSON` | 16-point target grid (read once from first bag) |

---

## Processing pipeline

```
Rosbags
  │
  ▼
1. Extract sessions
   Touch sessions are defined by /ice_touch_detection/touching going True → False.
   Within a session, a sample is only recorded when:
     - GNSS fix is valid (status ≥ 0) AND accuracy < gnss_max_accuracy_m (4 m)
     - All sensor values are available
     - Thickness is a positive finite number
   Grid point is assigned per session from the session-averaged lat/lon.
  │
  ▼
2. Filter sessions
   Drop sessions where:
     - duration_s < min_duration_s      (approach bumps, brief touches)
     - mean depth < min_depth_m         (surface-contact calibration touches)
  │
  ▼
3. Merge sessions
   Consecutive sessions at the same grid point with a gap ≤ max_gap_s
   are merged into one. Handles momentary contact loss at a single
   measurement location.
  │
  ▼
4. Depth stability filter  (per sample within each session)
   For each sample, compute the rolling minimum depth over a centred
   ±(depth_window_s / 2) window. Discard the sample if:
     depth_m > rolling_min_depth + max_depth_dev_m
   This removes moments where touching = True but the AUV oscillated
   a few centimetres away from the ice, which would inflate the
   thickness estimate.
  │
  ▼
5. Write CSVs
   measurements_raw.csv  — all retained samples
   measurements_av.csv   — one averaged row per session
```

---

## Configuration

All parameters live in `config.yaml`. Pass it with `--config`; any CLI flag overrides the corresponding config value.

### `config.yaml` reference

```yaml
# Input bags (MCAP rosbag2 directories)
bags:
  - /path/to/bag_1
  - /path/to/bag_2

# Output directory
out: /ros2_ws/measurements

# --- Physics constants ---

# Vertical: BlueRobotics pressure sensor below ice contact point (m)
# (Not the ultrasonic touch-detection sensor)
pressure_to_contact_z_m: 0.210

# Horizontal: BlueRobotics pressure sensor forward of ice contact point (m)
pressure_to_contact_x_m: 0.515

# Fresh water density (kg/m³). Adjust for saline/brackish water.
rho_water: 1000.0

# Ice density (kg/m³). Sea ice: 720–940 kg/m³ depending on salinity.
rho_ice: 917.0

# Gravitational acceleration (m/s²)
g: 9.81

# --- Sample validity filters ---

# Drop samples with |pitch| above this (degrees).
# High pitch = tower not perpendicular to ice = biased thickness.
max_pitch_deg: 25.0

# Drop samples with |roll| above this (degrees).
max_roll_deg: 10.0

# Drop samples where waterlinked GNSS accuracy sqrt(cov[0]) exceeds this (m).
gnss_max_accuracy_m: 4.0

# --- Session filtering ---

# Drop sessions shorter than this (seconds).
# Removes approach bumps and accidental brief contacts.
min_duration_s: 10.0

# Merge consecutive same-point sessions with gap ≤ this (seconds).
max_gap_s: 60.0

# Drop sessions with mean depth below this (metres).
# Filters surface-contact tests where the AUV was not submerged
# under the ice (observed depth ~0.19 m in Zermatt data).
min_depth_m: 0.5

# --- Depth stability filter ---

# Centred rolling window size for the local depth minimum (seconds).
# Wider = tolerates gradual depth drift across the session.
# Narrower = more sensitive to short oscillation bursts.
depth_window_s: 30.0

# Maximum depth above local rolling minimum to keep a sample (metres).
# 0.01 = 1 cm. Raise to 0.02 if the filter is too aggressive.
max_depth_dev_m: 0.01
```

### CLI flags (all optional)

```
--config FILE             Load parameters from a YAML file
--out DIR                 Output directory
# Physics
--pressure-to-contact-z-m M   Vertical: BlueRobotics pressure sensor below ice contact point (m)
--pressure-to-contact-x-m M   Horizontal: BlueRobotics pressure sensor forward of ice contact point (m)
--rho-water K             Water density (kg/m³)
--rho-ice K               Ice density (kg/m³)
--g A                     Gravitational acceleration (m/s²)
# Sample validity
--max-pitch-deg D         Max absolute pitch (degrees)
--max-roll-deg D          Max absolute roll (degrees)
--gnss-max-accuracy-m M   Max waterlinked GNSS accuracy (m)
# Session filtering
--min-duration-s S        Session minimum duration (seconds)
--max-gap-s S             Session merge gap threshold (seconds)
--min-depth-m M           Session minimum average depth (metres)
# Depth stability filter
--depth-window-s S        Rolling window size (seconds)
--max-depth-dev-m M       Depth deviation threshold (metres)
BAG [BAG ...]             Bag paths (positional, override config bags list)
```

---

## Grid point layout

The 16 target points form a 4 × 4 grid published as a GeoJSON on `/measurement_grid` at bag start. IDs are assigned column-first (west → east), row-first (north → south):

```
col →   0    1    2    3
row ↓
 0      0    4    8   12
 1      1    5    9   13
 2      2    6   10   14
 3      3    7   11   15
```

Each session in the averaged CSV includes the target grid point coordinates and the Haversine distance from the session's mean GNSS position to that target.

---

## Dependencies

- `rosbag2_py` (ROS 2 Humble)
- `rclpy`, `rosidl_runtime_py`
- `tf_transformations`
- `numpy`
- `PyYAML`
