# ice_estimates — Ice Thickness Node

ROS 2 node that estimates the thickness of ice above the submarine when it is pressing against the ice from below, using water pressure and Archimedes' principle.

## Node: `ice_estimation`

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `/sensors/keller26x/gauge_pressure` | `sensor_msgs/FluidPressure` | Gauge water pressure from the Keller 26X sensor (Pa) |
| `/odometry/filtered/local` | `nav_msgs/Odometry` | Filtered odometry — used for depth and roll/pitch |
| `/gps/filtered/global` | `sensor_msgs/NavSatFix` | Filtered GPS position for geo-tagging measurements |
| `/ice_touch_detection/touching` | `std_msgs/Bool` | Whether the submarine is currently in contact with the ice |

### Publishers

| Topic | Type | Description |
|---|---|---|
| `/ice_thickness` | `std_msgs/Float64MultiArray` | `[timestamp_s, thickness_m, pressure_pa, latitude, longitude, depth_m]` |

## Ice Thickness Calculation

The node uses gauge pressure to compute the depth of the ice underside, then derives total ice thickness via Archimedes' principle:

```
depth_to_ice_bottom = (pressure_gauge / (rho_water * g)) - 0.05

T = (rho_water / rho_ice) * (depth_to_ice_bottom - omega)
```

| Parameter | Value | Description |
|---|---|---|
| `rho_water` | 1000.0 kg/m³ | Water density |
| `rho_ice` | 917.0 kg/m³ | Ice density |
| `g` | 9.81 m/s² | Gravitational acceleration |
| `omega` | 0.138 m | Distance from the pressure sensor to the ice contact point |

Measurements are discarded if:
- The submarine is not touching the ice
- Pitch exceeds ±25° or roll exceeds ±10°
- The computed thickness is negative

## Data Logging

On startup the node creates a timestamped folder under `measurements/` at the repo root:

```
measurements/
└── YYYYMMDD_HHMMSS/
    ├── measurements_raw_YYYYMMDD_HHMMSS.csv   # every valid measurement point while touching
    └── measurements_av_YYYYMMDD_HHMMSS.csv    # one averaged row per touch session
```

Both files share the same columns:

| Column | Unit | Description |
|---|---|---|
| `timestamp_s` | s | ROS clock time |
| `latitude` | deg | GPS latitude |
| `longitude` | deg | GPS longitude |
| `pressure_pa` | Pa | Gauge water pressure |
| `depth_m` | m | Depth from odometry |
| `roll_deg` | deg | Roll angle |
| `pitch_deg` | deg | Pitch angle |
| `ice_thickness_m` | m | Estimated ice thickness |

The averaged file writes one row each time the submarine loses contact with the ice, containing the column-wise mean of all valid measurements collected during that touch session.
