# ice_touch_detection_pkg

ROS 2 package that detects contact between the AUV's tower and the ice ceiling.  
Publishes a single `std_msgs/Bool` topic that is `true` while the tower is touching ice.

---

## Hardware geometry

```
                  ICE SURFACE
─────────────────────────────────────────────
         ▲  Tower contact point (top of tower)
         │
       148 mm  ← tower_height_m
         │
         │          ← pressure sensor is 136 mm
         │             below tower top (pressure_sensor_offset_m)
    ─────┼─────  AUV hull (top)
    US   │
  sensor │  ← ultrasonic sensor fires upward along body-Z
         │
    650 mm (tower_horizontal_offset_m)
    Tower is this far *behind* the ultrasonic sensor (−body-X)
```

| Dimension | Value | Parameter |
|---|---|---|
| Tower top above ultrasonic | 148 mm | `tower_height_m` |
| Tower behind ultrasonic | 650 mm | `tower_horizontal_offset_m` |
| Pressure sensor below tower top | 136 mm | `pressure_sensor_offset_m` |

---

## Subscribed topics

| Topic | Type | Use |
|---|---|---|
| `/top/ultrasonic/distance` | `std_msgs/Float32` | Distance to ice ceiling (m) |
| `/imu/data` | `sensor_msgs/Imu` | Orientation quaternion for geometric correction |
| `/imu/acceleration` | `geometry_msgs/Vector3Stamped` | Acceleration magnitude for impact detection |
| `/sensors/keller26x/gauge_pressure` | `sensor_msgs/FluidPressure` | Gauge pressure (Pa) for depth estimation |

All topic names are overridable via launch arguments or ROS 2 parameters.

## Published topics

| Topic | Type | Description |
|---|---|---|
| `/ice_touch_detection/touching` | `std_msgs/Bool` | `true` = tower is in contact with ice |

---

## Detection strategy

Three signals are fused; the first healthy path that fires wins.

### 1. Geometric check (primary)

The ultrasonic sensor measures distance `d` along body-Z (upward).  
The tower sits at body position `(−0.650, 0, 0.148)` m relative to the sensor.

Both are projected onto world-Z using the third row of the IMU rotation matrix:

```
R₂₀ = 2·(qx·qz − qy·qw)
R₂₁ = 2·(qy·qz + qx·qw)
R₂₂ = 1 − 2·(qx² + qy²)

tower_world_z    = R₂₀·(−0.650) + R₂₂·0.148
ultrasonic_world_z = R₂₂ · d

touching  ⟺  ultrasonic_world_z ≤ tower_world_z + touch_tolerance_m
```

At level flight this simplifies to `d ≤ 0.148 m`.  
If roll or pitch exceeds `max_valid_angle_deg` (default 45°) the check is skipped.

### 2. Pressure fallback (ultrasonic fault)

If ≥ 80 % of the last 15 ultrasonic readings are exactly `0.0` the sensor is declared faulty.  
The node then falls back to the pressure sensor:

```
touching  ⟺  gauge_pressure < pressure_fallback_pa   (default 1800 Pa ≈ 0.18 m depth)
```

At the moment of contact the pressure sensor (136 mm below the tower top) sits at  
≈ 0.136 m depth → ≈ 1334 Pa gauge (freshwater). The 1800 Pa default adds a ~34 % margin.

### 3. IMU impact detection (enhancement)

A spike in total acceleration magnitude that exceeds the rolling window mean by  
`imu_collision_threshold_ms2` (default 3.5 m/s²) *while the pressure sensor confirms  
near-surface* (`< pressure_near_surface_pa`, default 2000 Pa) triggers a touch  
independently of the two paths above.

### Debounce

The raw signal must be `true` for `confirm_count` (default 3) consecutive ticks to  
latch `touching = true`, and `false` for `clear_count` (default 5) ticks to release it.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `tower_height_m` | `0.148` | Tower top above ultrasonic sensor (m) |
| `tower_horizontal_offset_m` | `0.650` | Tower behind ultrasonic sensor (m) |
| `pressure_sensor_offset_m` | `0.136` | Pressure sensor below tower top (m) |
| `touch_tolerance_m` | `0.02` | Geometric margin added to tower height (m) |
| `max_valid_angle_deg` | `45.0` | Max roll/pitch for geometric check to run (°) |
| `ultrasonic_zero_window` | `15` | Rolling window size for fault detection |
| `ultrasonic_zero_ratio_threshold` | `0.8` | Fraction of zeros that declare sensor faulty |
| `water_density_kgm3` | `1000.0` | Water density — 1000 freshwater, 1025 seawater |
| `pressure_near_surface_pa` | `2000.0` | Near-surface gate for IMU collision check (Pa) |
| `pressure_fallback_pa` | `1800.0` | Touch threshold when ultrasonic is invalid (Pa) |
| `use_imu_collision` | `true` | Enable/disable IMU impact detection |
| `imu_collision_window` | `30` | Samples in the acceleration rolling window |
| `imu_collision_min_samples` | `10` | Minimum samples before collision check fires |
| `imu_collision_threshold_ms2` | `3.5` | Spike above baseline to infer impact (m/s²) |
| `confirm_count` | `3` | Ticks to confirm a touch |
| `clear_count` | `5` | Ticks to clear a touch |
| `publish_rate_hz` | `10.0` | Detection and publish rate (Hz) |
| `ultrasonic_topic` | `/top/ultrasonic/distance` | |
| `imu_topic` | `/imu/data` | |
| `acceleration_topic` | `/imu/acceleration` | |
| `pressure_topic` | `/sensors/keller26x/gauge_pressure` | |
| `output_topic` | `/ice_touch_detection/touching` | |

---

## Usage

```bash
# Default launch (loads config/ice_touch_detection.yaml automatically)
ros2 launch ice_touch_detection_pkg ice_touch_detection.launch.py

# Point at a custom config file
ros2 launch ice_touch_detection_pkg ice_touch_detection.launch.py \
    params_file:=/path/to/my_params.yaml

# Run node directly with a parameter override
ros2 run ice_touch_detection_pkg ice_touch_detection_node \
    --ros-args -p water_density_kgm3:=1025.0 -p touch_tolerance_m:=0.03
```

---

## Build

```bash
cd <workspace_root>
colcon build --packages-select ice_touch_detection_pkg
source install/setup.bash
```
