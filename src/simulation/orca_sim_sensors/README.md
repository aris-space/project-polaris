# orca_sim_sensors — simulated sensors

Synthetic versions of Polaris's navigation sensors for the Gazebo + ArduSub SITL simulation.
Every sensor publishes **the same topics and message types as the vehicle's driver**, so the
vehicle's own conversion nodes and EKFs run unmodified on top of them. All sensors derive their
measurements from Gazebo ground truth (`/odom`: world pose in ENU, body twist in FLU). The water
surface is world z = 0, and world (0, 0, 0) is the mission origin
(`autonomy_bringup_pkg/missions/default_mission_origin.json`, also ArduSub's `--home`).

Simulation-only: built in the `:sim` image and never on the Jetson.

| Sensor | Real hardware | Node | Publishes (= driver topics) | Consumed by (real, unmodified) |
|---|---|---|---|---|
| DVL | Water Linked DVL-A50 | `sim_dvl_node` | `/sensors/dvl/velocity`, `/sensors/dvl/odometry` | `odometry_covariance_node` → `/sensors/dvl/odometry_cov` → both EKFs |
| IMU | Xsens Sirius (VRUAHS) | `sim_imu_node` | `/imu/data` | `imu_yaw_correction` → `/imu/data_corrected` → both EKFs; selector heading gate |
| Depth | Blue Robotics Bar30 (Pixhawk baro 2) | ArduSub SITL (`cfg/sub.parm`) | `/pixhawk/scaled_pressure` via `mavlink_publisher` | `pressure_z_ned_to_pose` → both EKFs; ArduSub depth hold |
| GNSS | u-blox X20P RTK | `sim_gnss_node` | `/fix`, `/ubx_nav_hp_pos_llh` | selector → `/gps/selected` → `gnss_datum_watchdog`, global EKF |
| SBL | Water Linked Underwater GPS G2 | `sim_sbl_node` | `/waterlinked_ugps/locator_position_global`, `/waterlinked_ugps/locator_acoustic_quality` | `to_navsatfix_translator` → selector → global EKF |

Plus `ekf_truth_error`, which compares the EKFs against ground truth (see [below](#ekf-vs-ground-truth)).

## Running

`ros2 launch orca_sim_bringup sim_launch.py` always starts these sensors, their static TFs and
the vehicle's conversion nodes. To have them drive the vehicle's real EKFs instead of ground truth:

```bash
ros2 launch orca_sim_bringup sim_launch.py ground_truth:=False                  # open water
ros2 launch orca_sim_bringup sim_launch.py ground_truth:=False ice_layer:=True  # under ice
ros2 launch orca_sim_bringup sim_launch.py ground_truth:=False sensor_seed:=42  # reproducible noise
```

| Launch arg | Default | Meaning |
|---|---|---|
| `ice_layer` | `False` | GNSS surfacing behaviour (see [GNSS](#gnss--u-blox-x20p-rtk)). |
| `sensor_seed` | `-1` | Seed for every sensor's random generator; `-1` = different each run. |

The sensors alone, without the rest of the sim (they need `/odom`, `/clock` and the Gazebo IMU):
`ros2 launch orca_sim_sensors sim_sensors.launch.py ice_layer:=true seed:=1`.

Every number below is a ROS parameter of the node. Most are read at startup, so to change one,
add it to the node's `parameters` in `launch/sim_sensors.launch.py` (or restart the node with
`--ros-args -p name:=value`).

Sensor mounting matches the vehicle. `sim_sensors.launch.py` publishes `base_link → imu_link`,
`dvl_a50_link`, `gnss_link` and `sbl_link` with the values from the vehicle's driver launch files,
and the nodes measure at those points (DVL lever arm, antenna and locator offsets).

---

## DVL — Water Linked DVL-A50

**Measurement.** The node computes the velocity of the DVL transducer, `v_base + ω × r`, from
ground truth, and expresses it in `dvl_a50_link` (mounted `roll = π, yaw = −π/4`, as on the
vehicle). It publishes at 10 Hz, as a `marine_acoustic_msgs/Dvl` with the driver's beam geometry
plus a twist-only `nav_msgs/Odometry` (`frame_id = child_frame_id = dvl_a50_link`), the same pair
the `dvl_a50` driver emits for a velocity report.

**Noise.** White Gaussian per axis, using the vehicle's measured bottom-lock variances from
`dvl_a50_pkg/measurement_noise_constants.py` (σ ≈ 2.1 / 2.8 / 0.3 mm/s for x / y / z). These are
imported, not copied, so they follow that file.

**Faults.**

| Fault | Default | What happens |
|---|---|---|
| Outliers | `outlier_probability` 0.01 per report, magnitude U(`outlier_min_mps` 0.2, `outlier_max_mps` 1.0) m/s in a random direction | The report **still claims bottom lock**, so `odometry_covariance_node` gives it the normal small covariance. This is the case the EKF must survive. |
| Dropouts | `dropout_rate_per_min` 1.0 (Poisson), length U(`dropout_min_s` 0.5, `dropout_max_s` 4.0) s | `dropout_silent_fraction` 0.3 of them are **silent** (no messages, like a TCP stall); the rest publish `beam_velocities_valid = false` (lost lock), and `odometry_covariance_node` inflates the covariance to 1e6. |
| Range | `min_altitude_m` 0.05, `max_altitude_m` 50 above `seafloor_z` −10 m | Lock is lost outside the A50's range. The seafloor is approximated as flat at the `sand.world` heightmap height. |

Not simulated: dead-reckoning reports (`/sensors/dvl/dead_reckoning`; the EKFs use only the DVL
twist), altitude-dependent ping rate, and loss of lock at high pitch or roll.

## IMU — Xsens Sirius

**Source.** A second Gazebo IMU sensor, `xsens_imu`, in the orca4 model (`model.sdf.in`), at the
vehicle's `imu_link` mount, FLU, 100 Hz (the vehicle's `output_data_rate`). `ros_gz_bridge` brings
it to `/sim/xsens/imu_raw`, and `sim_imu_node` turns it into `/imu/data` (frame `imu_link`). The
existing `imu_sensor` that feeds ArduSub is untouched.

| Field | Model |
|---|---|
| `angular_velocity` | Gazebo gyro + white noise σ = (1.44, 1.48, 1.45)·10⁻³ rad/s (measured on the vehicle, `xsens_mti_node.yaml`) + a constant bias drawn once with σ `gyro_bias_std` 1e-5 rad/s |
| `linear_acceleration` | Gazebo specific force (includes gravity, like the Xsens) + white noise σ = (14.4, 5.0, 7.1)·10⁻³ m/s² (same source) |
| `orientation` | Ground truth (true ENU) + white noise (`orientation_noise_deg` 0.02 / 0.02 / 0.01°) + a heading random walk (`yaw_drift_deg_per_sqrt_h` 1.0) that only grows while the vehicle rotates faster than `ahs_rate_threshold_deg_s` 0.5 °/s, like VRUAHS locking the heading when rotationally still |
| covariances | The values the vehicle's driver config publishes (`orientation_stddev` 3.5e-3 / 4.36e-3 / 1.75e-2 rad, and the gyro/accel stddevs above) |

**Spikes.** "Pretty good but spiky": per sample, with probability `gyro_spike_probability` 0.002
and `accel_spike_probability` 0.002 (≈ one every 5 s each at 100 Hz), a burst of 1 to
`spike_max_samples` 3 samples gets an error of U(0.5, 3) rad/s (gyro) or U(5, 30) m/s² (accel) on
one random axis. Orientation is **never** spiked, because the Xsens filter output is smooth; this
also keeps the selector's 30 s heading-stability gate (std < 0.1°) behaving as on the vehicle. The
node logs a spike count every 30 s.

**Heading reference.** Gazebo's yaw is already true ENU, so the sim runs `imu_yaw_correction` with
`yaw_offset_deg = 0`. To exercise the GNSS yaw calibration, set `initial_yaw_offset_deg` on
`sim_imu_node` (a boot-frame heading offset, like the real VRU).

The orientation comes from `/odom` (50 Hz) and is matched to the latest odometry, so it can lag the
100 Hz rates by up to 20 ms.

Not simulated: `/imu/acceleration` and the other auxiliary Xsens topics, temperature-dependent
bias, and accelerometer bias.

## Bar30 (depth)

On the vehicle the Bar30 is the Pixhawk's **second barometer**; `mavlink_publisher` republishes its
`SCALED_PRESSURE2` as `/pixhawk/scaled_pressure`. The sim keeps that path: ArduSub SITL simulates
both barometers from the Gazebo depth, and the noise is set in `orca_sim_bringup/cfg/sub.parm`:

```
SIM_BARO_RND 0.0087    # baro 1
SIM_BAR2_RND 0.0087    # baro 2 = Bar30 -> /pixhawk/scaled_pressure
```

SITL adds **uniform** noise of ±RND metres of depth per sample (`AP_Baro_SITL`), so
**std = RND / √3**. The default is **5 mm** (measured on the published depth in a test run: 4.9 mm).
Both baros get the same noise, so ArduSub's depth hold and the ROS side see the same sensor class.

| std | RND |
|---|---|
| 2 mm (what `pressure_z_ned_to_pose` assumes, `z_variance` 4e-6) | 0.0035 |
| 5 mm (default) | 0.0087 |
| 1 cm | 0.0173 |

Known quirks of the SITL baro, all handled by the vehicle's own node and none introduced here:

- The absolute pressure is about −18 MPa, because SITL computes underwater pressure from AMSL
  altitude and the mission origin is at 1822 m. `pressure_z_ned_to_pose` calibrates the surface
  pressure over its first 10 s, so the depth it outputs is still correct.
- SITL uses seawater density and the node uses 1000 kg/m³, so depth reads 2.4% too deep
  (measured: `z_pressure = 1.024 · z_true + 0.208`).
- The 0.208 m offset is the surface calibration, done while floating with base_link 0.165 m deep,
  plus the node's `sensor_z_offset_m` 0.039. The same happens on the vehicle if it calibrates
  while floating.
- `/pixhawk/scaled_pressure` has `stamp = 0` and `variance = 0` (from `mavlink_publisher`, on the
  vehicle too).

## GNSS — u-blox X20P RTK

Publishes `/fix` and `/ubx_nav_hp_pos_llh` at 2 Hz (the vehicle's `CFG_RATE_MEAS` 500 ms), with
identical stamps so the selector can pair them. The position is that of the antenna (`gnss_link`,
0.174 m above base_link). Status follows `ublox_nav_sat_fix_hp_node`: a carrier solution (float or
fixed) is `STATUS_GBAS_FIX`; no fix is `STATUS_NO_FIX`.

**Sky view: within 5 cm of the surface, measured at base_link.** The sim model floats at rest with
base_link at z = −0.165 m. That puts the antenna 9 mm above the water, but base_link itself is
never within 5 cm of z = 0. So "at the surface" means **base_link within `surface_depth_m`
(0.05 m) of its floating position `surfaced_base_link_z` (−0.165 m)**. If the model's buoyancy
changes, re-measure the floating z (`ros2 topic echo /odom --field pose.pose.position.z` while
disarmed) and update that parameter.

**Without a fix** (submerged, or acquiring), it publishes `STATUS_NO_FIX` with
`no_fix_h_acc_m` = **10 km** horizontal accuracy (in both the covariance and the UBX `h_acc`), the
`invalid_*` flags set, and the last solution's position, like a powered receiver with a wet
antenna. The selector and the datum watchdog reject it.

**Each time the vehicle surfaces**, one outcome is drawn for that surfacing:

| Mode | Outcome |
|---|---|
| `ice_layer:=False` | NO_FIX for `acquire_fix_s` 1 s → float (h_acc `float_h_acc_m` 1 m) → **RTK fixed** after U(`rtk_delay_min_s` 2, `rtk_delay_max_s` 5) s. Always. |
| `ice_layer:=True` | **RTK** with p = `ice_p_rtk` 0.5, after U(`ice_rtk_delay_min_s` 5, `ice_rtk_delay_max_s` 20) s of float; **float only** with p = `ice_p_float` 0.3, h_acc U(0.6, 2.0) m, never fixes (fails the 0.5 m datum gate); **nothing**, the remaining 0.2, NO_FIX for the whole surfacing |

**Startup: RTK is guaranteed** when the real EKF runs (`ground_truth:=False`). From launch until
`gnss_datum_watchdog` latches `/gnss_datum`, the receiver acts as surfaced in open water,
regardless of depth and `ice_layer`, so the datum always locks on an RTK fix. After that the rules
above apply. If no datum arrives within `startup_rtk_max_s` (300 s), the guarantee ends with a
warning. With `ground_truth:=True` no watchdog runs, so there is no startup hold.

**Errors.** RTK fixed: white noise, σ = `rtk_h_acc_m` 2 cm horizontal (2× vertical). Float: a
first-order Gauss-Markov process (time constant `float_error_tau_s` 30 s) with σ = the reported
h_acc, so it wanders slowly like a real float solution instead of jumping every epoch.

## SBL — Water Linked Underwater GPS G2

Publishes the two topics `uwgpsg2_ros2_interface` publishes, at 2 Hz (its `ros_rate`), with
identical stamps. The real `to_navsatfix_translator` and selector turn them into
`/waterlinked_ugps/navsatfix` and `/gps/selected`. The position is that of the locator
(`sbl_link`, 0.61 m aft of base_link).

**Accuracy scales with range.** With r the slant range from the topside antenna
(`antenna_enu`, default the mission origin at 1 m depth) to the locator:

```
σ = sigma_min_m + sigma_per_m · r = 1.5 m + 2 % · r      → 1.5 m close in, 2.5 m at 50 m, 3.5 m at 100 m
```

The horizontal error is white Gaussian with that σ per axis, and the reported acoustic std
(`locator_acoustic_quality.vector.x`) is σ, so the translator's covariance matches the real error
(measured in a test run: error std 1.73 / 1.79 m against a reported mean σ of 1.77 m). Depth comes
from the locator's own pressure sensor (σ `depth_noise_std_m` 5 cm). With probability
`invalid_probability` 0.05 per update, the G2 reports no valid position.

## EKF vs ground truth

`ekf_truth_error` runs with `ground_truth:=False` and publishes, for `local`
(`/odometry/filtered/local`) and `global` (`/odometry/filtered/global`):

| Topic | Type | |
|---|---|---|
| `/sim/ekf_error/<name>/position` | `geometry_msgs/Vector3Stamped` | estimate − truth, ENU [m] |
| `/sim/ekf_error/<name>/horizontal` | `std_msgs/Float64` | horizontal error magnitude [m] |
| `/sim/ekf_error/<name>/yaw_deg` | `std_msgs/Float64` | heading error [deg] |

It also logs an RMS/max summary every 30 s. Frame alignment:

- **local:** the odom frame starts where the vehicle is when `ekf_local` first publishes, so x/y
  are compared relative to the truth at that moment; z is absolute.
- **global:** the map frame origin is the latched `/gnss_datum`, converted to world ENU. Nothing is
  published for `global` before the datum exists.

Plot the `horizontal` topics in Foxglove next to `/fix` and `/gps/selected` to see GNSS and SBL
updates pull the global EKF in.

## Files

| File | |
|---|---|
| `orca_sim_sensors/sim_dvl_node.py`, `sim_imu_node.py`, `sim_gnss_node.py`, `sim_sbl_node.py` | the sensors |
| `orca_sim_sensors/ekf_truth_error_node.py` | EKF-vs-truth error |
| `orca_sim_sensors/common.py` | flat-earth ENU↔LLH, rotations |
| `launch/sim_sensors.launch.py` | sensors + base_link→sensor static TFs (included by `sim_launch.py`) |
| `STATUS.md` | what was asked, what was built, decisions, test results, open issues |
