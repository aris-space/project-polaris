# Simulated sensors: status

Branch `autonomy/sim-sensors` (from `autonomy/sim` @ `998ef64f`), 2026-10-06.
Sensor documentation: [README.md](README.md).

## 1. The task

As requested:

> Make a sub-branch from `autonomy/sim` → `autonomy/sim/sensors` and create simulations for the
> **DVL, IMU, Bar30 pressure sensor, GNSS and SBL**.
> - **DVL:** fallouts and outlier measurements here and there.
> - **IMU:** pretty good, but spiky data.
> - **GNSS:** only a fix within 5 cm of the surface. Not instant RTK: with some randomness it gets
>   RTK (after waiting a few seconds), but sometimes nothing. Make that a parameter
>   `ice_layer:=true`; with `false` it always gets RTK within a few seconds of surfacing.
> - **Bar30:** a standard deviation.
> - **SBL:** a standard deviation that scales with range; best case around 1.5 m.

Answers to my follow-up questions:

| Question | Answer |
|---|---|
| `autonomy/sim/sensors` can't coexist with the branch `autonomy/sim` (git ref file/directory clash) | use **`autonomy/sim-sensors`** |
| Where the Bar30 noise goes | **SITL barometer noise** in `cfg/sub.parm` (the Bar30 is the Pixhawk's baro 2 on the vehicle) |
| SBL σ model | **σ = 1.5 m + 2 % of slant range** |
| Under-ice outcomes | **RTK (late) / float only / nothing**, drawn once per surfacing |
| GNSS at startup | **always RTK**, so `gnss_datum_watchdog` never fails to set a datum |
| EKF integration | launch the **real `ekf_localization.launch.py`** |
| GNSS without fix | publish **NO_FIX with huge inaccuracy** |
| "Within 5 cm of the surface" measured at | **base_link** |
| Thruster velocity estimator | **ignore** (being removed) |
| EKF-vs-truth error monitor | **yes** |
| IMU source | **Gazebo IMU sensor** in the model (vehicle uses an Xsens Sirius) |
| Git | **commit and push** |
| Docs | a **README** describing each simulated sensor, and this **STATUS.md** |

## 2. What was built

| Requirement | Done | Where |
|---|---|---|
| DVL with dropouts and outliers | ✅ ~1 %/report outliers that still claim lock; ~1/min dropouts of 0.5–4 s (30 % silent, 70 % lock lost); range limits | `sim_dvl_node.py` |
| IMU, good but spiky | ✅ vehicle-measured noise; gyro + accel spikes (~1 per 5 s each, 1–3 samples); orientation not spiked | `sim_imu_node.py`, `xsens_imu` sensor in `model.sdf.in` |
| Bar30 with std | ✅ 5 mm std on both SITL baros (`SIM_BARO_RND` / `SIM_BAR2_RND` 0.0087, std = RND/√3) | `orca_sim_bringup/cfg/sub.parm` |
| GNSS: fix only near the surface | ✅ base_link within 5 cm of its floating position (see decision D3) | `sim_gnss_node.py` |
| GNSS: `ice_layer:=false` → RTK within seconds | ✅ 1 s no fix → float → RTK after 2–5 s | `sim_gnss_node.py` |
| GNSS: `ice_layer:=true` → random | ✅ 50 % RTK after 5–20 s / 30 % float only / 20 % nothing | `sim_gnss_node.py`, launch arg `ice_layer` |
| GNSS: always RTK at startup | ✅ held until `/gnss_datum` is latched | `sim_gnss_node.py` |
| GNSS: no fix = huge inaccuracy | ✅ NO_FIX, h_acc 10 km, `invalid_*` flags | `sim_gnss_node.py` |
| SBL σ scaling with range, best ≈ 1.5 m | ✅ 1.5 m + 2 %·r; reported σ = actual error σ | `sim_sbl_node.py` |
| Real `ekf_localization.launch.py` | ✅ with `ground_truth:=False` | `orca_sim_bringup/launch/sim_launch.py` |
| EKF-vs-truth error | ✅ `/sim/ekf_error/{local,global}/…` + 30 s log summary | `ekf_truth_error_node.py` |
| README / STATUS | ✅ | `README.md`, this file; `src/simulation/README.md` updated |

Launch wiring in `sim_launch.py`:
- The synthetic sensors, the vehicle's base_link→sensor static TFs, and the vehicle's conversion
  nodes (`odometry_covariance_node`, `imu_yaw_correction`, `pressure_z_ned_to_pose`,
  `to_navsatfix_translator`, `selector`) now run in **both** modes.
- `ground_truth:=False` additionally starts `ekf_localization.launch.py` (as `config_pkg`'s
  `navigation.launch.py` does: `gps_fix_topic:=/gps/selected`, plus `use_sim_time:=true`,
  `use_thruster_fallback:=false`) and `ekf_truth_error`.
- New launch args: `ice_layer`, `sensor_seed`.

## 3. Decisions and why

**D1. Inject upstream, so the vehicle's nodes run.** Every sensor publishes the *driver's* raw
topics, and the vehicle's own conversion nodes produce what the EKFs read. This follows the design
rule in `docs/SIM_MERGE_PLAN.md` §1.4: every bypassed node is one the sim no longer tests. The
SBL, for example, publishes the Water Linked interface's `GeoPointStamped` + acoustic-quality
pair, not a `NavSatFix`.

**D2. Noise values come from the vehicle, not datasheets.** The DVL lock variances are imported
from `dvl_a50_pkg/measurement_noise_constants.py`. The IMU gyro/accel stddevs and the reported
orientation covariance come from `xsens_mti_node.yaml` (measured, stationary_02). The GNSS rate
(2 Hz) and SBL rate (2 Hz) are the vehicle's configured rates.

**D3. "Within 5 cm of the surface" is measured from base_link's floating position.** In the sim,
the vehicle floats at rest with base_link at z = −0.165 m. That puts the antenna (+0.174 m) 9 mm
above the water. A literal "base_link within 5 cm of z = 0" could never happen, so the GNSS would
never get a fix after startup. The rule is therefore "base_link within 5 cm of its floating
position", with the floating z as a parameter (`surfaced_base_link_z`). **Check this matches your
intent.** The literal rule is `surfaced_base_link_z:=0.0`.

**D4. Startup RTK ends on `/gnss_datum`, not on a timer.** That is exactly "the watchdog never
fails". A 300 s safety timeout ends it if no datum ever comes. With `ground_truth:=True` there is
no watchdog, so the hold is off (`startup_rtk:=not ground_truth`).

**D5. IMU spikes only on rates and acceleration, never on orientation.** The Xsens orientation is
a filter output and doesn't spike on the vehicle. The selector also gates *all* GNSS and SBL on
yaw std < 0.1° over 30 s, so a spiky orientation would block `/gps/selected` (and the datum)
forever.

**D6. IMU orientation from ground truth, rates from the Gazebo sensor.** Gazebo's IMU orientation
reference frame is ambiguous across versions. Ground truth is unambiguously ENU, which is also what
the vehicle's corrected IMU is calibrated to. Heading drift is modelled as VRUAHS: it only grows
while turning.

**D7. Bar30 at 5 mm std.** You asked for a std but didn't give a number. The Bar30's resolution is
0.2 mbar ≈ 2 mm, which is what `pressure_z_ned_to_pose` assumes; I picked 5 mm so the noise is
visible and the EKF's 2 mm assumption gets exercised. The table in the README gives RND values for
other choices.

**D8. Under-ice float h_acc 0.6–2.0 m.** That always fails the watchdog's 0.5 m RTK gate. The
selector's 4 m @ 95 % gate still lets most of these fixes through to the global EKF, as on the
vehicle.

**D9. Thruster velocity estimator disabled in the sim** (`use_thruster_fallback:=false`). Per your
instruction it is not being fixed, and its node has no `use_sim_time`, so in the sim it would stamp
wall-clock time into the EKF.

**D10. Vehicle launch files can't be included for the converters.** `launch_dvl.launch.py`,
`launch_uwgpsg2.launch.py` etc. also start the hardware drivers, so `sim_launch.py` declares the
same nodes with the same parameters. Comments name the source file. Keep them in sync if those
change. `pressure_z_ned_to_pose.launch.py` has no driver and is included directly. All converters
get `use_sim_time` through a `SetParameter` group.

**D11. `imu_yaw_correction` with `yaw_offset_deg = 0`.** Gazebo's heading is true ENU. The sim
starts the node itself because `ekf_localization.launch.py` (and `config_pkg`'s launches) don't,
even though both EKF configs read `/imu/data_corrected`. Only `ekf_anchored.launch.py` and the
offline replays start it. See finding V4.

**D12. One Gazebo model change only.** A second IMU sensor `xsens_imu` (100 Hz, `imu_link` mount)
was added to `model.sdf.in`, and `model.sdf` regenerated with `generate_model.py` (regenerating the
unchanged template reproduces the old file byte-for-byte). The ArduSub-facing `imu_sensor` is
untouched.

## 4. Test results

Run in the `polaris:sim` image (`docker/build_sim.sh` on this branch; fully cached), headless,
`colcon build --packages-up-to orca_sim_bringup orca_sim_sensors`: clean. `flake8` on the new
package: clean.

**Run A — `ground_truth:=False`, open water, seed 7, 3 min incl. a dive profile** (MANUAL via
MAVLink: dive to 4 m, 20 m transit, yaw, transit, ascend, float):

| Check | Result |
|---|---|
| Datum | RTK at t ≈ 6 s; selector heading gate opened at 30 s (yaw std 0.0099°); datum locked on h_acc 2 cm; world ENU (−0.01, −0.01) m |
| GNSS sky view | NO_FIX when 0.10 m below floating depth; resurfaced → float after 1 s → RTK 3.5 s later |
| `/gps/selected` | 86 GNSS + 161 SBL messages; SBL takes over 2 s after GNSS loss |
| SBL | error std 1.73 / 1.79 m (E/N) vs reported σ mean 1.77 m (1.51–1.90) |
| DVL | 1214 valid / 10 invalid reports, median error 2.9 mm/s, 15 outliers > 0.15 m/s (1.2 %), one silent gap of 2.1 s |
| IMU | 100 Hz, 49 gyro + 39 accel spike samples in ~2 min |
| Bar30 | 4.9 mm std at rest (target 5 mm) |
| Local EKF vs truth | median 0.23 m, max 0.31 m (pure dead reckoning, slowly growing) |
| Global EKF vs truth | 0.02 m at the surface on RTK; ~1.0–1.3 m RMS (max 2.6 m) submerged on SBL; back to 0.25 m after resurfacing |

**Run B — `ground_truth:=False ice_layer:=True`, seed 3, six dive/surface cycles:** the startup
RTK guarantee held despite the ice. The six surfacings drew float only (h_acc 1.09 m), RTK after
15.8 s, RTK after 9.0 s, nothing, RTK after 15.0 s, and RTK after 10.9 s. All three outcomes were
exercised.

**Run C — default `ground_truth:=True`:** `odom_to_tf` drives `/odometry/filtered/local` at 50 Hz,
no EKF or watchdog runs, the sensors run, and the GNSS has no startup hold.

**Not tested:** a full Nav2 mission with `ground_truth:=False`; long (> 5 min) runs; the
`initial_yaw_offset_deg` / GNSS yaw-calibration path; the GUI (`gzclient:=True`).

### Test-environment note (not a code issue)
I ran the container as my host uid, which has no passwd entry inside the image. That breaks
gz-transport's default partition name, so `ros_gz_bridge` sees no Gazebo topics (no `/clock`, and
ArduSub logs "No JSON sensor message received"). Setting `GZ_PARTITION` explicitly fixes it.
Running as root, as the README's `docker run` does, is not affected.

## 5. Findings in the vehicle stack (pre-existing, not changed)

The sim surfaced these. They are not caused by this branch, and I left vehicle code untouched.

| # | Finding | Effect |
|---|---|---|
| V1 | `gnss_datum_watchdog` subscribes to `/odometry/filtered/local_validated` with incompatible QoS (RELIABILITY) | It never receives local odom and logs "No local odom … publishing the datum itself as /odom_origin". The Pixhawk origin is only right if the vehicle hasn't moved since the local EKF started. |
| V2 | With no DVL and no thruster fallback, `ekf_local` diverges to tens of km within a minute (seen while the sim DVL crashed during development) | No velocity source = unbounded drift. Worth guarding on the vehicle. Short dropouts (≤ 4 s) were fine. |
| V3 | The watchdog pairs every `/gps/selected` fix with the *latest* UBX h_acc, whatever the source. The selector forwards SBL whenever no GNSS has been accepted yet | An SBL fix could lock the datum using an RTK h_acc. Not observed in 3 runs (GNSS came first), but it's a race. |
| V4 | `ekf_localization.launch.py` / `config_pkg` don't start `imu_yaw_correction` (only `ekf_anchored.launch.py` and the offline replays do), yet both EKF configs read `/imu/data_corrected` | If the vehicle runs `ekf_localization.launch.py`, check what produces that topic. |
| V5 | `autonomy.launch.py`: `mission_waypoints_publisher` needs `foxglove_msgs`, which isn't in the `:sim` image, so it crashes, and its respawn hits `TypeError: '>' not supported between 'LaunchConfiguration' and 'float'` (respawn_delay passed as a substitution) | Mission waypoint visualisation is missing in the sim. |
| V6 | `pressure_z_ned_to_pose` assumes 1000 kg/m³; SITL uses seawater → depth 2.4 % too deep in the sim. Its surface calibration happens while floating (base_link 0.165 m deep) → 0.21 m offset | Sim-only scale error; the offset happens on the vehicle too if it calibrates while floating. |
| V7 | `mavlink_publisher` publishes `/pixhawk/scaled_pressure` with `stamp = 0` and `variance = 0` | Harmless today; anything time-aligning pressure would break. |

## 6. Open points

- **Confirm D3** (the meaning of the 5 cm rule) and **D7** (the 5 mm Bar30 std).
- The DVL seafloor is flat at z = −10 m; the real heightmap varies, so altitude is approximate.
  A Gazebo ray sensor would fix that if bottom-following matters.
- The SBL error is white. The real G2 error is partly correlated (multipath), so a Gauss-Markov
  term like the GNSS float model could be added.
- The sim converters in `sim_launch.py` duplicate vehicle launch parameters (D10). If those launch
  files get a "no driver" switch, include them instead.
