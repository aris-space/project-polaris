# mavlink_bridge

ROS2 package that bridges MAVLink (ArduSub/Pixhawk) telemetry to ROS2 topics and forwards ROS2 commands back to the vehicle.

---

## ArduSub Parameters Required for Topic Visualization

The bridge requests MAVLink streams at startup via `REQUEST_DATA_STREAM` / `REQUEST_MESSAGE` commands. However, certain ArduSub parameters must be configured on the flight controller for the corresponding topics to carry valid data.

### PID Tuning — `/pixhawk/PID_ACCZ`

| Parameter | Value | Description |
|---|---|---|
| `GCS_PID_MASK` | `8` (bit 3) | Enables `PID_TUNING` stream for the **AccZ (depth) controller**. Bitmask: bit 0 = Roll, bit 1 = Pitch, bit 2 = Yaw, bit 3 = AccZ. Set to `8` to stream only AccZ; set to `15` to stream all four axes. |

Without this parameter set, `/pixhawk/PID_ACCZ` will never receive data regardless of stream rate requests.

---

### Battery — `/pixhawk/battery`, `/pixhawk/battery_consumed`, `/pixhawk/battery_remaining`

| Parameter | Value | Description |
|---|---|---|
| `BATT_MONITOR` | `4` | Enable analog voltage + current monitoring. `4` = Analog Voltage and Current. Other valid values: `3` = Voltage only. |
| `BATT_VOLT_PIN` | Hardware-dependent | ADC pin for voltage sense (board-specific). |
| `BATT_CURR_PIN` | Hardware-dependent | ADC pin for current sense (board-specific). |
| `BATT_VOLT_MULT` | Hardware-dependent | Voltage divider multiplier for correct voltage reading. |
| `BATT_AMP_PERVLT` | Hardware-dependent | Amps-per-volt conversion for current sensor. |
| `BATT_CAPACITY` | e.g. `14400` | Battery capacity in mAh — used by the `BatteryTracker` node to compute `battery_remaining`. |

---

### Depth — `/pixhawk/DEPTH_ACHIEVED`, `/pixhawk/DEPTH_VELOCITY`, `/pixhawk/DEPTH_TARGET`

These topics come from `VFR_HUD` (msg ID 74) and `NAV_CONTROLLER_OUTPUT` (msg ID 62). The bridge requests both streams at 10 Hz on startup. The depth sensor (e.g. Bar30) must be recognised as a second barometer.

| Parameter | Value | Description |
|---|---|---|
| `BARO_PRIMARY` | `1` | Set to `1` to use the external barometer (depth sensor) as primary. This is what feeds `VFR_HUD.alt` and therefore `DEPTH_ACHIEVED`. |
| `EK3_SRC1_POSZ` | `1` | EKF3 vertical position source. `1` = Baro. Required for valid altitude/depth in `VFR_HUD`. |
| `EK3_ENABLE` | `1` | Enable EKF3 (required for `VFR_HUD` alt output to be meaningful in water). |

---

### Pressure — `/pixhawk/scaled_pressure`

This topic comes from `SCALED_PRESSURE2` (msg ID 137), which is the **second** barometer (i.e. the external depth/pressure sensor).

| Parameter | Value | Description |
|---|---|---|
| `BARO_PRIMARY` | `1` | Use external baro as primary so it registers as BARO2 in the MAVLink stream. |

No additional parameters needed — the bridge explicitly requests `SCALED_PRESSURE2` at 50 Hz in both publisher and receiver nodes.

---

### Attitude — `/pixhawk/attitude_quaternion`

Comes from `ATTITUDE_QUATERNION` (msg ID 31). The bridge requests this at 50 Hz on startup.

| Parameter | Value | Description |
|---|---|---|
| `AHRS_EKF_TYPE` | `3` | Use EKF3 for attitude estimation (recommended for ArduSub). |
| `EK3_ENABLE` | `1` | Enable EKF3. |

---

### External Odometry (sent TO vehicle) — `/odometry/filtered/local`

This topic carries the output of the companion computer's own EKF, which fuses a high-quality IMU, a DVL (Doppler Velocity Log), and the pressure/depth sensor. The bridge forwards it to the Pixhawk as a MAVLink `ODOMETRY` message so ArduSub can use it as its navigation reference. For the vehicle to accept and use this data:

| Parameter | Value | Description |
|---|---|---|
| `EK3_SRC1_POSXY` | `6` | Set XY position source to ExternalNav (6). |
| `EK3_SRC1_VELXY` | `6` | Set XY velocity source to ExternalNav (6). |
| `EK3_SRC1_POSZ` | `6` | Set Z position source to ExternalNav (6) if vision provides depth; otherwise keep `1` (Baro). |
| `VISO_TYPE` | `1` | Enable visual odometry input. `1` = MAVLink. |
| `EK3_VISION_DELAY_MS` | e.g. `80` | Expected vision pipeline latency in ms. Tune to match actual delay. |

---

### Stream Rate Defaults (fallback)

The bridge explicitly requests individual streams on startup, so `SR0_*` parameters are not strictly required. However, if the UDP/serial connection resets without a bridge restart, these defaults control what ArduSub streams automatically:

| Parameter | Recommended Value | Affects |
|---|---|---|
| `SR0_EXTRA1` | `50` | `ATTITUDE`, `ATTITUDE_QUATERNION` |
| `SR0_EXTRA2` | `10` | `VFR_HUD`, `NAV_CONTROLLER_OUTPUT` |
| `SR0_EXTRA3` | `10` | `AHRS`, `HWSTATUS`, `SCALED_PRESSURE` |
| `SR0_RAW_SENS` | `10` | `SCALED_PRESSURE2`, raw IMU |

Replace `SR0_*` with `SR1_*` / `SR2_*` if the bridge connects on a different GCS port.

---

## Published Topics

| Topic | Type | MAVLink Source | Rate |
|---|---|---|---|
| `/pixhawk/heartbeat` | `mavros_msgs/State` | `HEARTBEAT` (0) | 1 Hz |
| `/pixhawk/attitude_quaternion` | `geometry_msgs/Quaternion` | `ATTITUDE_QUATERNION` (31) | 50 Hz |
| `/pixhawk/scaled_pressure` | `sensor_msgs/FluidPressure` | `SCALED_PRESSURE2` (137) | 50 Hz |
| `/pixhawk/battery` | `sensor_msgs/BatteryState` | `BATTERY_STATUS` (147) | ~1 Hz |
| `/pixhawk/battery_consumed` | `std_msgs/Float32` | `BATTERY_STATUS` (147) | ~1 Hz |
| `/pixhawk/battery_remaining` | `std_msgs/Float32` | computed | 1 Hz |
| `/pixhawk/out/manual_control` | `std_msgs/Int16MultiArray` | `MANUAL_CONTROL` (69) | ~1 Hz |
| `/pixhawk/DEPTH_ACHIEVED` | `std_msgs/Float32` | `VFR_HUD` (74) `.alt` | 5 Hz |
| `/pixhawk/DEPTH_VELOCITY` | `std_msgs/Float32` | `VFR_HUD` (74) `.climb` | 5 Hz |
| `/pixhawk/DEPTH_TARGET` | `std_msgs/Float32` | `NAV_CONTROLLER_OUTPUT` (62) | 5 Hz |
| `/pixhawk/PID_ACCZ` | `std_msgs/Float32MultiArray` | `PID_TUNING` (98) axis=4 | 5 Hz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | multiple | 1 Hz |

## Subscribed Topics

| Topic | Type | MAVLink Destination |
|---|---|---|
| `/pixhawk/rc_override` | `mavros_msgs/OverrideRCIn` | `RC_CHANNELS_OVERRIDE` (70) |
| `/pixhawk/manual_control` | `std_msgs/Int16MultiArray` | `MANUAL_CONTROL` (69) |
| `/pixhawk/mode_cmd` | `std_msgs/String` | `SET_MODE` |
| `/pixhawk/arm_cmd` | `std_msgs/Bool` | `COMMAND_LONG` (arm/disarm) |
| `/pixhawk/reboot_cmd` | `std_msgs/Bool` | `COMMAND_LONG` (reboot) |
| `/odometry/filtered/local` | `nav_msgs/Odometry` | `ODOMETRY` (111) — companion EKF output (IMU + DVL + pressure) |
