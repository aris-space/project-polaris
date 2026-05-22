# CHECK_AXIS — Dry-run axis verification for GUIDED velocity control

**Vehicle on the bench, propellers OFF.** This document gives you concrete
servo-PWM predictions for each axis based on the actual custom mixer + the
current `MOT_n_DIRECTION` parameters, so you can pin down which layer is
broken (bridge, AHRS, mixer signs, motor wiring) without touching water.

> **Pre-conditions for every test below**
> - Vehicle on bench, **propellers removed or covered**
> - Bridge running: `ros2 node list | grep mavlink_bridge_receiver`
> - Vehicle armed in GUIDED mode (bridge ignores cmd_vel in any other mode)
> - One terminal recording a bag (template at the end)
> - One terminal with `ros2 topic echo /pixhawk/servo_output_raw`

---

## 0. Frame ground-truth — geometry, mixer, direction params

### 0.1 Vehicle coordinate system (Onshape — what the script uses)

Per [scripts/calculate_attitude_factors.py](scripts/calculate_attitude_factors.py)
header comment, the Onshape design frame is:

| Axis     | Direction        | Meaning                  |
|----------|------------------|--------------------------|
| Roll     | **−y**           | Forward direction        |
| Pitch    | **+x**           | Right direction          |
| Yaw      | **+z**           | Down direction           |

This is a **+90° rotation about z** away from canonical FRD
(`forward = +x, right = +y, down = +z`). Compensated on the autopilot side
by `AHRS_ORIENTATION = 2` (= `ROTATION_YAW_90`).

### 0.2 Motor map — position, role, mixer factors, MOT_DIRECTION

| MOT | PWM ch | Role            | Onshape position (m)          | Mixer (R, P, Y, T, F, L) | `MOT_n_DIRECTION` |
|-----|--------|-----------------|-------------------------------|--------------------------|-------------------|
| 1   | 1      | Forward         | (−1.910, −0.314, +0.312)      | 0, 0, 0, 0, **+1.0**, 0  | **−1**            |
| 2   | 2      | Lateral back    | (−1.583, −0.314, +0.312)      | 0, 0, **+0.775**, 0, 0, **−1.0** | **+1**            |
| 3   | 3      | Vertical left   | (−1.539, −0.166, +0.347)      | **+1.0**, **−0.833**, 0, **+0.6**, 0, 0 | **+1**            |
| 4   | 4      | Vertical right  | (−1.539, −0.462, +0.347)      | **−1.0**, **−0.833**, 0, **+0.6**, 0, 0 | **−1**            |
| 5   | 5      | Vertical front  | (−0.446, −0.314, +0.312)      | 0, **+1.0**, 0, **+1.0**, 0, 0 | **+1**            |
| 6   | 6      | Lateral front   | (−0.351, −0.314, +0.312)      | 0, 0, **−1.0**, 0, 0, **−1.0** | **−1**            |

COM at `(−1.046, −0.314, +0.312)`. Mixer columns are
`(roll, pitch, yaw, throttle, forward, lateral)` — the order in
`add_motor_raw_6dof(...)`.

### 0.3 Servo PWM = factor × axis_command × DIRECTION + 1500

For a single non-zero axis command, each motor's PWM offset from neutral is:

```
PWM_offset[n]  =  factor[n, axis]  ×  axis_output  ×  MOT_n_DIRECTION
PWM_raw[n]     =  1500  +  scale × PWM_offset[n]
```

So you can predict **direction of deflection** at each servo, for each
axis-only command, without running anything.

### 0.4 Predicted servo deflection table (props off, single-axis dry-run)

The bridge applies these sign flips before the PSC sees them:

```python
surge    = -float(msg.linear.x)   # negation in [ros2_receiver.py:875]
heave    = -float(msg.linear.z)   # spec-correct FLU→FRD z-flip
yaw_rate = -float(msg.angular.z)  # negation in [ros2_receiver.py:877]
vy       = 0.0                    # sway never sent
```

After PSC + ATC convert those to body-frame outputs `(R, P, Y, T, F, L)`,
the mixer × DIRECTION produces:

| ROS cmd_vel input        | Sign of `(R,P,Y,T,F,L)` PSC output | MOT_1 | MOT_2 | MOT_3 | MOT_4 | MOT_5 | MOT_6 |
|--------------------------|------------------------------------|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|
| `linear.x = +0.30`       | `F < 0` (negation in bridge)       | **↑**  | —     | —     | —     | —     | —     |
| `linear.x = −0.30`       | `F > 0`                            | **↓**  | —     | —     | —     | —     | —     |
| `linear.z = +0.10` (up)  | `T > 0`                            | —     | —     | **↑**  | **↓**  | **↑**  | —     |
| `linear.z = −0.10` (down)| `T < 0`                            | —     | —     | **↓**  | **↑**  | **↓**  | —     |
| `angular.z = +0.30` (CCW) | `Y < 0` (negation in bridge)      | —     | **↓**  | —     | —     | —     | **↓**  |
| `angular.z = −0.30` (CW) | `Y > 0`                            | —     | **↑**  | —     | —     | —     | **↑**  |
| `linear.y = +0.20` (left, **needs bridge patch**) | `L < 0` | —     | **↑**  | —     | —     | —     | **↓**  |
| `linear.y = −0.20` (right) | `L > 0`                          | —     | **↓**  | —     | —     | —     | **↑**  |

Legend: ↑ PWM > 1500 µs, ↓ PWM < 1500 µs, — neutral 1500 µs (within ±5 µs deadband).

> **Read this table before every test.** If the observed deflection pattern
> doesn't match the prediction *exactly* — including which channels stay neutral —
> you know which layer is broken before you change anything.

---

## 1. Setup verification (≤ 1 minute)

```bash
# Terminal A
ros2 topic echo /pixhawk/servo_output_raw --field data

# Terminal B
ros2 topic pub --once /pixhawk/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.05}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Within 0.3 s of publishing you should see *some* servo data change. If
nothing changes:

- Confirm `pixhawk_mode == "GUIDED"` via `ros2 topic echo /pixhawk/heartbeat`.
- Confirm `armed: true`.
- Confirm `ros2 topic hz /pixhawk/cmd_vel` reports ~20 Hz when streaming.

The bridge silently drops cmd_vel in non-GUIDED mode — that's
[ros2_receiver.py:863-864](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L863-L864).

---

## 2. Surge dry-run (X axis)

### 2.1 Run

```bash
ros2 topic pub -r 20 /pixhawk/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.30, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.30}}" \
  --times 60     # ~3 s of cmd_vel, then auto-stop (no watchdog corner case)
```

Watch all six channels of `servo_output_raw.data`.

### 2.2 Expected — single-channel response

Per the table in §0.4 row 1 (`linear.x = +0.30`):

| Channel | Expected      | If different |
|---------|---------------|---------------------------------|
| `data[0]` MOT_1 | **> 1500** (above neutral) | See diagnosis below |
| `data[1]` MOT_2 | 1500 ± 5      | Spurious yaw or lat factor on MOT_2 (audit mixer row) |
| `data[2]` MOT_3 | 1500 ± 5      | Spurious factor; or PSC is leaking throttle (vertical drift hold) |
| `data[3]` MOT_4 | 1500 ± 5      | Same as MOT_3 |
| `data[4]` MOT_5 | 1500 ± 5      | Same as MOT_3 |
| `data[5]` MOT_6 | 1500 ± 5      | Spurious yaw or lat factor on MOT_6 (audit mixer row) |

### 2.3 Diagnosis

| Observation                                  | Most likely cause                                     | Fix                                                  |
|----------------------------------------------|-------------------------------------------------------|------------------------------------------------------|
| MOT_1 above 1500, others neutral             | **Bridge negation + `MOT_1_DIRECTION=−1` cancel — surge "works" but is doubly inverted.** | Optional cleanup: set `MOT_1_DIRECTION=+1` AND change bridge to `surge = +msg.linear.x`. Functionally identical, removes hidden double-negation. |
| MOT_1 below 1500                             | One of the two negations isn't actually present       | Re-read [ros2_receiver.py:875](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L875) and `param show MOT_1_DIRECTION` |
| MOT_1 stays at 1500                          | Bridge isn't reaching ArduSub, or `forward` factor is 0 in mixer | §1 setup check; then read `AP_Motors6DOF.cpp` MOT_1 row |
| MOT_3 / MOT_4 / MOT_5 also deflect           | PSC `_z` integrator hot from a previous test, or depth-hold leakage | Disarm + arm to reset; or send `linear.z = 0` for 3 s first |
| All six channels move                        | Mixer table has spurious entries, OR PSC is producing all-axis noise | Re-read `AP_Motors6DOF.cpp` `SUB_FRAME_CUSTOM`; check ATC rate output |

### 2.4 The simplest clean-up

`MOT_1` is the **only** motor with a non-zero `forward` factor.
`MOT_1_DIRECTION = −1` *only* affects surge — it is safe to flip without
disturbing yaw, sway, heave, roll, or pitch. The cleanest state is:

```text
param set MOT_1_DIRECTION  1
reboot
```

**and** in [ros2_receiver.py:875](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L875):

```python
surge = +float(msg.linear.x)   # was: -float(msg.linear.x)
```

These two changes together preserve current behaviour (forward thrust on
forward command) while removing the double-negation.

---

## 3. Heave dry-run (Z axis)

### 3.1 Run

```bash
ros2 topic pub -r 20 /pixhawk/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.10}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --times 60
```

### 3.2 Expected — three vertical channels respond, asymmetrically

Per the table in §0.4 row 3 (`linear.z = +0.10`, ROS-FLU = up):

| Channel | Expected   | Why                                |
|---------|------------|------------------------------------|
| MOT_1   | 1500 ± 5   | No throttle factor                 |
| MOT_2   | 1500 ± 5   | No throttle factor                 |
| MOT_3   | **> 1500** | T=+0.6, DIR=+1 → +0.6 × T_out      |
| MOT_4   | **< 1500** | T=+0.6, **DIR=−1** → −0.6 × T_out  |
| MOT_5   | **> 1500** | T=+1.0, DIR=+1 → +1.0 × T_out      |
| MOT_6   | 1500 ± 5   | No throttle factor                 |

The asymmetry between MOT_3 (above 1500) and MOT_4 (below 1500) is
**by design** — `MOT_4_DIRECTION = −1` is compensating a reversed prop
or wiring on that motor, so all three vertical thrusters physically push
the same direction (up) for an "ascend" command.

### 3.3 Diagnosis

| Observation                                  | Cause                                                | Action                                            |
|----------------------------------------------|------------------------------------------------------|---------------------------------------------------|
| All three (3, 4, 5) match the prediction     | Heave wired and signed correctly                     | None (already known good per `z_vel_0_10_correct`) |
| MOT_3 below 1500, MOT_4 above, MOT_5 below   | Whole heave chain inverted upstream                  | The bridge sign is wrong: change `heave = +msg.linear.z`. Do NOT touch DIRECTION params. |
| MOT_3 and MOT_5 both above, MOT_4 also above | `MOT_4_DIRECTION` is incorrectly `+1` (or its prop is the same as 3/5) | Sub will pitch instead of heave; check wiring + `MOT_4_DIRECTION` |
| MOT_3 above, MOT_5 stays neutral             | Mixer entry for MOT_5 throttle column is wrong       | Audit `AP_Motors6DOF.cpp` MOT_5 row               |

> **Note:** Heave is the cleanest sign indicator the vehicle has, because
> z is gravity-aligned and immune to `AHRS_ORIENTATION`. If heave is right,
> the vertical chain is right; if it's wrong, the bug is upstream of AHRS.

---

## 4. Yaw dry-run (rotational Z axis)

### 4.1 Run — CCW (positive angular.z)

```bash
ros2 topic pub -r 20 /pixhawk/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.30}}" \
  --times 60
```

### 4.2 Expected — both lateral motors deflect *the same direction*

Per §0.4 row 5 (`angular.z = +0.30`, CCW from above):

| Channel | Expected   | Why                                                 |
|---------|------------|-----------------------------------------------------|
| MOT_1   | 1500 ± 5   | No yaw factor                                       |
| MOT_2   | **< 1500** | Y=+0.775, DIR=+1, ATC Y_out < 0  → 0.775×Y_out < 0  |
| MOT_3   | 1500 ± 5   | No yaw factor                                       |
| MOT_4   | 1500 ± 5   | No yaw factor                                       |
| MOT_5   | 1500 ± 5   | No yaw factor                                       |
| MOT_6   | **< 1500** | Y=−1.0, DIR=−1, ATC Y_out < 0   → +1.0×Y_out < 0    |

Both yaw motors should deflect **below** neutral. The reason they go *the
same direction in PWM space* is that `MOT_6_DIRECTION = −1` cancels the
sign flip in the mixer's `Y` column, so both motors physically produce
torque the same way (CCW rotation about z) for a single-sign yaw command.

### 4.3 Then immediately run — CW (negative angular.z)

```bash
ros2 topic pub -r 20 /pixhawk/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.30}}" \
  --times 60
```

Both MOT_2 and MOT_6 should now deflect **above** 1500.

### 4.4 Diagnosis

| CCW result (MOT_2, MOT_6)        | CW result (MOT_2, MOT_6)     | Diagnosis                                        | Fix                                                          |
|----------------------------------|------------------------------|--------------------------------------------------|--------------------------------------------------------------|
| Both ↓                           | Both ↑                       | **Sign chain clean** — bridge negation + ATC + mixer + DIR all consistent | Don't touch yaw signs. PID may still need tuning (§4.5) |
| Both ↑                           | Both ↓                       | Whole yaw chain inverted by 1 sign               | Either remove the bridge negation **or** flip the mixer Y column **or** swap MOT_2 and MOT_6 DIRECTIONs as a pair. Pick one — never two. |
| MOT_2 ↓, MOT_6 ↑                 | MOT_2 ↑, MOT_6 ↓             | The two yaw motors are fighting each other       | One of `MOT_2_DIRECTION` / `MOT_6_DIRECTION` is wrong individually. Run §6 to disambiguate. |
| One channel deflects, other neutral | —                          | Mixer Y entry zero for one motor, or motor disabled | Re-check `AP_Motors6DOF.cpp`, `SERVOn_FUNCTION`, `MOTn_PWM_*` |
| Neither deflects, but vehicle would clearly try to yaw | — | ATC produced almost-zero output           | Yaw PID is gutted (`ATC_RAT_YAW_P=0`). See §4.5. |

### 4.5 Yaw PID parameters (must apply BEFORE meaningful yaw test)

If MOT_2 and MOT_6 barely move (offsets < ±10 µs) on a 0.30 rad/s command,
the rate loop is too soft. Paste:

```text
param set ATC_RAT_YAW_P    0.18
param set ATC_RAT_YAW_I    0.018
param set ATC_RAT_YAW_FF   0.10
param set ATC_SLEW_YAW     200
param set ATC_RATE_Y_MAX   90
```

Reboot, re-run §4.1. Offsets should now be a clear ±50–150 µs.

---

## 5. Sway dry-run (Y axis) — bridge patch needed

The bridge currently sends `vy = 0` always
([ros2_receiver.py:901](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L901)).
For this test, **temporarily patch** the bridge:

```python
# In cmd_vel_cb (around line 875):
sway = -float(msg.linear.y)   # FLU left -> FRD right (negate) — same convention as heave
...
# Around line 901, replace the hardcoded 0.0:
self.port.mav.set_position_target_local_ned_send(
    ...,
    surge, sway, heave,    # was: surge, 0.0, heave
    ...
)
```

Rebuild + relaunch the bridge.

### 5.1 Run — sway left (positive linear.y)

```bash
ros2 topic pub -r 20 /pixhawk/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.20, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --times 60
```

### 5.2 Expected

Per §0.4 row 7 (`linear.y = +0.20`, ROS-FLU = left):

| Channel | Expected   |
|---------|------------|
| MOT_2   | **> 1500** |
| MOT_6   | **< 1500** |
| Others  | 1500 ± 5   |

This is **opposite-direction** between MOT_2 and MOT_6 in PWM space — but
again that's by design: physically both motors push the chassis the same
way (left) because `MOT_6_DIRECTION = −1` cancels the mixer's matching
sign on the lateral column.

### 5.3 Why the sway test pins down the yaw-fault location

§4 and §5 share two motors (MOT_2, MOT_6). Cross-referencing the two
results disambiguates the yaw fault:

| Sway result (left command) | Yaw result (CCW command) | Root cause                                          | Fix                                                       |
|---|---|---|---|
| MOT_2 ↑, MOT_6 ↓ (correct) | MOT_2 ↓, MOT_6 ↓ (correct) | Everything fine                                | None                                                      |
| MOT_2 ↓, MOT_6 ↑ (inverted) | MOT_2 ↑, MOT_6 ↑ (inverted) | Both motors wired backwards as a pair; mixer fine | `MOT_2_DIRECTION 1 → −1`, `MOT_6_DIRECTION −1 → +1`, reboot. **No reflash.** |
| MOT_2 ↑, MOT_6 ↓ (correct) | MOT_2 ↑, MOT_6 ↑ (inverted) | Mixer **Y column** signs wrong; lat column OK   | Reflash with the Y column flipped (see [DEBUG_GUIDED.md](DEBUG_GUIDED.md) §"Firmware fix"). Then drop the bridge yaw negation. |
| MOT_2 ↓, MOT_6 ↑ (inverted) | MOT_2 ↓, MOT_6 ↓ (correct) | Mixer **L column** signs wrong; yaw column OK   | Reflash with L column flipped on MOT_2 and MOT_6.         |

Without §5 you can't tell whether you're in row 2 (cheap fix, no reflash)
or row 3 (firmware reflash). **Always run sway before deciding the yaw fix.**

---

## 6. Pinpointing a per-motor wiring fault — `motortest`

If §4 + §5 disagree (one motor each direction) you have a single-motor
wiring or prop-rotation issue, not a mixer issue. Confirm with MAVProxy's
`motortest`:

```text
motortest 2 1 1700 2     # MOT_2, +200 µs above neutral, 2 s
motortest 2 1 1300 2     # MOT_2, −200 µs below neutral, 2 s
motortest 6 1 1700 2     # MOT_6
motortest 6 1 1300 2
```

For each command, observe the physical thrust direction (mark the prop or
feel the airflow). The expected pattern (with default `MOT_n_DIRECTION = +1`):

- PWM > 1500 → prop pushes water in its "default" direction (per the
  hardware spec sheet — typically the side the prop label faces away from).
- PWM < 1500 → opposite direction.

If the physical response is reversed for **only one** motor, set that
motor's `MOT_n_DIRECTION` to `−1` (or back to `+1`) and reboot.

---

## 7. AHRS_ORIENTATION — what velocity commands can and can't tell you

**Short answer: GUIDED-mode velocity commands cannot reliably detect a
wrong `AHRS_ORIENTATION`.** Here's why, then how to actually check.

### 7.1 Why velocity tests don't catch AHRS errors

The PSC velocity loop runs in **NED** internally:

1. cmd_vel arrives in BODY_FRD.
2. ArduSub rotates BODY_FRD → NED using AHRS yaw.
3. PSC computes NED error vs measured NED velocity, outputs NED accel.
4. ArduSub rotates NED → BODY using **the same AHRS yaw**.
5. Mixer takes BODY-frame `(forward, lateral, throttle)`.

Steps 2 and 4 use the same rotation, so any AHRS yaw bias **cancels out**
in steady state. A 180° error in `AHRS_ORIENTATION` does *not* flip the
sign of MOT_1 on a forward command.

The yaw rate loop is body-relative throughout — yaw rate setpoint feeds
directly into ATC rate without a NED detour. Yaw tests *also* don't
detect AHRS errors.

### 7.2 What does detect a wrong AHRS_ORIENTATION

- **Static heading check** (no commands needed).
  Park sub physically pointing north (use a building wall / known
  bearing). Read `ATTITUDE.yaw` from MissionPlanner. It should be ≈ 0
  (radians, NED, CW from north).
  - Off by π → flip to `AHRS_ORIENTATION = 6` (YAW_270) and re-check.
  - Off by ±π/2 → try 0 or 4.
  - Within 0.1 rad → AHRS is fine.
- **Static tilt check.**
  Tilt the sub nose-up by a known angle (a phone level on the chassis).
  Read `ATTITUDE.pitch`. Should match the physical pitch in sign and
  magnitude. If pitch swap with roll, the orientation has a non-z-axis
  rotation component (none of YAW_0/90/180/270 — try one of the
  `*_PITCH_*` enums).
- **EKF innovation messages.**
  If `EK3_SRC1_YAW = 6` (external nav, e.g. DVL or VISO) and the IMU's
  body frame disagrees with the external nav body frame by more than
  ~5°, the FC will print messages like
  `"EKF3 IMU0 yaw aligned to external nav"` once at startup, then
  emit large `XKF3.IYaw` innovations during flight. Pull the
  dataflash `.bin` and check `XKF3.IYaw` magnitude — > 0.2 rad
  sustained means AHRS and external nav disagree.

### 7.3 Indirect symptoms that *do* show up under velocity commands

- **Slow position drift in long-duration GUIDED hold.** AHRS error
  doesn't kill the velocity loop, but it does slowly poison the
  position-hold integrator. After 30 s of `vx=0`, the sub has drifted
  cm-scale because its NED → BODY projection is consistently biased.
  Detectable in water with a >30 s hold; not detectable on the bench
  in a 3 s test.
- **External nav rejection.** If the EKF rejects the external nav
  velocity because of yaw mismatch, the PSC falls back to dead-reckoning
  from the IMU. Velocity tracking gets noisy. Visible as larger-than-
  expected error in the bag's `/odometry/filtered/local.twist.linear.x`
  vs cmd, even on short tests.

### 7.4 Conclusion — which test goes in which bucket

| Symptom you're chasing               | Use this test                              |
|---|---|
| Surge / sway / yaw goes wrong direction | §2 / §4 / §5 (servo PWM table predictions)  |
| Need to confirm AHRS_ORIENTATION       | §7.2 static heading + tilt checks           |
| Velocity tracking degrades over seconds | Look at EKF innovations in dataflash log    |
| Position drift in hold                  | In-water test, ≥30 s hold, plot odometry    |

**Velocity dry-runs cannot distinguish "AHRS_ORIENTATION wrong" from
"mixer/wiring wrong" by themselves.** The two faults manifest at the
servo level the same way (or not at all). Always run §7.2 *before*
trusting any sign-flip recommendation from §2 / §4 / §5.

---

## 8. Lag / backward drift after stop

After any axis test, kill the publisher with `Ctrl+C` and time how
long until `servo_output_raw` returns to 1500 ± 5 µs.

| Observation                              | Cause                                  | Fix                                            |
|------------------------------------------|----------------------------------------|------------------------------------------------|
| All channels return to 1500 within 0.5 s | Normal                                 | None                                           |
| Slow decay (1–3 s) on the active motor   | PSC velocity integrator wind-up        | `param set PSC_VELXY_IMAX 50`, `PSC_VELXY_D 0.8`, `PSC_VELXY_I 0.2` |
| Active motor reverses past neutral, then oscillates | Same + `PSC_VELXY_D=0` (no damping) | Same param set. Reboot. |
| Channel never returns                    | Watchdog isn't firing                  | Re-read [ros2_receiver.py:911](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L911); check `Comms.SUB_QOS_DEPTH` |

```text
param set PSC_VELXY_D    0.8
param set PSC_VELXY_IMAX 50
param set PSC_VELXY_I    0.2
param set MOT_SAFE_DISARM 1
```

---

## 9. Recommended dry-run order

Run from the bench, props off, GUIDED + ARMED, in order. Stop and
diagnose at the first deviation from the §0.4 prediction table.

1. §1 setup verification — confirm the bridge is heard at all.
2. §7.2 static heading + tilt — rule out AHRS_ORIENTATION before
   interpreting any sign result.
3. §3 heave — gravity-locked, fastest sanity check.
4. §2 surge — only one motor responds; cleanest single-axis test.
5. §4 yaw (CCW + CW back-to-back).
6. §5 sway (after applying the bridge patch).
7. §6 if §4 and §5 disagree on individual motor signs.
8. §8 stop / lag check after every axis above.

After each test, **annotate the recorded bag** with the section number
and the observed pattern so the post-mortem cross-references cleanly.

---

## 10. Recording template

```bash
source /opt/ros/humble/setup.bash
source /home/polaris_pz/project-polaris/install/setup.bash
ros2 bag record -s mcap \
  -o ~/Downloads/Rosbags/$(date +%Y%m%d_%H%M%S)_dryrun \
  /pixhawk/cmd_vel \
  /pixhawk/servo_output_raw \
  /odometry/filtered/local \
  /imu/angular_velocity \
  /filter/euler \
  /pixhawk/attitude_quaternion \
  /pixhawk/heartbeat
```

Useful tag names: `dryrun_surge_+030`, `dryrun_heave_+010`,
`dryrun_yaw_ccw_030`, `dryrun_yaw_cw_030`, `dryrun_sway_left_020`.
Cross-reference each result against the predictions in §0.4.

---

## 11. Cross-reference

- Full diagnostic playbook + parameter strategy: [DEBUG_GUIDED.md](DEBUG_GUIDED.md)
- Bridge code: [src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py)
- Mixer source (custom frame): [project-polaris-ardusub/libraries/AP_Motors/AP_Motors6DOF.cpp](../project-polaris-ardusub/libraries/AP_Motors/AP_Motors6DOF.cpp) — `SUB_FRAME_CUSTOM` block
- Frame factor calculator: [scripts/calculate_attitude_factors.py](scripts/calculate_attitude_factors.py) (or wherever you keep it)
