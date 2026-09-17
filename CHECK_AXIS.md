# CHECK_AXIS — Dry-run axis verification for GUIDED velocity control

**Vehicle on the bench, propellers OFF.** This document gives you concrete
servo-PWM predictions for each axis based on the actual custom mixer + the
current `MOT_n_DIRECTION` parameters, so you can pin down which layer is
broken (bridge, AHRS, mixer signs, motor wiring) without touching water.

---

## CRITICAL CONTEXT — MANUAL mode works

`/pixhawk/manual_control` (the manual_control_node path → MAVLink
`MANUAL_CONTROL` → ArduSub MANUAL/STABILIZE/ALT_HOLD) **produces correct
physical motion on this vehicle**: forward command → forward thrust,
yaw-left command → CCW rotation, etc.

That single fact rules out a very large set of hypotheses. Anything in
the chain that MANUAL also exercises must be correct, because if it
weren't, MANUAL would also be wrong:

| Layer that MANUAL exercises          | Status (proven by MANUAL working)                  |
|--------------------------------------|----------------------------------------------------|
| ESC phase wiring (per motor)         | **Correct** for the foils + DIRECTIONs as fitted  |
| Propeller R/L foil handedness        | **Correct** (consistent with wiring)               |
| Motor mounting orientation           | **Correct**                                        |
| `MOT_n_DIRECTION` parameters         | **Correct** for the hardware as built              |
| Custom 6DOF mixer (`AP_Motors6DOF.cpp`) | **Correct** — body-frame `forward, lat, yaw, throttle, roll, pitch` inputs map to physical thrust as designed |
| AHRS chip orientation (gross)        | Believable — large errors would show up in MANUAL via attitude-stabilized modes |

What MANUAL does **not** exercise — and therefore does not validate:

- ArduSub's GUIDED-mode handling of `SET_POSITION_TARGET_LOCAL_NED` in
  `MAV_FRAME_BODY_FRD`. This path goes through the **PSC velocity loop**
  (BODY → NED → controller → NED → BODY), not directly into the mixer.
- Sign conventions ArduSub's GUIDED handler uses for `vx`, `vy`, `vz`,
  and `yaw_rate` in BODY_FRD. The current bridge has empirically
  determined that the x axis needs a negation that the spec does not
  predict — see [ros2_receiver.py:884](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L884) comment.

### What this changes about the diagnosis path

Most of the diagnosis tables in this document list "reflash the mixer"
or "flip `MOT_n_DIRECTION`" as candidate fixes. **With MANUAL working,
those fixes are almost always wrong** — they would break MANUAL while
trying to fix GUIDED. The remaining valid fix locations for a GUIDED-only
fault are, in order of cheapness:

1. **Bridge sign flips** in `cmd_vel_cb` — the right place to absorb
   ArduSub-firmware-quirk inversions, because the bridge only affects
   GUIDED-mode commands.
2. **Bridge `type_mask` construction** — affects only GUIDED.
3. **PSC parameters** (`PSC_VELXY_*`, `PSC_VELZ_*`, etc.) — affects only
   GUIDED.
4. **ATC rate-loop parameters** (`ATC_RAT_YAW_*`, etc.) — these affect
   GUIDED yaw rate tracking but also ACRO and angle-loop modes.
5. **GUIDED-mode firmware code** (`mode_guided.cpp`,
   `GCS_MAVLink_Sub.cpp`) — only as a last resort, and only after
   confirming with ArduSub upstream that the BODY_FRD interpretation
   is genuinely buggy.

**Reflashing `AP_Motors6DOF.cpp` or flipping `MOT_n_DIRECTION` to fix
a GUIDED-only fault is a category error.** Flag it in code review.

### A 60-second sanity-check before any GUIDED dry-run

Before each session, do one MANUAL surge + yaw to confirm MANUAL is
still good. If MANUAL is broken on a given day, GUIDED diagnosis is
meaningless until you fix MANUAL first (and the §6 / §B.2 hardware
layers come back into play). See §0.6 below for the procedure.

> **Pre-conditions for every test below**
> - Vehicle on bench, **propellers removed or covered**
> - Bridge running: `ros2 node list | grep mavlink_bridge_receiver`
> - Vehicle armed in GUIDED mode (bridge ignores cmd_vel in any other mode)
> - One terminal recording a bag (template at the end)
> - One terminal with `ros2 topic echo /pixhawk/servo_output_raw`

---

## 0. Frame ground-truth — geometry, mixer, direction params

### 0.1 Vehicle coordinate system — Onshape ENU world, FLU body

Onshape uses a right-handed **ENU world frame** (`+x = East, +y = North, +z = Up`).
The vehicle is modelled inside that world with body axes aligned **FLU**, which
matches ROS REP-103 exactly:

| Body axis     | Onshape direction | Cross-check (MOT_3 = "vertical LEFT thruster") |
|---------------|-------------------|------------------------------------------------|
| **Forward**   | **+x**            | MOT_5 (vertical *front*) is at relative `+x = +0.6` from COM ✓ |
| **Left**      | **+y**            | MOT_3 (vertical *left*) is at relative `+y = +0.148` from COM ✓ |
| **Up**        | **+z**            | MOT_5 / MOT_3 / MOT_4 sit slightly above COM (`+z`) ✓ |

> The script comment ("roll axis is −y, pitch +x, yaw +z") is **misleading** — the
> *math* in the script (`roll_col = +torque_x`, `pitch_col = −torque_y`,
> `yaw_col = −torque_z`) is what actually produced the C-code mixer, and that math
> is consistent with **forward = +x, left = +y, up = +z**. Trust the math, not the
> header comment.

ArduSub internally uses **FRD** (forward = +x, **right** = +y, **down** = +z).
The vehicle's Onshape FLU body is therefore **180° rolled about the forward axis**
relative to ArduSub FRD. That difference is *not* what `AHRS_ORIENTATION = 2`
(`ROTATION_YAW_90`) compensates — `YAW_90` only describes how the autopilot **chip**
is bolted into the hull, independent of the Onshape model frame. Heading sanity
must therefore be verified physically (§7), not inferred from the model.

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

The bridge currently applies these sign flips
([ros2_receiver.py:884-886](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L884-L886)):

```python
surge    = -float(msg.linear.x)   # negation present  — masks a downstream surge inversion
heave    = -float(msg.linear.z)   # negation present  — spec-correct FLU→FRD z-flip
yaw_rate = +float(msg.angular.z)  # NO negation       — passes ROS-FLU sign straight through
                                  #                     (NOT FRD-spec; masks a downstream yaw inversion)
vy       = 0.0                    # sway never sent
```

After PSC + ATC convert those to body-frame outputs `(R, P, Y, T, F, L)`,
the mixer × DIRECTION produces the table below.

> **Sign-chain note for yaw.** The earlier `yaw_rate = -msg.angular.z` was
> spec-correct (FLU CCW → FRD `-CW`) but produced **physical CW rotation** when
> commanded CCW (per the 2026-05-06 yaw bag). Removing the negation
> compensates for that downstream inversion (mixer Y column or motor wiring),
> so the *current* code yaws the right physical direction — but the bridge is no
> longer carrying the FLU→FRD spec flip, exactly the way `surge = -msg.linear.x`
> isn't either. Both axes are masking firmware-level inversions; see §2 / §4.

| ROS cmd_vel input        | Sign of `(R,P,Y,T,F,L)` PSC output | MOT_1 | MOT_2 | MOT_3 | MOT_4 | MOT_5 | MOT_6 |
|--------------------------|------------------------------------|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|
| `linear.x = +0.30`       | `F < 0` (bridge negation)          | **↑**  | —     | —     | —     | —     | —     |
| `linear.x = −0.30`       | `F > 0`                            | **↓**  | —     | —     | —     | —     | —     |
| `linear.z = +0.10` (up)  | `T > 0` (bridge negation + ArduSub up = +T) | —     | —     | **↑**  | **↓**  | **↑**  | —     |
| `linear.z = −0.10` (down)| `T < 0`                            | —     | —     | **↓**  | **↑**  | **↓**  | —     |
| `angular.z = +0.30` (CCW) | `Y > 0` (no bridge negation; ArduSub interprets +yaw_rate as CW per FRD) | —     | **↑**  | —     | —     | —     | **↑**  |
| `angular.z = −0.30` (CW) | `Y < 0`                            | —     | **↓**  | —     | —     | —     | **↓**  |
| `linear.y = +0.20` (left)  | `L < 0` (bridge `sway = -msg.linear.y`) | —     | **↑**  | —     | —     | —     | **↓**  |
| `linear.y = −0.20` (right) | `L > 0`                          | —     | **↓**  | —     | —     | —     | **↑**  |

> **Reading note for yaw rows.** What the mixer outputs for `Y > 0` is *what
> ArduSub thinks is CW* — but because the firmware Y-column or wiring has one
> inversion, "ArduSub CW" produces **physical CCW** here. So `angular.z = +0.30`
> (ROS CCW) → both yaw motors PWM ↑ → physical CCW rotation. The PWM signs in
> the table are what you'll observe at `servo_output_raw`, not what the spec
> would predict from a clean chain.

Legend: ↑ PWM > 1500 µs, ↓ PWM < 1500 µs, — neutral 1500 µs (within ±5 µs deadband).

> **Read this table before every test.** If the observed deflection pattern
> doesn't match the prediction *exactly* — including which channels stay neutral —
> you know which layer is broken before you change anything.

### 0.6 MANUAL-mode sanity check (run before every session)

Goal: confirm in 60 seconds that MANUAL still produces correct physical
motion. If it doesn't, **stop**, fix MANUAL first; the GUIDED-mode dry
runs in this document are based on the assumption that hardware layers
(§B.2 layers 1–4) are healthy, and they aren't reliable diagnostics if
that assumption breaks.

```bash
# Switch to MANUAL
ros2 topic pub --once /pixhawk/mode_cmd std_msgs/msg/String "data: 'MANUAL'"

# Arm
ros2 topic pub --once /pixhawk/arm_cmd std_msgs/msg/Bool "data: true"
```

Then, with props off, send each of these one-shot manual commands and
observe the chassis. The expected response is the **physical** direction
matching the command label — same convention as a pilot would expect.

```bash
# Forward (surge): chassis tries to thrust forward (MOT_1 spins to push fwd)
ros2 topic pub --once /pixhawk/manual_control std_msgs/msg/Int16MultiArray \
  "{data: [300, 0, 500, 0, 0, 0]}"   # surge=300, sway=0, heave=500 (neutral), yaw=0, roll=0, pitch=0

# Yaw left (CCW): chassis tries to rotate CCW
ros2 topic pub --once /pixhawk/manual_control std_msgs/msg/Int16MultiArray \
  "{data: [0, 0, 500, -300, 0, 0]}"  # yaw negative = left in MANUAL_CONTROL convention

# Heave up
ros2 topic pub --once /pixhawk/manual_control std_msgs/msg/Int16MultiArray \
  "{data: [0, 0, 800, 0, 0, 0]}"     # heave > 500 = up
```

(Heave channel is 0–1000 with 500 neutral; the others are −1000…+1000
with 0 neutral. See [send_6dof_command](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L407) in the bridge.)

**If all three match, hardware layers are healthy → continue to §1.**

**If any one doesn't match, stop:**

| MANUAL fault                       | Likely cause                                                    | Fix before continuing                                   |
|------------------------------------|-----------------------------------------------------------------|---------------------------------------------------------|
| Forward command → backward thrust  | A foil was swapped, ESC re-pinned, or `MOT_1_DIRECTION` changed | §6.1 foil inventory + §B.2.1                            |
| Yaw-left command → CW rotation     | Same on MOT_2 or MOT_6 (or the mixer Y column was edited)        | §6 — foil inventory first, then `motortest`             |
| Heave-up → sub sinks               | Same on MOT_3/4/5, or `MOT_4_DIRECTION` flipped                 | §6                                                      |

Do not proceed with the GUIDED dry-runs until MANUAL is back to its
known-good state. **GUIDED dry-run results with broken MANUAL are
uninterpretable.**

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
| MOT_1 above 1500, others neutral             | **Bridge `surge = -msg.linear.x` + `MOT_1_DIRECTION = −1` cancel — surge "works" but is doubly inverted.** | Optional cleanup: set `MOT_1_DIRECTION = +1` AND change bridge to `surge = +msg.linear.x`. Functionally identical, removes hidden double-negation. |
| MOT_1 below 1500                             | One of the two negations isn't actually present       | Re-read [ros2_receiver.py:884](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L884) and `param show MOT_1_DIRECTION` |
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

**and** in [ros2_receiver.py:884](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L884):

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

### 4.2 Expected — both lateral motors deflect *the same direction* (current bridge code)

Per §0.4 row 5 (`angular.z = +0.30`, ROS-FLU CCW; bridge passes `+0.30` straight to ArduSub):

| Channel | Expected   | Why                                                          |
|---------|------------|--------------------------------------------------------------|
| MOT_1   | 1500 ± 5   | No yaw factor                                                |
| MOT_2   | **> 1500** | ArduSub interprets `+yaw_rate` as CW → ATC `Y_out > 0` → `+0.775 × Y_out × +1 > 0` |
| MOT_3   | 1500 ± 5   | No yaw factor                                                |
| MOT_4   | 1500 ± 5   | No yaw factor                                                |
| MOT_5   | 1500 ± 5   | No yaw factor                                                |
| MOT_6   | **> 1500** | `−1.0 × Y_out × −1 > 0` (the two negatives cancel)           |

Both yaw motors should deflect **above** neutral. They go the same direction
in PWM space because `MOT_6_DIRECTION = −1` cancels the negative entry in the
mixer's `Y` column, so both physical motors push the chassis the same way for
a single-sign yaw command.

> **Physical direction expected:** ROS-FLU `+angular.z` should yaw the chassis
> **CCW** (left). If the chassis instead twists CW, the bridge is no longer
> compensating the firmware-level inversion correctly — see §4.4.

### 4.3 Then immediately run — CW (negative angular.z)

```bash
ros2 topic pub -r 20 /pixhawk/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.30}}" \
  --times 60
```

Both MOT_2 and MOT_6 should now deflect **below** 1500.

### 4.4 Diagnosis

| CCW result (MOT_2, MOT_6)        | CW result (MOT_2, MOT_6)     | Diagnosis                                        | Fix                                                          |
|----------------------------------|------------------------------|--------------------------------------------------|--------------------------------------------------------------|
| Both ↑, chassis twists CCW       | Both ↓, chassis twists CW    | **Symptoms match expectations** — bridge `+angular.z` + firmware inversion + DIR pairing all cancel out cleanly. Spec-non-compliant but functional. | Don't touch signs. PID may still need tuning (§4.5). For long-term cleanup see DEBUG_GUIDED.md. |
| Both ↑, chassis twists CW        | Both ↓, chassis twists CCW   | PWM table matches but physical direction is flipped — the firmware inversion the bridge was masking is **not actually present** | Re-add the bridge negation: `yaw_rate = -float(msg.angular.z)`. |
| Both ↓                           | Both ↑                       | Whole yaw chain inverted by 1 sign vs the table  | Easiest: re-add the bridge negation. Cleanest: leave bridge as-is, flip both `MOT_2_DIRECTION` and `MOT_6_DIRECTION` as a pair, reboot. Pick one — never both. |
| MOT_2 ↓, MOT_6 ↑                 | MOT_2 ↑, MOT_6 ↓             | The two yaw motors are fighting each other       | One of `MOT_2_DIRECTION` / `MOT_6_DIRECTION` is wrong individually. Run §6 to disambiguate. |
| One channel deflects, other neutral | —                          | Mixer Y entry zero for one motor, or motor disabled | Re-check `AP_Motors6DOF.cpp`, `SERVOn_FUNCTION`, `MOTn_PWM_*` |
| Neither deflects, but vehicle would clearly try to yaw | — | ATC produced almost-zero output           | Yaw PID is gutted (`ATC_RAT_YAW_P=0`). See §4.5. |

> **Important:** the *PWM direction* (↑/↓) and the *physical chassis rotation*
> are independent observations. Always log both. Two of the rows above
> distinguish themselves only by physical direction.

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

## 5. Sway dry-run (Y axis)

The bridge now forwards `linear.y` as `sway = -float(msg.linear.y)` (the
spec-correct FLU→FRD flip — see [ros2_receiver.py:885](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L885)).
No patch needed.

> The starting sign is the **spec-clean** convention, on the assumption
> that the lateral chain (mixer L column + `MOT_2_DIRECTION` + `MOT_6_DIRECTION`)
> is consistent. Since both surge and yaw turned out to need workaround
> negations, expect to discover the same here. The §5.3 truth-table tells
> you whether the fault is wiring (DIRECTION pair flip) or mixer geometry
> (reflash) — **don't change the bridge sign to mask it**, since that just
> hides the fault and breaks the spec-clean target end state (§A.5).

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

## 6. Pinpointing a per-motor fault — foil check + `motortest`

If §4 + §5 disagree (one motor each direction) you have a single-motor
fault, not a mixer issue. Before going to MAVProxy, do a 30-second visual
check:

### 6.1 Foil inventory (props off)

Pull or inspect each prop, read its **R** or **L** stamp (see §B.2 for
why this matters). Build the table:

| Motor | Foil stamp (observed) | Foil stamp (expected per design) | `MOT_n_DIRECTION` |
|-------|-----------------------|-----------------------------------|-------------------|
| MOT_1 | ____                  | ____                              | −1 (current)      |
| MOT_2 | ____                  | ____                              | +1                |
| MOT_3 | ____                  | ____                              | +1                |
| MOT_4 | ____                  | ____                              | −1                |
| MOT_5 | ____                  | ____                              | +1                |
| MOT_6 | ____                  | ____                              | −1                |

If observed ≠ expected for the suspect motor, the cheapest fix is to
**swap the foil back** to the design value. That removes one layer of
inversion, so you also need to flip `MOT_n_DIRECTION` back to whatever
its un-compensated value would be — otherwise you've moved the fault,
not removed it.

If observed = expected, the foil is fine; move on to §6.2.

### 6.2 Per-motor thrust direction with `motortest`

In MAVProxy:

```text
motortest 2 1 1700 2     # MOT_2, +200 µs above neutral, 2 s
motortest 2 1 1300 2     # MOT_2, −200 µs below neutral, 2 s
motortest 6 1 1700 2     # MOT_6
motortest 6 1 1300 2
```

For each command, observe the physical thrust direction (mark the prop
or feel the water/airflow). With the design foil fitted and
`MOT_n_DIRECTION = +1`:

- `PWM > 1500` → motor produces thrust in its **design "positive"**
  direction.
- `PWM < 1500` → opposite.

With `MOT_n_DIRECTION = −1` the polarities reverse — that's the param's
purpose.

### 6.3 Decision

| Observation                                      | Likely root cause                          | Fix                                                                |
|--------------------------------------------------|--------------------------------------------|--------------------------------------------------------------------|
| Foil stamp is wrong; thrust is reversed          | A foil swap during maintenance             | Re-fit the design foil **and** undo the compensating `MOT_n_DIRECTION` change. Net: one layer change, one layer change → cancel out. |
| Foil stamp is correct; thrust is reversed        | ESC phase miswire **or** mounting flipped  | Easiest: flip `MOT_n_DIRECTION`. Cheapest physical: swap two ESC leads. Don't do both. |
| Foil correct, thrust correct, but axis still wrong in §4/§5 | Fault is in the mixer, not this motor | Go to §5.3 truth-table; the fix is a reflash, not a per-motor change. |

Never apply two compensations to the same motor — one layer of fix per
layer of fault, otherwise you regenerate the "two wrongs make a right"
pattern and the next person to touch the sub will be debugging the same
ghost.

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

## 9. Procedure — what to do, in order, with branching

The goal is to walk every axis from "unknown" → "spec-clean root cause
identified" with the minimum amount of testing. Each step gates the next:
a failure narrows the search before you change anything.

```
        ┌─────────────────────────────────────────┐
        │ STEP 0. MANUAL sanity check (§0.6)      │
        │   Surge / yaw / heave each do the right │
        │   thing in MANUAL? If yes, hardware     │
        │   layers (mixer, DIRECTION, foils, ESC) │
        │   are RULED OUT and stay out unless     │
        │   MANUAL later breaks.                  │
        └──────────────────┬──────────────────────┘
                           │ all three correct
                           ▼
        ┌─────────────────────────────────────────┐
        │ STEP 1. Setup verification (§1)         │
        │   Switch to GUIDED. Send tiny heave;    │
        │   do servo PWMs move?                   │
        └──────────────────┬──────────────────────┘
                           │ yes
                           ▼
        ┌─────────────────────────────────────────┐
        │ STEP 2. Static AHRS check (§7.2)        │
        │   ATTITUDE.yaw matches physical north?  │
        │   Tilt by hand; ATTITUDE.pitch tracks?  │
        └──────────────────┬──────────────────────┘
                           │ yes — AHRS is ruled out
                           ▼
        ┌─────────────────────────────────────────┐
        │ STEP 3. Heave dry-run (§3)              │
        │   Verify the cleanest gravity-locked    │
        │   axis. Pattern: MOT_3↑ MOT_4↓ MOT_5↑   │
        └──────────────────┬──────────────────────┘
                           │ matches
                           ▼
        ┌─────────────────────────────────────────┐
        │ STEP 4. Surge dry-run (§2)              │
        │   Single-motor test. Pattern: MOT_1↑    │
        │   for ROS +linear.x, others neutral.    │
        └──────────────────┬──────────────────────┘
                           │ matches (one fault absorbed by 2 negations)
                           ▼
        ┌─────────────────────────────────────────┐
        │ STEP 5. Yaw dry-run (§4) — CCW then CW  │
        │   Pattern: MOT_2 and MOT_6 same dir.    │
        │   Record physical chassis direction.    │
        └──────────────────┬──────────────────────┘
                           │
                           ▼
        ┌─────────────────────────────────────────┐
        │ STEP 6. Sway dry-run (§5) — left, right │
        │   Pattern: MOT_2 and MOT_6 opposite dir.│
        │   This step disambiguates the yaw fix.  │
        └──────────────────┬──────────────────────┘
                           │
                           ▼
        ┌─────────────────────────────────────────┐
        │ STEP 7. Decision (§5.3 truth-table)     │
        │   Cross-reference §4 yaw + §5 sway:     │
        │   choose DIRECTION pair flip OR mixer   │
        │   reflash. Apply exactly ONE of them.   │
        └──────────────────┬──────────────────────┘
                           │
                           ▼
        ┌─────────────────────────────────────────┐
        │ STEP 8. Cleanup (§A.5)                  │
        │   Remove the bridge double-negations.   │
        │   Fix surge: MOT_1_DIRECTION = +1.      │
        │   Bridge → spec-clean (§A.3).           │
        └──────────────────┬──────────────────────┘
                           │
                           ▼
        ┌─────────────────────────────────────────┐
        │ STEP 9. Re-verify (rerun §3/§4/§5/§2)   │
        │   With spec-clean bridge, predictions   │
        │   should now match without negations.   │
        └─────────────────────────────────────────┘
```

### What to do at each branch — failure modes

| Step fails at  | Symptom                                                       | Branch to                                                                  |
|----------------|---------------------------------------------------------------|----------------------------------------------------------------------------|
| 0              | MANUAL surge/yaw/heave wrong                                  | Stop GUIDED diagnosis. Go to §6 (foil + ESC inventory). Hardware layers are now suspect; DON'T trust the §0.4 prediction table until MANUAL is fixed. |
| 1              | No servo PWMs move                                            | Bridge / mode / arm problem; not an axis fault. §1 fail-actions.           |
| 2              | `ATTITUDE.yaw` 180° off                                       | `AHRS_ORIENTATION = 6` (YAW_270), reboot, retry. **All later sign tests are meaningless until this is right.** |
| 2              | Pitch and roll swap when tilted                               | `AHRS_ORIENTATION` is on a non-z rotation; try one of the `*_PITCH_*` enums. Fix before continuing. |
| 3              | Wrong asymmetry pattern, MANUAL still works                   | Bridge `heave` sign is wrong, OR ArduSub GUIDED `vz` interpretation differs from spec. **Don't touch the mixer or MOT_DIRECTION** — MANUAL working proves those are right. Test by flipping the bridge `heave` sign only. |
| 3              | Wrong asymmetry pattern AND MANUAL also broke                  | Hardware regression since last MANUAL test (foil swap, ESC re-pin, param drift). Go to §6. |
| 4              | MOT_1 wrong direction in GUIDED, correct in MANUAL            | The bridge `surge = -msg.linear.x` is the correct ArduSub-GUIDED-quirk compensation; if you removed it as cleanup, put it back. Don't touch `MOT_1_DIRECTION`. |
| 4              | MOT_1 wrong direction in MANUAL too                           | Hardware regression. Go to §6.                                              |
| 5              | Yaw wrong in GUIDED, MANUAL still works                       | Bridge `yaw_rate` sign needs adjusting (current code passes ROS-FLU through unchanged). Try `yaw_rate = -msg.angular.z` and rerun §4. **Don't touch the mixer or DIRECTIONs.** |
| 5              | Both yaw motors deflect *opposite* ways (one ↑, one ↓)        | Single-motor wiring fault — but only believable if MANUAL also shows asymmetric yaw. If MANUAL works, this is a transient PSC/ATC artefact; rearm and retry. |
| 6              | Sway wrong in GUIDED, MANUAL surge/yaw still good             | Bridge `sway` sign needs adjusting. Try `sway = +msg.linear.y` and rerun §5. (Sway isn't on the §0.6 MANUAL check, so re-test sway in MANUAL too: `[0, 300, 500, 0, 0, 0]` to validate the lateral chain independently of GUIDED.) |
| 7              | Any reflash recommendation                                     | **Re-verify MANUAL still works first.** If MANUAL is correct, a mixer reflash will break it without fixing GUIDED. Always exhaust bridge-side fixes first. |

### Per-test recording discipline

Annotate each bag with the step number and the observed pattern, e.g.:

```
20260508_103115_step3_heave_+010_PASS_match-prediction
20260508_103330_step4_surge_+030_PASS_MOT1_above
20260508_103515_step5_yaw_ccw_030_FAIL_pwm-up_chassis-CW
```

The post-mortem in [DEBUG_GUIDED.md](DEBUG_GUIDED.md) cross-references these tags.

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

## A. Where can a sign inversion live? — bridge vs `MOT_DIRECTION` vs mixer factor

Every axis has the same chain:

```
ROS cmd_vel  →  bridge sign flip  →  PSC/ATC  →  mixer factor (R,P,Y,T,F,L)  →  MOT_n_DIRECTION  →  PWM
```

A sign inversion at any of those four layers (bridge, PSC, mixer factor, motor
direction) produces the same wrong-direction symptom. **Where you fix it
matters** because the three knobs are not interchangeable.

### A.1 `MOT_n_DIRECTION` — flips the entire motor

```
final_pwm[n] = 1500 + scale · output[n] · MOT_n_DIRECTION
```

`MOT_n_DIRECTION` multiplies the **whole** motor output, so it flips every
axis (R, P, Y, T, F, L) the motor contributes to *simultaneously*. It is a
**physical-polarity** knob: use it when the motor is mechanically reversed
(ESC phases swapped, or propeller pitched backwards).

Practical consequence:

| Motor (only non-zero columns)               | Effect of flipping its `MOT_n_DIRECTION`                  |
|---------------------------------------------|-----------------------------------------------------------|
| MOT_1 (`F` only)                            | Flips surge alone — safe.                                 |
| MOT_2 (`Y`, `L`)                            | Flips yaw **and** sway together — never use to fix one.   |
| MOT_3 (`R`, `P`, `T`)                       | Flips roll, pitch, throttle together — almost never useful. |
| MOT_4 (`R`, `P`, `T`)                       | Same as MOT_3 — flips all three.                          |
| MOT_5 (`P`, `T`)                            | Flips pitch and throttle together.                        |
| MOT_6 (`Y`, `L`)                            | Flips yaw and sway together.                              |

### A.2 Mixer factor sign in `add_motor_raw_6dof` — flips one axis on one motor

```c
add_motor_raw_6dof(MOT_2, R=0, P=0, Y=+0.775, T=0, F=0, L=-1.0, 2);
//                                  ^^^^^^^                ^^^^
//                                  yaw          lateral
```

Each scalar multiplies its axis's command independently. Flipping only `Y`
for MOT_2 flips yaw without touching lateral. This is the **geometry** knob:
use it when the mixer's idea of which way a motor's thrust × moment-arm
points around an axis is wrong, but the motor itself is fine.

Requires firmware reflash; no parameter equivalent.

### A.3 Bridge sign flip — flips one ROS axis globally

A negation in `cmd_vel_cb` flips one ROS-FLU axis before ArduSub ever sees
it. Effect is global (every motor that contributes to that axis flips).

This is *the* place where **two distinct things** belong:

1. **The FLU→FRD spec flip** for each axis (a frame conversion that is
   purely a coordinate-system fact):

   ```python
   surge    = +float(msg.linear.x)   # forward axis is invariant under FLU→FRD
   sway     = -float(msg.linear.y)   # left  → -right
   heave    = -float(msg.linear.z)   # up    → -down
   yaw_rate = -float(msg.angular.z)  # CCW   → -CW
   ```

2. **Compensation for ArduSub GUIDED-mode quirks** — sign conventions
   that differ between the MAVLink spec and what ArduSub's GUIDED
   handler (`mode_guided.cpp` / `GCS_MAVLink_Sub.cpp`) actually does.
   Because **MANUAL works** on this vehicle (see "CRITICAL CONTEXT"
   at the top of this doc), every hardware layer downstream of the
   PSC is correct. The only place an empirically-needed sign flip
   can come from in GUIDED is the GUIDED firmware path itself.

The current bridge state combines both into final coefficients:

```python
surge    = -float(msg.linear.x)   # spec +, ArduSub-quirk -, net -
heave    = -float(msg.linear.z)   # spec -, no quirk,       net -
yaw_rate = +float(msg.angular.z)  # spec -, ArduSub-quirk -, net +
sway     = -float(msg.linear.y)   # spec -, no quirk known, net - (verify with §5)
```

These are **not two-wrongs-make-a-right hacks** — they are the two
legitimate sign-handling roles the bridge plays, collapsed into single
coefficients. Trying to "clean them up" by removing them and reflashing
the firmware will break MANUAL while leaving GUIDED still wrong.

### A.4 When are two knobs equivalent? When are they not?

| Inversion in       | Equivalent way to absorb it                                             |
|--------------------|-------------------------------------------------------------------------|
| Surge only (MOT_1) | bridge sign **OR** `MOT_1_DIRECTION` **OR** mixer `F` factor — any one  |
| Yaw + sway as a pair (MOT_2 + MOT_6 wired backwards) | flip both `MOT_2_DIRECTION` and `MOT_6_DIRECTION` |
| Yaw only (mixer Y signs disagree with sway L signs) | flip mixer `Y` factor on MOT_2 and MOT_6 — **must reflash** |
| Sway only (mixer L signs disagree with yaw Y signs) | flip mixer `L` factor on MOT_2 and MOT_6 — **must reflash** |

Pick exactly one absorption point per inversion. If you stack two — bridge
negation **and** `MOT_n_DIRECTION = -1` — you get the current "two wrongs
make a right" situation that masks the underlying fault.

### A.5 Target end-state

Given that **MANUAL works**, the realistic target is **not** "spec-clean
bridge with no negations" — that would require the firmware path to be
ArduSub-GUIDED-spec-clean too, which it empirically isn't on 4.5.7 for
this hull. The realistic target is:

1. Bridge holds **the FLU→FRD spec flips PLUS the minimum number of
   ArduSub-GUIDED-quirk negations needed for correct physical motion**.
   On the current vehicle that resolves to: `surge` net `-`, `sway`
   net `-`, `heave` net `-`, `yaw_rate` net `+`. (See §A.3 for the
   per-coefficient breakdown.)
2. The bridge code comment for each line states **both** the spec
   flip *and* the GUIDED-quirk flip, so the next person can tell at
   a glance whether a coefficient comes from coordinate-system math
   or from firmware-quirk compensation. Example:
   ```python
   yaw_rate = +float(msg.angular.z)
   # Spec FLU→FRD would be -msg.angular.z (CCW → -CW). ArduSub 4.5.7
   # GUIDED inverts that again, so the net is +. Removing this flip
   # breaks GUIDED yaw direction. MANUAL is unaffected.
   ```
3. **No `MOT_n_DIRECTION` or mixer-factor changes are made for the
   purpose of fixing GUIDED.** All three are validated by MANUAL and
   stay where they are. Hardware-layer fixes happen if and only if
   MANUAL itself becomes broken (see §6).

This isn't theoretically pretty — there are sign flips in the bridge
that wouldn't exist in a spec-clean ArduSub. But it's the correct
**architectural** location for them, because they compensate something
in the GUIDED firmware path that doesn't affect MANUAL.

If you ever upgrade ArduSub or get the BODY_FRD interpretation patched
upstream, the GUIDED-quirk flips can be removed — and the per-line
comments make it obvious which ones to remove without breaking MANUAL.

---

## B. What "PWM > 1500" or "PWM < 1500" actually means

Throughout this document we predict each motor's deflection as **↑** (PWM
above 1500 µs) or **↓** (PWM below 1500 µs). That maps to a physical
behaviour, but the mapping has a few moving parts worth being explicit about.

### B.1 The PWM convention

ArduSub drives each motor (via its ESC) with an RC-style PWM pulse, in
microseconds. The current vehicle params:

| Param         | Value | Meaning                                        |
|---------------|------:|------------------------------------------------|
| `MOT_PWM_MIN` |  1100 | full thrust, "negative" direction              |
| `MOT_PWM_MAX` |  1900 | full thrust, "positive" direction              |
| neutral       |  1500 | motor stopped (zero thrust), centred deadband  |
| `MOT_SPIN_MIN`| 0.15  | minimum normalized output before motor spins   |
| `MOT_SPIN_ARM`| 0.10  | armed-but-not-spinning level                   |

So the motor signal is **bipolar around 1500 µs**:

```
1100 µs ───────── 1500 µs ───────── 1900 µs
  │                  │                  │
  full reverse     stopped         full forward
  thrust            (neutral)         thrust
```

A motor PWM above 1500 spins the motor in its "positive" direction; below
1500 spins it the other way. The ESC needs to be a **bidirectional**
ESC (which it is for BlueRobotics T200/T500 and equivalents) — not the
unidirectional ESCs used for aircraft propellers.

### B.2 What "positive direction" means physically

Whether `PWM > 1500` produces forward or backward thrust on a given motor
depends on **four** independent layers — flipping any one of them reverses
the thrust direction. Flipping any two cancels out. This is why surprise
"works after we swapped X" fixes happen.

1. **ESC phase wiring.** Swapping any two of the three motor leads reverses
   the direction the rotor spins for the same PWM signal. Cheap and
   silent — easy to do by accident at assembly.

2. **Propeller handedness — R-foil vs L-foil.** Our motors ship with two
   prop variants: one marked **R** (right-handed / CW prop, produces forward
   thrust when the rotor spins clockwise viewed from in front) and one
   marked **L** (left-handed / CCW prop, produces forward thrust when the
   rotor spins counter-clockwise). **Mounting an R-foil on a motor that
   spins CCW (or an L-foil on a CW motor) gives reverse thrust** at the
   same PWM. The motor itself is "fine"; the system is just plumbed wrong.

   On this sub, the foils were chosen so that with the **default**
   `MOT_n_DIRECTION = +1`, `PWM > 1500` on any motor produces thrust in
   that motor's "intended" direction — `MOT_1_DIRECTION = -1`,
   `MOT_4_DIRECTION = -1`, `MOT_6_DIRECTION = -1` indicate three motors
   where either the wiring **or** the foil **or** the mounting ended up
   reversed at integration, and `MOT_n_DIRECTION = -1` is the parameter
   knob compensating for it.

3. **Mounting orientation.** The same motor + foil bolted in upside-down,
   or rotated 180° about its thrust axis, produces opposite thrust along
   the body axis. Rare on this hull but worth eliminating.

4. **`MOT_n_DIRECTION` parameter.** The escape hatch that flips the entire
   motor output sign in software, after layers 1–3 are already locked in
   by hardware.

The vehicle integrator (you, when building the sub) needs **an even number
of inversions** across layers 1–4 for each motor, so that `output[n] > 0`
from the mixer formula

```
output[n] = R·roll + P·pitch + Y·yaw + T·throttle + F·forward + L·lateral
final_pwm[n] = 1500 + scale · output[n] · MOT_n_DIRECTION
```

ends up as the intended physical thrust direction. If you change *one*
of the four layers and don't update another, the sign flips.

> **Diagnostic implication:** when §6 / §B.4 says a motor's thrust is
> "wrong direction", you have four candidate fixes: swap two ESC leads,
> swap the R-foil for an L-foil (or vice versa), remount it 180°, or
> flip `MOT_n_DIRECTION`. **Pick the one that's cheapest to verify
> visually.** For a motor with the foil exposed: pulling the prop and
> reading the R/L stamp takes 10 seconds and rules out (or in) layer 2
> immediately. For a motor with sealed wiring: `MOT_n_DIRECTION` is
> faster than re-pinning the ESC.

### B.2.1 R/L foil quick check (props off, before any motor test)

Before running `motortest` (§6), inventory the foils:

1. Remove each prop (or inspect its stamp through the duct).
2. Note the **R** or **L** marking on each.
3. Compare to the design intent — typically opposite-pair motors should
   carry opposite foils so the counter-rotation cancels reaction torque
   on the chassis when commanded to thrust together.

A common failure mode: during a parts replacement (broken prop, vendor
substitution), an R-foil was swapped onto a motor that was originally
running an L-foil. This produces a hard "wrong direction on motor N"
fault that cannot be fixed by tweaking the bridge or the mixer — only
by swapping the prop back, or by flipping `MOT_n_DIRECTION` to mask it.

### B.3 Why prediction tables use ↑/↓ not "forward/backward"

The §0.4 table is computed from the mixer formula above — it tells you
**which side of 1500 µs each PWM should be on**, given a clean sign chain.
Whether ↑ produces "forward thrust on the chassis" depends on the three
hardware layers in §B.2.

That separation is deliberate, because it lets the diagnosis tables in §2
and §4 distinguish:

- **PWM matches table, physical direction matches** → entire chain is
  consistent (possibly via two-wrongs cancelling, but consistent).
- **PWM matches table, physical direction is wrong** → bug is at hardware
  layers (B.2 items 1, 2, or 3); the mixer + bridge are doing what the
  table predicted.
- **PWM doesn't match table** → bug is upstream of the mixer (bridge sign,
  PSC integrator, ATC sign, mixer factor).

These are different fixes. **Always log both** the PWM offset *and* the
physical chassis response for every dry-run test — the two together pin
down the layer.

### B.4 Reading `servo_output_raw` in practice

```bash
ros2 topic echo /pixhawk/servo_output_raw --field data
```

Output for, say, a clean `linear.x = +0.30` with the current bridge:

```
data:
- 1620   ← MOT_1, ↑ (above 1500 by ~120 µs)
- 1500   ← MOT_2, neutral
- 1500   ← MOT_3, neutral
- 1500   ← MOT_4, neutral
- 1500   ← MOT_5, neutral
- 1500   ← MOT_6, neutral
```

A typical clean offset on a step-input dry-run is **±50–200 µs** at
amplitudes of 0.20–0.30 m/s or rad/s. Less than ±20 µs means the PSC/ATC
output was near zero (gain too low, or integrator not yet built up); more
than ±300 µs means the loop is saturating or the gain is too high.

The deadband around 1500 is small (~±5 µs from the mixer; quantization on
the wire can add a few more). Treat anything within ±10 µs as "neutral"
for diagnosis.

---

## 11. Cross-reference

- Full diagnostic playbook + parameter strategy: [DEBUG_GUIDED.md](DEBUG_GUIDED.md)
- Bridge code: [src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py)
- Mixer source (custom frame): [project-polaris-ardusub/libraries/AP_Motors/AP_Motors6DOF.cpp](../project-polaris-ardusub/libraries/AP_Motors/AP_Motors6DOF.cpp) — `SUB_FRAME_CUSTOM` block
- Frame factor calculator: [scripts/calculate_attitude_factors.py](scripts/calculate_attitude_factors.py) (or wherever you keep it)
