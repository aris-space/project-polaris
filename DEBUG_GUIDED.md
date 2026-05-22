# Polaris — ArduSub GUIDED-mode velocity control debug playbook

This document is the post-mortem of the 2026-05 dive trials where SITL
worked but the real vehicle exhibited a surge sign inversion, wrong yaw
direction, post-stop lag/backward drift, yaw chatter, and an idle pitch
bias. It is structured as a punch list — read top to bottom, do each step,
verify, then move on.

> **Revision (2026-05-07):** earlier drafts of this doc said
> `AHRS_ORIENTATION=2` was `YAW_180`. **It is not.** ArduPilot's enum has
> `ROTATION_YAW_90 = 2` ([rotations.h:30](../project-polaris-ardusub/libraries/AP_Math/rotations.h#L30));
> `YAW_180 = 4`. The autopilot is mounted **rotated 90° about Z** from
> default. With the param set correctly to `YAW_90`, AHRS body **does**
> equal vehicle body — there is no 180° offset to be found here. This
> kills the original "surge inversion = EKF yaw 180° off" hypothesis.
>
> Bag analysis (the 2026-05-06 yaw-rate test, `y_rat_0_30_*.mcap`)
> further confirms `/odometry/filtered/local` reports yaw rate with the
> correct sign relative to the physical rotation (sub yawed CW; both
> `/imu/angular_velocity.z` and `/odometry/filtered/local.twist.angular.z`
> are negative, matching REP-103 ENU/FLU). **The ROS-side state is
> correct.** Therefore the surge / yaw faults live entirely in what the
> bridge sends to Pixhawk and how Pixhawk's stock GUIDED handler routes
> it through the **custom motor mixer**.

## TL;DR — the actual root causes

1. **Custom-frame yaw column is sign-inverted vs ArduSub's mixer
   convention.** [AP_Motors6DOF.cpp:181-186](../project-polaris-ardusub/libraries/AP_Motors/AP_Motors6DOF.cpp#L181-L186)
   of `project-polaris-ardusub` produces CCW for positive `yaw_in`;
   ArduSub's stock attitude controller assumes positive `yaw_in` →
   CW (FRD-positive). Confirmed by the `y_rat` bag: ROS commands
   `+0.30 rad/s` (CCW), bridge negates to `-0.30 rad/s` BODY_FRD
   (still CCW per spec), sub physically rotates CW. One inversion in
   the chain — the mixer.
2. **Surge sign inversion is *not* an AHRS/EKF problem.** With
   `AHRS_ORIENTATION=YAW_90` correctly set and the user-verified ROS
   odometry, AHRS body == vehicle body. The custom firmware diff vs
   stock 4.5.7 only touches `mode_althold.cpp` and the motor mixer —
   the GUIDED handler, PSC, and BODY_FRD interpretation are
   untouched. So the surge inversion lives in **either**:
   - the custom mixer's MOT_1 `forward` column combined with
     `MOT_1_DIRECTION=-1` (a wiring-vs-mixer polarity mismatch that
     happens to be hidden in MANUAL mode by joystick deadband or
     differing scaling), **or**
   - one of the other custom-frame motors having a non-zero
     `forward_factor` that wasn't intended (verify by re-reading the
     mixer table — see "Surge: targeted verification" section below).
   The empirical `surge = -msg.linear.x` workaround is masking a
   firmware/wiring issue that needs a one-shot single-axis test to
   pin down.
3. **Yaw PIDs are gutted.** `ATC_RAT_YAW_P=0.08, I=D=FF=0`,
   `ATC_ANG_YAW_P=0`, `ATC_SLEW_YAW=6000`. SITL doesn't override these
   so it runs ArduSub defaults (P≈0.18, I≈0.018, FF≈0). Whatever the
   bridge commands gets shredded by the under-tuned rate loop on the
   real vehicle. The `y_rat` bag's ±0.20 rad/s ringing on `/imu` while
   the sub was "stationary in yaw" is exactly this loop chattering.
4. **PSC velocity integrators wind up and then refuse to wind down.**
   `PSC_VELXY_I=0.5, IMAX=1000` builds large internal state during a
   long surge; the watchdog's "send 0 velocity" doesn't reset it, so
   after the publisher stops there's residual forward thrust that
   overshoots into backward thrust. SITL has `PSC_VELXY_D=0.8` (real
   has 0.0) which damps this. The `x_vel` bag shows
   `/odometry/filtered/local.twist.linear.x` swinging from −0.3 to
   +0.6 m/s while a constant +0.20 m/s was commanded — exactly the
   PSC-overshoot signature.

The cmd_vel callback **type_mask is correct as-is** — do not change it.
The bug isn't the bitmask, it's what's downstream of it.

## Surge: targeted verification before any fix

The user-verified facts narrow the search drastically:

- `/odometry/filtered/local` (ROS EKF) is correct — yaw and
  body-frame velocity both match the physical state.
- `AHRS_ORIENTATION=YAW_90` matches the physical mounting.
- ArduSub's GUIDED → PSC → motors path is stock.

So when the bridge sends BODY_FRD vx=+0.35 and the sub goes backward,
exactly one of these is happening:

**(a)** ArduSub is computing the right thing but a single motor's
`forward_factor` × `MOT_n_DIRECTION` chain produces backward thrust.
Most likely: MOT_1, since that's the only motor with non-zero
forward_factor in the custom mixer. Verify with a **GUIDED single-axis
test in air, props off**:

1. Arm in GUIDED mode with the sub on the bench, propellers removed
   or covered.
2. Send `ros2 topic pub --once /pixhawk/cmd_vel geometry_msgs/Twist
   "{linear: {x: 0.35}}"` with bridge code reverted to
   `surge = +msg.linear.x` (drop the negation temporarily for this
   test only).
3. Read `SERVO_OUTPUT_RAW.servo1_raw` from MissionPlanner. It should
   be **above 1500 µs** (forward = positive deflection from neutral).
   If it is below 1500 µs, MOT_1's effective sign is flipped — the
   mixer's `forward_factor=+1.0` and `MOT_1_DIRECTION=-1` are
   double-counting the wiring polarity.

**(b)** ArduSub's PSC is producing the right body-frame target but
some other motor is being driven by the velocity setpoint. Verify by
reading `SERVO_OUTPUT_RAW` for **all** servos during the same test.
Only `servo1_raw` should deflect on a pure forward command. If
servo2/3/4/5/6 also move, the mixer table has a stale `forward_factor`
in the wrong row.

**Once (a) or (b) is confirmed**, fix it at the source:
- (a) → set `MOT_1_DIRECTION=+1` (un-invert the wiring compensation),
  reflash is not required — it's a parameter.
- (b) → edit
  [AP_Motors6DOF.cpp:181-186](../project-polaris-ardusub/libraries/AP_Motors/AP_Motors6DOF.cpp#L181-L186)
  to zero out the spurious `forward_factor` and reflash.

Then remove the `surge = -msg.linear.x` workaround in the bridge.

## What does *not* need to change

- `cmd_vel_cb` `type_mask` construction at
  [ros2_receiver.py:882-891](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L882-L891).
  Leave the current seven `*_IGNORE` bits exactly as they are.
  `YAW_RATE_IGNORE` stays *not* set; `VY_IGNORE` stays *not* set
  (setting it would make ArduSub drop the entire velocity setpoint,
  since `VEL_IGNORE` is the OR of vx/vy/vz ignore bits in
  `GCS_Mavlink.cpp`).
- `ekf_odom_cb` — the ENU→NED + FLU→FRD math at
  [ros2_receiver.py:482-516](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L482-L516)
  is symbolically correct. Don't touch it until log analysis from a
  .mcap proves the yaw is actually wrong on the wire.
- The 0.3 s watchdog timeout at
  [ros2_receiver.py:222](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L222).
  It correctly bypasses ArduSub's 3 s `GUID_TIMEOUT`.

## Why "send zero velocity" is the worst stop command

The lag and backward overshoot when the publisher stops are not a
motor speed problem — they're an **integrator wind-up** problem in
ArduSub's PSC velocity loop. The loop is:

```
accel_cmd = P·err + I·∫err dt + D·d(err)/dt + FF·target
```

While you stream `vx = +0.20 m/s` for several seconds, the integrator
quietly accumulates until its contribution alone equals the thrust
needed to fight drag at 0.20 m/s. At that steady state `err ≈ 0`, the
P term ≈ 0, `FF·target = 0.04` m/s² (small), and **all the rest of
the thrust is the integrator**. The loop is "remembering" the
forward thrust the sub needs.

When the publisher stops and the watchdog sends `vx = 0`:

| Term  | Value just after the switch | Effect |
|-------|------------------------------|--------|
| `err` | `0 − 0.20 = −0.20` (sub is still gliding) |  |
| P     | `6 × −0.20 = −1.2` m/s²       | strong deceleration — good |
| I     | still **positive** from wind-up; bleeds at `I·err = 0.5 × −0.20 = −0.1` per second | takes ~2 s to discharge — bad |
| D     | `0` on real vehicle (`PSC_VELXY_D = 0`) | no damping — bad |
| FF    | `0`                           | inactive |

P brakes, I keeps pushing forward, no D to keep them honest. The sub
decelerates slowly. By the time actual velocity reaches 0, the
integrator has overshot **into negative territory** (the whole
deceleration phase added negative `err·dt` to its state). Now the
integrator commands backward thrust → sub drifts backward → new
positive error → integrator winds back. Without D this oscillates
instead of settling. Settling time is dominated by `1/I`, so with
`I = 0.5` it's seconds — exactly the lag you observe.

## Why this hits the real vehicle harder than SITL

Three things compound on hardware:

1. **`PSC_VELXY_D = 0`** vs SITL's `0.8`. No damping on the
   wind-up/wind-down oscillation.
2. **`PSC_VELXY_IMAX = 1000`** — effectively unbounded integrator. SITL
   doesn't expose this because it's less draggy and the integrator
   never gets that big.
3. **The watchdog sends `vx = 0` once, then goes silent.** ArduSub
   then has a stale velocity target and a hot integrator until
   `GUID_TIMEOUT` (3 s) eventually flips it into position-hold.
   Whatever the integrator does in that 3 s window is what you see.

## Fixes — cheapest first

1. **Cap the integrator (param only).** Set `PSC_VELXY_IMAX = 50`
   cm/s². Wind-up bounded → wind-down bounded. Biggest cheap win.
2. **Add D damping (param only).** Set `PSC_VELXY_D = 0.8` (matches
   SITL). Kills the oscillation outright.
3. **Reduce I (param only).** Set `PSC_VELXY_I = 0.2`. Slower wind-up
   = less to discharge later. Trade-off: slightly worse steady-state
   tracking.
4. **Watchdog sends position-hold, not velocity-zero (code).** Set the
   `VX/VY/VZ_IGNORE` bits and give ArduSub a position target. That
   forces a *setpoint-class transition* and ArduSub resets the
   velocity integrator internally on that transition — no wind-down
   problem at all. See "What the cmd_vel watchdog should do" below.
5. **Bridge ramps to zero (code).** When the watchdog fires, instead
   of one `vx = 0` message, send a 0.5 s ramp from the last commanded
   value down to 0 at 20 Hz. The integrator sees a smooth target
   change, not a step, and stays in the linear regime.
6. **Upstream controller ramps its own outputs (architectural).**
   `pure_pursuit_controller_3d` should never step from full speed to
   zero. If it ramps down before going silent, the watchdog never
   fires.

Items 1 + 2 alone usually take the visible lag from "noticeable
seconds" to "barely there." Add #4 if you want the integrator state
actually purged on stop.

## What the cmd_vel watchdog should do

Today the watchdog at
[ros2_receiver.py:911-948](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L911-L948)
sends a clean zero-velocity setpoint with the same `type_mask` used in
normal flight. It fires once per timeout, then sets
`_cmd_vel_was_active = False` so it doesn't keep spamming. That's
structurally fine but it is **not enough** for the lag / backward-drift
behavior. Three escalations to apply, in order of effort:

### 1. Send a position-hold setpoint instead of velocity-zero (recommended)

Velocity-zero with `VEL_IGNORE` *cleared* tells the PSC "track 0 m/s,"
which keeps the velocity integrator alive trying to cancel residual
motion. A position-hold setpoint, with the velocity-ignore bits *set*,
tells ArduSub "switch loop class — position controller, hold here,"
which internally resets the velocity integrator on setpoint-class
transition.

The minimal change is a second `set_position_target_local_ned_send` in
the watchdog with this mask:

```
type_mask =
      POSITION_TARGET_TYPEMASK_VX_IGNORE
    | POSITION_TARGET_TYPEMASK_VY_IGNORE
    | POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | POSITION_TARGET_TYPEMASK_AX_IGNORE
    | POSITION_TARGET_TYPEMASK_AY_IGNORE
    | POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
```

with `x=y=z=0` in `MAV_FRAME_BODY_OFFSET_NED` (relative to current
position) — i.e. "stop here." This is the lowest-risk change that
actually breaks the integrator wind-down.

### 2. Send the position-hold *immediately on mode exit*, not just on cmd_vel timeout

The watchdog currently checks `pixhawk_mode != "GUIDED"` and clears
`_cmd_vel_was_active` without sending anything
([ros2_receiver.py:919-921](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L919-L921)).
That's the source of the "backward drift after disarm" — when the
operator switches out of GUIDED or disarms, the PSC integrator state is
still hot, and whichever mode the sub falls into inherits the residual.

Add a one-shot position-hold send on the `pixhawk_mode != "GUIDED"`
branch, before clearing the flag. Same mask as above.

### 3. Set `MOT_SAFE_DISARM=1` (firmware param, not code)

Currently 0 — on disarm, motors hold their last PWM until the ESC times
out. Setting to 1 forces immediate idle. This is independent of the
watchdog but addresses the same "backward drift" symptom.

### What the watchdog should *not* do

- Don't send a `MAV_CMD_DO_REPOSITION` or change mode. ArduSub's
  velocity-target stale handling is well-tested; we just need to give it
  a clean setpoint-class transition.
- Don't reduce the 0.3 s timeout. Lower than that and a legitimate
  publisher hiccup (TCP retransmit, swap pause) triggers a needless
  position-hold.
- Don't try to reset the PSC integrator with `PARAM_SET` — there's no
  parameter for it, only the setpoint-class transition path works.

## Sub-param changes — what to copy from SITL `sub.parm`

The SITL `sub.parm` is the trusted reference: same firmware tree, same
custom frame, but every test passes. Strategy: **paste the SITL
overrides verbatim** where the same value applies, then add the small
set of params SITL leaves at default but the real vehicle has been
hand-edited away from defaults.

### Tier 1 — paste verbatim from SITL `sub.parm`

These are settings SITL explicitly sets where the real vehicle has
either drifted off or never matched. Copy as-is, no thinking required.
Five rows in **bold** are the high-value deltas — those are what
distinguishes SITL from real PSC behaviour. Everything else is already
aligned.

| Param            | SITL value | Real value | Notes                                 |
|------------------|-----------:|-----------:|---------------------------------------|
| `PSC_POSXY_P`    | 2.5        | 1.0        | XY position-loop responsiveness       |
| `PSC_VELXY_P`    | 5.0        | 6.0        |                                       |
| `PSC_VELXY_I`    | 0.5        | 0.5        | already matches                       |
| **`PSC_VELXY_D`**| **0.8**    | **0.0**    | **damps the wind-up oscillation**     |
| `PSC_VELZ_P`     | 8          | 6.0        |                                       |
| `PSC_POSZ_P`     | 3          | 3          | already matches                       |
| `PSC_ACCZ_P`     | 2          | 1.2        |                                       |
| **`PSC_ACCZ_I`** | **4**      | **0**      | **without this, depth has SS error**  |
| **`PSC_ACCZ_FF`**| **0.75**   | **0**      | **without FF, depth tracks badly**    |
| `GPS_TYPE`       | 0          | 0          | already matches                       |
| `VISO_TYPE`      | 1          | 1          | already matches                       |
| `EK3_SRC1_POSXY` | 6          | 6          | already matches                       |
| `EK3_SRC1_VELXY` | 6          | 6          | already matches                       |
| `EK3_SRC1_POSZ`  | 6          | 6          | already matches                       |
| `EK3_SRC1_YAW`   | 6          | 6          | already matches                       |
| `COMPASS_USE`    | 0          | 0          | already matches (`COMPASS_ENABLE=0`)  |
| `COMPASS_USE2`   | 0          | 0          | already matches                       |
| `COMPASS_USE3`   | 0          | 0          | already matches                       |
| `FRAME_CONFIG`   | 7          | 7          | already matches (custom mixer)        |
| `SERIAL1_PROTOCOL` | 2        | 2          | already matches (MAVLink)             |

### Tier 2 — copy concept, change value

SITL sets these one way; real hardware needs a different value because
the underlying physical thing differs.

| Param            | SITL value | Real value | Set to | Reason                                                            |
|------------------|-----------:|-----------:|-------:|-------------------------------------------------------------------|
| `EK3_SRC1_VELZ`  | 0          | 6          | 0      | SITL uses no external VELZ; on real, noisy DVL VELZ corrupts depth. **Try 0 first; revert if depth degrades.** |
| `BARO_EXT_BUS`   | 0          | 1          | 1      | Real has external Bar30; SITL doesn't. Leave real at 1.            |
| `PSC_VELZ_I`     | (default 0.5) | 0.75    | 0.5    | SITL inherits default; real's 0.75 is too hot.                     |
| `PSC_VELXY_FF`   | (default 0)   | 0.2     | 0.0    | SITL inherits default; real's 0.2 unnecessarily biases the loop.   |
| `PSC_VELXY_IMAX` | (default 1000)| 1000    | 50     | Both inherit silly default; cap aggressively to bound wind-up.     |

### Tier 3 — params SITL doesn't touch but the real vehicle has been gutted

These don't appear in SITL `sub.parm` at all (so SITL runs ArduSub
defaults and works fine), but on the real vehicle they've been
hand-edited to zero or near-zero. Restore to either ArduSub defaults
or the values below.

| Param              | Real value | Set to | Reason                                              |
|--------------------|-----------:|-------:|-----------------------------------------------------|
| `ATC_RAT_YAW_P`    | 0.08       | 0.18   | ArduSub default for ROVs                            |
| `ATC_RAT_YAW_I`    | 0          | 0.018  | Closes steady-state yaw error                       |
| `ATC_RAT_YAW_D`    | 0          | 0      | Keep 0 unless overshoot appears                     |
| `ATC_RAT_YAW_FF`   | 0          | 0.10   | Eliminates "stop, then start" yaw chatter           |
| `ATC_RAT_YAW_IMAX` | 0.222      | 0.222  | Already correct                                     |
| `ATC_RAT_PIT_I`    | 0          | 0.05   | Compensates buoyancy/CG bias steady-state           |
| `ATC_RAT_PIT_IMAX` | 0          | 0.222  | Lets the pitch integrator actually accumulate       |
| `ATC_ANG_YAW_P`    | 0          | 0      | Intentional — pure rate control, no heading hold    |
| `ATC_SLEW_YAW`     | 6000       | 200    | 6000 deg/s² is step-territory; 200 is a real ramp   |
| `ATC_RATE_Y_MAX`   | 180        | 90     | Realistic max yaw rate for this hull                |
| `MOT_SAFE_DISARM`  | 0          | 1      | Stops the "drift after disarm"                      |

### Tier 4 — DO NOT copy from SITL `sub.parm`

These appear in SITL but are either vehicle-specific calibrations or
sim-only flags. **Never paste these to the real vehicle:**

- `INS_ACC*OFFS_*`, `INS_ACC*SCAL_*`, `INS_GYR*OFFS_*` — accel and
  gyro calibration. The real autopilot has its own measured offsets;
  SITL's are placeholders. Overwriting destroys the real calibration.
- `COMPASS_OFS_*`, `COMPASS_DIA_*`, `COMPASS_ODI_*` — compass
  calibration. Same reason. Compass is also disabled (`COMPASS_USE*=0`)
  so the values don't matter, but don't overwrite them.
- `BARO*_GND_PRESS` — ground-pressure reference, learned at boot. Do
  not copy.
- `BTN*_FUNCTION`, `BTN*_SFUNCTION` — joystick button mappings;
  irrelevant for autonomous control.
- `RNGFND1_*` (TYPE/PIN/SCALING/MAX_CM) — SITL configures a simulated
  rangefinder. Real has different sensors (Ping sonar, ultrasonic)
  configured elsewhere.
- `MNT_*` — gimbal mount, sim-only.
- `SIM_*` — simulator-only parameters.
- `AHRS_ORIENTATION` — see below.

### `AHRS_ORIENTATION` — leave alone unless the heading sanity check fails

`AHRS_ORIENTATION = 2` is **`ROTATION_YAW_90`** (autopilot mounted
rotated 90° about Z), not `YAW_180`. SITL runs with default `0`
because there is no "mounting" to compensate for. **Do not copy
SITL's value here.**

But: the surge inversion suggests AHRS body and motor body might be
180° apart in yaw despite this param looking right. That happens if
the autopilot is physically rotated **−90°** (= 270°) but the param
says **+90°** — net error 180°. The fix is `AHRS_ORIENTATION = 6`
(`ROTATION_YAW_270`).

#### Why heave (Z) is unaffected by all this

Vertical thrust is gravity-aligned. `AHRS_ORIENTATION` values 0, 2,
4, 6 are all rotations about the body-z axis, and z-axis rotations
preserve the z component of any vector or angular velocity. So:

- **Heave (vz)**: along z → invariant under any yaw rotation → works.
- **Yaw rate (ωz)**: rotation about z → invariant under any yaw rotation → not affected by `AHRS_ORIENTATION` *value*, only by the mixer.
- **Surge (vx) / sway (vy)**: in the horizontal plane → directly sensitive to which way "forward" points → broken if AHRS body is rotated relative to motor body.

That's why **only X and yaw fail and Z works**: Z is gravity-locked,
the others ride on yaw alignment.

#### What changing `AHRS_ORIENTATION` actually does

Surge and yaw don't move together — flipping the param fixes one
group of bugs and leaves the other alone, depending on the bridge
code state.

| Step                                                         | `AHRS_ORIENTATION` | Bridge `surge =`     | Bridge `yaw_rate =`  | Surge      | Yaw                                |
|--------------------------------------------------------------|--------------------:|----------------------|----------------------|------------|------------------------------------|
| **Now (broken yaw, "working" surge via two-wrongs)**         | 2 (YAW_90)          | `-msg.linear.x`      | `-msg.angular.z`     | forward ✓  | wrong direction ✗                  |
| Just flip AHRS                                               | 6 (YAW_270)         | `-msg.linear.x`      | `-msg.angular.z`     | backward ✗ | still wrong ✗                      |
| Flip AHRS + remove surge negation                            | 6 (YAW_270)         | `+msg.linear.x`      | `-msg.angular.z`     | forward ✓  | still wrong ✗                      |
| ... + remove yaw negation too (two-wrongs for yaw)           | 6 (YAW_270)         | `+msg.linear.x`      | `+msg.angular.z`     | forward ✓  | correct ✓ (firmware bug still there) |
| Clean: AHRS + bridge fixes + reflash mixer                   | 6 (YAW_270)         | `+msg.linear.x`      | `+msg.angular.z`     | forward ✓  | correct ✓                          |

Yaw is immune to `AHRS_ORIENTATION` — it's a rotation about z, and
all four candidate values (0/2/4/6) are also rotations about z. The
yaw fault is in the firmware/wiring, not in the AHRS layer.

#### Heading sanity check before you flip

1. Park the sub on the bench pointing physically north
   (compass-confirmed).
2. Read `ATTITUDE.yaw` from MissionPlanner (radians, NED, CW from
   north).
3. Read `/filter/euler.z` from Foxglove (radians, ENU, CCW from
   east).

They should satisfy `ATTITUDE.yaw ≈ π/2 − filter.euler.z` (mod 2π).

- Off by ~π → AHRS is 180° wrong → flip `AHRS_ORIENTATION` 2 → 6.
- Off by ~π/2 → autopilot rotated by 90° in a different direction
  than YAW_90 expects → try `AHRS_ORIENTATION = 0` or `4`.
- Matches → AHRS layer is fine; the surge inversion is elsewhere
  (look at MOT_1 wiring per the section below).

## Reading the custom mixer + `MOT_n_DIRECTION` pairing

Each motor's physical thrust is computed by ArduSub as:

```
mixer_output[i] = roll·R + pitch·P + yaw·Y + throt·T + fwd·F + lat·L
                 (factors come from SUB_FRAME_CUSTOM in AP_Motors6DOF.cpp)
final_pwm[i]   = mixer_output[i] × MOT_i_DIRECTION
```

**`MOT_n_DIRECTION` flips the final output of the whole motor**, not a
single axis. Setting `MOT_2_DIRECTION = −1` simultaneously inverts
that motor's contribution to forward, sway, yaw, throttle, roll, and
pitch. It's a polarity switch on the whole motor — equivalent to
swapping two of the three ESC phase wires.

Cross-referencing your custom mixer against your `MOT_n_DIRECTION`
params:

| Motor | Factors (R,P,Y,T,F,L)   | DIR  | Active axes          |
|-------|-------------------------|-----:|----------------------|
| MOT_1 | 0, 0, 0, 0, **+1**, 0   | **−1** | surge only           |
| MOT_2 | 0, 0, **+0.775**, 0, 0, **−1** | +1   | **yaw + sway**       |
| MOT_3 | **+1**, −0.833, 0, 0.6, 0, 0 | +1 | roll + pitch + heave |
| MOT_4 | **−1**, −0.833, 0, 0.6, 0, 0 | **−1** | roll + pitch + heave |
| MOT_5 | 0, **+1**, 0, **+1**, 0, 0 | +1  | pitch + heave        |
| MOT_6 | 0, 0, **−1**, 0, 0, **−1** | **−1** | **yaw + sway**       |

Motors with non-zero **yaw_factor**: MOT_2, MOT_6.
Motors with **DIRECTION = −1**: MOT_1, MOT_4, MOT_6.
**Overlap: only MOT_6.** They don't line up, so DIRECTION flips on the
yaw motors won't isolate "yaw only" — they'll change sway at the same
time. You cannot fix just yaw via DIRECTION param edits.

### When DIRECTION is enough, vs when you must reflash

Decision tree, based on a one-shot bench sway test:

| Bench observation                        | Cause                                              | Cheapest fix                          |
|------------------------------------------|----------------------------------------------------|---------------------------------------|
| Yaw wrong **and** sway also wrong (both inverted on MOT_2 + MOT_6) | Both motors wired backwards / props inverted; mixer fine | `param set MOT_2_DIRECTION -1`<br>`param set MOT_6_DIRECTION 1`<br>**No reflash, just reboot.** |
| Yaw wrong, sway correct                  | Mixer's yaw column signs are wrong; sway signs happen to be right | Reflash with the mixer fix in the firmware section |
| Yaw correct on one of {MOT_2, MOT_6} but wrong on the other | One specific motor is individually mis-wired         | Flip only that motor's DIRECTION. Reboot.       |
| Yaw correct, sway wrong                  | Mixer's lateral column signs are wrong              | Reflash, fix lat factors                       |

Right now your bridge always sends `vy = 0`, so the sway axis has
never been exercised on hardware. **You can't tell which row of the
table you're in** without running a sway test first.

### One-shot sway test

Bench, props off, GUIDED, bridge running normally:

```bash
./scripts/cmd_vel_ramp.py ramp sway 0.20 --ramp 4 --hold 4
```

Watch which way the chassis tries to roll/translate (or read
`SERVO_OUTPUT_RAW` for MOT_2 and MOT_6). If the command is "go right"
and the response is "go left", sway is inverted → top row of the
table → flip both DIRECTIONs and reboot.

### `MOT_n_DIRECTION` is a parameter — no reflash needed

```text
param set MOT_2_DIRECTION -1
param set MOT_6_DIRECTION  1
param fetch
reboot
```

`reboot` is required for `MOT_*` params to apply (ArduPilot only
re-runs motor-mixer setup at boot). That's 5–10 seconds vs the
firmware reflash + parameter re-fetch loop.

### Predict thrust directions before flipping anything

Before changing any DIRECTION, run individual motor tests with props
off to see which way each one spins at PWM > 1500 (its "default"
direction). From MAVProxy:

```text
motortest 1 1 1500 5      # MOT_1, neutral PWM, 5 s
motortest 1 1 1700 2      # MOT_1, +200 µs above neutral, 2 s
motortest 1 1 1300 2      # MOT_1, −200 µs below neutral, 2 s
```

(Repeat with motor index 2..6.) Note which physical direction the prop
pushes at "above neutral." Combined with the factor signs in the
mixer table above, you can predict every axis's response without
running a single GUIDED command.

### Bulk-apply via MAVProxy

Tier 1 + 2 + 3 as a single block. Paste each line into the MAVProxy
console, or save to a `.parm` file and use `param load`:

```text
# Tier 1 — paste from SITL sub.parm (high-value deltas only)
param set PSC_POSXY_P    2.5
param set PSC_VELXY_P    5.0
param set PSC_VELXY_D    0.8
param set PSC_VELZ_P     8
param set PSC_ACCZ_P     2
param set PSC_ACCZ_I     4
param set PSC_ACCZ_FF    0.75

# Tier 2 — concept reused, value adjusted for hardware
param set EK3_SRC1_VELZ  0
param set PSC_VELZ_I     0.5
param set PSC_VELXY_FF   0.0
param set PSC_VELXY_IMAX 50

# Tier 3 — restore ArduSub defaults that the vehicle has been edited away from
param set ATC_RAT_YAW_P    0.18
param set ATC_RAT_YAW_I    0.018
param set ATC_RAT_YAW_FF   0.10
param set ATC_RAT_PIT_I    0.05
param set ATC_RAT_PIT_IMAX 0.222
param set ATC_SLEW_YAW     200
param set ATC_RATE_Y_MAX   90
param set MOT_SAFE_DISARM  1
```

Reboot the FC after applying. Verify each value with
`param show <NAME>` before flying.

## Firmware fix — custom 6DOF mixer yaw column

[AP_Motors6DOF.cpp:181-186](../project-polaris-ardusub/libraries/AP_Motors/AP_Motors6DOF.cpp#L181-L186),
`SUB_FRAME_CUSTOM`. Current:

```cpp
add_motor_raw_6dof(AP_MOTORS_MOT_2, 0, 0,  0.775, 0,  0, -1.0f, 2);
add_motor_raw_6dof(AP_MOTORS_MOT_6, 0, 0, -1.0f,  0,  0, -1.0f, 6);
```

Both yaw factors should be flipped:

```cpp
add_motor_raw_6dof(AP_MOTORS_MOT_2, 0, 0, -0.775, 0,  0, -1.0f, 2);
add_motor_raw_6dof(AP_MOTORS_MOT_6, 0, 0,  1.0f,  0,  0, -1.0f, 6);
```

Reasoning: ArduSub's mixer convention is `positive yaw_factor → positive
FRD yaw torque → CW from above`. The current factors produce CCW for
positive yaw input (verified by tracing thrust direction at MOT_2 (rear)
and MOT_6 (front) given lat=-1 and the resulting moment about the down
axis). The bridge currently masks this by negating `angular.z` in
`cmd_vel_cb`; once the firmware is fixed, that negation must also be
removed.

## Bridge code change

### Yaw negation — remove after firmware mixer fix

After reflashing with the yaw mixer fix above, the bridge's
[ros2_receiver.py:877](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L877)
`yaw_rate = -msg.angular.z` becomes `yaw_rate = +msg.angular.z`. That
preserves the spec-correct FLU→FRD inversion (which the bridge
performs) and removes the second inversion that was masking the mixer
bug.

### Surge negation — only after the bench test

[ros2_receiver.py:875](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L875)
currently has `surge = -msg.linear.x`. **Don't touch this until the
single-axis bench test under "Surge: targeted verification" identifies
where the inversion lives.** Removing the negation prematurely, with
the underlying firmware/wiring bug still present, will only swap the
direction of the failure, not fix it.

Once that test pins down whether the issue is `MOT_1_DIRECTION`, a
spurious mixer entry, or something else, fix it at the source and
*then* drop the bridge negation:

```python
surge    = +float(msg.linear.x)
heave    = -float(msg.linear.z)   # FRD-spec correct, leave as -msg.linear.z
yaw_rate = +float(msg.angular.z)  # after firmware mixer fix
```

`heave = -msg.linear.z` stays — that's the legitimate FLU→FRD z-axis
flip per MAVLink spec.

The `type_mask` at
[ros2_receiver.py:882-891](src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L882-L891)
stays unchanged. Do not add conditional `YAW_RATE_IGNORE`.

## Verification order

1. **Apply parameter set above** via MissionPlanner / `param set` /
   `mavproxy.py`. Reboot the FC. (No `AHRS_ORIENTATION` change — it
   matches the mounting.)
2. **Bench surge test** with props off, GUIDED, bridge temporarily
   patched to `surge = +msg.linear.x`. Read all six
   `SERVO_OUTPUT_RAW.servoN_raw` values for a forward command. Pin
   down whether the inversion is `MOT_1_DIRECTION`, a spurious
   mixer entry, or something else (see "Surge: targeted verification"
   above).
3. **Apply the fix** (param flip or mixer reflash) identified by step 2.
4. **Reflash firmware** with the mixer yaw column fix per the
   firmware-fix section.
5. **Apply bridge code change** (remove both negations).
6. **In-water test**: forward, lateral hold, yaw left, yaw right, all
   in GUIDED. Verify each direction matches ROS REP-103 sign
   expectations.
7. **Watchdog test**: send 5 s of forward, then `Ctrl+C` the publisher.
   Sub should stop within 0.5 s with no backward overshoot. Plot
   `PIDN.I`/`PIDE.I` from the dataflash log — should decay to 0 within
   that window.

## Running test profiles — `scripts/cmd_vel_ramp.py`

`ros2 topic pub` only republishes a fixed message; it can't ramp or
shape inputs. [scripts/cmd_vel_ramp.py](scripts/cmd_vel_ramp.py)
publishes `geometry_msgs/Twist` to `/pixhawk/cmd_vel` at the bridge's
20 Hz expectation, with the value moving over time. Same wire format
as a `ros2 topic pub`, just shaped.

### Setup once per terminal

```bash
source /opt/ros/humble/setup.bash
source /home/polaris_pz/project-polaris/install/setup.bash
cd /home/polaris_pz/project-polaris
```

If you run the rest of the stack inside the dev container, source the
container's environment instead (`./develop.sh` / `./entrypoint.sh`
shell, then the same two `source` lines above) so DDS discovery on
`ROS_DOMAIN_ID` finds the `mavlink_bridge` node. Verify with:

```bash
ros2 topic info /pixhawk/cmd_vel    # must report Type: geometry_msgs/msg/Twist
ros2 node list | grep mavlink       # must show mavlink_bridge_receiver
```

### Profiles

| Profile     | Shape                                           | When to use                                                  |
|-------------|-------------------------------------------------|--------------------------------------------------------------|
| `step`      | 0 → target, hold, target → 0                    | Like `ros2 topic pub` but with auto-stop. Worst case for PSC integrator. |
| `ramp`      | linear up `--ramp` s, hold `--hold` s, linear down | Smooth target. Lets you measure tracking lag without integrator wind-up. |
| `triangle`  | linear up, immediate linear down (no flat top)  | "Did it move at all?" test. No integrator saturation. Cleanest sign-check. |
| `bipolar`   | +target leg → 0 → −target leg → 0               | Catches asymmetric thrust (e.g. `MOT_n_DIRECTION` mismatches). |
| `staircase` | 25 % → 50 % → 75 % → 100 % of target, then 0    | Sweeps amplitude in one run. Finds motor deadband and characterises tracking vs amplitude. |

### Examples (run from the project root)

```bash
# Pure step — same as the original ros2 topic pub command, with auto-stop
./scripts/cmd_vel_ramp.py step surge 0.35 --hold 6

# Trapezoid — 0 → 0.20 m/s over 5 s, hold 5 s, back to 0 over 5 s
./scripts/cmd_vel_ramp.py ramp surge 0.20 --ramp 5 --hold 5

# Triangle — never holds; minimal integrator excitation
./scripts/cmd_vel_ramp.py triangle surge 0.20 --ramp 4

# Bipolar — forward then backward at same magnitude
./scripts/cmd_vel_ramp.py bipolar surge 0.20 --ramp 3 --hold 2

# Staircase — 0.075 → 0.15 → 0.225 → 0.30 m/s, each held 3 s
./scripts/cmd_vel_ramp.py staircase surge 0.30 --hold 3

# Yaw rate ramp — exposes ATC_RAT_YAW_* tracking quality vs chatter
./scripts/cmd_vel_ramp.py ramp yaw 0.30 --ramp 4 --hold 4

# Heave (depth velocity) — verify FRD-spec sign and PSC_VELZ tracking
./scripts/cmd_vel_ramp.py ramp heave 0.10 --ramp 4 --hold 4
```

### Recording while testing

In a separate terminal, before running a profile:

```bash
source /opt/ros/humble/setup.bash
source /home/polaris_pz/project-polaris/install/setup.bash
ros2 bag record -s mcap -o ~/Downloads/Rosbags/$(date +%Y%m%d_%H%M%S)_test \
  /pixhawk/cmd_vel /odometry/filtered/local /sensors/dvl/odometry \
  /imu/angular_velocity /filter/euler /pixhawk/attitude_quaternion \
  /pixhawk/heartbeat /pixhawk/servo_output_raw
```

Those eight topics are enough to reconstruct every plot in this
playbook (cmd vs response, sign chain, mode/arm state, motor outputs).

### Tail behavior — what happens when the profile ends

The script sends 10 explicit zero messages after the profile completes
(configurable with `--tail-zeros`). That's a clean stop signal so you
can see the watchdog wind-down separately from ArduSub's ~3 s
`GUID_TIMEOUT` path. Set `--tail-zeros 0` if you want to test what
happens when the publisher just goes silent (i.e. the watchdog
itself).

### Safety

- Always test new profiles **on the bench with props off** before
  running them in water.
- The mode must be GUIDED and the vehicle armed for the bridge to
  forward setpoints; the script does not arm or change mode itself.
- `Ctrl+C` aborts the script. The bridge will then send a zero-velocity
  setpoint within 0.3 s via the watchdog (assuming GUIDED is still
  active).

## Bag-confirmed observations from the 2026-05-06 trials

From `y_rat_0_30_2026_05_06-14_42_29_0.mcap` and
`x_vel_0_20_2026_05_06-14_26_13_0.mcap`:

- **Yaw inversion confirmed at the firmware-mixer level.** ROS
  commanded +0.30 rad/s, sub physically rotated CW (negative ENU yaw
  rate on `/imu` and `/odometry/filtered/local`). Bridge negation +
  spec-correct BODY_FRD interpretation = ArduSub PSC drove the
  attitude controller toward CCW; mixer inverted that to CW.
- **Yaw chatter visible.** `/imu/angular_velocity.z` rings ±0.20 rad/s
  during what should be steady yaw. Matches the `ATC_RAT_YAW_*`
  diagnosis above.
- **PSC overshoot visible.** With a constant +0.20 m/s surge command,
  `/odometry/filtered/local.twist.linear.x` swings −0.3 to +0.6 m/s.
  Confirms the velocity-loop integrator wind-up.
- **ROS-side state is correct.** No 180° yaw offset between the EKF
  output and physical reality — so the surge inversion is *not*
  upstream of `cmd_vel_cb`.

## Still useful next session — the dataflash `.bin` log

The rosbag answered the ROS-side questions. The remaining open
question is what the FC's EKF is actually doing: whether
`EK3_SRC1_YAW=6` is being fused or silently rejected. That requires
the FC's `LOG/n.bin` from the same run — check `XKF1.Yaw`,
`XKF3.IYaw` (yaw innovation), and the `MSG` channel for "EKF3 IMU0
fusing external nav". If innovations are huge or the message says
"rejected," the FC EKF isn't using the external yaw and is dead-
reckoning from gyro alone — which would change the picture for both
the surge and yaw fixes.
