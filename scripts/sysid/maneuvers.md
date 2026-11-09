# System identification — maneuver catalogue

Offline identification of the POLARIS dynamic model from rosbag data. This file defines the
experiments; the fitting code lives next to it in `scripts/sysid/`.

## 1. Model and scope

Starting point is Fossen's 6-DOF equation, with `nu_r = nu - nu_c` (velocity relative to water):

```
(M_RB + M_A) * d(nu_r)/dt + (C_RB(nu) + C_A(nu_r)) * nu_r + D(nu_r) * nu_r + g(eta) = tau_thrust + tau_dist
```

- `tau_thrust`: from thruster commands through the mixer and the thruster curve (not measured directly).
- `tau_dist`: current (via `nu_c`), tether, waves (surface only). Not modelled; absorbed by nuisance terms.
- Start with **surge only** (1 DOF), then yaw, then sway/heave. Coupling is checked afterwards.
- Command path: `/pixhawk/manual_control` (`Int16MultiArray`, 6 values, surge in [-1000, 1000]) in
  **MANUAL** mode. Open loop through the ArduSub mixer. Do **not** use `/pixhawk/cmd_vel`: that is a
  GUIDED velocity setpoint and puts ArduSub's controller inside the loop.

### What can be commanded separately

All six axes are independently commandable in one message, `data = [x, y, z, r, s, t]`:

| index | axis | range | neutral |
|---|---|---|---|
| 0 | `x` surge  | -1000 … 1000 | 0 |
| 1 | `y` sway   | -1000 … 1000 | 0 |
| 2 | `z` heave  | 0 … 1000     | 500 |
| 3 | `r` yaw    | -1000 … 1000 | 0 |
| 4 | `s` roll   | -1000 … 1000 | 0 |
| 5 | `t` pitch  | -1000 … 1000 | 0 |

In practice commanded values are capped at +-500.

The vehicle is **symmetric about the xz plane** (port-starboard mirror symmetry). For a
vehicle with that symmetry the 6-DOF system splits into two groups that are dynamically
decoupled to first order:

- **longitudinal:** surge, heave, pitch
- **lateral:** sway, roll, yaw

The cross terms between the two groups vanish under this symmetry, which is what makes the
one-axis-at-a-time plan in section 2 valid: exciting surge alone does not feed sway, roll or
yaw, so each group can be identified from its own maneuvers without solving the full coupled
system. Coupling that survives the symmetry (mainly surge-yaw, section 2.6) still has to be
checked separately.


## 2. Commanded maneuvers to excite terms

Each maneuver is applied per axis where relevant (surge, yaw, then sway/heave). Locations are
proposed and still to be confirmed. Script names are placeholders until the scripts exist.

### 2.1 Static tests — Location: Pool

First tests to run. No commanded profile (manual displacement or a known applied moment), and they
fix the restoring terms that the dynamic runs rely on. Yaw has no hydrostatic restoring, so it
does not oscillate; yaw is covered by the dynamic maneuvers below.

#### 2.1.1 Static tilt

- **Identifies:** hydrostatic restoring stiffness, from the tilt angle produced by a known applied moment.
- **Script:** fit `fit_static_tilt.py` (placeholder)
- **Relevant rosbags:** _to be added_
- **Results:** _to be added_

#### 2.1.2 Pitch free decay

- **Identifies:** pitch added inertia and pitch damping. Displace in pitch and release; with the
  stiffness known from 2.1.1, the oscillation frequency gives the added inertia and the rate of decay
  gives the damping. Amplitude dependence of the decay indicates linear versus quadratic damping.
- **Script:** fit `fit_free_decay.py` (placeholder)
- **Relevant rosbags:** _to be added_
- **Results:** _to be added_

#### 2.1.3 Roll free decay (not needed)

- **Not needed while roll is not modelled** (decision 2026-10-15, see the comment at the top of
  `model.yaml`). POLARIS is passively roll-stable and the EKF roll is used wherever the angle
  appears. Kept here in case roll is re-enabled.
- **Identifies:** roll added inertia and roll damping, same method as 2.1.2 about the roll axis.
- **Script:** fit `fit_free_decay.py` (placeholder)
- **Relevant rosbags:** _to be added_
- **Results:** _to be added_

### 2.2 Step from rest — Location: TBD (proposed: lake, surface)

- **Identifies:** mass plus added mass. Drag vanishes at zero velocity, so the initial acceleration
  gives the inertia.
- **Script:** maneuver player `sysid_maneuvers.py` (placeholder), fit `fit_step.py` (placeholder)
- **Relevant rosbags:** _to be added_
- **Results:** _to be added_

### 2.3 Multi-level steps — Location: TBD (proposed: lake, surface)

- **Identifies:** the damping curve. Steady-state velocity at four to five thrust levels separates
  linear from quadratic damping.
- **Script:** maneuver player `sysid_maneuvers.py` (placeholder), fit `fit_damping.py` (placeholder)
- **Relevant rosbags:** _to be added_
- **Results:** _to be added_

### 2.4 Coast-down — Location: TBD (proposed: lake, surface)

- **Identifies:** damping, independently of the thrust curve calibration. With thrust cut from steady
  speed there is no thrust term in the force balance, and the decay gives the drag.
- **Script:** maneuver player `sysid_maneuvers.py` (placeholder), fit `fit_coastdown.py` (placeholder)
- **Relevant rosbags:** _to be added_
- **Results:** _to be added_

### 2.5 Forced oscillation — Location: TBD (proposed: lake, surface)

- **Identifies:** added mass and damping together, from the amplitude and phase of the response to a
  sinusoidal command swept in frequency, over a short travel distance.
- **Script:** maneuver player `sysid_maneuvers.py` (placeholder), fit `fit_oscillation.py` (placeholder)
- **Relevant rosbags:** _to be added_
- **Results:** _to be added_

### 2.6 Combined axes — Location: TBD (proposed: lake, surface)

- **Identifies:** the coupling terms (surge and yaw commanded together) that mission profiles do not produce.
- **Script:** maneuver player `sysid_maneuvers.py` (placeholder), fit `fit_coupling.py` (placeholder)
- **Relevant rosbags:** _to be added_
- **Results:** _to be added_


## 3. Validation
The following script plots and comapares a propagation of a identified model against given mcap rosbag runs. 
Hereby we read the relevant mcap file to synchronize timestamps etc, then the model is integrated forward and the relevant estimates are publsihed to /sysid/odom (state estimate), and to /sysid/model (to check the version of the model used). 

### Reading the results: free-run vs re-init

Use `--reinit-s 0` (free rollout over the whole bag) to judge whether the model is actually
right — a wrong thrust scale, drag or mass shows up as steadily growing error and nothing
hides it. Short re-init periods measure something narrower (how well the model predicts N
seconds ahead from a known state), and they keep the EKF reference trustworthy, since the
reference drifts too over long horizons.

Keep the re-init runs as a **diagnostic**, not the verdict: re-initialising gives many
independent samples instead of one, and the shape of the error-vs-horizon curve says which
parameter is wrong.

- error growing **linearly** with horizon → velocity bias → thrust scale or drag is off
- error growing **quadratically** → acceleration error → mass / added mass is off

Caveat: a single free rollout is one realisation, so a current or a bad patch of DVL can
dominate it. Compare a few bags rather than trusting one long number.



## 4. Important Points

The T500 thrust curves are from the T500 documentation. Interpolation was used between the curves at voltages 12, 14, 16 and 18V. The force reconstruction hence uses both pwm and the current voltage readings. 

How the model is stepped forwared in time: Runge Kutta 4th order method. Here instead of using a method like euler forward u_next = u + dt*f(u), we rather use the rk4 function that projects forward according to the following schema:
k1 = f(u)                    # slope at the start
k2 = f(u + dt/2 * k1)        # slope at the midpoint, estimated using k1
k3 = f(u + dt/2 * k2)        # slope at the midpoint again, estimated using k2
k4 = f(u + dt * k3)          # slope at the end, estimated using k3
u_next = u + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

There is no exact discretization because we dont have a linear model available, hence the matrix exponential cannot be computed. 


TODO: Currently the model.yaml has a allocation section with the lever arms from the custom_frame_helper.py, these lever arms will naturally change as we update the pixhawk mixer as well for the new geometry. 

base_link FLU is used as frame convention for the Fossen EOM moving forward. 

Plan is to use a per axis approach to allow for proper error attribution. So apart from the one axis that we intend to test, we give the model the remaining quantities. Thereafter we fit on a certain set of runs, and leave another set of holdouts as validation. For each axis we can fit using the different maneuvres. 

For coupling we can command two axes together and check if the per axis models still hold well. if so, then there is little coupling and the symmetry argument holds. if there is a systematic resutal, then we can fit a correction while leaving hte per axis numbers as they are.


### Delays to measure before fitting added mass

Added mass is read off the *initial acceleration* of a step, so anything that delays or smooths
the first second of the response biases it — and always in the same direction: a response that
looks slower than it was makes the vehicle look heavier, i.e. **over-estimates added mass**.
Drag is unaffected, because it comes from steady state or from a decay over many seconds, where
a few hundred ms is negligible. The bias only matters when a delay is comparable to the physical
time constant being measured.

Reference timescales (measured on `aut_no_z_surface_big_40_grid_03`):

| | period | notes |
|---|---|---|
| surge time constant `m_tot / X_u` | **~1.4 s** | the thing being measured (dummy values; real value TBD) |
| EKF `/odometry/filtered/local` | 33 ms (30 Hz) | ~44 samples per time constant |
| DVL `/sensors/dvl/odometry_cov` | 132 ms (7.6 Hz) | ~11 samples per time constant - adequate |
| DVL `/sensors/dvl/velocity` | 230 ms (4.3 Hz) | |
| `servo_output_raw` | 100 ms (10 Hz) | 2 Hz in the older rectangle bags |

So sample rate is not the limiting factor. The delays are.

There are two *different* delays, and they need two different measurements:

1. **Actuation + transport delay — compare `servo_output_raw` against the IMU.** This is the lag
   from the PWM being logged to the vehicle actually moving. It mixes the MAVLink/bridge/
   recorder transport delay (tens of ms; note `servo_output_raw` has no header, so its MCAP
   log_time is when the Jetson recorded it, not when the Pixhawk applied it) with motor and
   propeller spin-up (order 100-300 ms). This is exactly what `thrust.servo_delay_s` exists to
   absorb, and it is still 0.0 - measuring it from a step is how it should be set.

   **Use `/imu/data` `linear_acceleration.x`, not the DVL**, because you are timing an *edge*: a
   thrust step is a near-instant STEP in acceleration but only the onset of a RAMP in velocity,
   and the start of a ramp is intrinsically fuzzy to locate. The rates compound the difference -
   IMU ~86 Hz (~12 ms resolution) against DVL 7.6 Hz (~130 ms, and soft). The signal is strong:
   ~17 N on ~20 kg is about 0.85 m/s^2, far above MEMS noise. The IMU is identity-rotated to
   base_link, so `linear_acceleration.x` is body-forward directly. The DVL is then a magnitude
   cross-check - does the velocity ramp match the integrated acceleration - rather than the
   primary instrument.

   Transport and spin-up cannot be separated without a Pixhawk-side timestamp, and they do not
   need to be: `servo_delay_s` shifts servo log_time to align with everything else, so the total
   is exactly the quantity wanted.

   Use both edges. `/sysid/event` publishes `thrust_on` and `thrust_cut`, and the cut is just as
   sharp (acceleration drops to the drag-only value), so every run yields two measurements.

   One confounder: the accelerometer measures specific force, so trim pitch projects `g*sin(theta)`
   onto x, and the vehicle pitches slightly during the step because MOT_1 sits above the CoM. That
   is a slow change arriving *after* the edge, so it does not corrupt the leading-edge timing -
   just do not use the absolute magnitude without removing the trim component.

   **Bonus: this also gives added mass more cleanly than any velocity signal.** `m - X_udot = F/a`
   is read straight off the accelerometer at the instant of the step, with drag still ~0 from rest.
   No differentiating a velocity signal and no filter in the path, so the EKF-smoothing concern in
   (2) simply does not apply to that parameter.

2. **Filter lag — compare the DVL against the EKF.** Both observe the same physical motion, so
   their difference isolates the EKF's smoothing, with actuation delay common to both and
   cancelled. Magnitude depends on the EKF tuning (process vs measurement noise), order
   100-400 ms, i.e. 10-30 % of the surge time constant.

Do this on one clean step before fitting added mass. Three outcomes: if DVL and EKF rise
together, use the EKF for everything and drop the concern; if the EKF visibly trails, fit added
mass against the DVL and keep the EKF for everything slower; if both look mushy, the step is too
slow relative to the sample rates and the fix is a sharper command, not a different sensor.

Caveat: the DVL is less lagged, not unlagged — check what internal ping averaging the A50 is
configured for before treating it as the reference on the transient. 