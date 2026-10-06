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

#### 2.1.3 Roll free decay

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