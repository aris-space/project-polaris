> **Status (2026-09-27, merged into `project-polaris` on `autonomy/sim`):**
> - Phase 1 is done: `FRAME_CONFIG 7` is in `src/simulation/orca_sim_bringup/cfg/sub.parm`.
> - Phase 2 (SDF thruster geometry) is **already done** in the simulation repo: `aaa8878` (2026-05-04),
>   "Added also a new thruster configuration in the model", which is on its `main` and was vendored here.
>   `generate_model.py` now defines the six Polaris thruster poses individually (`t1_*`..`t6_*`). The
>   `thruster_x` / `vert_thruster_*` constants and line numbers in the Phase 2 section below are from before
>   that commit and no longer exist. What remains is verifying the numbers — see `docs/SIM_MERGE_PLAN.md` Phase 4.
> - The hydrodynamics caveat at the end is also stale: drag uses a Polaris hull cylinder (1.78 m × 0.16 m radius)
>   and added mass is a fraction of the vehicle sheet. Both are estimates, not BlueROV2 values. The visual mesh
>   and buoyancy box are still BlueROV2-sized.
> - The fork SHA named below (`41b8a10e8d`) is stale. The pinned commit is `0a75bdf` on `polaris-pressure-filter`.
> - File paths below refer to the original simulation repo layout (`orca_bringup/`, `orca_description/` at repo root); here they live under `src/simulation/`.

# Plan: Integrate SUB_FRAME_CUSTOM (Polaris) into the Sim

## Context

This sim is a fork of Orca4 with a hardcoded BlueROV2 thruster layout in the SDF and no `FRAME_CONFIG` set in the parm file (so SITL falls back to `SUB_FRAME_VECTORED = 1`). The Polaris AUV uses a 6-thruster geometry that's nothing like BlueROV2 — surge/sway/yaw/heave are produced by completely different motor combinations. Our ArduSub fork already has a `SUB_FRAME_CUSTOM = 7` case in `AP_Motors6DOF.cpp` with the matching mixer matrix, but it has only ever run on real hardware. Goal: make the same firmware flyable in SITL+Gazebo so we can validate autonomy code against the real vehicle.

User-confirmed scope:
- **Single vehicle.** Only Polaris with the fork's ArduSub. No BlueROV2 fallback path, no `vehicle:=` launch arg, no variant flag. Edit in place.
- **Host:** Ubuntu 22 PC (native Linux). No WSL/Docker concerns for the dev machine; Docker only matters in Phase 4 for team reproducibility.
- **Inertia:** mass and inertia in [generate_model.py:52-58](../orca_description/scripts/generate_model.py#L52-L58) are already CAD/spreadsheet-derived for Polaris (per the comments). Only thruster placement at lines 119-124 is still BlueROV2.
- **Hydrodynamics:** added mass and quadratic drag stay as they are. Out of scope for this milestone.
- **Fork is stable.** Pin its current commit in the Dockerfile, like the `.apj` pin for the real vehicle.

## Reference data

- `SUB_FRAME_CUSTOM = 7` (from `AP_Motors6DOF.h` in the fork).
- Mixer matrix at `AP_Motors6DOF.cpp:179-187` (columns: roll, pitch, yaw, throttle, forward, lateral):
  - MOT_1: pure surge (forward = 1.0).
  - MOT_2: rear lateral — sway + yaw (yaw = 0.775, lateral = -1.0).
  - MOT_3: up-left vertical — heave + roll + pitch (roll = 1.0, pitch = -0.833, throttle = 0.6).
  - MOT_4: up-right vertical — mirror of MOT_3 (roll = -1.0).
  - MOT_5: up-front vertical (pitch = 1.0, throttle = 1.0).
  - MOT_6: front lateral — sway + yaw (yaw = -1.0, lateral = -1.0).
- Fork: `https://github.com/aris-space/project-polaris-ardusub.git`, branch `polaris-pressure-filter`, current HEAD `41b8a10e8d`.
- Servo defaults: `MOT_n` → `SERVOn_FUNCTION = 33 + n - 1`. Don't override.

---

## Phase 1 — Verify the firmware path

**Goal:** prove SITL is running our fork and that `FRAME_CONFIG = 7` activates `SUB_FRAME_CUSTOM`. SDF still shows BlueROV2 geometry; we only check command-to-channel wiring at this stage.

### Actions

1. **Build SITL from the fork on the Ubuntu PC.**
   ```
   cd <fork-path>
   git submodule update --init --recursive
   modules/waf/waf-light configure --board sitl
   modules/waf/waf-light build --target bin/ardusub
   ```
   Then `export ARDUPILOT_HOME=<fork-path>` before sourcing [setup.bash](../setup.bash). PATH prepend at line 19 picks up `<fork>/build/sitl/bin/ardusub`. **Fail-fast:** if the fork's submodule pins are incompatible with the build, fix it on the firmware side before continuing.

2. **Add the frame param.** Append to [orca_bringup/cfg/sub.parm](../orca_bringup/cfg/sub.parm):
   ```
   FRAME_CONFIG 7
   ```

3. **Confirm the fork is loaded.** Launch sim, watch `ardusub` stdout for the frame string. The fork sets `_frame_class_string = "CUSTOM"` in `AP_Motors6DOF.cpp:180`. Read `FRAME_CONFIG` over MAVLink (mavproxy `param show FRAME_CONFIG`) — must return 7.

4. **Per-channel sanity test (SDF still BlueROV2 — vehicle motion will be wrong, ignore it).** Arm in MANUAL. For each DOF, command via MAVLink `MANUAL_CONTROL` and watch `gz topic -e -t /model/orca4/joint/thrusterN_joint/cmd_thrust` for N=1..6:
   - Pure surge (`x=+1000`): only MOT_1 should leave neutral.
   - Pure heave (`z=+1000`): MOT_3, MOT_4, MOT_5 active.
   - Pure yaw (`r=+1000`): MOT_2 and MOT_6 active in opposite signs.
   - Pure lateral (`y=+1000`): MOT_2 and MOT_6 active in same direction.

### Verification (gate to Phase 2)
- Boot log says `Frame: CUSTOM`.
- MAVLink param read returns `FRAME_CONFIG = 7`.
- Per-DOF channel-activation pattern matches the matrix rows.

### Files touched
- [orca_bringup/cfg/sub.parm](../orca_bringup/cfg/sub.parm) — append `FRAME_CONFIG 7`.

---

## Phase 2 — Rewrite the SDF thruster geometry for Polaris

**Goal:** replace the BlueROV2 thruster layout in the existing model with Polaris geometry. No new model dir, no variant flag — edit in place.

### Blocker — needed from user before Phase 2

For each thruster MOT_1..MOT_6, from Onshape CAD (origin at vehicle CoG):
- **Position** `(x, y, z)` in body frame (meters).
- **Thrust axis unit vector** in body frame — direction of positive thrust force.
- **Body-frame convention** the CAD numbers use. ArduSub expects x-forward / y-right / z-down (NED-aligned body); Gazebo's body frame in the existing SDF appears to be x-forward / y-port / z-up, with the conversion baked into [model.sdf:355-359](../orca_description/models/orca4/model.sdf#L355-L359). Pick one and document it; do NOT modify those two transform blocks unless you've consciously decided to change conventions.

The matrix rows in `AP_Motors6DOF.cpp` already encode each thruster's contribution; the CAD positions and thrust axes must be consistent with those rows or the firmware will fight the geometry.

### Actions

1. **Update thruster placement constants.** In [orca_description/scripts/generate_model.py:119-124](../orca_description/scripts/generate_model.py#L119-L124), replace the four BlueROV2-specific values (`thruster_x`, `thruster_y`, `thruster_z`, `vert_thruster_y`, `vert_thruster_z`) with explicit per-thruster Polaris pose data — six `(pose_xyz, pose_rpy, thrust_coefficient_sign)` tuples derived from CAD. The current scheme assumes 2 vertical + 4 horizontal vectored; Polaris doesn't fit that pattern, so the cleanest approach is to define each thruster's pose individually rather than reuse symmetric constants.

2. **Update `model.sdf.in` template.** In [orca_description/models/orca4/model.sdf.in](../orca_description/models/orca4/model.sdf.in), replace each `<link name="thrusterN">` `<pose>` and the linked `<joint>` block with placeholders the new generate_model.py fills from the per-thruster tuples. Keep ArduPilotPlugin `<control channel="N">` blocks 0..5 → thruster1_joint..thruster6_joint exactly as today (matches default `MOT_n` → `SERVOn_FUNCTION = 33+n-1`). Keep `cmd_topic` paths under `/model/orca4/joint/...` (model name stays "orca4" — it's just a label internally; we don't need to rename the model dir).

3. **Encoding convention for thrust direction.** Match the existing pattern: keep all `<axis>0 0 -1</axis>` and bake direction into link `<pose>` rpy. Use `<thrust_coefficient>` sign (+0.02 / -0.02) only for propeller spin reversal, not for general direction. Avoids divergence from upstream conventions.

4. **Regenerate `model.sdf`.** Run `python3 generate_model.py model.sdf.in model.sdf 0` (existing CLI signature, no new flags). Verify Gazebo parses it: `gz sdf -k orca_description/models/orca4/model.sdf`.

### Verification (gate to Phase 3)
- Sim launches without SDF errors.
- Thruster meshes appear at the Polaris CAD positions in Gazebo's GUI.
- 6 thruster command topics still exist under `/model/orca4/joint/thrusterN_joint/cmd_thrust`.

### Files touched
- [orca_description/scripts/generate_model.py](../orca_description/scripts/generate_model.py) — replace thruster placement block at lines 119-124 with per-thruster Polaris data.
- [orca_description/models/orca4/model.sdf.in](../orca_description/models/orca4/model.sdf.in) — rewrite the 6 `<link name="thrusterN">` and matching `<joint>` blocks to use new placeholders.
- [orca_description/models/orca4/model.sdf](../orca_description/models/orca4/model.sdf) — regenerated artifact.

### Watch out
- `model.sdf` is generated, not hand-edited. Re-run `generate_model.py` after every change.
- Frame transform blocks at [model.sdf:355-359](../orca_description/models/orca4/model.sdf#L355-L359) (`modelXYZToAirplaneXForwardZDown`, `gazeboXYZToNED`) stay constant. Touching them to "fix" a sign error usually breaks navigation in subtler ways — use thruster pose rpy instead.

---

## Phase 3 — Validation (per-motor wrench + mode walkthrough)

**Goal:** prove that for each commanded motor, the resulting Gazebo body wrench matches the corresponding column of the custom mixer matrix, then verify each flight mode converges.

### Actions

1. **Per-motor isolation test.** With sim running and gravity zeroed (`SIM_GRAVITY_E/N/D 0` — temporary), command each motor individually via MAVLink `MOTOR_TEST` (mavproxy `motortest <n> 1 50 5` — motor n, throttle 50%, 5 s). For each:
   - Watch `/model/orca4/joint/thrusterN_joint/cmd_thrust` (expected: nonzero only on the commanded motor).
   - Record body-frame linear and angular acceleration from `/model/orca4/odometry` and IMU.
   - Compare measured 6-DOF response to the matrix row for that motor. Tolerate small cross-axis coupling from CoG offset; signs must match.

2. **Sign-flip detector.** If MOT_1 produces -X acceleration when matrix says +X (forward = 1.0), the link `<pose>` rpy is 180° off. If only roll/pitch signs invert while surge is correct, it's a different rotation. Most common bug — budget two iterations on signs.

3. **Mode-by-mode** (gravity restored):
   - **MANUAL**: stick forward small, watch `/odom` linear.x positive, `cmd_thrust` mostly on MOT_1.
   - **STABILIZE**: hands off sticks, vehicle holds level. Manually disturb roll via Gazebo wrench injection; vertical thrusters react.
   - **ALT_HOLD**: arm at depth, neutral sticks, depth holds ±10 cm.
   - **GUIDED**: send `SET_POSITION_TARGET_LOCAL_NED` waypoint 2 m forward, vehicle reaches it.

4. **Topics to monitor**: `/model/orca4/odometry`, `/imu`, `/model/orca4/joint/thrusterN_joint/cmd_thrust` (×6), MAVLink `VFR_HUD` and `ATTITUDE`. Record bags using the existing `bag:=True` launch arg.

### Verification (gate to Phase 4)
- Per-motor measured wrench signs match the matrix row for all 6 motors.
- ALT_HOLD holds depth within 10 cm.
- GUIDED reaches a 2 m waypoint.

### Files touched
None during validation. Sign corrections feed back into Phase 2 (`generate_model.py` + regenerate `model.sdf`).

### Watch out
- Don't paper over sign errors with `MOT_n_DIRECTION` parm flips. Fix the SDF.

---

## Phase 4 — Persist for the team

**Goal:** lock the configuration so a fresh checkout reproduces it. Pin the fork commit (analogous to the `.apj` pin for the real vehicle).

### Actions

1. **Pin the fork in Docker.** Edit [docker/Dockerfile:86](../docker/Dockerfile#L86) and the equivalent line in [docker/Dockerfile.laptop](../docker/Dockerfile.laptop):
   ```
   RUN git clone https://github.com/aris-space/project-polaris-ardusub.git ardupilot --recurse-submodules \
     && cd ardupilot && git checkout <SHA-validated-in-Phase-3>
   ```
   Replace upstream URL outright. Both Dockerfiles produce a fork-firmware image.

2. **Update [docker/README.md](../docker/README.md)**: which fork commit is pinned, how to bump it (one line: change the SHA, rebuild image).

3. **Document the integration** in top-level [README.md](../README.md): note that SITL runs the Polaris fork with `FRAME_CONFIG = 7` and that the SDF geometry comes from CAD. Link to this plan.

### Verification
- Fresh `git clone` + `docker compose build` + `docker compose run` + `ros2 launch orca_bringup sim_launch.py` reproduces a working Polaris sim end-to-end.
- Phase 3 mode checklist passes on the freshly-built image.

### Files touched
- [docker/Dockerfile](../docker/Dockerfile)
- [docker/Dockerfile.laptop](../docker/Dockerfile.laptop)
- [docker/README.md](../docker/README.md)
- [README.md](../README.md)

---

## Cross-cutting watch-outs

- Servo function defaults (`MOT_n` → `SERVOn_FUNCTION = 33 + n - 1`) — don't override unless deliberately remapping.
- `model.sdf` is generated — never hand-edit. Always edit `model.sdf.in` + `generate_model.py`, then re-run.
- Frame transforms at `model.sdf:355-359` stay constant. Body-frame convention chosen in Phase 2 must match these or be deliberately changed.
- MAVLink bridge at [comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py:482-506](../comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py#L482-L506) is frame-agnostic. No changes anywhere in `comms/`.
- Hydrodynamics caveat: added mass and quadratic drag remain BlueROV2-derived. Polaris dynamics will be approximate. Track as a follow-up tuning task once Phase 3 passes.

---

## End-to-end verification

1. On the Ubuntu PC, fresh shell: `export ARDUPILOT_HOME=<fork>; source setup.bash`.
2. `ros2 launch orca_bringup sim_launch.py` — Gazebo opens with Polaris geometry, ArduSub boots and prints `Frame: CUSTOM`, MAVLink `FRAME_CONFIG` reads 7.
3. mavproxy `motortest 1 1 50 5` — only thruster1 fires, vehicle accelerates +X body.
4. Arm in GUIDED, command waypoint, vehicle reaches it.