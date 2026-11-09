# sysid TODO

Working list. Details and reasoning are in `docs/sysid_notes/sysid_notes.pdf` (section refs below).
Tick things off here; add a line to "Notes log" whenever the notes change.

## Fit script (`fitting/fit_surge.py`) — notes §10.6
- [x] Filter every term with the same filter: build `|u|u` from raw `u` then smooth, `du = gradient(smooth(u))`
- [x] Analysis grid >= 100 Hz (`RATE_HZ`)
- [x] Check it recovers the truth on an ideal synthetic bag (m_tot -1.6 %, X_u -0.1 %, X_uu +0.1 %)
- [ ] Add a time window (`--t-start/--t-end`) to crop EKF jumps; pass it to `log_fit`
- [ ] Add servo delay handling / scan
- [ ] Later: output-error fit (no du/dt needed) — notes §10.4

## Before trusting any m_tot
- [ ] Decide `servo_delay_s` sign convention (lag L is entered as **-L** today) — notes Ch. 9, Ch. 13, §14.2 item 2
- [ ] Measure actuation delay from IMU / speed edges at a step
- [ ] Raise `servo_output_raw` rate to ~50 Hz (rectangle bags are 2 Hz -> m_tot unidentifiable)
      or reconstruct PWM edges from `/manual_control` + `/sysid/event`
- [ ] Try `/filter/free_acceleration` (Xsens, gravity already removed, world frame -> rotate by heading) as du/dt — notes §10.6.3
- [ ] Fix `/imu/data` timestamps (arrive in bursts, gaps up to 72 ms) before using IMU for timing
- [ ] Read the real command -> PWM mapping off a lake bag (bench assumes 1500 + 0.4*cmd)
- [ ] Check whether `servo_output_raw` reflects `MOT_n_DIRECTION`. Unknown whether ArduSub logs the
      PWM *after* applying a reversed motor direction or before it. The heave allocation in
      `model.yaml` assumes PWM > 1500 pushes up on all three vertical thrusters (ch 2, 3, 4, +1 each)
      once the directions are set. Test (bench, props in water, or a pure-heave bag): command pure
      heave up and check that ch 2, 3, 4 all go above 1500 *and* all three actually push up. If the
      logged PWM is the pre-reversal value, the force sign per channel must come from
      `MOT_n_DIRECTION` (a per-channel PWM mirror, like the surge `sign`), not from the log.

## Old-bag first fit — see chat plan / notes §10.6
- [x] Confirm which thruster was fitted when the old bags were recorded: **T200** (the T500 is on
      the new hardware only). `model.yaml`'s T500 map is wrong for every old bag; use `--dummy`
- [ ] Get a proper T200 PWM x voltage table (Blue Robotics T200 performance spreadsheet) to replace
      the DUMMY polynomial (assumed PWM mapping, single voltage) for the old bags
- [ ] Confirm channel 0 = surge with the same sign in those bags
- [ ] Crop EKF jumps (June 842 m, rectangle_08 14 m, grid_02 10 m); drop 0 V BMS samples in grid_02
- [ ] Fit into a copy of the model (`model_firstfit.yaml`), not `model.yaml`
- [ ] Treat results as a pipeline sanity check, not POLARIS coefficients

## Pool / lake measurements
- [ ] Weigh the vehicle (m)
- [ ] Measure net buoyancy B - W with a known weight (no land rig needed). This is the absolute
      anchor for the heave cut-thrust test below. Possible methods:
      1. **Trim to neutral:** with thrusters off, add small known weights (if buoyant) until the
         vehicle hovers, or a known float (if heavy). B - W = weight in water of what was added,
         m_ballast * g * (1 - rho_water/rho_material) (steel ~0.87, lead ~0.91). Hovering is hard to
         judge: bracket it (smallest weight that sinks, largest that still rises) and average.
      2. **Two cut-thrust runs, one with an extra known weight** (in-water weight D): the initial
         accelerations right after the cut (w ~ 0, no drag) are a1 = (B - W)/m_z and
         a2 = (B - W - D)/m_z, so m_z = D/(a1 - a2) and B - W = m_z * a1. Does not need the vehicle
         near neutral and reuses the test below.
- [ ] Heave cut-thrust test (Z_w, Z_ww without any vertical thrust curve): depth hold,
      then cut all thrusters, mark it with `thrust_cut`, and let the vehicle rise/sink freely.
      Fit m_z dw/dt = (B - W) - Z_w w - Z_ww |w|w on the decay. **With zero thrust only ratios are
      identifiable** ((B - W)/m_z, Z_w/m_z, Z_ww/m_z): scaling every term by the same factor gives
      the same motion. Turn them into N and kg with B - W from the item above (or m_z from method 2).
      - Start depth: **>= 2.5-3 m below the surface** (if buoyant) or above the bed (if heavy).
        Rough estimate, assuming B - W = 1-5 N, Z_ww ~ 100 N s^2/m^2 (slender hull side-on,
        Cd ~ 1, ~0.2 m^2) and m_z ~ 60 kg (mass + heave added mass): terminal speed ~0.1-0.2 m/s,
        ~5 s to reach it (~0.5 m travel), then ~5 s at terminal speed (~1 m), plus >= 1 m margin
        so the fit stops well before surface (or bed) effects. Lake, not pool (0.8 m deep).
      - Cut the fit window before the vehicle comes within ~1 m of the surface or bed.
      - Record pitch: cutting the vertical thrusters can pitch the vehicle (CG/CB offset);
        use only the part where pitch stays small. Slack tether.
- [ ] Trim pitch theta_0 from a still, powered-down float
- [ ] Pitch tilt test (M_theta) and pitch free decay
- [ ] Clean lake steps 200-500 + coast-downs with `/sysid/event` and `/imu/data`, some held out
- [ ] Longer surge run (validation): check how well the fitted parameters predict it, and how much
      tether effects grow with distance (decides whether a tether term is needed)
- [ ] Check roll stays small (|dphi|95 < ~5 deg) in bags used for sway/heave/yaw — notes §3.5.1
- [ ] Does POLARIS run at the surface? Then heave needs waterplane stiffness (no slot yet) — notes §3.5

## Thrust bench (DD2838 curves) — notes §6.8, concept doc "Thrust-Messstand" (28 Oct)
- [ ] Order the S-type 10 kg cell + two rod ends; check its mV/V, thread and wire colours on arrival
- [ ] NAU7802 on the Jetson: bus/address with `i2cdetect`; excitation ~3.0 V (4.0 V not reachable from 3.3 V)
- [ ] Use the vehicle's ESC model; bench Pixhawk output with `SERVOn_FUNCTION = 0`; log `SERVO_OUTPUT_RAW`
- [ ] Build the lever/pendulum in water; calibrate in place with reference weights, both directions
- [ ] Short test run (5 s per step) for wiring, sign and zero, then measure the settling time
- [ ] Sweeps forward AND reverse, 10 us steps near neutral, 3-4 voltages bracketing 13-16.8 V (at the ESC)
- [ ] Convert to `force_vs_voltage.csv` format (kgf, common PWM grid) and replace the PLACEHOLDER maps
- [ ] While on the bench: answer the `servo_output_raw` vs `MOT_n_DIRECTION` question (needs a motor output)
- [ ] After the hardware is final and the bench curves exist: yaw and sway steps from rest (both directions,
      held several seconds) to separate timing mismatch (edge kicks) from static error (steady
      cross-axis force); use the steady part to refine the allocation in `model.yaml` and the
      ArduSub mixer factors — notes §9.5

## Scaffolding (no data needed) — notes Ch. 14, table "Scaffolding"
- [x] S1 synthetic ground-truth bags (`simulation/synth_bag.py`)
- [ ] S2 event reader / segmenter for `/sysid/event`
- [ ] S3 IMU + DVL readers in `bag_io.py`
- [ ] S4 shared uniform-grid loader
- [ ] S5 bag QA script for the field (topics, rates, events, EKF jumps, voltage, roll)
- [ ] S6 per-horizon RMS bins in propagation summary
- [ ] S7 comment-preserving `update_parameters`
- [ ] S8 remaining command scripts (reverse steps, sine sweep, surge+yaw)
- [x] S9 model schema: sway, heave, B (roll removed)
- [ ] S10 run registry `runs.yaml` (which bag = which manoeuvre / fit vs held out)
- [ ] S11 = servo_delay_s sign decision (above)
- [x] S12 fit log (`results_log.py`, `results/fit_log.yaml`)
- [ ] S13 validation log (same machinery as the fit log)
- [ ] S14 synth bag option: thrust follows the command, so the servo log rate is testable

## Potential ideas (maybe, not decided)
Might be useful; move up into a real section once decided, or delete.
- [ ] Tether-length proxy for a disturbance term: path length travelled since a mission-start
      marker (integrate the EKF speed, not position differences, so EKF jumps don't count) as a
      stand-in for paid-out tether, e.g. F_tether ~ k * path length. Low priority; only if the
      long-run test shows tether effects matter. Caveat: path length only grows, so it can't be
      told apart from slow time drifts (battery sag, current) except via pauses or speed changes.

## Housekeeping
- [ ] Commit `results/fit_log.yaml` together with the code (nothing committed yet)
- [ ] Commit before the first `update_parameters()` on `model.yaml` (it drops comments)
- [ ] Update `maneuvers.md` for the `servo_delay_s` sign once decided

## Notes log (what was added to `docs/sysid_notes`)
Full list: Appendix C (change log). Orange "Revised" boxes mark every change in the text.

| Date | Added / changed | Where |
|---|---|---|
| 9 Oct | First edition | all |
| 14 Oct | Synthetic test bench; servo_delay_s sign; smoothing-window bias; scaffolding list S1-S11 | Ch. 13, Ch. 9, §10.3, Ch. 14 |
| 15 Oct | Model schema (sway, heave, B); roll removed + roll evidence from bags | §3.5, §3.5.1 |
| 16 Oct | Savitzky-Golay mechanics | §10.3.1 |
| 16 Oct | Bias study: filter consistency, grid rate, IMU vs du/dt, servo log rate | §10.6 |
| 16 Oct | Fit log | §10.8 |
| 16 Oct | Plain-words explanation of the same-filter rule; EKF has no acceleration | §10.6.1 |
| 16 Oct | fit_surge.py fixed (consistent filter, 100 Hz) + results | end of §10.6 |
| 17 Oct | Personal notes chapter (Feynman-method entries in my own words) | Ch. 15 |
| 17 Oct | High-level pipeline overview (my four steps), before Ch. 1 | front |
| 21 Oct | Noise budget: EKF+SG vs Xsens IMU; EKF speed noise corrected (3.4 mm/s, not 0.03 m/s) | §10.3.1, §10.6.3 |
| 28 Oct | Thrust bench (planned): set-up, procedure, analysis, review corrections | §6.8 |
| 30 Oct | Thruster delay: what the IMU onset contains, timing vs static error, mixer refinement | §9.5 |
|  |  |  |

Regenerate data: `python docs/sysid_notes/gen_figdata.py`, `gen_synthdata.py`, `gen_biasstudy.py`;
build: `cd docs/sysid_notes && latexmk -pdf sysid_notes.tex`.
