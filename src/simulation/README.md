# Simulation

Gazebo Harmonic + ArduSub SITL simulation of Polaris. Only built in the `:sim` image
(`POLARIS_SIM=1`); never built on the Jetson (`COLCON_IGNORE` is committed here).

> **No PSC, ATC or PID value may be transferred from this sim to the real vehicle.**
> Thruster geometry is still BlueROV2, so forces and torques do not match Polaris.
> See `docs/SIM_MERGE_PLAN.md` Phase 4.

Full usage documentation follows in T5.3 of `docs/SIM_MERGE_PLAN.md`.

## Attribution

`orca_description` and parts of `orca_sim_bringup` are derived from
[orca4](https://github.com/clydemcqueen/orca4) by Clyde McQueen, MIT licensed — see `LICENSE`
in this directory. Adapted for Polaris via Paul Zambelli's `project-polaris-simulation-personal`.
