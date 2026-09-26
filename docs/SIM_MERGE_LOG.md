# Simulation merge log

Append-only. Records every place where a VERIFY-FIRST check in `docs/SIM_MERGE_PLAN.md`
contradicted the plan, and what was done instead. Newest entries at the bottom.

## 2026-09-27 — T0.1

- Branch is `autonomy/sim` (not `simulation` as the plan names it), cut from
  `origin/autonomy/main` @ `45aad65`. Decision by Noel.
- `CLAUDE.local.md` states `gnss_datum_watchdog` was removed from the active launch. On
  `autonomy/main` it is still wired in `ekf_localization.launch.py` (`use_navsat_transform`).
  Per invariant 2, `autonomy/main` is treated as truth.
- `scripts/cmd_vel_ramp.py` exists at the repo root on `autonomy/main` (T1.2) — not vendored again.
