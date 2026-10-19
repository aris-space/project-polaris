# autonomy_bringup_pkg

Nav2 bringup for the vehicle: `launch/autonomy.launch.py`, the Nav2 params and behaviour tree,
the mission CSVs and the operator tools (`nav2_activate`, `wgs84_mission_starter`, …).

The mission is chosen in [`params/mission.yaml`](params/mission.yaml) (`mission_csv`, required).

Simulation, running a mission in the sim, and recording/exporting the PurePursuit tracking errors
(`export_tracking_bag_csv`) are documented in the
[simulation README](../../simulation/README.md#32-run-a-mission-csv).
