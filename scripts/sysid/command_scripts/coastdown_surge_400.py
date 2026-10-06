#!/usr/bin/env python3
"""
Coast-down (maneuvers.md 2.4) — surge = 400, held long enough to reach steady speed.

Publishes Int16MultiArray [400, 0, 500, 0, 0, 0] on /pixhawk/manual_control
for 20s, then returns to neutral. The coast-down is the decay after neutral,
which starts at the 'thrust_cut' marker on /sysid/event (maneuver "coastdown"):
keep recording for at least 15s after the script exits and do not start the
next maneuver before the vehicle has stopped. Vehicle must already be ARMED
and in MANUAL mode (see CHECK_AXIS.md section 0.6).

Same command as the step scripts; only the hold is longer. 20s is a margin
over the expected time to steady speed (a few seconds), and the long steady
segment also averages DVL noise and waves for the steady-state drag point.
At ~0.4 m/s this covers ~8 m plus the coast, so it needs lake space, not the pool.

Usage:
    python3 coastdown_surge_400.py [duration_s]
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from _step_common import main_for

SURGE = 400
DURATION_S = 20.0

if __name__ == '__main__':
    sys.exit(main_for(SURGE, DURATION_S, f'sysid_coastdown_surge_{SURGE}', 'coastdown'))
