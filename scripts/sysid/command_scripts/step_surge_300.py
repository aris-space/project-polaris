#!/usr/bin/env python3
"""
Multi-level step input (maneuvers.md 2.3) — surge = 300, mid-low thrust level.

Publishes Int16MultiArray [300, 0, 500, 0, 0, 0] on /pixhawk/manual_control
for 10s, then returns to neutral. Vehicle must already be ARMED and in
MANUAL mode (see CHECK_AXIS.md section 0.6).

Usage:
    python3 step_surge_300.py [duration_s]
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from _step_common import main_for

SURGE = 300
DURATION_S = 10.0

if __name__ == '__main__':
    sys.exit(main_for(SURGE, DURATION_S, f'sysid_step_surge_{SURGE}'))
