"""Integrate yaw angle from IMU angular_velocity.z (rad/s) using message timestamps.

Does not use magnetometer / orientation quaternion for yaw. Gyro integration drifts over
long periods; fine for short maneuvers and Foxglove plots vs quaternion yaw.
"""

from __future__ import annotations

import math
from typing import Sequence


def stamp_to_seconds(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) * 1e-9


def integrate_yaw_rate_forward_euler(
    stamps_sec: Sequence[float],
    omega_z: Sequence[float],
    *,
    max_dt_sec: float = 0.25,
) -> list[float]:
    """Cumulative yaw (rad), yaw[0]=0.0.

    For i >= 1: yaw[i] = yaw[i-1] + omega_z[i-1] * dt_i where dt_i = stamps[i]-stamps[i-1].
    If dt < 0 or dt > max_dt_sec, dt is skipped (treated as 0) to avoid bag glitches.
    """
    n = len(stamps_sec)
    if n != len(omega_z):
        raise ValueError("stamps_sec and omega_z length mismatch")
    if n == 0:
        return []
    out: list[float] = [0.0]
    for i in range(1, n):
        dt = float(stamps_sec[i]) - float(stamps_sec[i - 1])
        if dt <= 0.0 or dt > max_dt_sec or not math.isfinite(dt):
            out.append(out[-1])
            continue
        w = float(omega_z[i - 1])
        if not math.isfinite(w):
            out.append(out[-1])
            continue
        out.append(out[-1] + w * dt)
    return out
