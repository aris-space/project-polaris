"""Quaternion (x,y,z,w) to roll, pitch, yaw (radians).

Convention: intrinsic Tait–Bryan XYZ (equivalent extrinsic ZYX): roll about x, pitch about y,
yaw about z. Matches common ROS / robot_localization usage for mapping quaternion orientation
to Euler angles for plotting.

Note: Euler angles are not unique at singularities; yaw is still useful for level flight.
"""

from __future__ import annotations

import math


def quaternion_to_rpy_xyz(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Return (roll, pitch, yaw) in radians."""
    # Shorthand
    x, y, z, w = qx, qy, qz, qw

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
