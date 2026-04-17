"""
Convert nav_msgs/Odometry (ROS map/odom ENU + base_link FLU) to MAVLink ODOMETRY
fields for ArduPilot external navigation (MAV_FRAME_LOCAL_FRD pose, MAV_FRAME_BODY_FRD twist).

See comms/mavlink_bridge/docs/EXTERNAL_NAV_PLAN.md and:
https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from geometry_msgs.msg import Quaternion


def _quat_to_R(q: Quaternion) -> List[List[float]]:
    """Rotation matrix R such that v_world = R @ v_body (FLU body expressed in ENU world).
    This is all still for ROS ENU world and FLU body, the MAVLink conversion happens in ros_odom_to_mavlink_odometry().
    So a vector from body frame (with FLU) gets represented in map frame (with ENU)."""
    
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ]


#Multiplication and transpose for 3x3 matrices, since we don't want to pull in numpy just for this.
def _matmul(A: Sequence[Sequence[float]], B: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [
            A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]
            for j in range(3)
        ]
        for i in range(3)
    ]

def _transpose(M: Sequence[Sequence[float]]) -> List[List[float]]:
    return [[M[j][i] for j in range(3)] for i in range(3)]



def _R_to_quat_wxyz(R: Sequence[Sequence[float]]) -> Tuple[float, float, float, float]:
    """Unit quaternion (w, x, y, z) from rotation matrix (body to parent)."""
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0.0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2][1] - R[1][2]) * s
        y = (R[0][2] - R[2][0]) * s
        z = (R[1][0] - R[0][1]) * s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    return w / n, x / n, y / n, z / n


# ENU position (x=east, y=north, z=up) -> NED (north, east, down)
_R_ned_from_enu = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))

# FRD body basis expressed in FLU body: x_f=x_f, y_r=-y_l, z_d=-z_u
_R_flu_from_frd = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))


def ros_odom_to_mavlink_odometry(
    pos_enu_x: float,
    pos_enu_y: float,
    pos_enu_z: float,
    orient_flu_in_enu: Quaternion,
    twist_linear_flu: Tuple[float, float, float],
    twist_angular_flu: Tuple[float, float, float],
) -> Tuple[
    float,
    float,
    float,
    Tuple[float, float, float, float],
    Tuple[float, float, float],
    Tuple[float, float, float],
]:
    """
    Returns:
      position (x,y,z) in MAV_FRAME_LOCAL_FRD (m),
      quaternion (w,x,y,z) LOCAL_FRD <- BODY_FRD,
      linear velocity (vx,vy,vz) in BODY_FRD (m/s),
      angular velocity (roll,pitch,yaw rates rad/s) in BODY_FRD.
    """
    pn = pos_enu_y
    pe = pos_enu_x
    pd = -pos_enu_z

    R_enu_from_flu = _quat_to_R(orient_flu_in_enu)
    R_ned_from_frd = _matmul(_matmul(_R_ned_from_enu, R_enu_from_flu), _R_flu_from_frd)

    yaw_ned = math.atan2(R_ned_from_frd[1][0], R_ned_from_frd[0][0])
    cp = math.cos(yaw_ned)
    sp = math.sin(yaw_ned)
    R_ned_from_lfrd = [[cp, -sp, 0.0], [sp, cp, 0.0], [0.0, 0.0, 1.0]]
    R_lfrd_from_ned = _transpose(R_ned_from_lfrd)

    x_lfrd = pn * cp + pe * sp
    y_lfrd = -pn * sp + pe * cp
    z_lfrd = pd

    R_lfrd_from_frd = _matmul(R_lfrd_from_ned, R_ned_from_frd)
    wq, xq, yq, zq = _R_to_quat_wxyz(R_lfrd_from_frd)

    lx, ly, lz = twist_linear_flu
    vx_frd = lx
    vy_frd = -ly
    vz_frd = -lz

    wx, wy, wz = twist_angular_flu
    rollspeed = wx
    pitchspeed = -wy
    yawspeed = -wz

    return (
        float(x_lfrd),
        float(y_lfrd),
        float(z_lfrd),
        (float(wq), float(xq), float(yq), float(zq)),
        (float(vx_frd), float(vy_frd), float(vz_frd)),
        (float(rollspeed), float(pitchspeed), float(yawspeed)),
    )


def nan_pose_covariance() -> List[float]:
    c = [float("nan")] * 21
    return c


def nan_velocity_covariance() -> List[float]:
    c = [float("nan")] * 21
    return c
