"""Helpers to convert ROS nav_msgs/Odometry into MAVLink ODOMETRY (msg 331).

ROS follows REP-103 conventions:
    - World frame (odom/map): ENU  (X=East, Y=North, Z=Up)
    - Body frame  (base_link): FLU (X=Forward, Y=Left, Z=Up)

MAVLink (ArduPilot external nav / AP_VisualOdom) wants:
    - World frame: NED  (X=North, Y=East, Z=Down)
    - Body frame : FRD  (X=Forward, Y=Right, Z=Down)

So every odometry sample needs two rotations applied:
    - pose  : ENU world + FLU body  ->  NED world + FRD body
    - twist : FLU body              ->  FRD body   (body-frame linear & angular)

Axis math:
    ENU -> NED : (x, y, z) -> (y, x, -z)   [180 deg rotation around (1,1,0)/sqrt(2)]
    FLU -> FRD : (x, y, z) -> (x, -y, -z)  [180 deg rotation around X]

Quaternion transform (both corrections in (w, x, y, z) order):
    q_enu_to_ned = (0, sqrt(2)/2, sqrt(2)/2, 0)
    q_flu_to_frd = (0, 1, 0, 0)
    q_mav        = q_enu_to_ned  *  q_ros  *  q_flu_to_frd
"""
from __future__ import annotations

from math import sqrt
from typing import List, Tuple


_SQRT2_OVER_2 = sqrt(2.0) / 2.0

_Q_ENU_TO_NED: Tuple[float, float, float, float] = (0.0, _SQRT2_OVER_2, _SQRT2_OVER_2, 0.0)
_Q_FLU_TO_FRD: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 0.0)


def _quat_mul(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Hamilton product of two quaternions given as (w, x, y, z)."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def ros_quat_to_mav_quat(q_ros) -> Tuple[float, float, float, float]:
    """Convert a ROS geometry_msgs/Quaternion (ENU<-FLU) to MAVLink (NED<-FRD).

    Returned as (w, x, y, z) — the order MAVLink ODOMETRY expects.
    """
    q = (float(q_ros.w), float(q_ros.x), float(q_ros.y), float(q_ros.z))
    return _quat_mul(_quat_mul(_Q_ENU_TO_NED, q), _Q_FLU_TO_FRD)


def ros_odom_to_mavlink_odometry(
    x_enu: float,
    y_enu: float,
    z_enu: float,
    q_ros,
    linear_flu: Tuple[float, float, float],
    angular_flu: Tuple[float, float, float],
):
    """Convert a single ROS Odometry sample into MAVLink ODOMETRY fields.

    Assumes pose is in an ENU world frame and the twist is expressed in the
    FLU body frame (REP-103; matches robot_localization output by default).

    Returns a tuple:
        (x_ned, y_ned, z_ned, quat_wxyz, vel_frd, rates_frd)
    """
    x_ned = y_enu
    y_ned = x_enu
    z_ned = -z_enu

    quat_wxyz = ros_quat_to_mav_quat(q_ros)

    vx, vy, vz = linear_flu
    vel_frd = (vx, -vy, -vz)

    wx, wy, wz = angular_flu
    rates_frd = (wx, -wy, -wz)

    return x_ned, y_ned, z_ned, quat_wxyz, vel_frd, rates_frd


def nan_pose_covariance() -> List[float]:
    """Upper-triangular 6x6 pose covariance filled with NaN.

    ArduPilot's AP_VisualOdom_MAV::handle_msg_odometry treats a NaN leading
    element as "use the backend defaults". Use this when your EKF does not
    publish a reliable covariance and you would rather let ArduPilot pick
    conservative values than pass in zeros (which the EKF would interpret
    as "infinitely precise").
    """
    return [float("nan")] * 21


def nan_velocity_covariance() -> List[float]:
    """Upper-triangular 6x6 velocity covariance filled with NaN."""
    return [float("nan")] * 21


def ros_cov_to_mav_upper_triangle(ros_cov) -> List[float]:
    """Convert a ROS 6x6 row-major covariance (length 36) to the 21-element
    upper-triangle packing that MAVLink ODOMETRY expects.

    Note: this does *not* rotate the covariance into the NED/FRD frame. If
    your covariance is anisotropic and the frame rotation matters, perform
    that rotation on the ROS side before calling. For a diagonal covariance
    coming out of robot_localization, the values are identical after an
    axis permutation and this function is good enough.
    """
    if len(ros_cov) != 36:
        return nan_pose_covariance()
    out: List[float] = []
    for row in range(6):
        for col in range(row, 6):
            out.append(float(ros_cov[row * 6 + col]))
    return out
