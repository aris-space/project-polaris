"""Pure (ROS-free) math for SBL topside-heading calibration.

Kept free of rclpy / ROS message imports so it can be unit-tested off-robot.

Conventions:
  * IMU orientation comes from /imu/data_corrected: ENU, yaw about +Z = CCW from East (rad).
  * Compass heading / box static_orientation: degrees CW from North.
  * SBL filtered position (locator_position_relative_wrt_topside): topside BODY frame,
    x = forward, y = right (z = down, unused). A travel course in this frame, measured CW from the
    topside Forward axis, is gamma = atan2(y_right, x_forward). The topside Forward axis itself points
    at compass bearing theta (the configured static_orientation), so the North-referenced course is
    theta + gamma. Hence to make the SBL agree with a known true course:
        theta_correct = true_course - gamma      (all degrees, CW from North).
"""

import math

import numpy as np


def norm360(deg):
    """Wrap to [0, 360)."""
    return deg % 360.0


def norm180(deg):
    """Wrap to (-180, 180]."""
    d = (deg + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


def yaw_enu_from_quat(qx, qy, qz, qw):
    """Yaw about +Z (rad). For an ENU orientation this is CCW from East (REP-103)."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def enu_yaw_to_compass_deg(yaw_enu_rad):
    """ENU yaw (rad, CCW from East) -> compass bearing (deg, CW from North)."""
    return norm360(90.0 - math.degrees(yaw_enu_rad))


def circular_mean_std_deg(angles_deg):
    """Circular mean and std (deg) of a list/array of angles in degrees."""
    a = np.radians(np.asarray(angles_deg, dtype=float))
    s, c = float(np.mean(np.sin(a))), float(np.mean(np.cos(a)))
    mean = math.degrees(math.atan2(s, c))
    R = math.hypot(s, c)
    std = math.degrees(math.sqrt(-2.0 * math.log(R))) if R > 1e-9 else 180.0
    return norm360(mean), std


def fit_course(points_xy):
    """Fit the principal direction of topside-body-frame (x_forward, y_right) points.

    Returns (gamma_deg, length_m, ratio):
      gamma_deg : travel course CW from topside Forward, disambiguated by start->end displacement.
      length_m  : extent of the points along the principal axis.
      ratio     : minor/major singular-value ratio (0 = perfectly straight line).
    """
    pts = np.asarray(points_xy, dtype=float)
    if pts.shape[0] < 2:
        raise ValueError("need >= 2 points to fit a course")
    centered = pts - pts.mean(axis=0)
    _, s, vh = np.linalg.svd(centered, full_matrices=False)
    v = vh[0]
    disp = pts[-1] - pts[0]
    if float(np.dot(v, disp)) < 0.0:
        v = -v
    gamma = norm360(math.degrees(math.atan2(v[1], v[0])))  # atan2(right, forward) = CW from forward
    proj = centered @ v
    length = float(proj.max() - proj.min())
    ratio = float(s[1] / s[0]) if s[0] > 1e-9 else 1.0
    return gamma, length, ratio


def corrected_heading(true_course_deg, gamma_deg):
    """theta_correct = true_course - gamma  (deg, CW from North, wrapped to [0,360))."""
    return norm360(true_course_deg - gamma_deg)
