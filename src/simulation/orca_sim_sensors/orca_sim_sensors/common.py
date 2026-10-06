"""Shared helpers for the synthetic sensors.

Ground truth comes from Gazebo's OdometryPublisher on /odom (ros_gz_bridge):
pose in the Gazebo world frame (ENU, the water surface is z = 0, x/y = 0 is the
mission origin = ArduSub --home), twist in the body frame (FLU), the same
convention as the vehicle's base_link.
"""

import json
import math
import os

import numpy as np
from ament_index_python.packages import get_package_share_directory

EARTH_RADIUS_M = 6378137.0


def default_origin_file():
    return os.path.join(
        get_package_share_directory('autonomy_bringup_pkg'),
        'missions', 'default_mission_origin.json')


def load_origin(path):
    """(lat, lon, alt) of the mission origin, which is Gazebo world (0, 0, 0)."""
    with open(path, encoding='utf-8') as f:
        o = json.load(f)
    return float(o['lat']), float(o['lon']), float(o['alt'])


def enu_to_llh(origin, e, n, u):
    """Flat-earth ENU -> lat/lon/alt. Fine over the few hundred metres a sim mission covers."""
    lat0, lon0, alt0 = origin
    lat = lat0 + math.degrees(n / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(e / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon, alt0 + u


def llh_to_enu(origin, lat, lon, alt):
    lat0, lon0, alt0 = origin
    n = math.radians(lat - lat0) * EARTH_RADIUS_M
    e = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return e, n, alt - alt0


def quat_to_matrix(q):
    """Rotation matrix (body -> world) from a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_matrix(roll, pitch, yaw):
    """Fixed-axis RPY (same convention as static_transform_publisher --roll/--pitch/--yaw)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def quat_multiply(a, b):
    """Hamilton product of (x, y, z, w) tuples."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi
