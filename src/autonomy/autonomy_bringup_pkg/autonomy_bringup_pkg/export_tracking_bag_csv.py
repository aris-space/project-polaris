#!/usr/bin/env python3
# MIT License - same as orca_bringup
r"""
Export PurePursuit tracking topics from a rosbag2 folder to plain CSV (no ROS needed to analyze CSVs).

Rosbag2: a *bag directory* contains ``metadata.yaml`` plus ``*.db3`` or ``*.mcap``.

PurePursuit topics (recorded when ``publish_tracking_error`` is true on PurePursuitController3D):

  std_msgs/Float64: cross_track_xy, vertical_error, yaw_error
  geometry_msgs/PointStamped: closest point on path (``pure_pursuit_closest_point_map``, frame = map)
  geometry_msgs/PoseStamped: robot pose in map (``pure_pursuit_robot_pose_map``)
  geometry_msgs/TwistStamped: robot twist from Nav2 odom (``pure_pursuit_robot_twist``, frame = robot pose frame)

Also supported if present in the bag:

  geometry_msgs/Vector3: ocean current m/s (``/ocean_current``, sim bridge or ``current_vector_node``)
  sensor_msgs/NavSatFix: filtered global GPS fix (``/gps/filtered/global``, navsat_transform output)
  geometry_msgs/Twist: commanded velocity to the Pixhawk (``/pixhawk/cmd_vel``, controller output)
  nav_msgs/Odometry: measured body twist from the local EKF (``/odometry/filtered/local``)
  nav_msgs/Path: planned reference path, one msg per leg (``/plan``)
  geometry_msgs/PoseArray: mission waypoints / Nav2 goals (``/mission_waypoints_enu``, latched)

Outputs:

  tracking_errors_long.csv - unified long format (see README in export folder)
  tracking_errors_wide.csv - one row per cross-track sample; errors + closest + pose + twist + ocean_current (as-of merged)
  gps_filtered_global.csv  - time_sec, latitude, longitude, altitude from /gps/filtered/global
  velocity_cmd_vs_measured.csv - measured (/odometry/filtered/local) vs commanded (/pixhawk/cmd_vel) twist; works without Nav2
  mission_waypoints.csv    - the goals sent to Nav2 (from /mission_waypoints_enu) - truthful waypoint markers
  planned_path.csv         - every /plan leg - the truthful reference path (NOT the closest-point trace)
  tracking_export_README.txt

Usage:
source /opt/ros/humble/setup.bash && python3 /home/polaris_pz/project-polaris/src/autonomy/autonomy_bringup_pkg/autonomy_bringup_pkg/export_tracking_bag_csv.py "/home/polaris_pz/Downloads/OneDrive_2026-05-28/Important Ones/move_to_goal_lake_11_2026_05_26-14_38_21"
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import DefaultDict, List, Tuple

TOPIC_CROSS = '/pure_pursuit_cross_track_xy'
TOPIC_VERT = '/pure_pursuit_vertical_error'
TOPIC_YAW = '/pure_pursuit_yaw_error'
TOPIC_CLOSEST = '/pure_pursuit_closest_point_map'
TOPIC_POSE = '/pure_pursuit_robot_pose_map'
TOPIC_TWIST = '/pure_pursuit_robot_twist'
TOPIC_OCEAN_CURRENT = '/ocean_current'
TOPIC_GPS_FILTERED = '/gps/filtered/global'
TOPIC_CMD_VEL = '/pixhawk/cmd_vel'
# Measured body velocity from the local EKF (odom -> base_link). This is the
# vehicle's actual twist, available even when Nav2 is NOT running (e.g. surge/yaw
# controller-characterization runs). The Nav2 TWIST topic above only exists while
# PurePursuit is active, so it cannot be the measured source for those runs.
TOPIC_ODOM_LOCAL = '/odometry/filtered/local'
# Ground-truth geometry: the goals sent to Nav2 and the path the controller tracks.
# Exported to dedicated sidecar CSVs (not the long/wide tables) so analysis plots
# never have to hardcode waypoints or mistake the closest-point trace for the plan.
TOPIC_PLAN = '/plan'
TOPIC_WAYPOINTS = '/mission_waypoints_enu'

TOPICS_FLOAT = (TOPIC_CROSS, TOPIC_VERT, TOPIC_YAW)
LONG_HEADER = [
    'time_sec',
    'topic',
    'msg_type',
    'v0',
    'v1',
    'v2',
    'v3',
    'v4',
    'v5',
    'v6',
    'v7',
    'v8',
    'v9',
    'v10',
    'v11',
    'v12',
]


def _storage_id_from_metadata(bag_dir: Path) -> str:
    meta = bag_dir / 'metadata.yaml'
    if not meta.is_file():
        return 'sqlite3'
    text = meta.read_text(encoding='utf-8', errors='replace')
    m = re.search(r'storage_identifier:\s*(\S+)', text)
    return m.group(1) if m else 'sqlite3'


def _read_bag_mixed(bag_dir: Path):
    from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Twist, TwistStamped, Vector3
    from nav_msgs.msg import Odometry
    from nav_msgs.msg import Path as PathMsg
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import Float64

    want = {
        TOPIC_CROSS,
        TOPIC_VERT,
        TOPIC_YAW,
        TOPIC_CLOSEST,
        TOPIC_POSE,
        TOPIC_TWIST,
        TOPIC_OCEAN_CURRENT,
        TOPIC_GPS_FILTERED,
        TOPIC_CMD_VEL,
        TOPIC_ODOM_LOCAL,
        TOPIC_PLAN,
        TOPIC_WAYPOINTS,
    }
    sid = _storage_id_from_metadata(bag_dir)
    storage_options = StorageOptions(uri=str(bag_dir), storage_id=sid)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    long_rows: List[List[object]] = []
    float_scalar: DefaultDict[str, List[Tuple[float, float]]] = defaultdict(list)
    closest: List[Tuple[float, Tuple[float, float, float]]] = []
    pose: List[Tuple[float, Tuple[float, ...]]] = []
    twist: List[Tuple[float, Tuple[float, ...]]] = []
    ocean_current: List[Tuple[float, Tuple[float, float, float]]] = []
    gps_filtered: List[Tuple[float, Tuple[float, float, float]]] = []
    cmd_vel: List[Tuple[float, Tuple[float, ...]]] = []
    # odom_twist: measured body twist (linear x,y,z, angular x,y,z) from the local EKF.
    odom_twist: List[Tuple[float, Tuple[float, ...]]] = []
    # plans: one entry per /plan message (a leg). Each is (t_sec, [(x,y,z), ...]).
    plans: List[Tuple[float, List[Tuple[float, float, float]]]] = []
    # waypoints: one entry per /mission_waypoints_enu message (latched -> usually 1).
    waypoints: List[Tuple[float, List[Tuple[float, float, float]]]] = []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in want:
            continue
        t_sec = t_ns * 1e-9
        if topic in TOPICS_FLOAT:
            msg = deserialize_message(data, Float64)
            float_scalar[topic].append((t_sec, msg.data))
            row = [t_sec, topic, 'float64', msg.data] + [''] * 12
            long_rows.append(row)
        elif topic == TOPIC_CLOSEST:
            msg = deserialize_message(data, PointStamped)
            x, y, z = msg.point.x, msg.point.y, msg.point.z
            closest.append((t_sec, (x, y, z)))
            row = [t_sec, topic, 'point', x, y, z] + [''] * 10
            long_rows.append(row)
        elif topic == TOPIC_POSE:
            msg = deserialize_message(data, PoseStamped)
            p = msg.pose.position
            o = msg.pose.orientation
            tup = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
            pose.append((t_sec, tup))
            row = [t_sec, topic, 'pose', *tup] + [''] * 6
            long_rows.append(row)
        elif topic == TOPIC_TWIST:
            msg = deserialize_message(data, TwistStamped)
            l = msg.twist.linear
            a = msg.twist.angular
            tup = (l.x, l.y, l.z, a.x, a.y, a.z)
            twist.append((t_sec, tup))
            row = [t_sec, topic, 'twist', *tup] + [''] * 7
            long_rows.append(row)
        elif topic == TOPIC_OCEAN_CURRENT:
            msg = deserialize_message(data, Vector3)
            x, y, z = msg.x, msg.y, msg.z
            ocean_current.append((t_sec, (x, y, z)))
            row = [t_sec, topic, 'vector3', x, y, z] + [''] * 10
            long_rows.append(row)
        elif topic == TOPIC_GPS_FILTERED:
            msg = deserialize_message(data, NavSatFix)
            lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
            gps_filtered.append((t_sec, (lat, lon, alt)))
            row = [t_sec, topic, 'navsatfix', lat, lon, alt] + [''] * 10
            long_rows.append(row)
        elif topic == TOPIC_CMD_VEL:
            msg = deserialize_message(data, Twist)
            l = msg.linear
            a = msg.angular
            tup = (l.x, l.y, l.z, a.x, a.y, a.z)
            cmd_vel.append((t_sec, tup))
            row = [t_sec, topic, 'twist', *tup] + [''] * 7
            long_rows.append(row)
        elif topic == TOPIC_ODOM_LOCAL:
            msg = deserialize_message(data, Odometry)
            l = msg.twist.twist.linear
            a = msg.twist.twist.angular
            tup = (l.x, l.y, l.z, a.x, a.y, a.z)
            odom_twist.append((t_sec, tup))
            row = [t_sec, topic, 'odom_twist', *tup] + [''] * 7
            long_rows.append(row)
        elif topic == TOPIC_PLAN:
            msg = deserialize_message(data, PathMsg)
            pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
            if pts:
                plans.append((t_sec, pts))
        elif topic == TOPIC_WAYPOINTS:
            msg = deserialize_message(data, PoseArray)
            pts = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
            if pts:
                waypoints.append((t_sec, pts))

    long_rows.sort(key=lambda r: r[0])
    return (
        long_rows, float_scalar, closest, pose, twist,
        ocean_current, gps_filtered, cmd_vel, odom_twist, plans, waypoints,
    )


def _asof_backward(
    master: List[Tuple[float, float]],
    slave: List[Tuple[float, float]],
) -> List[float]:
    if not master:
        return []
    slave = sorted(slave, key=lambda x: x[0])
    j = 0
    last = float('nan')
    out: List[float] = []
    for t, _ in master:
        while j < len(slave) and slave[j][0] <= t:
            last = slave[j][1]
            j += 1
        out.append(last)
    return out


def _asof_backward_tuple(
    master: List[Tuple[float, float]],
    slave: List[Tuple[float, Tuple[float, ...]]],
    dim: int,
) -> List[Tuple[float, ...]]:
    if not master:
        return []
    slave = sorted(slave, key=lambda x: x[0])
    j = 0
    nan_t = tuple(float('nan') for _ in range(dim))
    last = nan_t
    out: List[Tuple[float, ...]] = []
    for t, _ in master:
        while j < len(slave) and slave[j][0] <= t:
            last = slave[j][1]
            j += 1
        out.append(last)
    return out


def _write_long_csv(path: Path, rows: List[List[object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(LONG_HEADER)
        w.writerows(rows)


def _write_wide_csv(
    path: Path,
    by_float: DefaultDict[str, List[Tuple[float, float]]],
    closest: List[Tuple[float, Tuple[float, float, float]]],
    pose: List[Tuple[float, Tuple[float, ...]]],
    twist: List[Tuple[float, Tuple[float, ...]]],
    ocean_current: List[Tuple[float, Tuple[float, float, float]]],
    gps_filtered: List[Tuple[float, Tuple[float, float, float]]],
    cmd_vel: List[Tuple[float, Tuple[float, ...]]],
) -> None:
    master = sorted(by_float.get(TOPIC_CROSS, []), key=lambda x: x[0])
    vert = sorted(by_float.get(TOPIC_VERT, []), key=lambda x: x[0])
    yaw = sorted(by_float.get(TOPIC_YAW, []), key=lambda x: x[0])

    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        'time_sec',
        'cross_track_xy_m',
        'vertical_error_m',
        'yaw_error_rad',
        'closest_x_m',
        'closest_y_m',
        'closest_z_m',
        'robot_x_m',
        'robot_y_m',
        'robot_z_m',
        'robot_qx',
        'robot_qy',
        'robot_qz',
        'robot_qw',
        'twist_linear_x',
        'twist_linear_y',
        'twist_linear_z',
        'twist_angular_x',
        'twist_angular_y',
        'twist_angular_z',
        'ocean_current_x_m_s',
        'ocean_current_y_m_s',
        'ocean_current_z_m_s',
        'gps_latitude',
        'gps_longitude',
        'gps_altitude',
        'cmd_vel_linear_x',
        'cmd_vel_angular_z',
    ]

    def fmt_num(x: float) -> object:
        return '' if isinstance(x, float) and math.isnan(x) else x

    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(header)

        if not master:
            return

        vcol = _asof_backward(master, vert)
        ycol = _asof_backward(master, yaw)
        ccol = _asof_backward_tuple(master, closest, 3)
        pcol = _asof_backward_tuple(master, pose, 7)
        tcol = _asof_backward_tuple(master, twist, 6)
        ocol = _asof_backward_tuple(master, ocean_current, 3)
        gcol = _asof_backward_tuple(master, gps_filtered, 3)
        cvcol = _asof_backward_tuple(master, cmd_vel, 6)

        for (t, cx), v, y, c3, p7, t6, o3, g3, cv6 in zip(
            master, vcol, ycol, ccol, pcol, tcol, ocol, gcol, cvcol,
        ):
            w.writerow(
                [
                    t,
                    cx,
                    fmt_num(v),
                    fmt_num(y),
                    fmt_num(c3[0]),
                    fmt_num(c3[1]),
                    fmt_num(c3[2]),
                    fmt_num(p7[0]),
                    fmt_num(p7[1]),
                    fmt_num(p7[2]),
                    fmt_num(p7[3]),
                    fmt_num(p7[4]),
                    fmt_num(p7[5]),
                    fmt_num(p7[6]),
                    fmt_num(t6[0]),
                    fmt_num(t6[1]),
                    fmt_num(t6[2]),
                    fmt_num(t6[3]),
                    fmt_num(t6[4]),
                    fmt_num(t6[5]),
                    fmt_num(o3[0]),
                    fmt_num(o3[1]),
                    fmt_num(o3[2]),
                    fmt_num(g3[0]),
                    fmt_num(g3[1]),
                    fmt_num(g3[2]),
                    fmt_num(cv6[0]),
                    fmt_num(cv6[5]),
                ],
            )


def _write_velocity_csv(
    path: Path,
    odom_twist: List[Tuple[float, Tuple[float, ...]]],
    cmd_vel: List[Tuple[float, Tuple[float, ...]]],
) -> int:
    """Measured (local-EKF) body twist vs commanded /pixhawk/cmd_vel.

    Keyed on the odometry samples (NOT cross-track), so it is populated even when
    Nav2 is not running -- the use case for surge/yaw controller-characterization
    runs. cmd_vel is as-of (backward) merged onto each odometry timestamp.

    Column names match what the analysis scripts already read from the wide CSV
    (twist_*, cmd_vel_*), so those plot scripts work unchanged when pointed here.
    Returns the number of rows written.
    """
    master = sorted(odom_twist, key=lambda x: x[0])
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        'time_sec',
        'twist_linear_x',
        'twist_linear_y',
        'twist_linear_z',
        'twist_angular_x',
        'twist_angular_y',
        'twist_angular_z',
        'cmd_vel_linear_x',
        'cmd_vel_angular_z',
    ]

    def fmt_num(x: float) -> object:
        return '' if isinstance(x, float) and math.isnan(x) else x

    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(header)
        if not master:
            return 0
        cvcol = _asof_backward_tuple(master, cmd_vel, 6)
        for (t, m6), cv6 in zip(master, cvcol):
            w.writerow(
                [
                    t,
                    fmt_num(m6[0]),
                    fmt_num(m6[1]),
                    fmt_num(m6[2]),
                    fmt_num(m6[3]),
                    fmt_num(m6[4]),
                    fmt_num(m6[5]),
                    fmt_num(cv6[0]),
                    fmt_num(cv6[5]),
                ],
            )
        return len(master)


def _write_gps_csv(
    path: Path,
    gps_filtered: List[Tuple[float, Tuple[float, float, float]]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['time_sec', 'latitude', 'longitude', 'altitude'])
        for t, (lat, lon, alt) in sorted(gps_filtered, key=lambda r: r[0]):
            w.writerow([t, lat, lon, alt])


def _write_waypoints_csv(
    path: Path,
    waypoints: List[Tuple[float, List[Tuple[float, float, float]]]],
) -> int:
    """Write the mission waypoints (the goals sent to Nav2) as ground truth.

    /mission_waypoints_enu is latched, so the last message holds the final set;
    we export that one. Returns the number of waypoints written.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    pts = waypoints[-1][1] if waypoints else []
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['wp_index', 'wp_x_m', 'wp_y_m', 'wp_z_m'])
        for i, (x, y, z) in enumerate(pts):
            w.writerow([i, x, y, z])
    return len(pts)


def _write_planned_path_csv(
    path: Path,
    plans: List[Tuple[float, List[Tuple[float, float, float]]]],
) -> int:
    """Write every /plan message (the actual reference path the controller tracks).

    One block per leg, tagged by leg_index and the plan's publish time. This is the
    truthful 'reference path' for XY plots -- not the closest-point trace. Returns
    the number of legs written.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['leg_index', 'plan_time_sec', 'seq', 'plan_x_m', 'plan_y_m', 'plan_z_m'])
        for leg, (t_sec, pts) in enumerate(sorted(plans, key=lambda r: r[0])):
            for seq, (x, y, z) in enumerate(pts):
                w.writerow([leg, t_sec, seq, x, y, z])
    return len(plans)


def _write_readme(path: Path, bag_dir: Path) -> None:
    text = f"""Tracking export (CSV)
=====================

Source bag: {bag_dir}

tracking_errors_long.csv
  Columns: {', '.join(LONG_HEADER)}
  msg_type:
    float64 - v0 is scalar error (.data)
    point   - v0,v1,v2 = closest path point x,y,z (map frame, see bag headers)
    pose    - v0..v6 = position x,y,z and orientation quaternion x,y,z,w (map)
    twist   - v0..v5 = linear x,y,z then angular x,y,z (Nav2 body twist, or /pixhawk/cmd_vel command)
    vector3 - v0,v1,v2 = ocean current x,y,z (m/s, /ocean_current)

tracking_errors_wide.csv
  One row per cross-track sample (time_sec). vertical/yaw/closest/pose/twist/ocean_current/gps/cmd_vel
  columns use the last sample at or before that time (same controller tick -> aligned).
  GPS columns: gps_latitude, gps_longitude, gps_altitude (from /gps/filtered/global).
  cmd_vel columns: cmd_vel_linear_x, cmd_vel_angular_z (from /pixhawk/cmd_vel).

gps_filtered_global.csv
  One row per /gps/filtered/global (NavSatFix) sample. Columns: time_sec, latitude, longitude, altitude.
  Independent of the controller cadence (not as-of merged into the wide CSV).

velocity_cmd_vs_measured.csv
  Measured body twist (from /odometry/filtered/local, the local EKF) vs commanded
  /pixhawk/cmd_vel. One row per odometry sample; cmd_vel is as-of (backward) merged.
  Columns: time_sec, twist_linear_x/y/z, twist_angular_x/y/z, cmd_vel_linear_x, cmd_vel_angular_z.
  USE THIS for surge/yaw controller-characterization runs where Nav2 is NOT running
  (the wide CSV is empty there because it is keyed on the cross-track error series).
  Column names match the wide CSV, so the cmd-vs-measured plot scripts work unchanged.

mission_waypoints.csv
  The goals sent to Nav2 (from /mission_waypoints_enu, latched -> last set).
  Columns: wp_index, wp_x_m, wp_y_m, wp_z_m (map/ENU frame).
  USE THIS for waypoint markers in plots -- never hardcode waypoint coordinates.

planned_path.csv
  Every /plan message (the actual reference path the controller tracks), one block
  per leg. Columns: leg_index, plan_time_sec, seq, plan_x_m, plan_y_m, plan_z_m (map frame).
  USE THIS as the 'reference path' in XY plots. Do NOT use the closest-point trace
  (closest_*_m in the wide CSV) as the reference -- it is the controller foot-point
  projection and jumps discontinuously at each leg handoff.

Plain UTF-8; no ROS needed to analyze. Produced by export_tracking_bag_csv.py
"""
    path.write_text(text, encoding='utf-8')


def _maybe_plot(by_topic: DefaultDict[str, List[Tuple[float, float]]], title: str) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed; skipping --plot.', file=sys.stderr)
        return

    topics = TOPICS_FLOAT
    t0 = min((by_topic[t][0][0] for t in topics if by_topic.get(t)), default=0.0)
    labels = (
        'cross_track_xy (m)',
        'vertical_error (m)',
        'yaw_error (rad)',
    )
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7), constrained_layout=True)
    for ax, topic, ylab in zip(axes, topics, labels):
        series = by_topic.get(topic, [])
        if series:
            xs = [x - t0 for x, _ in series]
            ys = [y for _, y in series]
            ax.plot(xs, ys, '.', markersize=2)
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('time (s) from first sample')
    fig.suptitle(title)
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Export PurePursuit tracking rosbag topics to CSV for offline analysis.',
    )
    parser.add_argument('bag', type=Path, help='rosbag2 directory (contains metadata.yaml)')
    parser.add_argument(
        '-o',
        '--output-dir',
        type=Path,
        default=None,
        help='Output directory (default: <bag>/csv_export)',
    )
    parser.add_argument('--plot', action='store_true', help='Plot error scalars with matplotlib')
    args = parser.parse_args()

    bag_dir = args.bag.resolve()
    if not bag_dir.is_dir():
        print(f'Not a directory: {bag_dir}', file=sys.stderr)
        return 1

    try:
        (
            long_rows, by_float, closest, pose, twist,
            ocean_current, gps_filtered, cmd_vel, odom_twist, plans, waypoints,
        ) = _read_bag_mixed(bag_dir)
    except Exception as e:
        print(f'Failed to read bag: {e}', file=sys.stderr)
        print('Source ROS 2 setup (rosbag2_py, rclpy, geometry_msgs).', file=sys.stderr)
        return 1

    out_dir = (args.output_dir or (bag_dir / 'csv_export')).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    long_path = out_dir / 'tracking_errors_long.csv'
    wide_path = out_dir / 'tracking_errors_wide.csv'
    gps_path = out_dir / 'gps_filtered_global.csv'
    velocity_path = out_dir / 'velocity_cmd_vs_measured.csv'
    waypoints_path = out_dir / 'mission_waypoints.csv'
    planned_path = out_dir / 'planned_path.csv'
    readme_path = out_dir / 'tracking_export_README.txt'

    _write_long_csv(long_path, long_rows)
    _write_wide_csv(wide_path, by_float, closest, pose, twist, ocean_current, gps_filtered, cmd_vel)
    _write_gps_csv(gps_path, gps_filtered)
    n_vel = _write_velocity_csv(velocity_path, odom_twist, cmd_vel)
    n_wp = _write_waypoints_csv(waypoints_path, waypoints)
    n_legs = _write_planned_path_csv(planned_path, plans)
    _write_readme(readme_path, bag_dir)

    print(f'Wrote {len(long_rows)} long rows -> {long_path}')
    print(f'Wide CSV -> {wide_path}')
    print(f'GPS CSV ({len(gps_filtered)} samples) -> {gps_path}')
    print(f'Velocity CSV ({n_vel} odom samples) -> {velocity_path}')
    print(f'Waypoints CSV ({n_wp} waypoints) -> {waypoints_path}')
    print(f'Planned path CSV ({n_legs} legs) -> {planned_path}')
    print(f'Notes -> {readme_path}')

    if args.plot:
        _maybe_plot(by_float, bag_dir.name)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
