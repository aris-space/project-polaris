#!/usr/bin/env python3
"""Publish EKF stack health on /diagnostics_software for Foxglove.

Kept separate from /diagnostics (hardware vitals: battery, ultrasonics,
temps, pixhawk heartbeat) so the two surfaces can be shown in their own
Foxglove panels. Mirrors the pattern in `nav2_lifecycle_diagnostics`:
one parent status ('EKF Stack', level = worst child) plus one child per
subsystem of the EKF localization pipeline.

Children:
  - EKF Stack: GNSS datum lock        — has gnss_datum_watchdog locked
                                         a datum, and how recent the
                                         /odometry/gps_map stream is.
  - EKF Stack: map→odom TF            — is ekf_global_node broadcasting,
                                         and how stale is the latest TF.
  - EKF Stack: Local odometry         — /odometry/filtered/local heartbeat
                                         + position covariance health.
  - EKF Stack: Global odometry        — /odometry/filtered/global heartbeat
                                         + position covariance health.
  - EKF Stack: GPS pipeline           — raw / validated / filtered rates
                                         (catches dead watchdog or the
                                         velocity filter rejecting all).
  - EKF Stack: State vs GPS divergence — |state − /odometry/gps_map|
                                         (catches an EKF that's drifting
                                         despite GPS being available).

Pre-lock window: before the watchdog fires (typically within the first
30 s after startup), the global / TF / divergence children report WARN
with message 'pre-lock'; the parent stays OK as long as local odometry
and the GPS pipeline are healthy.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

import tf2_ros


_PARENT_NAME = 'EKF Stack'
_PUBLISH_PERIOD_SEC = 1.0
_DIAG_TOPIC = '/diagnostics_software'

# Topic names — mid_v2 canonical pipeline.
_LOCAL_ODOM_TOPIC = '/odometry/filtered/local'
_GLOBAL_ODOM_TOPIC = '/odometry/filtered/global'
_GPS_RAW_TOPIC = '/gps/selected'
_GPS_VALIDATED_TOPIC = '/gps/validated'
_GPS_FILTERED_TOPIC = '/gps/validated_filtered'
_GPS_MAP_TOPIC = '/odometry/gps_map'

# Thresholds (matching the operational envelope of the mid_v2 config).
_ODOM_AGE_WARN_S = 0.5
_ODOM_AGE_ERROR_S = 2.0
_TF_AGE_WARN_S = 1.0
_TF_AGE_ERROR_S = 5.0
_DIVERGENCE_WARN_M = 2.0
_DIVERGENCE_ERROR_M = 10.0
_COV_WARN_M2 = 1.0
_COV_ERROR_M2 = 10.0
_RATE_WINDOW_S = 5.0
# Until the watchdog has had a chance to lock, "global EKF not running"
# and "no map→odom TF" are normal. Suppress them as WARN (not ERROR) until
# this many seconds after node start; afterward they're real failures.
_PRE_LOCK_GRACE_S = 30.0


def _kv(key, value) -> KeyValue:
    return KeyValue(key=str(key), value=str(value))


class EkfDiagnostics(Node):

    def __init__(self) -> None:
        super().__init__('ekf_diagnostics')

        # Latest cached subsystem state.
        self._latest_local: Optional[Odometry] = None
        self._latest_local_t: Optional[float] = None
        self._latest_global: Optional[Odometry] = None
        self._latest_global_t: Optional[float] = None
        self._latest_gps_map: Optional[Odometry] = None
        self._latest_gps_map_t: Optional[float] = None

        # Rolling-window ring buffers (timestamps of msgs in the last
        # _RATE_WINDOW_S). Rate = len(deque) / _RATE_WINDOW_S.
        self._raw_times: deque[float] = deque()
        self._validated_times: deque[float] = deque()
        self._filtered_times: deque[float] = deque()

        self._t_start = self._now_s()

        # TF buffer for the map→odom check. ekf_global_node broadcasts
        # this once it's running; missing/stale TF is the canonical signal
        # that the global EKF has crashed or never started.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            Odometry, _LOCAL_ODOM_TOPIC, self._on_local,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, _GLOBAL_ODOM_TOPIC, self._on_global,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, _GPS_MAP_TOPIC, self._on_gps_map,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix, _GPS_RAW_TOPIC,
            lambda _msg: self._raw_times.append(self._now_s()),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix, _GPS_VALIDATED_TOPIC,
            lambda _msg: self._validated_times.append(self._now_s()),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix, _GPS_FILTERED_TOPIC,
            lambda _msg: self._filtered_times.append(self._now_s()),
            qos_profile_sensor_data,
        )

        self._diag_pub = self.create_publisher(
            DiagnosticArray, _DIAG_TOPIC, 10,
        )
        self.create_timer(_PUBLISH_PERIOD_SEC, self._tick)

        self.get_logger().info(
            f'Publishing EKF stack health to {_DIAG_TOPIC} at '
            f'{1.0 / _PUBLISH_PERIOD_SEC:.1f} Hz'
        )

    # ── Helpers ────────────────────────────────────────────────────────

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pre_lock_grace_active(self, now_s: float) -> bool:
        return (now_s - self._t_start) < _PRE_LOCK_GRACE_S

    # ── Subscriber callbacks ───────────────────────────────────────────

    def _on_local(self, msg: Odometry) -> None:
        self._latest_local = msg
        self._latest_local_t = self._now_s()

    def _on_global(self, msg: Odometry) -> None:
        self._latest_global = msg
        self._latest_global_t = self._now_s()

    def _on_gps_map(self, msg: Odometry) -> None:
        self._latest_gps_map = msg
        self._latest_gps_map_t = self._now_s()

    # ── Per-subsystem checks ───────────────────────────────────────────

    def _check_datum_lock(self, now_s: float) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = f'{_PARENT_NAME}: GNSS datum lock'
        if self._latest_gps_map_t is None:
            if self._pre_lock_grace_active(now_s):
                s.level = DiagnosticStatus.WARN
                s.message = 'pre-lock: waiting for first /odometry/gps_map'
            else:
                s.level = DiagnosticStatus.ERROR
                s.message = 'no /odometry/gps_map past grace window'
            s.values.append(_kv('uptime_s', f'{now_s - self._t_start:.1f}'))
            return s
        age = now_s - self._latest_gps_map_t
        if age > _ODOM_AGE_ERROR_S:
            s.level = DiagnosticStatus.ERROR
            s.message = f'datum locked but gps_map stale ({age:.1f} s)'
        elif age > _ODOM_AGE_WARN_S:
            s.level = DiagnosticStatus.WARN
            s.message = f'datum locked, gps_map aging ({age:.2f} s)'
        else:
            s.level = DiagnosticStatus.OK
            s.message = 'datum locked, gps_map fresh'
        s.values.extend([
            _kv('gps_map_age_s', f'{age:.3f}'),
            _kv('frame_id', self._latest_gps_map.header.frame_id),
        ])
        return s

    def _check_tf(self, now_s: float) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = f'{_PARENT_NAME}: map→odom TF'
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time(),
                Duration(seconds=0.05),
            )
            stamp_s = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = now_s - stamp_s
            if age > _TF_AGE_ERROR_S:
                s.level = DiagnosticStatus.ERROR
                s.message = (
                    f'TF stale ({age:.1f} s) — '
                    'ekf_global_node may have stopped broadcasting'
                )
            elif age > _TF_AGE_WARN_S:
                s.level = DiagnosticStatus.WARN
                s.message = f'TF aging ({age:.2f} s)'
            else:
                s.level = DiagnosticStatus.OK
                s.message = f'TF fresh ({age:.2f} s)'
            s.values.extend([
                _kv('age_s', f'{age:.3f}'),
                _kv('tx_m', f'{tf.transform.translation.x:.3f}'),
                _kv('ty_m', f'{tf.transform.translation.y:.3f}'),
                _kv('tz_m', f'{tf.transform.translation.z:.3f}'),
            ])
        except Exception as e:
            if self._pre_lock_grace_active(now_s):
                s.level = DiagnosticStatus.WARN
                s.message = 'pre-lock: no map→odom yet'
            else:
                s.level = DiagnosticStatus.ERROR
                s.message = f'map→odom TF missing ({type(e).__name__})'
            s.values.append(_kv('uptime_s', f'{now_s - self._t_start:.1f}'))
        return s

    def _check_odom(
        self,
        label: str,
        latest: Optional[Odometry],
        latest_t: Optional[float],
        now_s: float,
        pre_lock_ok: bool,
    ) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = f'{_PARENT_NAME}: {label}'
        if latest is None:
            if pre_lock_ok and self._pre_lock_grace_active(now_s):
                s.level = DiagnosticStatus.WARN
                s.message = f'pre-lock: no messages yet'
            else:
                s.level = DiagnosticStatus.ERROR
                s.message = f'no messages received'
            s.values.append(_kv('uptime_s', f'{now_s - self._t_start:.1f}'))
            return s

        age = now_s - latest_t
        p_xx = float(latest.pose.covariance[0])
        p_yy = float(latest.pose.covariance[7])
        cov_max = max(p_xx, p_yy)

        # Worst of (age, covariance) wins.
        if age > _ODOM_AGE_ERROR_S:
            level, msg = DiagnosticStatus.ERROR, f'stale ({age:.1f} s)'
        elif cov_max > _COV_ERROR_M2:
            level, msg = (
                DiagnosticStatus.ERROR,
                f'covariance exploded ({cov_max:.2f} m²)',
            )
        elif age > _ODOM_AGE_WARN_S:
            level, msg = DiagnosticStatus.WARN, f'aging ({age:.2f} s)'
        elif cov_max > _COV_WARN_M2:
            level, msg = (
                DiagnosticStatus.WARN,
                f'covariance high ({cov_max:.2f} m²)',
            )
        else:
            level, msg = (
                DiagnosticStatus.OK,
                f'fresh ({age:.2f} s, σ_xy ≤ {math.sqrt(cov_max):.2f} m)',
            )
        s.level = level
        s.message = msg
        s.values.extend([
            _kv('age_s', f'{age:.3f}'),
            _kv('P_xx_m2', f'{p_xx:.4f}'),
            _kv('P_yy_m2', f'{p_yy:.4f}'),
            _kv('x_m', f'{latest.pose.pose.position.x:.3f}'),
            _kv('y_m', f'{latest.pose.pose.position.y:.3f}'),
            _kv('z_m', f'{latest.pose.pose.position.z:.3f}'),
        ])
        return s

    def _check_gps_pipeline(self, now_s: float) -> DiagnosticStatus:
        # Prune timestamps outside the rate window.
        for dq in (self._raw_times, self._validated_times, self._filtered_times):
            while dq and now_s - dq[0] > _RATE_WINDOW_S:
                dq.popleft()
        raw_hz = len(self._raw_times) / _RATE_WINDOW_S
        valid_hz = len(self._validated_times) / _RATE_WINDOW_S
        filt_hz = len(self._filtered_times) / _RATE_WINDOW_S

        s = DiagnosticStatus()
        s.name = f'{_PARENT_NAME}: GPS pipeline'
        if raw_hz <= 0.0:
            if self._pre_lock_grace_active(now_s):
                s.level = DiagnosticStatus.WARN
                s.message = 'pre-lock: no raw GPS yet'
            else:
                s.level = DiagnosticStatus.ERROR
                s.message = 'no raw GPS in last window'
        elif valid_hz <= 0.0:
            s.level = DiagnosticStatus.ERROR
            s.message = 'watchdog rejecting all raw fixes (h_acc / null-island gate)'
        elif filt_hz <= 0.0:
            s.level = DiagnosticStatus.WARN
            s.message = 'velocity filter rejecting all validated fixes'
        elif filt_hz < 0.5 * valid_hz and valid_hz > 0.5:
            s.level = DiagnosticStatus.WARN
            s.message = (
                f'velocity filter rejecting > 50% '
                f'(filtered {filt_hz:.2f}/{valid_hz:.2f} Hz)'
            )
        else:
            s.level = DiagnosticStatus.OK
            s.message = (
                f'raw {raw_hz:.2f} → validated {valid_hz:.2f} '
                f'→ filtered {filt_hz:.2f} Hz'
            )
        s.values.extend([
            _kv('raw_hz', f'{raw_hz:.3f}'),
            _kv('validated_hz', f'{valid_hz:.3f}'),
            _kv('filtered_hz', f'{filt_hz:.3f}'),
            _kv('window_s', f'{_RATE_WINDOW_S:.1f}'),
        ])
        return s

    def _check_divergence(self, now_s: float) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = f'{_PARENT_NAME}: State vs GPS divergence'
        if self._latest_global is None or self._latest_gps_map is None:
            if self._pre_lock_grace_active(now_s):
                s.level = DiagnosticStatus.WARN
                s.message = (
                    'pre-lock: need /odometry/filtered/global '
                    'and /odometry/gps_map'
                )
            else:
                s.level = DiagnosticStatus.ERROR
                s.message = 'global state or gps_map missing past grace window'
            s.values.append(_kv('uptime_s', f'{now_s - self._t_start:.1f}'))
            return s
        dx = (self._latest_global.pose.pose.position.x
              - self._latest_gps_map.pose.pose.position.x)
        dy = (self._latest_global.pose.pose.position.y
              - self._latest_gps_map.pose.pose.position.y)
        d = math.hypot(dx, dy)
        if d > _DIVERGENCE_ERROR_M:
            level, msg = (
                DiagnosticStatus.ERROR,
                f'EKF diverged: {d:.2f} m from GPS',
            )
        elif d > _DIVERGENCE_WARN_M:
            level, msg = (
                DiagnosticStatus.WARN,
                f'EKF lagging: {d:.2f} m from GPS',
            )
        else:
            level, msg = (
                DiagnosticStatus.OK,
                f'EKF tracking: {d:.2f} m from GPS',
            )
        s.level = level
        s.message = msg
        s.values.extend([
            _kv('delta_m', f'{d:.3f}'),
            _kv('delta_x_m', f'{dx:.3f}'),
            _kv('delta_y_m', f'{dy:.3f}'),
        ])
        return s

    # ── Tick ───────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now_s = self._now_s()
        children = [
            self._check_datum_lock(now_s),
            self._check_tf(now_s),
            self._check_odom(
                'Local odometry', self._latest_local, self._latest_local_t,
                now_s, pre_lock_ok=False,
            ),
            self._check_odom(
                'Global odometry', self._latest_global, self._latest_global_t,
                now_s, pre_lock_ok=True,
            ),
            self._check_divergence(now_s),
            self._check_gps_pipeline(now_s),
        ]

        worst_level = DiagnosticStatus.OK
        worst_child = None
        ok_count = warn_count = err_count = 0
        for c in children:
            if c.level == DiagnosticStatus.OK:
                ok_count += 1
            elif c.level == DiagnosticStatus.WARN:
                warn_count += 1
            else:
                err_count += 1
            if c.level > worst_level:
                worst_level = c.level
                worst_child = c

        parent = DiagnosticStatus()
        parent.name = _PARENT_NAME
        parent.level = worst_level
        if worst_level == DiagnosticStatus.OK:
            parent.message = f'{ok_count}/{len(children)} OK'
        elif worst_child is not None:
            short_name = worst_child.name.split(': ', 1)[-1]
            parent.message = f'{short_name}: {worst_child.message}'
        else:
            parent.message = 'unknown'
        parent.values = [
            _kv('ok', str(ok_count)),
            _kv('warn', str(warn_count)),
            _kv('error', str(err_count)),
            _kv('total', str(len(children))),
        ]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [parent, *children]
        self._diag_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = EkfDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
