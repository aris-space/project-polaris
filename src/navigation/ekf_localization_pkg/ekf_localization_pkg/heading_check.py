"""
heading_check — live check of the heading the stack uses against RTK GNSS.

While the vehicle moves at the surface, compares our heading (/imu/data_corrected, ENU)
with the heading implied by GNSS: course over ground from /odometry/gps_map (true-ENU map
positions, no IMU yaw involved) minus the crab angle atan2(vy, vx) from the body-frame
velocity in /odometry/filtered/local (DVL), so sideways drift does not count as heading
error.

Publishes (latched, so the last value is visible after the vehicle stops and is in bags):
  /heading_check/error_deg   Float64  our heading − true heading, median of recent windows
  /heading_check/status      String   JSON: per-window error, suggested yaw_offset_deg, ...

A window is evaluated only when the GNSS displacement over window_s is ≥ min_distance_m and
the vehicle did not turn more than max_yaw_change_deg, i.e. a reasonably straight leg.
Suggested offset = true heading − raw IMU yaw (/imu/data) = what imu_yaw_correction's
yaw_offset_deg should be.

Run on demand:  ros2 run ekf_localization_pkg heading_check
"""
from __future__ import annotations

import json
import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def circ_mean_rad(angles) -> float:
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def heading_error_deg(dx: float, dy: float, vx_body: float, vy_body: float,
                      our_yaw_rad: float) -> tuple[float, float]:
    """Return (our heading − true heading, true heading) in degrees, ENU.

    Course over ground atan2(dy, dx) = heading + crab, with crab = atan2(vy, vx) in the body
    frame, so true heading = course − crab.
    """
    course = math.degrees(math.atan2(dy, dx))
    crab = math.degrees(math.atan2(vy_body, vx_body))
    true_heading = wrap_deg(course - crab)
    return wrap_deg(math.degrees(our_yaw_rad) - true_heading), true_heading


class HeadingCheck(Node):

    def __init__(self) -> None:
        super().__init__("heading_check")
        self.declare_parameter("gps_map_topic", "/odometry/gps_map")
        self.declare_parameter("imu_corrected_topic", "/imu/data_corrected")
        self.declare_parameter("imu_raw_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odometry/filtered/local")
        self.declare_parameter("window_s", 4.0)
        self.declare_parameter("min_distance_m", 0.4)
        self.declare_parameter("max_yaw_change_deg", 15.0)
        self.declare_parameter("warn_threshold_deg", 10.0)
        self.declare_parameter("median_of_last", 5)

        self._gps: deque = deque(maxlen=400)   # (t, x, y)
        self._yaw: deque = deque(maxlen=2000)  # (t, our yaw rad)
        self._raw: deque = deque(maxlen=2000)  # (t, raw yaw rad)
        self._vel: deque = deque(maxlen=1000)  # (t, vx, vy) body frame
        self._errors: deque = deque(maxlen=int(self.get_parameter("median_of_last").value))

        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._err_pub = self.create_publisher(Float64, "/heading_check/error_deg", latched)
        self._status_pub = self.create_publisher(String, "/heading_check/status", latched)

        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, self.get_parameter("gps_map_topic").value, self._on_gps, sd)
        self.create_subscription(Imu, self.get_parameter("imu_corrected_topic").value, self._on_yaw, sd)
        self.create_subscription(Imu, self.get_parameter("imu_raw_topic").value, self._on_raw, sd)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._on_odom, sd)
        self.get_logger().info("heading_check: drive straight at the surface (RTK) to evaluate heading")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_yaw(self, msg: Imu) -> None:
        self._yaw.append((self._now(), yaw_from_quat(msg.orientation)))

    def _on_raw(self, msg: Imu) -> None:
        self._raw.append((self._now(), yaw_from_quat(msg.orientation)))

    def _on_odom(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._vel.append((self._now(), v.x, v.y))

    @staticmethod
    def _since(buf, t0):
        return [s for s in buf if s[0] >= t0]

    def _on_gps(self, msg: Odometry) -> None:
        t = self._now()
        p = msg.pose.pose.position
        self._gps.append((t, p.x, p.y))
        window = float(self.get_parameter("window_s").value)
        t0 = t - window
        old = [g for g in self._gps if g[0] <= t0]
        if not old:
            return
        start = old[-1]
        # /odometry/gps_map only carries fixes that passed the datum watchdog's accuracy
        # gate, so dives leave gaps. Only evaluate continuous good-GNSS windows: the start
        # sample must be ~window_s old and no gap > 1.5 s inside the window.
        if t - start[0] > window + 1.0:
            return
        win = [g[0] for g in self._gps if g[0] >= start[0]]
        if any(b - a > 1.5 for a, b in zip(win, win[1:])):
            return
        dx, dy = p.x - start[1], p.y - start[2]
        dist = math.hypot(dx, dy)
        if dist < float(self.get_parameter("min_distance_m").value):
            return
        yaws = [y for _, y in self._since(self._yaw, start[0])]
        raws = [y for _, y in self._since(self._raw, start[0])]
        vels = self._since(self._vel, start[0])
        if len(yaws) < 5 or len(raws) < 5 or len(vels) < 3:
            return
        mean_yaw = circ_mean_rad(yaws)
        yaw_change = max(abs(wrap_deg(math.degrees(y - mean_yaw))) for y in yaws) * 2.0
        if yaw_change > float(self.get_parameter("max_yaw_change_deg").value):
            return
        vx = sum(v[1] for v in vels) / len(vels)
        vy = sum(v[2] for v in vels) / len(vels)
        if math.hypot(vx, vy) < 0.03:
            return  # body velocity too small to estimate the crab angle
        err, true_heading = heading_error_deg(dx, dy, vx, vy, mean_yaw)
        self._errors.append(err)
        med = sorted(self._errors)[len(self._errors) // 2]
        suggested = wrap_deg(true_heading - math.degrees(circ_mean_rad(raws)))
        self._err_pub.publish(Float64(data=float(med)))
        self._status_pub.publish(String(data=json.dumps(dict(
            error_deg=round(err, 1),
            error_median_deg=round(med, 1),
            n_windows=len(self._errors),
            gnss_distance_m=round(dist, 2),
            our_heading_enu_deg=round(math.degrees(mean_yaw), 1),
            true_heading_enu_deg=round(true_heading, 1),
            crab_deg=round(math.degrees(math.atan2(vy, vx)), 1),
            suggested_yaw_offset_deg=round(suggested, 1),
        ))))
        thr = float(self.get_parameter("warn_threshold_deg").value)
        msg_txt = (f"heading error {err:+.1f}° (median {med:+.1f}° over {len(self._errors)}), "
                   f"suggested yaw_offset_deg {suggested:+.1f}°")
        if abs(med) > thr:
            self.get_logger().warn("HEADING OFF: " + msg_txt, throttle_duration_sec=2.0)
        else:
            self.get_logger().info("heading OK: " + msg_txt, throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
