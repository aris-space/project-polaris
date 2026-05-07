#!/usr/bin/env python3
"""
Send shaped /pixhawk/cmd_vel test signals at 20 Hz.

Profiles:
  ramp        : 0 → target over `ramp_s`, hold `hold_s`, target → 0 over `ramp_s`, stop.
  trapezoid   : alias of ramp.
  step        : instant 0 → target, hold `hold_s`, instant target → 0.
  triangle    : 0 → target → 0 with linear edges only (no flat top).
  bipolar     : 0 → +target → 0 → -target → 0, each leg is `ramp_s` ramp + `hold_s` hold.
  staircase   : 0 → 0.25*target → 0.50*target → 0.75*target → target → 0.
                Each level held for `hold_s`. Sweeps amplitude in one run; useful
                for finding motor deadband and characterising tracking vs amplitude.

Axis: surge (linear.x), heave (linear.z), yaw (angular.z), or sway (linear.y).

Examples:
  ./cmd_vel_ramp.py ramp      surge 0.20 --ramp 5 --hold 5
  ./cmd_vel_ramp.py step      surge 0.35 --hold 6        # equivalent to ros2 topic pub
  ./cmd_vel_ramp.py triangle  surge 0.20 --ramp 4        # rising edge then immediate fall
  ./cmd_vel_ramp.py bipolar   surge 0.20 --ramp 3 --hold 2
  ./cmd_vel_ramp.py staircase surge 0.30 --hold 3        # 0.075 → 0.15 → 0.225 → 0.30
  ./cmd_vel_ramp.py ramp      yaw   0.30 --ramp 4 --hold 4
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


AXIS_MAP = {
    "surge":  ("linear",  "x"),   # forward in ROS REP-103 FLU
    "sway":   ("linear",  "y"),   # left
    "heave":  ("linear",  "z"),   # up
    "yaw":    ("angular", "z"),   # CCW
    "pitch":  ("angular", "y"),
    "roll":   ("angular", "x"),
}


def build_profile(profile, target, ramp_s, hold_s):
    """Return list of (t_seconds_from_start, value) keyframes. Linear interp between."""
    if profile in ("ramp", "trapezoid"):
        return [
            (0.0,                        0.0),
            (ramp_s,                     target),
            (ramp_s + hold_s,            target),
            (ramp_s + hold_s + ramp_s,   0.0),
        ]
    if profile == "step":
        return [
            (0.0,        0.0),
            (1e-3,       target),
            (hold_s,     target),
            (hold_s + 1e-3, 0.0),
        ]
    if profile == "triangle":
        return [
            (0.0,            0.0),
            (ramp_s,         target),
            (ramp_s + ramp_s, 0.0),
        ]
    if profile == "bipolar":
        return [
            (0.0,                                 0.0),
            (ramp_s,                              +target),
            (ramp_s + hold_s,                     +target),
            (ramp_s + hold_s + ramp_s,            0.0),
            (ramp_s + hold_s + ramp_s + ramp_s,   -target),
            (ramp_s + hold_s + ramp_s + ramp_s + hold_s, -target),
            (ramp_s + hold_s + ramp_s + hold_s + 3*ramp_s, 0.0),
        ]
    if profile == "staircase":
        # 4 levels at 25/50/75/100% of target, instant edges between, each held hold_s.
        levels = [0.25, 0.50, 0.75, 1.00]
        kfs = [(0.0, 0.0), (1e-3, levels[0] * target)]
        t = hold_s
        for i, frac in enumerate(levels[1:], start=1):
            kfs.append((t,        levels[i-1] * target))
            kfs.append((t + 1e-3, frac        * target))
            t += hold_s
        kfs.append((t,        target))
        kfs.append((t + 1e-3, 0.0))
        return kfs
    raise ValueError(f"unknown profile: {profile}")


def lerp_keyframes(t, kfs):
    """Piecewise-linear interpolation. Returns 0 outside the keyframe range."""
    if t <= kfs[0][0]:
        return kfs[0][1]
    if t >= kfs[-1][0]:
        return kfs[-1][1]
    for (t0, v0), (t1, v1) in zip(kfs, kfs[1:]):
        if t0 <= t <= t1:
            if t1 == t0:
                return v1
            a = (t - t0) / (t1 - t0)
            return v0 + a * (v1 - v0)
    return 0.0


class RampPublisher(Node):
    def __init__(self, args):
        super().__init__("cmd_vel_ramp")
        self.pub = self.create_publisher(Twist, args.topic, 1)
        self.kfs = build_profile(args.profile, args.target, args.ramp, args.hold)
        self.duration = self.kfs[-1][0]
        self.t0 = time.monotonic()
        self.field, self.attr = AXIS_MAP[args.axis]
        self.tail_zeros = args.tail_zeros
        self.rate_hz = args.rate
        self._tail_count = 0
        self.timer = self.create_timer(1.0 / args.rate, self._tick)
        self.get_logger().info(
            f"profile={args.profile} axis={args.axis} target={args.target} "
            f"ramp={args.ramp}s hold={args.hold}s total={self.duration:.1f}s @ {args.rate} Hz"
        )

    def _tick(self):
        t = time.monotonic() - self.t0
        msg = Twist()
        if t <= self.duration:
            v = lerp_keyframes(t, self.kfs)
            getattr(getattr(msg, self.field), self.attr).__class__  # no-op type check
            setattr(getattr(msg, self.field), self.attr, float(v))
            self.pub.publish(msg)
            if int(t * 5) != int((t - 1.0/self.rate_hz) * 5):  # ~5 Hz log
                self.get_logger().info(f"t={t:5.2f}s   {self.field}.{self.attr} = {v:+.4f}")
        else:
            # Tail: send a few explicit zero messages so the bridge watchdog
            # sees a clean stop instead of timing out (which is what we are
            # actually testing under "step" or "ramp" profiles).
            if self._tail_count < self.tail_zeros:
                self.pub.publish(Twist())
                self._tail_count += 1
                if self._tail_count == 1:
                    self.get_logger().info(f"profile done at t={t:.2f}s; sending {self.tail_zeros} explicit zeros")
            else:
                self.get_logger().info("done")
                rclpy.shutdown()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("profile", choices=["ramp", "trapezoid", "step", "triangle", "bipolar", "staircase"])
    p.add_argument("axis", choices=list(AXIS_MAP.keys()))
    p.add_argument("target", type=float, help="target value (m/s for linear, rad/s for angular)")
    p.add_argument("--ramp", type=float, default=4.0, help="ramp duration in seconds (each edge)")
    p.add_argument("--hold", type=float, default=4.0, help="hold duration at peak in seconds")
    p.add_argument("--rate", type=float, default=20.0, help="publish rate Hz (matches bridge expectation)")
    p.add_argument("--topic", default="/pixhawk/cmd_vel")
    p.add_argument("--tail-zeros", type=int, default=10, help="explicit zero messages after profile (-1 to skip — lets watchdog handle stop)")
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RampPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
