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
  zikzak      : compound-axis only. First axis runs ONE ramp→hold→ramp cycle (the
                "bounding" axis). All remaining axes zig-zag with alternating sign
                (each leg = ramp+hold+ramp returning to 0), repeating +leg, −leg, +leg, …
                until the bounding axis finishes. Any mid-leg at cutoff is truncated
                cleanly back to 0.

Axis: surge (linear.x), heave (linear.z), yaw (angular.z), or sway (linear.y).
Compound axes: join with underscores to drive several axes in the same Twist,
e.g. surge_yaw, heave_yaw, surge_sway_yaw. Each axis then gets its own
target/ramp/hold value, mapped positionally.

Examples:
  ./cmd_vel_ramp.py ramp      surge 0.20 --ramp 5 --hold 5
  ./cmd_vel_ramp.py step      surge 0.35 --hold 6        # equivalent to ros2 topic pub
  ./cmd_vel_ramp.py triangle  surge 0.20 --ramp 4        # rising edge then immediate fall
  ./cmd_vel_ramp.py bipolar   surge 0.20 --ramp 3 --hold 2
  ./cmd_vel_ramp.py staircase surge 0.30 --hold 3        # 0.075 → 0.15 → 0.225 → 0.30
  ./cmd_vel_ramp.py ramp      yaw   0.30 --ramp 4 --hold 4
  # Compound: surge with (target=0.40, ramp=1.33, hold=30) AND yaw with (0.30, 10, 50)
  ./cmd_vel_ramp.py ramp      surge_yaw 0.40 0.30 --ramp 1.33 10 --hold 30 50
  # zikzak: surge runs one ramp cycle (20+1000+20 = 1040s); yaw zig-zags +/-0.2
  # with each leg = 10+5+10 = 25s, until surge ends.
  ./cmd_vel_ramp.py zikzak    surge_yaw 0.25 0.20 --ramp 20 10 --hold 1000 5
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


def build_zikzak_zigzag(target, ramp_s, hold_s, bound_duration):
    """Alternating +leg/-leg keyframes (each leg = ramp+hold+ramp back to 0),
    truncated cleanly to 0 at bound_duration. Used by the zikzak profile for
    non-bounding axes."""
    leg = ramp_s + hold_s + ramp_s  # one full +leg or -leg duration
    kfs = [(0.0, 0.0)]
    t = 0.0
    sign = +1.0
    while t < bound_duration:
        leg_end = t + leg
        if leg_end <= bound_duration:
            kfs.append((t + ramp_s,              sign * target))
            kfs.append((t + ramp_s + hold_s,     sign * target))
            kfs.append((leg_end,                 0.0))
            t = leg_end
            sign = -sign
        else:
            # Partial final leg: ramp up as far as we can, then ramp back to 0
            # so we end cleanly at bound_duration with value 0.
            remaining = bound_duration - t
            if remaining <= 2 * ramp_s:
                # Not enough room for a full triangle — scale a symmetric triangle peak.
                half = remaining / 2.0
                peak = sign * target * (half / ramp_s) if ramp_s > 0 else 0.0
                kfs.append((t + half,            peak))
                kfs.append((bound_duration,      0.0))
            else:
                hold_partial = remaining - 2 * ramp_s
                kfs.append((t + ramp_s,                       sign * target))
                kfs.append((t + ramp_s + hold_partial,        sign * target))
                kfs.append((bound_duration,                   0.0))
            break
    return kfs


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
        # One channel per axis; each has its own keyframes/duration but shares the profile shape.
        self.channels = []
        if args.profile == "zikzak":
            # First axis = bounding (one ramp cycle). Remaining axes zig-zag for the
            # bounding axis's full duration.
            bound_axis, *zigzag_axes = args.axes
            bound_target, *zigzag_targets = args.targets
            bound_ramp, *zigzag_ramps = args.ramps
            bound_hold, *zigzag_holds = args.holds
            bound_kfs = build_profile("ramp", bound_target, bound_ramp, bound_hold)
            bound_duration = bound_kfs[-1][0]
            field, attr = AXIS_MAP[bound_axis]
            self.channels.append({
                "axis": bound_axis, "field": field, "attr": attr,
                "kfs": bound_kfs, "target": bound_target, "ramp": bound_ramp, "hold": bound_hold,
            })
            for axis_name, target, ramp_s, hold_s in zip(zigzag_axes, zigzag_targets, zigzag_ramps, zigzag_holds):
                field, attr = AXIS_MAP[axis_name]
                kfs = build_zikzak_zigzag(target, ramp_s, hold_s, bound_duration)
                self.channels.append({
                    "axis": axis_name, "field": field, "attr": attr,
                    "kfs": kfs, "target": target, "ramp": ramp_s, "hold": hold_s,
                })
        else:
            for axis_name, target, ramp_s, hold_s in zip(args.axes, args.targets, args.ramps, args.holds):
                field, attr = AXIS_MAP[axis_name]
                kfs = build_profile(args.profile, target, ramp_s, hold_s)
                self.channels.append({
                    "axis": axis_name, "field": field, "attr": attr,
                    "kfs": kfs, "target": target, "ramp": ramp_s, "hold": hold_s,
                })
        self.duration = max(ch["kfs"][-1][0] for ch in self.channels)
        self.t0 = time.monotonic()
        self.tail_zeros = args.tail_zeros
        self.rate_hz = args.rate
        self._tail_count = 0
        self.timer = self.create_timer(1.0 / args.rate, self._tick)
        desc = ", ".join(
            f"{ch['axis']}(target={ch['target']} ramp={ch['ramp']}s hold={ch['hold']}s)"
            for ch in self.channels
        )
        self.get_logger().info(
            f"profile={args.profile} channels=[{desc}] total={self.duration:.1f}s @ {args.rate} Hz"
        )

    def _tick(self):
        t = time.monotonic() - self.t0
        msg = Twist()
        if t <= self.duration:
            parts = []
            for ch in self.channels:
                v = lerp_keyframes(t, ch["kfs"]) if t <= ch["kfs"][-1][0] else 0.0
                setattr(getattr(msg, ch["field"]), ch["attr"], float(v))
                parts.append(f"{ch['field']}.{ch['attr']}={v:+.4f}")
            self.pub.publish(msg)
            if int(t * 5) != int((t - 1.0/self.rate_hz) * 5):  # ~5 Hz log
                self.get_logger().info(f"t={t:5.2f}s   " + "  ".join(parts))
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
                self.get_logger().info("done — profile complete, node keeping alive (Ctrl-C to stop)")
                self.timer.cancel()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("profile", choices=["ramp", "trapezoid", "step", "triangle", "bipolar", "staircase", "zikzak"])
    p.add_argument("axis", help="single axis (surge, yaw, …) or compound joined by underscores (surge_yaw, heave_yaw_sway)")
    p.add_argument("targets", type=float, nargs="+",
                   help="target per axis (m/s for linear, rad/s for angular); one value per axis in the compound name")
    p.add_argument("--ramp", dest="ramps", type=float, nargs="+", default=None,
                   help="ramp seconds per axis (each edge); single value broadcasts to all axes")
    p.add_argument("--hold", dest="holds", type=float, nargs="+", default=None,
                   help="hold seconds per axis at peak; single value broadcasts to all axes")
    p.add_argument("--rate", type=float, default=20.0, help="publish rate Hz (matches bridge expectation)")
    p.add_argument("--topic", default="/pixhawk/cmd_vel")
    p.add_argument("--tail-zeros", type=int, default=10, help="explicit zero messages after profile (-1 to skip — lets watchdog handle stop)")
    args = p.parse_args()

    args.axes = args.axis.split("_")
    unknown = [a for a in args.axes if a not in AXIS_MAP]
    if unknown:
        p.error(f"unknown axis name(s): {unknown}. Valid: {list(AXIS_MAP.keys())}")
    if len(set(args.axes)) != len(args.axes):
        p.error(f"duplicate axes in compound name: {args.axes}")
    if args.profile == "zikzak" and len(args.axes) < 2:
        p.error("zikzak requires a compound axis with at least 2 axes (e.g. surge_yaw)")

    n = len(args.axes)

    def _broadcast(name, values, default):
        if values is None:
            return [default] * n
        if len(values) == 1:
            return values * n
        if len(values) != n:
            p.error(f"--{name} expects 1 or {n} values to match axis '{args.axis}', got {len(values)}")
        return values

    if len(args.targets) == 1 and n > 1:
        args.targets = args.targets * n
    elif len(args.targets) != n:
        p.error(f"target expects 1 or {n} values to match axis '{args.axis}', got {len(args.targets)}")
    args.ramps = _broadcast("ramp", args.ramps, 4.0)
    args.holds = _broadcast("hold", args.holds, 4.0)
    return args


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
