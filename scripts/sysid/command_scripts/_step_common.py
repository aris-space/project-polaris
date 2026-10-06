#!/usr/bin/env python3
"""
Shared runner for open-loop surge step maneuvers (maneuvers.md section 2.3).

Publishes a constant Int16MultiArray on /pixhawk/manual_control for a fixed
duration, then returns to neutral. Does NOT switch mode or arm the vehicle —
do that first (see CHECK_AXIS.md section 0.6):

    ros2 topic pub --once /pixhawk/mode_cmd std_msgs/msg/String "data: 'MANUAL'"
    ros2 topic pub --once /pixhawk/arm_cmd std_msgs/msg/Bool "data: true"

Int16MultiArray layout: [x, y, z, r, s, t] = [surge, sway, heave, yaw, roll, pitch].
Raw field range is -1000..1000 (0 = neutral); heave is 0..1000 (500 = neutral).
Commanded values are capped at +-500 here, per the operational limit in use.

Event markers: every run also publishes std_msgs/String on /sysid/event, so the
recorded bag can be segmented without guessing times from the PWM. The payload is
JSON, e.g. {"maneuver": "coastdown", "event": "thrust_cut", "surge": 400, ...}:

    thrust_on   published together with the first surge command
    thrust_cut  published together with the first neutral command (the moment
                the thrusters are switched off; the start of a coast-down)
    done        after the neutral burst; 'interrupted' replaces 'done' on Ctrl-C

The marker is only useful if /sysid/event is in the recorded topic list.
"""

import json
import sys
import time

import rclpy
from std_msgs.msg import Int16MultiArray, String

NEUTRAL = [0, 0, 500, 0, 0, 0]
PUBLISH_HZ = 20.0
SETTLE_SEC = 0.5  # wait for the publisher to be discovered before sending
SURGE_CAP = 500  # operational cap on commanded surge, not the raw -1000..1000 field range
EVENT_TOPIC = '/sysid/event'


def run_constant_surge(surge: int, duration_s: float, node_name: str,
                       maneuver: str = 'step') -> int:
    """Publish a constant surge command for duration_s, then neutral. Returns exit code."""
    surge = max(-SURGE_CAP, min(SURGE_CAP, int(surge)))
    step_msg = Int16MultiArray()
    step_msg.data = [surge, 0, 500, 0, 0, 0]
    neutral_msg = Int16MultiArray()
    neutral_msg.data = list(NEUTRAL)

    rclpy.init(args=None)
    node = rclpy.create_node(node_name)
    pub = node.create_publisher(Int16MultiArray, '/pixhawk/manual_control', 10)
    event_pub = node.create_publisher(String, EVENT_TOPIC, 10)

    def mark(event: str, **extra) -> None:
        payload = {'maneuver': maneuver, 'event': event, 'node': node_name,
                   'surge': surge, 'hold_s': duration_s, **extra}
        msg = String()
        msg.data = json.dumps(payload)
        event_pub.publish(msg)
        print(f"\n[{node_name}] EVENT {event}" if event != 'thrust_on'
              else f"[{node_name}] EVENT {event}")

    interrupted = False
    t0 = None
    try:
        print(f"[{node_name}] {maneuver}  surge={surge}  duration={duration_s:g}s")
        print(f"[{node_name}] vehicle must already be ARMED and in MANUAL mode.")
        time.sleep(SETTLE_SEC)

        period = 1.0 / PUBLISH_HZ
        n_steps = int(duration_s * PUBLISH_HZ)
        t0 = time.monotonic()
        for i in range(n_steps):
            pub.publish(step_msg)
            if i == 0:
                mark('thrust_on')
            rclpy.spin_once(node, timeout_sec=0.0)
            elapsed = time.monotonic() - t0
            print(f"\r[{node_name}] running  {elapsed:5.1f}s / {duration_s:g}s", end="", flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - t0) % period))
    except KeyboardInterrupt:
        interrupted = True
        print(f"\n[{node_name}] interrupted, returning to neutral")
    finally:
        held = None if t0 is None else round(time.monotonic() - t0, 3)
        pub.publish(neutral_msg)
        mark('thrust_cut', held_s=held, interrupted=interrupted)
        for _ in range(9):
            time.sleep(1.0 / PUBLISH_HZ)
            pub.publish(neutral_msg)
        mark('interrupted' if interrupted else 'done')
        time.sleep(0.2)  # let the last marker go out before the node is torn down
        node.destroy_node()
        rclpy.shutdown()

    print(f"[{node_name}] done")
    return 0


def main_for(surge: int, default_duration_s: float, node_name: str,
             maneuver: str = 'step') -> int:
    """Small CLI wrapper: optional positional duration override in seconds."""
    duration_s = default_duration_s
    if len(sys.argv) > 1:
        duration_s = float(sys.argv[1])
    return run_constant_surge(surge, duration_s, node_name, maneuver)
