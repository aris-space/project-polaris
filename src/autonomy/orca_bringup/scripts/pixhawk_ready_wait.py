#!/usr/bin/env python3
"""Pixhawk readiness via ``/pixhawk/heartbeat`` — fast fail, no long polling."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from std_msgs.msg import Bool, String


def parse_heartbeat(data: str) -> Optional[tuple[str, bool, int]]:
    """
    Parse ``mode=...;armed=0|1;system_status=N`` from mavlink_bridge.

    Returns (mode_str, armed, system_status) or None if malformed.
    """
    parts: dict[str, str] = {}
    for segment in data.split(';'):
        segment = segment.strip()
        if not segment:
            continue
        key, _, val = segment.partition('=')
        parts[key.strip()] = val.strip()
    try:
        mode = parts['mode']
        armed = parts['armed'] == '1'
        system_status = int(parts['system_status'])
    except (KeyError, ValueError):
        return None
    return mode, armed, system_status


@dataclass
class PixhawkState:
    """Latest parsed heartbeat; updated by subscription callback."""

    mode: str = ''
    armed: bool = False
    system_status: int = 0
    last_msg_time: float = 0.0

    def update_from_string(self, data: str, now: float) -> None:
        parsed = parse_heartbeat(data)
        if parsed is None:
            return
        self.mode, self.armed, self.system_status = parsed
        self.last_msg_time = now


def _spin(executor, timeout_sec: float) -> None:
    executor.spin_once(timeout_sec=timeout_sec)


def _spin_for(executor, duration_sec: float, poll_sec: float = 0.05) -> None:
    end = time.time() + duration_sec
    while rclpy.ok() and time.time() < end:
        _spin(executor, poll_sec)


def mode_matches(mode: str, want: str) -> bool:
    """Heartbeat mode is e.g. GUIDED or UNKNOWN(4); accept prefix / case-insensitive."""
    m = mode.strip().upper()
    w = want.strip().upper()
    if m == w or m.startswith(w + '('):
        return True
    if w == 'GUIDED' and m.startswith('UNKNOWN('):
        try:
            inner = m.split('(', 1)[1].split(')', 1)[0]
            return inner.isdigit() and int(inner) == 4
        except (IndexError, ValueError):
            pass
    return False


def ensure_armed_and_mode_guided(
    executor,
    node,
    arm_pub,
    mode_pub,
    state: PixhawkState,
    *,
    heartbeat_wait_sec: float = 1.5,
    settle_after_arm_sec: float = 1.5,
    settle_after_guided_sec: float = 1.5,
    poll_sec: float = 0.05,
) -> bool:
    """
    Fast gate before Nav2: if Pixhawk is not already linkable and armable **now**, return False.

    Does **not** block for tens of seconds waiting for SITL to become healthy — one short
    heartbeat wait, one arm command, one GUIDED command, brief settle each time, then check.

    ``node`` is unused; kept for call-site compatibility.
    """
    _ = node
    deadline = time.time() + heartbeat_wait_sec
    print(
        f'Checking /pixhawk/heartbeat (up to {heartbeat_wait_sec:.1f}s)',
        flush=True,)
    while rclpy.ok() and time.time() < deadline:
        if state.last_msg_time > 0.0:
            break
        _spin(executor, poll_sec)
    else:
        print(
            'No Pixhawk heartbeat yet.',
            flush=True,
        )
        return False

    print(
        f'Heartbeat: mode={state.mode}, armed={int(state.armed)} — attempting once',
        flush=True,
    )

    print('>>> Arming <<<', flush=True)
    arm_pub.publish(Bool(data=True))
    _spin_for(executor, settle_after_arm_sec, poll_sec)
    if not state.armed:
        print(
            f'Not armed; skipping mission.',
            flush=True,
        )
        return False
    print('Pixhawk reports ARMED.', flush=True)

    print('>>> Setting Pixhawk mode to GUIDED <<<', flush=True)
    mode_pub.publish(String(data='GUIDED'))
    _spin_for(executor, settle_after_guided_sec, poll_sec)
    if not mode_matches(state.mode, 'GUIDED'):
        print(
            f'Not in GUIDED; skipping mission.',
            flush=True,
        )
        return False
    print(f'Pixhawk reports mode {state.mode}.', flush=True)
    return True


def make_heartbeat_callback(state: PixhawkState) -> Callable[[String], None]:
    """Subscription callback factory."""

    def _cb(msg: String) -> None:
        state.update_from_string(msg.data, time.time())

    return _cb
