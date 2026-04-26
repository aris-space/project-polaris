#!/usr/bin/env python3
"""
Operator command topics in raw ROS 2 MCAP bags (recordings/rosbags only).

Deserializes /joy, /joy_controller (sensor_msgs/Joy) and /pixhawk/manual_control
(std_msgs/Int16MultiArray). Summarizes rates, which axes/channels move, and
time intervals (bag-relative seconds) where commands deviate from neutral or
change sharply — useful for maneuver segmentation.

Dependency: pip install rosbags

Does not use __bodyframe companion bags.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("Install: pip install rosbags", file=sys.stderr)
    raise

# Align with manual_control_node neutral: [0, 0, 500, 0, 0, 0]
MC_NEUTRAL = (0, 0, 500, 0, 0, 0)
MC_LABELS = ("x_surge", "y_sway", "z_heave", "r_yaw", "s_roll", "t_pitch")
MC_THRESH = 45  # int16 units (~4.5% of full scale); tweak for noisier logs

# Joy: FOXGLOVE layout (config_pkg.constants, CONTROLLER_LAYOUT == FOXGLOVE)
JOY_AXIS_SURGE = 1
JOY_AXIS_SWAY = 0
JOY_AXIS_YAW = 2
JOY_AXIS_PITCH = 3
JOY_AXIS_L2 = 4
JOY_AXIS_R2 = 5
JOY_STICK_DEAD = 0.09
JOY_TRIG_NET_DEAD = 0.12  # on heave_net after (r2-l2)/2 style


def _under_raw_rosbags(path: Path) -> bool:
    lower = [p.lower() for p in path.parts]
    if "rosbags" not in lower:
        return False
    if any("bodyframe" in p.lower() for p in path.parts):
        return False
    return True


def find_raw_bag_dirs(root: Path) -> list[Path]:
    out: list[Path] = []
    for meta in root.rglob("metadata.yaml"):
        d = meta.parent.resolve()
        if not _under_raw_rosbags(d):
            continue
        if d.name.endswith("__bodyframe"):
            continue
        if any(d.glob("*.mcap")):
            out.append(d)
    return sorted(set(out))


def _merge_close_runs(
    runs: list[tuple[int, int]], max_gap_ns: int = 250_000_000
) -> list[tuple[int, int]]:
    if not runs:
        return []
    merged = [runs[0]]
    for s, e in runs[1:]:
        ps, pe = merged[-1]
        if s - pe <= max_gap_ns:
            merged[-1] = (ps, e)
        else:
            merged.append((s, e))
    return merged


def _runs_from_flags(times_ns: list[int], flags: list[bool]) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    run_start: int | None = None
    run_end: int | None = None
    for t, f in zip(times_ns, flags):
        if f:
            if run_start is None:
                run_start = t
            run_end = t
        else:
            if run_start is not None and run_end is not None:
                runs.append((run_start, run_end))
            run_start = None
            run_end = None
    if run_start is not None and run_end is not None:
        runs.append((run_start, run_end))
    return runs


def _manual_deviations(data: list[int]) -> tuple[bool, list[bool]]:
    """Return (any_deviation, per-channel bool)."""
    if len(data) < 6:
        return False, [False] * 6
    ch: list[bool] = []
    for i, v in enumerate(data[:6]):
        vi = int(v)
        n = MC_NEUTRAL[i]
        ch.append(abs(vi - n) > MC_THRESH)
    return any(ch), ch


def _joy_deviations(msg: Any) -> tuple[bool, list[bool]]:
    """FOXGLOVE-style axes; heave from L2/R2. Per-axis active flags (6 axes)."""
    axes = list(msg.axes) if msg.axes is not None else []
    n = max(6, len(axes))
    while len(axes) < n:
        axes.append(0.0)
    sticks = [False] * 6
    if len(axes) > JOY_AXIS_SURGE:
        sticks[JOY_AXIS_SURGE] = abs(axes[JOY_AXIS_SURGE]) > JOY_STICK_DEAD
    if len(axes) > JOY_AXIS_SWAY:
        sticks[JOY_AXIS_SWAY] = abs(axes[JOY_AXIS_SWAY]) > JOY_STICK_DEAD
    if len(axes) > JOY_AXIS_YAW:
        sticks[JOY_AXIS_YAW] = abs(axes[JOY_AXIS_YAW]) > JOY_STICK_DEAD
    if len(axes) > JOY_AXIS_PITCH:
        sticks[JOY_AXIS_PITCH] = abs(axes[JOY_AXIS_PITCH]) > JOY_STICK_DEAD
    heave = False
    if len(axes) > max(JOY_AXIS_L2, JOY_AXIS_R2):
        l2 = (axes[JOY_AXIS_L2] + 1.0) * 0.5
        r2 = (axes[JOY_AXIS_R2] + 1.0) * 0.5
        heave = abs(r2 - l2) > JOY_TRIG_NET_DEAD
    buttons = list(msg.buttons) if msg.buttons is not None else []
    roll_btn = False
    if len(buttons) > 5:
        roll_btn = bool(buttons[4]) or bool(buttons[5])  # L1, R1 typical FOXGLOVE
    any_act = any(sticks) or heave or roll_btn
    return any_act, sticks + [heave, roll_btn]  # length 8; caller uses summary


def _large_step_manual(prev: list[int] | None, cur: list[int], step: int = 120) -> bool:
    if prev is None or len(prev) < 6 or len(cur) < 6:
        return False
    for i in range(6):
        if abs(int(cur[i]) - int(prev[i])) >= step:
            return True
    return False


def analyze_topic_manual_control(reader: AnyReader, t0_ns: int) -> dict[str, Any]:
    conns = [c for c in reader.connections if c.topic == "/pixhawk/manual_control"]
    if not conns:
        return {"present": False}

    times: list[int] = []
    flags_any: list[bool] = []
    per_ch: list[list[bool]] = [[] for _ in range(6)]
    max_abs = [0, 0, 0, 0, 0, 0]
    prev: list[int] | None = None
    step_flags: list[bool] = []
    n = 0

    for c, ts, raw in reader.messages(connections=conns):
        n += 1
        msg = reader.deserialize(raw, c.msgtype)
        data = list(msg.data) if getattr(msg, "data", None) is not None else []
        if len(data) < 6:
            continue
        for i in range(6):
            max_abs[i] = max(max_abs[i], abs(int(data[i])))
        any_d, chb = _manual_deviations(data)
        times.append(ts)
        flags_any.append(any_d)
        for i in range(6):
            per_ch[i].append(chb[i])
        step_flags.append(_large_step_manual(prev, data))
        prev = [int(x) for x in data[:6]]

    if not times:
        return {"present": False, "messages": 0, "note": "no_messages_on_topic"}

    span_s = (times[-1] - times[0]) / 1e9
    hz = n / span_s if span_s > 0 else 0.0
    t0 = t0_ns
    runs_any = _merge_close_runs(_runs_from_flags(times, flags_any))
    runs_step = _merge_close_runs(_runs_from_flags(times, step_flags))
    ch_runs: dict[str, list[tuple[float, float]]] = {}
    for i, name in enumerate(MC_LABELS):
        rs = _merge_close_runs(_runs_from_flags(times, per_ch[i]))
        ch_runs[name] = [((s - t0) / 1e9, (e - t0) / 1e9) for s, e in rs]

    def _fmt_runs(runs: list[tuple[int, int]]) -> list[tuple[float, float]]:
        return [((s - t0) / 1e9, (e - t0) / 1e9) for s, e in runs]

    active_frac = sum(flags_any) / len(flags_any) if flags_any else 0.0

    return {
        "present": True,
        "messages": n,
        "approx_mean_hz": round(hz, 2),
        "max_abs_per_channel": dict(zip(MC_LABELS, max_abs)),
        "fraction_msgs_any_axis_active": round(active_frac, 3),
        "intervals_any_deviation_s": _fmt_runs(runs_any),
        "intervals_large_step_s": _fmt_runs(runs_step),
        "intervals_per_channel_s": ch_runs,
    }


def analyze_topic_joy(reader: AnyReader, topic: str, t0_ns: int) -> dict[str, Any]:
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        return {"present": False}

    times: list[int] = []
    flags: list[bool] = []
    max_axes: list[float] = []
    n_axes = 0
    n_buttons = 0
    n = 0

    for c, ts, raw in reader.messages(connections=conns):
        n += 1
        msg = reader.deserialize(raw, c.msgtype)
        ax = list(msg.axes) if msg.axes is not None else []
        bt = list(msg.buttons) if msg.buttons is not None else []
        n_axes = max(n_axes, len(ax))
        n_buttons = max(n_buttons, len(bt))
        while len(max_axes) < len(ax):
            max_axes.append(0.0)
        for i, v in enumerate(ax):
            max_axes[i] = max(max_axes[i], abs(float(v)))
        active, _ = _joy_deviations(msg)
        times.append(ts)
        flags.append(active)

    if not times:
        return {"present": False, "messages": 0, "note": "no_messages_on_topic"}

    span_s = (times[-1] - times[0]) / 1e9
    hz = n / span_s if span_s > 0 else 0.0
    runs = _merge_close_runs(_runs_from_flags(times, flags))
    t0 = t0_ns
    intervals = [((s - t0) / 1e9, (e - t0) / 1e9) for s, e in runs]
    active_frac = sum(flags) / len(flags) if flags else 0.0

    return {
        "present": True,
        "messages": n,
        "approx_mean_hz": round(hz, 2),
        "max_num_axes_seen": n_axes,
        "max_num_buttons_seen": n_buttons,
        "max_abs_per_axis_index": max_axes,
        "fraction_msgs_active_heuristic": round(active_frac, 3),
        "intervals_active_s": intervals,
    }


def _bag_start_time_ns(mcaps: list[Path]) -> int:
    """Earliest message time in recording (rosbags metadata), stable time origin."""
    with AnyReader(mcaps) as reader:
        return int(reader.start_time)


def analyze_bag_dir(bag_dir: Path) -> dict[str, Any]:
    mcaps = sorted(bag_dir.glob("*.mcap"))
    if not mcaps:
        return {"bag_dir": str(bag_dir), "error": "no_mcap"}

    t0_ns = _bag_start_time_ns(mcaps)

    with AnyReader(mcaps) as reader:
        mc = analyze_topic_manual_control(reader, t0_ns)
    with AnyReader(mcaps) as reader:
        joy = analyze_topic_joy(reader, "/joy", t0_ns)
    with AnyReader(mcaps) as reader:
        jc = analyze_topic_joy(reader, "/joy_controller", t0_ns)

    return {
        "bag_dir": str(bag_dir),
        "bag_name": bag_dir.name,
        "time_origin": "reader.start_time (earliest message in bag metadata)",
        "topics": {
            "/pixhawk/manual_control": mc,
            "/joy": joy,
            "/joy_controller": jc,
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "root",
        type=Path,
        nargs="?",
        default=Path("recordings/rosbags"),
        help="e.g. recordings/rosbags (default: recordings/rosbags)",
    )
    ap.add_argument("--json", action="store_true", help="Print one JSON array")
    args = ap.parse_args()
    root = args.root.resolve()
    if not root.is_dir():
        print(f"Not a directory: {root}", file=sys.stderr)
        return 1

    bags = find_raw_bag_dirs(root)
    results = [analyze_bag_dir(b) for b in bags]

    if args.json:
        print(json.dumps(results, indent=2))
        return 0

    for r in results:
        print(f"\n=== {r.get('bag_name', '?')} ===")
        for tname, info in r.get("topics", {}).items():
            if not info.get("present"):
                print(f"  {tname}: absent")
                continue
            print(
                f"  {tname}: n={info.get('messages')} ~{info.get('approx_mean_hz')} Hz "
                f"active_frac={info.get('fraction_msgs_any_axis_active', info.get('fraction_msgs_active_heuristic'))}"
            )
            if "max_abs_per_channel" in info:
                print(f"    max_abs: {info['max_abs_per_channel']}")
            if "intervals_any_deviation_s" in info and info["intervals_any_deviation_s"]:
                iv = info["intervals_any_deviation_s"][:8]
                more = len(info["intervals_any_deviation_s"]) - len(iv)
                print(f"    deviation intervals (s, first {len(iv)}): {iv}" + (f" ... +{more} more" if more > 0 else ""))
            if "intervals_active_s" in info and info["intervals_active_s"]:
                iv = info["intervals_active_s"][:6]
                more = len(info["intervals_active_s"]) - len(iv)
                print(f"    active intervals (s): {iv}" + (f" ... +{more} more" if more > 0 else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
