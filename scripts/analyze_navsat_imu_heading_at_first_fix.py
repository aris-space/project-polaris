#!/usr/bin/env python3
"""
Check IMU heading stability at the time navsat_transform latches its datum.

navsat_transform (use_odometry_yaw=false, wait_for_datum=false) captures the
IMU orientation from the first /fix that has status >= STATUS_FIX (0). This
script replicates that logic and reports:

  - When (relative to bag start) the first qualified fix arrives
  - Whether the IMU is still in its early-convergence window (< 5 min)
  - IMU yaw at that moment (degrees, ENU frame after yaw_offset + declination)
  - Yaw std-dev over the 30 s window centred on the first fix (stability gauge)

navsat_transform.yaml relevant settings:
  use_odometry_yaw: false
  yaw_offset: 1.57079632679   (pi/2 — ENU convention offset)
  magnetic_declination_radians: 0.0623
  wait_for_datum: false

Usage:
  python scripts/analyze_navsat_imu_heading_at_first_fix.py [root_dir]

Dependencies: pip install rosbags numpy
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    print("pip install rosbags numpy", file=sys.stderr)
    raise

TOPIC_FIX = "/fix"
TOPIC_IMU = "/imu/data"

# Match navsat_transform: fix is valid when status.status >= 0
STATUS_FIX = 0

# navsat_transform.yaml
YAW_OFFSET = math.pi / 2.0          # radians
MAGNETIC_DECL = 0.0623              # radians

# Warn if first fix arrives within this many seconds of bag start
CONVERGENCE_WINDOW_S = 300.0        # 5 minutes

# Max gap between first fix stamp and nearest IMU stamp
MAX_IMU_GAP_S = 0.5

# Half-width of the yaw-stability window (centred on first fix)
STABILITY_HALF_WIN_S = 30.0


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw (ENU, rad) from unit quaternion — same formula as tf2::getYaw."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_enu_deg(raw_yaw_rad: float) -> float:
    """Apply navsat_transform's yaw_offset + magnetic declination and convert to degrees."""
    # navsat_transform: yaw_enu = raw_imu_yaw + yaw_offset + magnetic_decl
    return math.degrees(raw_yaw_rad + YAW_OFFSET + MAGNETIC_DECL)


def analyze_bag(bag_dir: Path) -> dict | None:
    fix_msgs: list[tuple[float, int]] = []   # (stamp_s, status)
    imu_stamps: list[float] = []
    imu_yaws: list[float] = []              # raw yaw from quaternion (rad)

    with AnyReader([bag_dir]) as reader:
        conns_fix = [c for c in reader.connections if c.topic == TOPIC_FIX]
        conns_imu = [c for c in reader.connections if c.topic == TOPIC_IMU]
        if not conns_fix or not conns_imu:
            return None

        for conn, _log_ts, raw in reader.messages(connections=conns_fix + conns_imu):
            try:
                msg = reader.deserialize(raw, conn.msgtype)
            except Exception:
                continue
            if conn.topic == TOPIC_FIX:
                t = _stamp_s(msg.header.stamp)
                status = int(msg.status.status)
                fix_msgs.append((t, status))
            elif conn.topic == TOPIC_IMU:
                t = _stamp_s(msg.header.stamp)
                q = msg.orientation
                yaw = _quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
                imu_stamps.append(t)
                imu_yaws.append(yaw)

    if not fix_msgs or not imu_stamps:
        return None

    # Sort by stamp
    fix_msgs.sort(key=lambda x: x[0])
    order = np.argsort(imu_stamps)
    imu_stamps_arr = np.array(imu_stamps, dtype=np.float64)[order]
    imu_yaws_arr = np.array(imu_yaws, dtype=np.float64)[order]

    bag_start = min(fix_msgs[0][0], float(imu_stamps_arr[0]))

    # Find first qualified fix
    first_fix_t: float | None = None
    for t, status in fix_msgs:
        if status >= STATUS_FIX:
            first_fix_t = t
            break

    if first_fix_t is None:
        return {"bag": bag_dir.name, "no_valid_fix": True}

    time_since_start = first_fix_t - bag_start

    # Nearest IMU message within MAX_IMU_GAP_S
    idx = int(np.argmin(np.abs(imu_stamps_arr - first_fix_t)))
    imu_gap = abs(float(imu_stamps_arr[idx]) - first_fix_t)
    if imu_gap > MAX_IMU_GAP_S:
        return {
            "bag": bag_dir.name,
            "first_fix_t_rel": time_since_start,
            "imu_gap_s": imu_gap,
            "no_imu_nearby": True,
        }

    raw_yaw_rad = float(imu_yaws_arr[idx])
    enu_yaw_deg = _yaw_to_enu_deg(raw_yaw_rad)

    # Yaw stability: std over ±STABILITY_HALF_WIN_S window
    lo = first_fix_t - STABILITY_HALF_WIN_S
    hi = first_fix_t + STABILITY_HALF_WIN_S
    window_mask = (imu_stamps_arr >= lo) & (imu_stamps_arr <= hi)
    window_yaws = imu_yaws_arr[window_mask]

    yaw_std_deg: float | None = None
    if len(window_yaws) >= 5:
        # Unwrap to handle wrap-around at ±π
        unwrapped = np.unwrap(window_yaws)
        yaw_std_deg = float(np.std(unwrapped, ddof=1)) * 180.0 / math.pi

    n_fixes_total = len(fix_msgs)
    n_qualified = sum(1 for _, s in fix_msgs if s >= STATUS_FIX)

    return {
        "bag": bag_dir.name,
        "bag_start_s": bag_start,
        "first_fix_t_rel": time_since_start,
        "imu_gap_s": imu_gap,
        "raw_yaw_rad": raw_yaw_rad,
        "enu_yaw_deg": enu_yaw_deg,
        "yaw_std_deg": yaw_std_deg,
        "n_imu_in_window": int(window_mask.sum()),
        "n_fixes_total": n_fixes_total,
        "n_qualified_fixes": n_qualified,
        "still_converging": time_since_start < CONVERGENCE_WINDOW_S,
    }


def find_bag_dirs(root: Path) -> list[Path]:
    dirs: list[Path] = []
    for meta in root.rglob("metadata.yaml"):
        d = meta.parent.resolve()
        if any(d.glob("*.mcap")):
            dirs.append(d)
    return sorted(dirs)


def _fmt(val: float | None, fmt: str, unit: str = "") -> str:
    if val is None:
        return "n/a"
    return f"{val:{fmt}}{unit}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "root",
        type=Path,
        nargs="?",
        default=Path("recordings/rosbags"),
        help="Root directory to scan for bags (default: recordings/rosbags)",
    )
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    root = args.root if args.root.is_absolute() else repo_root / args.root
    if not root.is_dir():
        print(f"Not a directory: {root}", file=sys.stderr)
        return 1

    bag_dirs = find_bag_dirs(root)
    if not bag_dirs:
        print(f"No bags found under {root}", file=sys.stderr)
        return 1

    results = []
    for d in bag_dirs:
        r = analyze_bag(d)
        if r is not None:
            results.append(r)

    if not results:
        print("No bags contained both /fix and /imu/data.", file=sys.stderr)
        return 1

    # ── print table ──────────────────────────────────────────────────────────
    COL = 50
    print()
    print("=" * 110)
    print("navsat_transform datum heading check — IMU yaw at first qualified /fix")
    print(f"  Quality gate: status >= {STATUS_FIX}  |  IMU match window: ±{MAX_IMU_GAP_S} s")
    print(f"  Convergence warning: first fix within {CONVERGENCE_WINDOW_S:.0f} s of bag start")
    print(f"  Stability window: ±{STABILITY_HALF_WIN_S:.0f} s around first fix")
    print("=" * 110)

    hdr = (
        f"{'Bag':<{COL}}  {'fix@t':>7}  {'IMU gap':>7}  "
        f"{'raw_yaw':>8}  {'ENU yaw':>9}  {'yaw_std':>8}  {'conv?':>6}  {'note'}"
    )
    print(hdr)
    print("-" * 110)

    for r in results:
        name = r["bag"][:COL - 1]
        if r.get("no_valid_fix"):
            print(f"{name:<{COL}}  {'—':>7}  {'—':>7}  {'no valid /fix in bag':}")
            continue
        if r.get("no_imu_nearby"):
            gap = f"{r['imu_gap_s']:.2f} s"
            t_rel = _fmt(r.get("first_fix_t_rel"), ".1f", " s")
            print(f"{name:<{COL}}  {t_rel:>7}  {gap:>7}  {'NO IMU within ±0.5 s':}")
            continue

        t_rel = f"{r['first_fix_t_rel']:.1f} s"
        gap = f"{r['imu_gap_s']*1000:.0f} ms"
        raw_yaw = f"{math.degrees(r['raw_yaw_rad']):.1f}°"
        enu_yaw = f"{r['enu_yaw_deg']:.1f}°"
        std_str = _fmt(r.get("yaw_std_deg"), ".2f", "°")
        conv = "WARN" if r["still_converging"] else "ok"

        notes = []
        if r["still_converging"]:
            notes.append(f"only {r['first_fix_t_rel']:.0f}s since bag start (<5 min)")
        if r.get("yaw_std_deg") is not None and r["yaw_std_deg"] > 2.0:
            notes.append(f"yaw moving (std={r['yaw_std_deg']:.2f}°)")
        if r.get("yaw_std_deg") is not None and r["yaw_std_deg"] < 0.5:
            notes.append("yaw stable")

        print(
            f"{name:<{COL}}  {t_rel:>7}  {gap:>7}  "
            f"{raw_yaw:>8}  {enu_yaw:>9}  {std_str:>8}  {conv:>6}  "
            + ("  ".join(notes) if notes else "")
        )

    print("=" * 110)
    print()
    print("Columns:")
    print("  fix@t    — time of first qualified /fix relative to bag start")
    print("  IMU gap  — |IMU stamp - fix stamp| (should be < 10 ms at 100 Hz)")
    print("  raw_yaw  — quaternion yaw from /imu/data at that moment")
    print(f"  ENU yaw  — raw_yaw + yaw_offset({math.degrees(YAW_OFFSET):.0f}°) + decl({math.degrees(MAGNETIC_DECL):.2f}°)")
    print("             this is what navsat_transform uses to rotate GPS into ENU")
    print(f"  yaw_std  — std of unwrapped yaw over ±{STABILITY_HALF_WIN_S:.0f} s window")
    print("  conv?    — WARN if first fix arrived within 5 min of bag start")
    print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
