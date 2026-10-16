"""
Audit two things that determine whether GPS actually pulls the global EKF:

1. R_gps  — the position covariance carried by /odometry/gps (= the R the EKF
   sees on the GPS update). Sourced from the replayed bag.
2. K[x via GPS-x], K[y via GPS-y] — the Kalman gain components the EKF
   actually computes on each /odometry/gps update. Sourced from the
   robot_localization debug log written by `ekf_global.yaml`'s
   `debug_out_file: /tmp/ekf_global_debug.log`.

If R_gps is ~0.1 m² and K is ~1.0, GPS is being trusted and applied. State
divergence under those conditions implies the bug is elsewhere.
If R_gps is huge (e.g. 1e6) or K is tiny (e.g. 1e-3), GPS isn't being
weighted in — the EKF treats GPS as noise and integrates velocity off into
nowhere.

Usage:
    python3 scripts/inspect_global_ekf_kgain.py <bag_dir> [--debug-log /tmp/ekf_global_debug.log]
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


# ---------------------------------------------------------------------------
# Bag side: R_gps over time
# ---------------------------------------------------------------------------

def read_gps_covariance(bag_dir: Path) -> list[tuple[int, float, float]]:
    """Return [(t_ns, sigma_x, sigma_y), ...] for /odometry/gps.

    sigma_* = sqrt(pose.covariance[0]) and sqrt(pose.covariance[7]) — the EKF
    treats these (squared) as R_gps.
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    samples: list[tuple[int, float, float]] = []
    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == "/odometry/gps"]
        if not conns:
            print("WARNING: no /odometry/gps in bag", file=sys.stderr)
            return samples
        for conn, _t, raw in reader.messages(connections=conns):
            ros = reader.deserialize(raw, conn.msgtype)
            t_ns = int(ros.header.stamp.sec) * 10**9 + int(ros.header.stamp.nanosec)
            cov = list(ros.pose.covariance)
            sx = cov[0] ** 0.5 if cov[0] >= 0 else float("nan")
            sy = cov[7] ** 0.5 if cov[7] >= 0 else float("nan")
            samples.append((t_ns, float(cov[0]), float(cov[7])))
    return samples


# ---------------------------------------------------------------------------
# Debug log side: Kalman gain on every odom1_pose (GPS) update
# ---------------------------------------------------------------------------

# Regex anchors. The robot_localization debug log lays out each correction as:
#
#   ---------------------- Ekf::correct ----------------------
#   State is:
#   [...]
#   Topic is:
#   odom1_pose
#   ...
#   Kalman gain subset is:
#   [<15 rows of 2 floats each>]
#   Innovation is:
#   [<two floats>]
#   ...
#
# We want, per odom1_pose correct: K[x via GPS x] = row 0 col 0
#                                  K[y via GPS y] = row 1 col 1

_TOPIC_PATTERN = re.compile(r"^Topic is:\s*\n\s*(\S+)\s*$", re.MULTILINE)


def extract_kgains(debug_log: Path) -> list[tuple[float, float]]:
    """Return [(K_xx, K_yy), ...] for every odom1_pose Kalman correct."""
    text = debug_log.read_text(errors="replace")

    # Split the log on "---------------------- Ekf::correct ----------------------"
    # — each segment that follows is one correction call.
    segs = text.split("---------------------- Ekf::correct ----------------------")
    out: list[tuple[float, float]] = []

    for seg in segs[1:]:  # first segment is preamble
        # Stop at the closing /Ekf::correct so we don't bleed into the next correct.
        end = seg.find("---------------------- /Ekf::correct ----------------------")
        body = seg if end < 0 else seg[:end]

        # Topic
        m = _TOPIC_PATTERN.search(body)
        if not m or m.group(1).strip() != "odom1_pose":
            continue

        # Kalman gain subset: a 15x2 matrix on consecutive lines after the marker.
        kg_idx = body.find("Kalman gain subset is:")
        if kg_idx < 0:
            continue
        # Find the first '[' after that marker — that's the start of the matrix.
        bracket_open = body.find("[", kg_idx)
        bracket_close = body.find("]", bracket_open)
        if bracket_open < 0 or bracket_close < 0:
            continue
        matrix_text = body[bracket_open + 1 : bracket_close]
        # Tokenise into floats; should yield exactly 15 * 2 = 30 numbers.
        numbers = re.findall(r"[-+0-9.eE]+", matrix_text)
        if len(numbers) < 4:
            continue
        try:
            # Row 0 of the 15x2: K[x via GPS x], K[x via GPS y]
            k_xx = float(numbers[0])
            # Row 1 of the 15x2: K[y via GPS x], K[y via GPS y]
            k_yy = float(numbers[3])
        except ValueError:
            continue
        out.append((k_xx, k_yy))

    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def fmt_stats(label: str, vals: list[float]) -> str:
    if not vals:
        return f"  {label:<30} (no samples)"
    n = len(vals)
    vals_sorted = sorted(vals)
    median = vals_sorted[n // 2]
    return (
        f"  {label:<30} n={n:>5}  "
        f"min={min(vals):.4g}  median={median:.4g}  max={max(vals):.4g}"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument(
        "--debug-log",
        type=Path,
        default=Path("/tmp/ekf_global_debug.log"),
        help="Path to robot_localization debug_out_file (default /tmp/ekf_global_debug.log).",
    )
    args = ap.parse_args()

    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)

    print("=" * 70)
    print("R_gps — what the EKF sees as GPS measurement covariance")
    print("(from /odometry/gps in the recorded bag)")
    print("=" * 70)
    gps_cov_samples = read_gps_covariance(bag_dir)
    if gps_cov_samples:
        cov_xx_vals = [s[1] for s in gps_cov_samples]
        cov_yy_vals = [s[2] for s in gps_cov_samples]
        sigma_xx_vals = [v ** 0.5 for v in cov_xx_vals if v >= 0]
        sigma_yy_vals = [v ** 0.5 for v in cov_yy_vals if v >= 0]
        print(fmt_stats("pose.covariance[0]  (m²)", cov_xx_vals))
        print(fmt_stats("pose.covariance[7]  (m²)", cov_yy_vals))
        print(fmt_stats("σ_x = √cov[0]       (m)", sigma_xx_vals))
        print(fmt_stats("σ_y = √cov[7]       (m)", sigma_yy_vals))
        print(
            f"\n  → If σ is much larger than ~0.1–1 m here, R_gps is huge and "
            f"K = P/(P+R) is tiny\n"
            f"     even when P[x,x] is healthy. GPS arrives but is ignored."
        )
    else:
        print("  (no /odometry/gps messages in bag)")

    print()
    print("=" * 70)
    print("K[x] and K[y] — what the EKF actually applied on each GPS update")
    print(f"(from {args.debug_log})")
    print("=" * 70)
    if not args.debug_log.exists():
        print(f"  ERROR: debug log not found at {args.debug_log}", file=sys.stderr)
        sys.exit(1)
    kgains = extract_kgains(args.debug_log)
    if kgains:
        kxx_vals = [k[0] for k in kgains]
        kyy_vals = [k[1] for k in kgains]
        print(fmt_stats("K[x via GPS x]", kxx_vals))
        print(fmt_stats("K[y via GPS y]", kyy_vals))
        print(
            f"\n  → K close to 1.0  → GPS pulls the state every update.\n"
            f"  → K close to 0.0  → GPS is ignored. State drifts on velocity integration alone.\n"
        )
        # Show first/middle/last so we can see if K collapses over time.
        n = len(kgains)
        for label, idx in [("first", 0), ("middle", n // 2), ("last", n - 1)]:
            kxx, kyy = kgains[idx]
            print(f"  {label:<6} odom1_pose update: K[x]={kxx:.6g}  K[y]={kyy:.6g}")
    else:
        print("  (no odom1_pose corrections found in debug log)")


if __name__ == "__main__":
    main()
