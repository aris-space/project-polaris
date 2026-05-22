"""
Two-NavSatFix overlay: /gps/filtered/global  vs  /waterlinked_ugps/navsatfix.

Both topics are NavSatFix (lat/lon). The script projects each independently
to a common UTM frame using the first valid /gps/filtered/global message
as the datum, pairs samples by timestamp, and reports raw Euclidean distance
error per pair. There is NO first-point shift correction — the whole point
of this comparison is to measure the natural agreement between the EKF's
back-projected position and the SBL fix without hiding any constant offset
behind alignment.

`/waterlinked_ugps/navsatfix` is the Waterlinked UGPS G2 acoustic positioning
NavSatFix and serves as ground truth here when the AUV is under-ice or
otherwise without GNSS lock. `/gps/filtered/global` is the EKF's map-frame
state inverse-projected back to lat/lon via the datum (by
global_ekf_to_navsatfix or gnss_anchored_pose).

For surface bags with GNSS available, prefer `gps_filtered_vs_fix_overlay.py`
(higher-rate, lower-noise reference).

Usage
-----
    python scripts/gps_filtered_vs_sbl_overlay.py /path/to/bag_dir [...]

Multiple bag dirs may be passed; each gets its own report block.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import utm
from mcap_ros2.reader import read_ros2_messages


_TOPIC_GLOBAL = "/gps/filtered/global"
_TOPIC_REF = "/waterlinked_ugps/navsatfix"
_PAIR_MAX_GAP_NS = 200_000_000  # 200 ms — be generous, SBL is ~2 Hz


@dataclass
class FixSample:
    t_ns: int
    lat: float
    lon: float
    status: int


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def read_bag(bag_dir: Path) -> tuple[list[FixSample], list[FixSample]]:
    mcap_path = bag_dir / f"{bag_dir.name}_0.mcap"
    if not mcap_path.exists():
        candidates = list(bag_dir.glob("*_0.mcap"))
        if not candidates:
            print(f"ERROR: no *_0.mcap found in {bag_dir}", file=sys.stderr)
            sys.exit(1)
        mcap_path = candidates[0]

    wanted = {_TOPIC_GLOBAL, _TOPIC_REF}
    global_track: list[FixSample] = []
    ref_track: list[FixSample] = []

    for msg in read_ros2_messages(str(mcap_path)):
        topic = msg.channel.topic
        if topic not in wanted:
            continue
        ros = msg.ros_msg
        try:
            t = _stamp_ns(ros.header.stamp)
            if t == 0:
                continue
            sample = FixSample(
                t_ns=t,
                lat=float(ros.latitude),
                lon=float(ros.longitude),
                status=int(ros.status.status),
            )
        except AttributeError:
            continue
        if topic == _TOPIC_GLOBAL:
            global_track.append(sample)
        else:
            ref_track.append(sample)

    global_track.sort(key=lambda s: s.t_ns)
    ref_track.sort(key=lambda s: s.t_ns)
    return global_track, ref_track


def project_track(samples: list[FixSample],
                  zone_number: int, zone_letter: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project each sample to UTM in the *given* zone, return E, N, t_ns arrays.
    Forcing the zone keeps both tracks in the same projected frame even at
    zone-boundary edge cases."""
    es = np.empty(len(samples), dtype=float)
    ns = np.empty(len(samples), dtype=float)
    ts = np.empty(len(samples), dtype=np.int64)
    for i, s in enumerate(samples):
        e, n, _, _ = utm.from_latlon(s.lat, s.lon,
                                     force_zone_number=zone_number,
                                     force_zone_letter=zone_letter)
        es[i] = e
        ns[i] = n
        ts[i] = s.t_ns
    return es, ns, ts


def pair_errors(global_e: np.ndarray, global_n: np.ndarray, global_t: np.ndarray,
                ref_e: np.ndarray, ref_n: np.ndarray, ref_t: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray]:
    """For each SBL sample, find the nearest /gps/filtered/global sample
    within PAIR_MAX_GAP_NS. Return (t_rel_s, err_m) arrays."""
    if global_t.size == 0 or ref_t.size == 0:
        return np.array([]), np.array([])
    t_rel = []
    err = []
    t0 = int(min(int(global_t[0]), int(ref_t[0])))
    for i in range(ref_t.size):
        ts = int(ref_t[i])
        j = int(np.argmin(np.abs(global_t - ts)))
        if abs(int(global_t[j]) - ts) > _PAIR_MAX_GAP_NS:
            continue
        e = math.hypot(global_e[j] - ref_e[i], global_n[j] - ref_n[i])
        err.append(e)
        t_rel.append((ts - t0) * 1e-9)
    return np.asarray(t_rel), np.asarray(err)


def report(label: str, global_track: list[FixSample], ref_track: list[FixSample]) -> None:
    print()
    print("=" * 92)
    print(f"file: {label}")
    print("=" * 92)
    print(f"  /gps/filtered/global samples: {len(global_track)}")
    print(f"  /waterlinked_ugps/navsatfix samples: {len(ref_track)}")

    if not global_track or not ref_track:
        print("  (one of the tracks is empty — cannot compare)")
        return

    # Datum: first valid /gps/filtered/global sample (status >= 0, not null-island).
    datum = next(
        (s for s in global_track
         if s.status >= 0 and not (abs(s.lat) < 0.1 and abs(s.lon) < 0.1)),
        None,
    )
    if datum is None:
        datum = global_track[0]
    _, _, zone_number, zone_letter = utm.from_latlon(datum.lat, datum.lon)
    print(f"  datum: lat={datum.lat:.7f} lon={datum.lon:.7f}  UTM zone {zone_number}{zone_letter}")

    # Project both tracks to the SAME UTM zone, in *absolute* UTM metres.
    g_e, g_n, g_t = project_track(global_track, zone_number, zone_letter)
    r_e, r_n, r_t = project_track(ref_track, zone_number, zone_letter)

    # Subtract datum so the numbers are small (and so a quick plot doesn't have
    # 6-digit axes). NB: this is not an alignment — the SAME constant is
    # subtracted from both tracks, which preserves any offset between them.
    datum_e, datum_n, _, _ = utm.from_latlon(datum.lat, datum.lon,
                                             force_zone_number=zone_number,
                                             force_zone_letter=zone_letter)
    gx = g_e - datum_e
    gy = g_n - datum_n
    rx = r_e - datum_e
    ry = r_n - datum_n

    # Pair errors (no alignment).
    t_rel, err = pair_errors(g_e, g_n, g_t, r_e, r_n, r_t)
    if err.size == 0:
        print("  (no paired samples within the 200 ms gap)")
        return

    err_sorted = np.sort(err)
    n = err_sorted.size
    mean = float(np.mean(err))
    med = float(np.median(err))
    p95 = float(err_sorted[int(0.95 * (n - 1))])
    mx = float(err_sorted[-1])

    print()
    print("  Distance error  /gps/filtered/global  vs  /waterlinked_ugps/navsatfix")
    print("  (NO first-point alignment — raw natural-anchor lat/lon distance):")
    print(f"    n_pairs = {n}")
    print(f"    mean    = {mean:.3f} m")
    print(f"    median  = {med:.3f} m")
    print(f"    p95     = {p95:.3f} m")
    print(f"    max     = {mx:.3f} m")

    # Track extents to make any "the AUV moved X m" sanity check easy.
    print()
    print("  GLOBAL track (whole run, datum-relative):")
    print(f"    x range  = [{gx.min():+8.2f}, {gx.max():+8.2f}]  width  = {gx.max()-gx.min():.2f} m")
    print(f"    y range  = [{gy.min():+8.2f}, {gy.max():+8.2f}]  height = {gy.max()-gy.min():.2f} m")
    print(f"    max distance from datum = {float(np.max(np.hypot(gx, gy))):.2f} m")
    print("  SBL track (whole run, datum-relative):")
    print(f"    x range  = [{rx.min():+8.2f}, {rx.max():+8.2f}]  width  = {rx.max()-rx.min():.2f} m")
    print(f"    y range  = [{ry.min():+8.2f}, {ry.max():+8.2f}]  height = {ry.max()-ry.min():.2f} m")
    print(f"    max distance from datum = {float(np.max(np.hypot(rx, ry))):.2f} m")


def plot(label: str, global_track: list[FixSample], ref_track: list[FixSample],
         output_path: Path) -> None:
    if not global_track or not ref_track:
        return
    datum = next(
        (s for s in global_track
         if s.status >= 0 and not (abs(s.lat) < 0.1 and abs(s.lon) < 0.1)),
        global_track[0],
    )
    _, _, zone_number, zone_letter = utm.from_latlon(datum.lat, datum.lon)
    datum_e, datum_n, _, _ = utm.from_latlon(datum.lat, datum.lon,
                                             force_zone_number=zone_number,
                                             force_zone_letter=zone_letter)
    g_e, g_n, g_t = project_track(global_track, zone_number, zone_letter)
    r_e, r_n, r_t = project_track(ref_track, zone_number, zone_letter)
    gx = g_e - datum_e
    gy = g_n - datum_n
    rx = r_e - datum_e
    ry = r_n - datum_n
    t_rel, err = pair_errors(g_e, g_n, g_t, r_e, r_n, r_t)

    fig, (ax_traj, ax_err) = plt.subplots(1, 2, figsize=(13, 6))
    ax_traj.plot(rx, ry, color="black", linewidth=1.0, alpha=0.6,
                 label="/waterlinked_ugps/navsatfix (SBL)")
    ax_traj.plot(gx, gy, color="tab:red", linewidth=1.0, alpha=0.85,
                 label="/gps/filtered/global")
    ax_traj.scatter([0], [0], color="tab:blue", s=30, zorder=5, label="datum")
    ax_traj.set_xlabel("E − datum [m]")
    ax_traj.set_ylabel("N − datum [m]")
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.set_title("Trajectories (UTM, datum-relative, no alignment)")
    ax_traj.grid(alpha=0.3)
    ax_traj.legend(fontsize=9, loc="best")

    if err.size:
        ax_err.plot(t_rel, err, color="tab:red", linewidth=1.0, alpha=0.85)
        ax_err.set_xlabel("t [s] (since first paired sample)")
        ax_err.set_ylabel("|global − SBL| [m]")
        ax_err.set_title(f"Pairwise distance error  (n={err.size})")
        ax_err.grid(alpha=0.3)
        med = float(np.median(err))
        mean = float(np.mean(err))
        ax_err.axhline(med, color="tab:blue", linestyle="--", linewidth=0.8,
                       label=f"median = {med:.2f} m")
        ax_err.axhline(mean, color="tab:green", linestyle=":", linewidth=0.8,
                       label=f"mean   = {mean:.2f} m")
        ax_err.legend(fontsize=9, loc="best")
    else:
        ax_err.text(0.5, 0.5, "no paired samples", ha="center", va="center",
                    transform=ax_err.transAxes)

    fig.suptitle(label, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=110)
    plt.close(fig)
    print(f"  wrote plot: {output_path}")


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag_dirs", nargs="+", type=Path,
                   help="One or more rosbag2 directories (each contains a *_0.mcap).")
    p.add_argument("--output-dir", type=Path, default=None,
                   help="Where to write per-bag PNGs. Default: <bag_dir>/gps_vs_sbl_overlay.png")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip plotting; only print the report.")
    args = p.parse_args(argv)

    rc = 0
    for bag_dir in args.bag_dirs:
        if not bag_dir.is_dir():
            print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
            rc = 1
            continue
        global_track, ref_track = read_bag(bag_dir)
        report(str(bag_dir), global_track, ref_track)
        if not args.no_plot:
            out_path = (args.output_dir / f"{bag_dir.name}_gps_vs_sbl.png"
                        if args.output_dir
                        else bag_dir / "gps_vs_sbl_overlay.png")
            plot(str(bag_dir), global_track, ref_track, out_path)
    return rc


if __name__ == "__main__":
    sys.exit(main())
