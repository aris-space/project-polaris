"""
Plot the global EKF state evolution against its inputs to find where divergence starts.

Reads /odometry/filtered/global, /odometry/filtered/local, /odometry/gps, and /fix
from a replayed bag and produces a five-panel figure on a shared time axis:

  1. Position magnitude (log-y): global vs local vs /odometry/gps
  2. Per-axis position: x, y for global / local / /odometry/gps overlaid
  3. Velocity from /odometry/filtered/global twist (vx, vy, vz)
  4. Position covariance diagonals from /odometry/filtered/global pose.covariance
     (sqrt → m std)
  5. GPS innovation magnitude (|/odometry/gps − /odometry/filtered/global| at
     each /odometry/gps timestamp)

Use this on any bag produced by replay_ekf_bag.sh, including the diverged ones,
to see exactly *when* the filter deviates from its inputs and *which state* is
the leading indicator.

Usage:
    python3 scripts/plot_global_ekf_divergence.py <bag_dir> [--output-dir <dir>]
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


_TOPIC_GLOBAL = "/odometry/filtered/global"
_TOPIC_LOCAL = "/odometry/filtered/local"
_TOPIC_GPS_ODOM = "/odometry/gps"
_TOPIC_FIX = "/fix"


@dataclass
class OdomSample:
    t_ns: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    px_var: float  # pose covariance diagonal: x
    py_var: float
    pz_var: float


@dataclass
class FixSample:
    t_ns: int
    lat: float
    lon: float


@dataclass
class BagData:
    global_odom: list = field(default_factory=list)
    local_odom: list = field(default_factory=list)
    gps_odom: list = field(default_factory=list)
    fix: list = field(default_factory=list)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _odom_to_sample(ros_msg) -> OdomSample:
    p = ros_msg.pose.pose.position
    v = ros_msg.twist.twist.linear
    cov = list(ros_msg.pose.covariance)  # 6×6 row-major: x,y,z,roll,pitch,yaw
    return OdomSample(
        t_ns=_stamp_ns(ros_msg.header.stamp),
        x=float(p.x), y=float(p.y), z=float(p.z),
        vx=float(v.x), vy=float(v.y), vz=float(v.z),
        px_var=float(cov[0]), py_var=float(cov[7]), pz_var=float(cov[14]),
    )


def read_bag(bag_dir: Path) -> BagData:
    """Read the four topics of interest from any rosbag2 backend (mcap or sqlite3)."""
    wanted = {_TOPIC_GLOBAL, _TOPIC_LOCAL, _TOPIC_GPS_ODOM, _TOPIC_FIX}
    data = BagData()

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic in wanted]
        for connection, _timestamp, rawdata in reader.messages(connections=connections):
            ros = reader.deserialize(rawdata, connection.msgtype)
            try:
                t = _stamp_ns(ros.header.stamp)
                if t == 0:
                    continue
                if connection.topic == _TOPIC_GLOBAL:
                    data.global_odom.append(_odom_to_sample(ros))
                elif connection.topic == _TOPIC_LOCAL:
                    data.local_odom.append(_odom_to_sample(ros))
                elif connection.topic == _TOPIC_GPS_ODOM:
                    data.gps_odom.append(_odom_to_sample(ros))
                elif connection.topic == _TOPIC_FIX:
                    data.fix.append(FixSample(
                        t_ns=t, lat=float(ros.latitude), lon=float(ros.longitude)
                    ))
            except AttributeError:
                continue

    return data


def _t_seconds(samples, t0_ns: int) -> np.ndarray:
    return np.array([(s.t_ns - t0_ns) * 1e-9 for s in samples])


def plot(bag_data: BagData, bag_name: str, out_dir: Path) -> Path:
    if not bag_data.global_odom:
        print("ERROR: no /odometry/filtered/global in bag", file=sys.stderr)
        sys.exit(1)

    t0_ns = bag_data.global_odom[0].t_ns

    g_t = _t_seconds(bag_data.global_odom, t0_ns)
    g_x = np.array([s.x for s in bag_data.global_odom])
    g_y = np.array([s.y for s in bag_data.global_odom])
    g_vx = np.array([s.vx for s in bag_data.global_odom])
    g_vy = np.array([s.vy for s in bag_data.global_odom])
    g_vz = np.array([s.vz for s in bag_data.global_odom])
    g_px_std = np.sqrt(np.array([s.px_var for s in bag_data.global_odom]))
    g_py_std = np.sqrt(np.array([s.py_var for s in bag_data.global_odom]))
    g_mag = np.sqrt(g_x ** 2 + g_y ** 2)

    l_t = _t_seconds(bag_data.local_odom, t0_ns)
    l_x = np.array([s.x for s in bag_data.local_odom])
    l_y = np.array([s.y for s in bag_data.local_odom])
    l_mag = np.sqrt(l_x ** 2 + l_y ** 2)

    gps_t = _t_seconds(bag_data.gps_odom, t0_ns)
    gps_x = np.array([s.x for s in bag_data.gps_odom])
    gps_y = np.array([s.y for s in bag_data.gps_odom])
    gps_mag = np.sqrt(gps_x ** 2 + gps_y ** 2)

    # GPS innovation: at each /odometry/gps timestamp, find nearest global EKF
    # sample and compute |gps − global|.
    innovations = np.zeros_like(gps_t)
    if len(g_t):
        for i, t in enumerate(gps_t):
            j = int(np.argmin(np.abs(g_t - t)))
            dx = gps_x[i] - g_x[j]
            dy = gps_y[i] - g_y[j]
            innovations[i] = np.sqrt(dx * dx + dy * dy)

    fig, axes = plt.subplots(5, 1, figsize=(14, 18), sharex=True)

    ax = axes[0]
    ax.plot(g_t, g_mag, label="|/odometry/filtered/global|", lw=1.0)
    ax.plot(l_t, l_mag, label="|/odometry/filtered/local|", lw=1.0)
    ax.plot(gps_t, gps_mag, ".", ms=3, label="|/odometry/gps|", alpha=0.6)
    ax.set_ylabel("|position| (m)")
    ax.set_yscale("symlog", linthresh=10)
    ax.axhline(100, color="orange", lw=0.5, ls="--", label="100 m onset")
    ax.axhline(1000, color="red", lw=0.5, ls="--", label="1 km onset")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title(f"Global EKF divergence diagnostic — {bag_name}")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(g_t, g_x, label="global x", color="C0", lw=1.0)
    ax.plot(g_t, g_y, label="global y", color="C0", ls="--", lw=1.0)
    ax.plot(l_t, l_x, label="local x", color="C1", lw=1.0)
    ax.plot(l_t, l_y, label="local y", color="C1", ls="--", lw=1.0)
    ax.plot(gps_t, gps_x, ".", color="C2", ms=2, label="gps x", alpha=0.6)
    ax.plot(gps_t, gps_y, "x", color="C2", ms=3, label="gps y", alpha=0.6)
    ax.set_ylabel("position (m)")
    ax.set_yscale("symlog", linthresh=10)
    ax.legend(loc="upper left", fontsize=8, ncol=3)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(g_t, g_vx, label="global vx", lw=1.0)
    ax.plot(g_t, g_vy, label="global vy", lw=1.0)
    ax.plot(g_t, g_vz, label="global vz", lw=1.0, alpha=0.6)
    ax.set_ylabel("velocity (m/s, body frame)")
    ax.set_yscale("symlog", linthresh=0.1)
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[3]
    ax.plot(g_t, g_px_std, label="σ_x", lw=1.0)
    ax.plot(g_t, g_py_std, label="σ_y", lw=1.0)
    ax.set_ylabel("global pose σ (m)")
    ax.set_yscale("log")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3, which="both")

    ax = axes[4]
    ax.plot(gps_t, innovations, ".", ms=3, label="|GPS − global| at each fix")
    ax.set_ylabel("GPS innovation (m)")
    ax.set_xlabel("time since first /odometry/filtered/global (s)")
    ax.set_yscale("symlog", linthresh=1)
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3, which="both")

    fig.tight_layout()
    out = out_dir / f"{bag_name}_global_ekf_divergence.png"
    fig.savefig(out, dpi=110)
    print(f"Saved: {out}")
    return out


def _parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("--output-dir", type=Path, default=None)
    return ap.parse_args(argv)


def main():
    args = _parse_args()
    bag_dir = args.bag_dir.resolve()
    if not bag_dir.is_dir():
        print(f"ERROR: not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)
    out_dir = args.output_dir or bag_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    bag_data = read_bag(bag_dir)
    print(
        f"Messages: global={len(bag_data.global_odom)} "
        f"local={len(bag_data.local_odom)} "
        f"gps={len(bag_data.gps_odom)} "
        f"fix={len(bag_data.fix)}"
    )
    plot(bag_data, bag_dir.name, out_dir)


if __name__ == "__main__":
    main()
