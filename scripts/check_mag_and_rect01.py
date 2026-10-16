"""Two-part script:

1. STATIONARY MAG CHECK
   Looks at stationary_11 — was the IMU actually stationary, were the thrusters
   off, and did the published mag wander relative to the gyro-integrated yaw?
   Goal: validate or refute the claim "mag drifts 11-29° in 7 s on the boat".

2. DIRECT YAW-BIAS-OVER-TIME TEST on rect_01 / grid_01 / grid_02
   Find SBL-track straight segments with chord >= 5 m. For each segment,
   compute the SBL bearing and the median IMU yaw and DVL forward-velocity
   sign. Subtract; that gives the IMU yaw bias relative to true heading.
   Track the bias over time. If the bias drifts, that's gyro integration
   error.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages

sys.path.insert(0, str(Path(__file__).resolve().parent))


# ── helpers ──────────────────────────────────────────────────────────────────

def stamp_ns(s):
    return int(s.sec) * 1_000_000_000 + int(s.nanosec)


def quat_to_rpy(qx, qy, qz, qw):
    """Return (roll, pitch, yaw) in radians, ZYX convention, ENU body frame."""
    # roll
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def mag_yaw_from_components(mx, my, mz, roll, pitch):
    """Compute compass-style yaw (ENU convention) from raw 3-axis mag,
    given roll and pitch from accel/gyro to project mag into the
    horizontal plane."""
    cosr, sinr = math.cos(roll), math.sin(roll)
    cosp, sinp = math.cos(pitch), math.sin(pitch)
    # Rotate mag from body to level frame (NED-style first, then convert)
    mx_h = mx * cosp + mz * sinp
    my_h = mx * sinr * sinp + my * cosr - mz * sinr * cosp
    # NED yaw (0 = N, +π/2 = E, CW from N)
    yaw_ned = math.atan2(my_h, mx_h)
    # Convert to ENU yaw (0 = E, +π/2 = N, CCW from E)
    yaw_enu = math.pi / 2.0 - yaw_ned
    return math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))


# ── PART 1: Stationary mag check ─────────────────────────────────────────────

def check_stationary(bag_dir: Path, out_png: Path):
    print(f"\n========== STATIONARY MAG CHECK: {bag_dir.name} ==========")
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    if mcap is None:
        print("ERROR: no MCAP")
        return

    imu_t, imu_yaw = [], []
    gyro_t, gyro_z = [], []
    mag_t, mag_x, mag_y, mag_z = [], [], [], []
    accel_t, accel_norm = [], []
    dvl_speed_t, dvl_speed = [], []
    thruster_t, thruster_active = [], []

    for msg in read_ros2_messages(str(mcap)):
        topic = msg.channel.topic
        ros = msg.ros_msg
        try:
            if topic == "/imu/data":
                t = stamp_ns(ros.header.stamp)
                if t == 0: continue
                q = ros.orientation
                _, _, yaw = quat_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))
                imu_t.append(t)
                imu_yaw.append(yaw)
                # angular vel z
                w = ros.angular_velocity
                gyro_t.append(t)
                gyro_z.append(float(w.z))
                # accel
                a = ros.linear_acceleration
                accel_t.append(t)
                accel_norm.append(math.sqrt(float(a.x)**2 + float(a.y)**2 + float(a.z)**2))
            elif topic == "/imu/mag":
                t = stamp_ns(ros.header.stamp)
                if t == 0: continue
                v = ros.magnetic_field if hasattr(ros, "magnetic_field") else None
                if v is None and hasattr(ros, "vector"):
                    v = ros.vector
                if v is None:
                    continue
                mag_t.append(t)
                mag_x.append(float(v.x))
                mag_y.append(float(v.y))
                mag_z.append(float(v.z))
            elif topic in ("/sensors/dvl/odometry_cov", "/sensors/dvl/velocity"):
                t = stamp_ns(ros.header.stamp)
                if t == 0: continue
                if topic == "/sensors/dvl/velocity":
                    v = ros.velocity
                    sp = math.sqrt(float(v.x)**2 + float(v.y)**2 + float(v.z)**2)
                else:
                    v = ros.twist.twist.linear
                    sp = math.sqrt(float(v.x)**2 + float(v.y)**2 + float(v.z)**2)
                dvl_speed_t.append(t)
                dvl_speed.append(sp)
            elif topic in ("/pixhawk/manual_control", "/pixhawk/out/manual_control"):
                t = stamp_ns(ros.header.stamp) if hasattr(ros, "header") else 0
                if t == 0: continue
                # manual_control_setpoint: x, y, z, r are [-1000, 1000]
                lvl = abs(float(ros.x)) + abs(float(ros.y)) + abs(float(ros.r))
                # threshold 50 (small enough to detect non-zero command)
                thruster_t.append(t)
                thruster_active.append(lvl > 50.0)
        except (AttributeError, IndexError, ValueError):
            continue

    if not imu_t:
        print("ERROR: no IMU data")
        return

    print(f"IMU samples:  {len(imu_t)}")
    print(f"Mag samples:  {len(mag_t)}")
    print(f"DVL samples:  {len(dvl_speed_t)}")
    print(f"Thr samples:  {len(thruster_t)}")

    imu_t_arr = np.array(imu_t, dtype=np.int64)
    imu_yaw_arr = np.unwrap(np.array(imu_yaw))
    gyro_z_arr = np.array(gyro_z)
    accel_norm_arr = np.array(accel_norm)

    t0 = int(imu_t_arr[0])
    duration = (imu_t_arr[-1] - t0) / 1e9
    print(f"Duration: {duration:.1f} s")

    # IMU yaw drift over the bag
    imu_yaw_deg = np.degrees(imu_yaw_arr - imu_yaw_arr[0])
    yaw_drift_total = float(imu_yaw_deg[-1] - imu_yaw_deg[0])
    print(f"IMU yaw drift over the bag: {yaw_drift_total:+.3f}°")
    print(f"|gyro_z| stats: median {np.degrees(np.median(np.abs(gyro_z_arr))):.4f} deg/s, "
          f"p95 {np.degrees(np.percentile(np.abs(gyro_z_arr), 95)):.4f} deg/s, "
          f"max {np.degrees(np.max(np.abs(gyro_z_arr))):.4f} deg/s")
    print(f"|accel_norm - 9.81| p95: {np.percentile(np.abs(accel_norm_arr - 9.81), 95):.3f} m/s² "
          f"(if mostly stationary on the boat, dominated by surface motion)")
    if dvl_speed:
        ds = np.array(dvl_speed)
        print(f"DVL speed median: {float(np.median(ds)):.3f} m/s, p95: {float(np.percentile(ds, 95)):.3f} m/s")
    if thruster_t:
        # Time-weighted fraction with thrust input
        ta = np.array(thruster_active)
        tt = np.array(thruster_t, dtype=np.int64)
        tsorted = np.argsort(tt)
        tt = tt[tsorted]; ta = ta[tsorted]
        if len(tt) >= 2:
            dt = np.diff(tt) / 1e9
            frac = float((dt * ta[:-1]).sum() / dt.sum()) if dt.sum() > 0 else None
            print(f"Manual control active fraction: {frac:.3f}" if frac is not None else "n/a")

    # Compute mag-derived yaw at each mag sample, using the nearest IMU
    # roll/pitch
    if mag_t:
        # We need roll/pitch at each mag sample. Recompute roll/pitch from imu data.
        imu_rpy = []
        for t, q_yaw_idx in enumerate(imu_yaw):
            pass  # we only stored yaw above; need to redo

        # Reread IMU for roll/pitch (faster than reprocessing whole bag again)
        # Just reuse what's published: pitch / roll from quat
        # Already have orientation samples, recompute below in a second pass

    # Second pass for IMU roll/pitch + mag-yaw construction
    imu_quat_t, imu_quat = [], []
    for msg in read_ros2_messages(str(mcap)):
        if msg.channel.topic != "/imu/data":
            continue
        try:
            t = stamp_ns(msg.ros_msg.header.stamp)
            if t == 0: continue
            q = msg.ros_msg.orientation
            imu_quat_t.append(t)
            imu_quat.append((float(q.x), float(q.y), float(q.z), float(q.w)))
        except AttributeError:
            pass
    imu_quat_t_arr = np.array(imu_quat_t, dtype=np.int64)

    mag_yaw_t = []
    mag_yaw_arr = []
    for i, t in enumerate(mag_t):
        # Find nearest IMU quat
        idx = int(np.argmin(np.abs(imu_quat_t_arr - t)))
        if abs(int(imu_quat_t_arr[idx]) - t) > int(0.1 * 1e9):
            continue
        roll, pitch, _ = quat_to_rpy(*imu_quat[idx])
        myw = mag_yaw_from_components(mag_x[i], mag_y[i], mag_z[i], roll, pitch)
        mag_yaw_t.append(t)
        mag_yaw_arr.append(myw)

    mag_yaw_t_arr = np.array(mag_yaw_t, dtype=np.int64)
    mag_yaw_arr = np.unwrap(np.array(mag_yaw_arr))
    if len(mag_yaw_arr):
        mag_yaw_deg = np.degrees(mag_yaw_arr - mag_yaw_arr[0])
        # Center: subtract first value of IMU yaw to put on common axis
        mag_yaw_drift_total = float(mag_yaw_deg[-1] - mag_yaw_deg[0])
        # Stats on raw mag yaw vs IMU yaw, after aligning starts
        # Compute mag-yaw - IMU-yaw at matching times
        idxs = np.searchsorted(imu_t_arr, mag_yaw_t_arr)
        idxs = np.clip(idxs, 0, len(imu_yaw_arr) - 1)
        diff = mag_yaw_arr - imu_yaw_arr[idxs]
        diff = np.array([math.atan2(math.sin(d), math.cos(d)) for d in diff])
        diff_deg = np.degrees(diff)
        # Print the first 7 s drift like the YAML claim
        t_rel = (mag_yaw_t_arr - mag_yaw_t_arr[0]) / 1e9
        if t_rel.max() >= 7.0:
            mask7 = t_rel <= 7.0
            mag_drift_7s = float(np.degrees(mag_yaw_arr[mask7][-1] - mag_yaw_arr[mask7][0]))
            print(f"\nMag-yaw drift in first 7 s of bag: {mag_drift_7s:+.3f}°")
        # over full bag
        print(f"Mag-yaw drift over bag (wraps included): {mag_yaw_drift_total:+.3f}°")
        print(f"Mag-yaw vs IMU-yaw difference  std: {float(np.std(diff_deg)):.3f}°  "
              f"p95(|diff|): {float(np.percentile(np.abs(diff_deg), 95)):.3f}°")

    # Plot
    fig, axes = plt.subplots(5, 1, figsize=(11, 10), sharex=True,
                              gridspec_kw={"hspace": 0.25})

    t_rel = (imu_t_arr - t0) / 1e9
    axes[0].plot(t_rel, imu_yaw_deg, color="navy", lw=0.8, label="IMU yaw (gyro-integrated)")
    if len(mag_yaw_arr):
        m_t_rel = (mag_yaw_t_arr - t0) / 1e9
        # Align mag yaw start to IMU yaw start by subtracting initial offset
        offset = imu_yaw_deg[0] - np.degrees(mag_yaw_arr[0])
        # actually we want them in the same coordinate system; since both unwrapped
        # from their first sample, plot mag yaw relative to its own start
        m_yaw_deg = np.degrees(mag_yaw_arr - mag_yaw_arr[0])
        axes[0].plot(m_t_rel, m_yaw_deg, color="crimson", lw=0.8, alpha=0.8,
                     label="Mag yaw (from raw /imu/mag)")
    axes[0].set_ylabel("yaw (deg, rel start)")
    axes[0].grid(True, lw=0.3, alpha=0.5)
    axes[0].legend(fontsize=8, loc="upper left")
    axes[0].set_title(f"{bag_dir.name}  —  duration {duration:.0f}s")

    axes[1].plot(t_rel, np.degrees(gyro_z_arr), color="darkorange", lw=0.5)
    axes[1].axhline(0, color="black", lw=0.3, ls=":")
    axes[1].set_ylabel("gyro z\n(deg/s)")
    axes[1].grid(True, lw=0.3, alpha=0.5)

    axes[2].plot(t_rel, accel_norm_arr, color="darkviolet", lw=0.5)
    axes[2].axhline(9.81, color="black", lw=0.3, ls=":")
    axes[2].set_ylabel("|accel|\n(m/s²)")
    axes[2].grid(True, lw=0.3, alpha=0.5)

    if dvl_speed:
        d_t = np.array(dvl_speed_t, dtype=np.int64)
        d_t_rel = (d_t - t0) / 1e9
        axes[3].plot(d_t_rel, dvl_speed, color="green", lw=0.5)
    axes[3].set_ylabel("DVL speed\n(m/s)")
    axes[3].grid(True, lw=0.3, alpha=0.5)

    if thruster_t:
        th_t = np.array(thruster_t, dtype=np.int64)
        th_t_rel = (th_t - t0) / 1e9
        th_a = np.array(thruster_active, dtype=int)
        axes[4].plot(th_t_rel, th_a, color="red", lw=0.5, drawstyle="steps-post")
    axes[4].set_ylabel("thruster\nactive")
    axes[4].set_yticks([0, 1])
    axes[4].set_xlabel("Time (s)")
    axes[4].grid(True, lw=0.3, alpha=0.5)

    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_png}")


# ── PART 2: Direct yaw-bias-over-time on bagged missions ─────────────────────

def find_straight_segments_long_chord(track, min_chord_m=3.0, ratio_max=1.20):
    """Same as before but with bigger min_chord to reduce SBL-noise impact."""
    E = track.E
    N = track.N_utm
    n = len(E)
    segs = []
    cursor = 0
    while cursor < n - 1:
        e0, n0 = float(E[cursor]), float(N[cursor])
        last_e, last_n = e0, n0
        path = 0.0
        found = None
        for j in range(cursor + 1, n):
            e, ny = float(E[j]), float(N[j])
            path += math.hypot(e - last_e, ny - last_n)
            last_e, last_n = e, ny
            chord = math.hypot(e - e0, ny - n0)
            if chord <= 0:
                continue
            ratio = path / chord
            if chord >= min_chord_m and ratio <= ratio_max:
                found = j
            elif found is not None and ratio > ratio_max:
                break
        if found is not None:
            segs.append((cursor, found))
            cursor = found + 1
        else:
            break
    return segs


def yaw_drift_test(bag_dir, label):
    from ekf_sbl_overlay import read_bag, gate_sbl, navsatfix_to_track
    print(f"\n========== {label}: yaw drift via SBL-bearing vs IMU-yaw on straight segments ==========")
    bag = read_bag(bag_dir, want_local=True)
    sbl_msgs = gate_sbl(bag.sbl, max_sbl_std=5.0)
    sbl_track = navsatfix_to_track(sbl_msgs, "SBL")

    # Read IMU yaw from /imu/data
    mcap = next(iter(bag_dir.glob("*.mcap")), None)
    imu_t, imu_yaw = [], []
    dvl_t, dvl_vx_body = [], []
    for msg in read_ros2_messages(str(mcap)):
        if msg.channel.topic == "/imu/data":
            try:
                t = stamp_ns(msg.ros_msg.header.stamp)
                if t == 0: continue
                q = msg.ros_msg.orientation
                _, _, yaw = quat_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))
                imu_t.append(t)
                imu_yaw.append(yaw)
            except AttributeError:
                pass
        elif msg.channel.topic in ("/sensors/dvl/velocity", "/sensors/dvl/odometry_cov"):
            try:
                t = stamp_ns(msg.ros_msg.header.stamp)
                if t == 0: continue
                if msg.channel.topic == "/sensors/dvl/velocity":
                    v = msg.ros_msg.velocity
                    vx = float(v.x)
                else:
                    v = msg.ros_msg.twist.twist.linear
                    vx = float(v.x)
                dvl_t.append(t); dvl_vx_body.append(vx)
            except AttributeError:
                pass
    if not imu_t:
        print("no IMU data")
        return

    imu_t_arr = np.array(imu_t, dtype=np.int64)
    imu_yaw_arr = np.unwrap(np.array(imu_yaw))
    dvl_t_arr = np.array(dvl_t, dtype=np.int64)
    dvl_vx_arr = np.array(dvl_vx_body)

    segs = find_straight_segments_long_chord(sbl_track, min_chord_m=3.0, ratio_max=1.20)
    print(f"Found {len(segs)} SBL straight segments (chord >= 3 m, ratio <= 1.20).")

    rows = []
    for (i0, i1) in segs:
        t0_ns = int(sbl_track.t_ns[i0])
        t1_ns = int(sbl_track.t_ns[i1])
        dE = sbl_track.E[i1] - sbl_track.E[i0]
        dN = sbl_track.N_utm[i1] - sbl_track.N_utm[i0]
        chord = math.hypot(dE, dN)
        if chord < 3.0:
            continue
        sbl_brg = math.degrees(math.atan2(dE, dN))
        # IMU yaw window
        mask = (imu_t_arr >= t0_ns) & (imu_t_arr <= t1_ns)
        if mask.sum() < 5:
            continue
        # Median IMU yaw in window (circular-aware)
        yaws = imu_yaw_arr[mask]
        cs = np.cos(yaws).mean()
        sn = np.sin(yaws).mean()
        imu_yaw_med = math.degrees(math.atan2(sn, cs))
        # DVL fwd velocity (body x) in window — sign tells us direction
        dmask = (dvl_t_arr >= t0_ns) & (dvl_t_arr <= t1_ns)
        dvx_med = float(np.median(dvl_vx_arr[dmask])) if dmask.sum() else 0.0
        # Convention: sbl_brg is direction of motion (atan2(dE, dN), CW from N).
        # imu_yaw is ENU yaw (atan2 over E, N in standard math, CCW from E/N).
        # Convert imu_yaw to compass-like (CW from N) for direct comparison.
        imu_yaw_compass = ((90.0 - imu_yaw_med) + 360) % 360
        sbl_brg_compass = (sbl_brg + 360) % 360
        # If AUV moving backward (vx_body < 0), heading = bearing + 180
        expected_heading = sbl_brg_compass if dvx_med >= 0 else (sbl_brg_compass + 180) % 360
        diff = ((imu_yaw_compass - expected_heading + 540) % 360) - 180
        rows.append({
            "t_mid": ((t0_ns + t1_ns) / 2 - int(sbl_track.t_ns[0])) / 1e9,
            "chord": chord,
            "sbl_brg": sbl_brg_compass,
            "dvl_vx": dvx_med,
            "imu_yaw_compass": imu_yaw_compass,
            "expected_heading": expected_heading,
            "yaw_bias": diff,
        })

    if not rows:
        print("No segments found.")
        return
    print(f"  {'t_mid':>7s}  {'chord':>6s}  {'dvx':>5s}  {'sbl_brg':>8s}  {'imu_brg':>8s}  {'expected':>8s}  {'bias':>7s}")
    for r in rows:
        print(f"  {r['t_mid']:7.1f}  {r['chord']:6.1f}  {r['dvl_vx']:+5.2f}  "
              f"{r['sbl_brg']:8.2f}  {r['imu_yaw_compass']:8.2f}  {r['expected_heading']:8.2f}  "
              f"{r['yaw_bias']:+7.2f}")

    biases = np.array([r["yaw_bias"] for r in rows])
    biases_unwrapped = np.unwrap(np.radians(biases))
    biases_deg = np.degrees(biases_unwrapped)
    if len(biases) >= 2:
        print(f"\nBias range: {biases_deg.min():+.2f}° to {biases_deg.max():+.2f}° "
              f"(spread {biases_deg.max() - biases_deg.min():.2f}°)")
        print(f"Bias drift first→last: {biases_deg[-1] - biases_deg[0]:+.2f}°")


def main():
    out_dir = Path("diagnosis/zermatt/mag_check")
    out_dir.mkdir(parents=True, exist_ok=True)

    # --- PART 1 (already ran; skip rerun)
    # stat_dir = Path("recordings/rosbags/2026-04-19/stationary_11_2026_04_19-16_18_49")
    # if stat_dir.exists():
    #     check_stationary(stat_dir, out_dir / "stationary_11_mag_check.png")

    # --- PART 2 ---
    for label, p in [
        ("rect_01", Path("recordings/rosbags/2026-04-29/zermatt_rectangle_01_2026_04_29-12_17_06")),
        ("grid_01", Path("recordings/rosbags/2026-04-30/zermatt_grid_01_2026_04_30-12_04_16")),
        ("grid_02", Path("recordings/rosbags/2026-04-30/zermatt_grid_02_2026_04_30-13_00_44")),
    ]:
        if p.exists():
            yaw_drift_test(p, label)


if __name__ == "__main__":
    main()
