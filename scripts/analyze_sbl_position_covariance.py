"""
Prompt 3: SBL Position and Covariance Check
Lake test 2026-04-19 analysis.

Bags: vertical_01, yaw_turns_01/02/03
Note: /fix is all-zero in these bags (no GNSS fix), so GNSS comparison is not possible.
Instead: SBL relative position, covariance, quality, /gps/selected source.
"""

import pathlib
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mcap_ros2.reader import read_ros2_messages
import utm

BAG_BASE = pathlib.Path("C:/Users/gleb0/Downloads/rosbags (2)/rosbags")
OUT_DIR = pathlib.Path("C:/Gleb/Uni/Fokusprojekt/project-polaris-main/scripts")

BAGS = [
    "vertical_01_2026_04_19-12_03_52",
    "yaw_turns_01_2026_04_19-11_51_21",
    "yaw_turns_02_2026_04_19-11_56_59",
    "yaw_turns_03_2026_04_19-11_59_26",
]

TOPICS = [
    "/waterlinked_ugps/locator_position_global",
    "/waterlinked_ugps/navsatfix",
    "/waterlinked_ugps/locator_acoustic_quality",
    "/waterlinked_ugps/locator_position_relative_wrt_topside",
    "/gps/selected",
    "/fix",
]


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def nsf_covariance_hstd(cov):
    """sqrt of max eigenvalue of 2x2 ENU horizontal block from NavSatFix covariance (indices 0,1,3,4)."""
    a, b, d = cov[0], cov[1], cov[4]
    trace = a + d
    det = a * d - b * b
    disc = max(0.0, (trace / 2) ** 2 - det)
    lam_max = trace / 2 + math.sqrt(disc)
    return math.sqrt(max(0.0, lam_max))


def load_bag(bag_name):
    bag_dir = BAG_BASE / bag_name
    mcap_file = next(bag_dir.glob("*.mcap"))
    data = {t: [] for t in TOPICS}
    for msg in read_ros2_messages(str(mcap_file)):
        topic = msg.channel.topic
        if topic not in data:
            continue
        ros_msg = msg.ros_msg
        t = stamp_to_sec(ros_msg.header.stamp)
        if t == 0.0:
            continue
        data[topic].append((t, ros_msg))
    return data


def analyse_bag(bag_name):
    print(f"\n{'=' * 65}")
    print(f"Bag: {bag_name}")
    print(f"{'=' * 65}")

    data = load_bag(bag_name)

    # ── message counts ────────────────────────────────────────────
    for topic in TOPICS:
        print(f"  {topic:<55}: {len(data[topic])} msgs")

    sbl_nsf = data["/waterlinked_ugps/navsatfix"]
    sbl_rel = data["/waterlinked_ugps/locator_position_relative_wrt_topside"]
    sbl_glo = data["/waterlinked_ugps/locator_position_global"]
    quality = data["/waterlinked_ugps/locator_acoustic_quality"]
    selected = data["/gps/selected"]

    if not sbl_nsf:
        print("  [SKIP] No SBL navsatfix data.")
        return

    t0_all = sbl_nsf[0][0]

    # ── 1. Frozen-SBL detection ───────────────────────────────────
    unique_lats = len({round(m.latitude, 9) for _, m in sbl_nsf})
    unique_lons = len({round(m.longitude, 9) for _, m in sbl_nsf})
    sbl_frozen = (unique_lats == 1 and unique_lons == 1)
    print(f"\n  SBL FROZEN (1 unique position): {sbl_frozen}")
    print(f"    Unique lat: {unique_lats}  |  unique lon: {unique_lons}")

    # ── 2. Covariance analysis ────────────────────────────────────
    cov_all_zero = all(all(c == 0.0 for c in m.position_covariance) for _, m in sbl_nsf)
    print(f"  Covariance all-zero         : {cov_all_zero}")
    sbl_cov_t = np.array([t for t, _ in sbl_nsf]) - t0_all
    sbl_h_std = np.array([nsf_covariance_hstd(list(m.position_covariance)) for _, m in sbl_nsf])
    sbl_v_std = np.array([math.sqrt(max(0.0, m.position_covariance[8])) for _, m in sbl_nsf])
    h_nonzero = sbl_h_std[sbl_h_std > 0]
    if len(h_nonzero) > 0:
        print(f"  Horiz std (sqrt max eigenval): min={h_nonzero.min():.2f} m  mean={h_nonzero.mean():.2f} m  max={h_nonzero.max():.2f} m")
    else:
        print("  Horiz std: all zero (covariance not set by translator)")

    # ── 3. Acoustic quality ───────────────────────────────────────
    if quality:
        q_t = np.array([t for t, _ in quality]) - t0_all
        q_v = np.array([m.vector.x for _, m in quality])
        print(f"  Acoustic quality: min={q_v.min():.1f}  mean={q_v.mean():.1f}  max={q_v.max():.1f}")
        print(f"  Fraction quality==5 (worst) : {np.mean(q_v == 5):.1%}")
        print(f"  Fraction quality >= 3       : {np.mean(q_v >= 3):.1%}")
    else:
        q_t, q_v = np.array([]), np.array([])

    # ── 4. Relative position (x,y,z from topside in metres) ──────
    if sbl_rel:
        rel_t = np.array([t for t, _ in sbl_rel]) - t0_all
        rel_x = np.array([m.vector.x for _, m in sbl_rel])
        rel_y = np.array([m.vector.y for _, m in sbl_rel])
        rel_z = np.array([m.vector.z for _, m in sbl_rel])
        horiz_spread = np.hypot(rel_x - rel_x.mean(), rel_y - rel_y.mean())
        print(f"\n  Relative position (topside frame, metres):")
        print(f"    x: mean={rel_x.mean():.2f}  std={rel_x.std():.2f}  range=[{rel_x.min():.2f}, {rel_x.max():.2f}]")
        print(f"    y: mean={rel_y.mean():.2f}  std={rel_y.std():.2f}  range=[{rel_y.min():.2f}, {rel_y.max():.2f}]")
        print(f"    z: mean={rel_z.mean():.2f}  std={rel_z.std():.2f}  range=[{rel_z.min():.2f}, {rel_z.max():.2f}]")
        print(f"    horiz spread p50={np.percentile(horiz_spread,50):.2f} m  p95={np.percentile(horiz_spread,95):.2f} m")
        outliers_20m = np.sum(horiz_spread > 20)
        print(f"    wild outliers >20 m from centroid: {outliers_20m}")
    else:
        rel_t = rel_x = rel_y = rel_z = np.array([])
        horiz_spread = np.array([])

    # ── 5. Global position (UTM scatter) ─────────────────────────
    sbl_utm_e, sbl_utm_n = [], []
    for _, m in sbl_glo:
        if m.position.latitude == 0.0 and m.position.longitude == 0.0:
            continue
        e, n, _, _ = utm.from_latlon(m.position.latitude, m.position.longitude)
        sbl_utm_e.append(e)
        sbl_utm_n.append(n)
    sbl_utm_e = np.array(sbl_utm_e)
    sbl_utm_n = np.array(sbl_utm_n)

    # ── 6. /gps/selected source breakdown ────────────────────────
    n_gnss = sum(1 for _, m in selected if m.header.frame_id == "gnss_link")
    n_sbl_sel = sum(1 for _, m in selected if m.header.frame_id == "sbl_link")
    n_other = len(selected) - n_gnss - n_sbl_sel
    if selected:
        print(f"\n  /gps/selected source: GNSS={n_gnss} ({n_gnss/len(selected):.1%})  "
              f"SBL={n_sbl_sel} ({n_sbl_sel/len(selected):.1%})  other={n_other}")
    else:
        print("\n  /gps/selected: no messages")

    # ── 7. Plots ──────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 14))
    frozen_tag = " *** SBL FROZEN ***" if sbl_frozen else " (SBL active)"
    fig.suptitle(f"Prompt 3 – SBL Position & Covariance\n{bag_name}{frozen_tag}",
                 fontsize=10, color="crimson" if sbl_frozen else "black")
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # 7a. Relative position scatter (x, y)
    ax_xy = fig.add_subplot(gs[0, 0])
    if len(rel_x) > 0:
        sc = ax_xy.scatter(rel_x, rel_y, c=rel_t, cmap="plasma", s=18, alpha=0.7)
        ax_xy.scatter(rel_x.mean(), rel_y.mean(), marker="+", s=120, c="red", zorder=5, label="centroid")
        plt.colorbar(sc, ax=ax_xy, label="Time [s]")
        ax_xy.legend(fontsize=8)
    ax_xy.set_xlabel("x (East from topside) [m]")
    ax_xy.set_ylabel("y (North from topside) [m]")
    ax_xy.set_title("SBL relative position scatter (horiz)")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, lw=0.3)

    # 7b. Depth (z) over time
    ax_z = fig.add_subplot(gs[0, 1])
    if len(rel_z) > 0:
        ax_z.plot(rel_t, rel_z, lw=1, color="steelblue")
        ax_z.set_xlabel("Time [s]")
        ax_z.set_ylabel("z depth from topside [m]")
        ax_z.set_title("SBL depth over time (z positive downward?)")
        ax_z.grid(True, lw=0.3)

    # 7c. Acoustic quality + covariance overlay
    ax_q = fig.add_subplot(gs[1, 0])
    if len(q_v) > 0:
        ax_q.step(q_t, q_v, where="post", color="seagreen", lw=1.2, label="quality (0-5)")
        ax_q.set_ylim(-0.2, 6)
        ax_q.set_ylabel("Acoustic quality")
        ax_q.set_xlabel("Time [s]")
        ax_q.set_title("SBL Acoustic Quality over time")
        ax_q.axhline(3, color="gray", lw=0.6, ls="--", label="quality=3")
        ax_q.legend(fontsize=8)
    ax_q.grid(True, lw=0.3)

    # 7d. Covariance (h_std) over time
    ax_cov = fig.add_subplot(gs[1, 1])
    if np.any(sbl_h_std > 0):
        ax_cov.plot(sbl_cov_t, sbl_h_std, color="darkorange", lw=1, label="horiz std [m]")
        ax_cov.plot(sbl_cov_t, sbl_v_std, color="tomato", lw=1, ls="--", label="vert std [m]")
        ax_cov.legend(fontsize=8)
    else:
        ax_cov.text(0.5, 0.5, "Covariance all-zero\n(translator not setting std_m)",
                    ha="center", va="center", transform=ax_cov.transAxes, color="crimson", fontsize=10)
    ax_cov.set_xlabel("Time [s]")
    ax_cov.set_ylabel("std [m]")
    ax_cov.set_title("SBL covariance from /navsatfix")
    ax_cov.grid(True, lw=0.3)

    # 7e. UTM global scatter
    ax_utm = fig.add_subplot(gs[2, 0])
    if len(sbl_utm_e) > 0:
        e0, n0 = sbl_utm_e.mean(), sbl_utm_n.mean()
        ax_utm.scatter(sbl_utm_e - e0, sbl_utm_n - n0, s=14, c="steelblue", alpha=0.6)
        ax_utm.scatter(0, 0, marker="+", s=120, c="red", zorder=5, label="centroid")
        ax_utm.set_xlabel("East offset [m]")
        ax_utm.set_ylabel("North offset [m]")
        ax_utm.set_title("SBL global position (UTM, relative to centroid)")
        ax_utm.set_aspect("equal", adjustable="datalim")
        ax_utm.legend(fontsize=8)
    ax_utm.grid(True, lw=0.3)

    # 7f. Horiz spread histogram
    ax_hist = fig.add_subplot(gs[2, 1])
    if len(horiz_spread) > 0:
        ax_hist.hist(horiz_spread, bins=40, color="steelblue", edgecolor="white", lw=0.4)
        ax_hist.axvline(np.percentile(horiz_spread, 95), color="crimson", lw=1.2, ls="--",
                        label=f"p95={np.percentile(horiz_spread,95):.1f} m")
        ax_hist.axvline(20, color="black", lw=0.8, ls=":", label="20 m threshold")
        ax_hist.legend(fontsize=8)
    ax_hist.set_xlabel("Horiz distance from centroid [m]")
    ax_hist.set_ylabel("Count")
    ax_hist.set_title("SBL relative position cluster radius")
    ax_hist.grid(True, lw=0.3)

    out_path = OUT_DIR / f"sbl_analysis_{bag_name[:22]}.png"
    fig.savefig(out_path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"\n  Plot saved: {out_path.name}")


def main():
    print("Prompt 3 — SBL Position and Covariance Check")
    print("Note: /fix is all-zero in these bags; GNSS–SBL comparison not possible.")
    for bag_name in BAGS:
        try:
            analyse_bag(bag_name)
        except Exception as exc:
            import traceback
            print(f"  ERROR in {bag_name}: {exc}")
            traceback.print_exc()
    print("\nDone.")


if __name__ == "__main__":
    main()
