#!/usr/bin/env python3
"""
Regenerate measurements_unfiltered.csv on the host (outside docker).

The host doesn't have foxglove_msgs, so we can't decode /measurement_grid.
Instead we load the gridpoint targets (id, lat, lon) from an existing
measurements_av.csv and patch extract_zermatt_measurements.TOPICS to drop
the grid topic before any reader sees it.

Outputs measurements_raw.csv, measurements_av.csv, measurements_unfiltered.csv
into --out (default: src/ice_estimates/zermatt_results/).
"""
import argparse
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_ROOT = os.path.dirname(HERE)
sys.path.insert(0, SCRIPTS_ROOT)

import extract_zermatt_measurements as ext  # noqa: E402


def load_grid_points_from_av(av_csv):
    seen = {}
    with open(av_csv) as f:
        for row in csv.DictReader(f):
            pid = int(row["grid_point_id"])
            if pid in seen:
                continue
            seen[pid] = {
                "id": pid,
                "lat": float(row["grid_point_lat"]),
                "lon": float(row["grid_point_lon"]),
            }
    return list(seen.values())


def main():
    repo_root = os.path.abspath(os.path.join(HERE, "../../.."))
    default_bags = [
        os.path.join("/mnt/c/Users/ridhc/Repos/project-polaris/recordings",
                     "zermatt_grid_01_2026_04_30-12_04_16_0.mcap"),
        os.path.join("/mnt/c/Users/ridhc/Repos/project-polaris/recordings",
                     "zermatt_grid_02_2026_04_30-13_00_44_0.mcap"),
    ]
    default_out = os.path.join(repo_root, "src/ice_estimates/zermatt_results")
    default_av  = os.path.join(default_out, "measurements_av.csv")

    p = argparse.ArgumentParser()
    p.add_argument("--bags", nargs="+", default=default_bags)
    p.add_argument("--grid-from", default=default_av,
                   help="Existing measurements_av.csv to source gridpoint targets from")
    p.add_argument("--out", default=default_out)
    p.add_argument("--config", default=os.path.join(HERE, "config.yaml"))
    args = p.parse_args()

    cfg = ext._load_config(args.config)

    def cfg_get(key, default):
        return cfg.get(key, default)

    sensor_z  = cfg_get("pressure_to_contact_z_m", ext.DEFAULT_PRESSURE_TO_CONTACT_Z_M)
    sensor_x  = cfg_get("pressure_to_contact_x_m", ext.DEFAULT_PRESSURE_TO_CONTACT_X_M)
    rho_water = cfg_get("rho_water",               ext.DEFAULT_RHO_WATER)
    rho_ice   = cfg_get("rho_ice",                 ext.DEFAULT_RHO_ICE)
    g         = cfg_get("g",                       ext.DEFAULT_G)
    max_pitch = cfg_get("max_pitch_deg",           ext.DEFAULT_MAX_PITCH_DEG)
    max_roll  = cfg_get("max_roll_deg",            ext.DEFAULT_MAX_ROLL_DEG)
    gnss_acc  = cfg_get("gnss_max_accuracy_m",     ext.DEFAULT_GNSS_MAX_ACCURACY_M)
    min_dur   = cfg_get("min_duration_s",          ext.DEFAULT_MIN_DURATION_S)
    max_gap   = cfg_get("max_gap_s",               ext.DEFAULT_MAX_GAP_S)
    min_depth = cfg_get("min_depth_m",             ext.DEFAULT_MIN_DEPTH_M)
    depth_win = cfg_get("depth_window_s",          ext.DEFAULT_DEPTH_WINDOW_S)
    max_dev   = cfg_get("max_depth_dev_m",         ext.DEFAULT_MAX_DEPTH_DEV_M)

    # Patch: remove /measurement_grid from the topic filter so the reader never
    # surfaces a message we can't decode (no foxglove_msgs on host).
    ext.TOPICS = [t for t in ext.TOPICS if t != "/measurement_grid"]

    grid_points = load_grid_points_from_av(args.grid_from)
    print(f"Loaded {len(grid_points)} gridpoints from {args.grid_from}: "
          f"{sorted(p['id'] for p in grid_points)}")

    os.makedirs(args.out, exist_ok=True)

    all_sessions = []
    for bag in args.bags:
        print(f"Extracting from {os.path.basename(bag)}...")
        sessions = ext.extract_sessions(
            bag, grid_points,
            gnss_max_accuracy_m=gnss_acc,
            pressure_to_contact_z_m=sensor_z, pressure_to_contact_x_m=sensor_x,
            rho_water=rho_water, rho_ice=rho_ice, g=g,
            max_pitch_deg=max_pitch, max_roll_deg=max_roll,
        )
        print(f"  {len(sessions)} raw sessions")
        all_sessions.extend(sessions)

    print(f"\nFiltering (min {min_dur}s, min depth {min_depth}m) "
          f"and merging (gap <= {max_gap}s)...")
    sessions = ext.filter_and_merge(all_sessions, min_dur, max_gap, min_depth)
    print(f"  {len(sessions)} sessions remaining")

    print(f"\nApplying depth stability filter "
          f"(window={depth_win}s, max_dev={max_dev*100:.0f} cm)...")
    sessions = ext.apply_depth_stability_filter(sessions, max_dev, depth_win)

    raw_path = os.path.join(args.out, "measurements_raw.csv")
    av_path  = os.path.join(args.out, "measurements_av.csv")
    unf_path = os.path.join(args.out, "measurements_unfiltered.csv")

    total_kept = 0
    total_unf = 0
    with open(raw_path, "w", newline="") as rf, \
         open(av_path, "w", newline="") as af, \
         open(unf_path, "w", newline="") as uf:
        raw_w = csv.DictWriter(rf, fieldnames=ext.RAW_FIELDS)
        av_w  = csv.DictWriter(af, fieldnames=ext.AV_FIELDS)
        unf_w = csv.DictWriter(uf, fieldnames=ext.UNFILTERED_FIELDS)
        raw_w.writeheader(); av_w.writeheader(); unf_w.writeheader()

        for s in sessions:
            for row in s["raw_rows"]:
                unf_w.writerow({k: row.get(k, "") for k in ext.UNFILTERED_FIELDS})
                total_unf += 1
                if not row["rejection_reason"]:
                    raw_w.writerow({k: row[k] for k in ext.RAW_FIELDS})
                    total_kept += 1
            av_row = ext.build_av_row(s)
            if av_row is not None:
                av_w.writerow(av_row)

    print(f"\nDone.  sessions={len(sessions)}  kept={total_kept}  total={total_unf}")
    print(f"  {raw_path}")
    print(f"  {av_path}")
    print(f"  {unf_path}")


if __name__ == "__main__":
    main()
