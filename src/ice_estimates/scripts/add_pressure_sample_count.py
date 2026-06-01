#!/usr/bin/env python3
"""Augment measurements_av.csv with a true pressure-event count per session.

The extractor writes one row per arriving message on ANY subscribed topic
(pressure, odometry, GNSS, ...), so n_samples in measurements_av.csv
reflects the combined topic-event rate (~130 Hz), not the 50 Hz pressure
rate. This script post-processes the existing CSVs and adds:

  n_pressure_samples  – kept rows whose pressure_pa differs from the
                        previous kept row in the same session, i.e. the
                        count of distinct pressure events that survived
                        the attitude/depth-stability filter.

Writes measurements_av.csv in place with the new column appended.
"""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

DEFAULT_UNF = Path(__file__).resolve().parent.parent / "zermatt_results" / "measurements_unfiltered.csv"
DEFAULT_AV  = Path(__file__).resolve().parent.parent / "zermatt_results" / "measurements_av.csv"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--unfiltered", default=str(DEFAULT_UNF))
    ap.add_argument("--av",         default=str(DEFAULT_AV))
    ap.add_argument("--out",        default=None,
                    help="Output CSV path (default: overwrite --av).")
    args = ap.parse_args()

    out_path = Path(args.out) if args.out else Path(args.av)

    # Group unfiltered rows by (bag, gp), sorted by timestamp.
    by_bag_gp = defaultdict(list)
    with open(args.unfiltered) as f:
        for r in csv.DictReader(f):
            key = (r["bag"], int(r["grid_point_id"]))
            by_bag_gp[key].append((
                float(r["timestamp_s"]),
                r["pressure_pa"],
                r.get("rejection_reason") or "",
            ))
    for k in by_bag_gp:
        by_bag_gp[k].sort(key=lambda x: x[0])

    with open(args.av) as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames)
        av_rows = list(reader)

    if "n_pressure_samples" not in fieldnames:
        fieldnames.append("n_pressure_samples")

    if "n_pressure_raw" not in fieldnames:
        fieldnames.append("n_pressure_raw")

    for row in av_rows:
        bag = row["bag"]
        gp = int(row["grid_point_id"])
        t_start = float(row["session_start_s"])
        t_end   = float(row["session_end_s"])
        candidates = by_bag_gp.get((bag, gp), [])

        n_p_kept = 0; prev_p_kept = None
        n_p_raw  = 0; prev_p_raw  = None
        for t, p, reason in candidates:
            if t < t_start or t > t_end:
                continue
            if p != prev_p_raw:
                n_p_raw += 1
                prev_p_raw = p
            if reason:
                continue
            if p != prev_p_kept:
                n_p_kept += 1
                prev_p_kept = p
        row["n_pressure_samples"] = n_p_kept
        row["n_pressure_raw"]     = n_p_raw

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(av_rows)

    # Summary
    print(f"wrote {out_path}")
    print(f"\n{'gp':>3}  {'dur_s':>7}  {'n_samples':>10}  {'n_pres_kept':>12}  {'n_pres_raw':>11}  {'Hz_raw':>7}  {'Hz_kept':>7}")
    for row in av_rows:
        dur = float(row["duration_s"])
        n_s = int(row["n_samples"])
        n_pk = int(row["n_pressure_samples"])
        n_pr = int(row["n_pressure_raw"])
        rate_raw  = n_pr / dur if dur > 0 else float("nan")
        rate_kept = n_pk / dur if dur > 0 else float("nan")
        print(f"{int(row['grid_point_id']):>3}  {dur:>7.1f}  {n_s:>10}  {n_pk:>12}  {n_pr:>11}  {rate_raw:>6.1f}  {rate_kept:>6.1f}")


if __name__ == "__main__":
    main()
