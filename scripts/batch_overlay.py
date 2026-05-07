#!/usr/bin/env python3
"""Batch wrapper around odom_to_gnss_overlay.py.

Runs the overlay script on a fixed list of bags from the 2026-04-23 session,
then aggregates the per-bag metadata JSONs into a summary CSV + Markdown table.
"""

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

BAGS = [
    "global_run_gnss_01_2026_04_23-14_00_14",
    "global_run_gnss_03_2026_04_23-14_03_53",
    "global_run_gnss_04_2026_04_23-14_06_35",
    "straight_surge_02_2026_04_23-13_35_18",
    "straight_surge_03_2026_04_23-13_37_29",
    "straight_surge_06_2026_04_23-13_45_45",
    "straight_surge_07_2026_04_23-13_49_11",
]

CSV_HEADER = [
    "bag",
    "status",
    "duration_s",
    "distance_m",
    "drift_rate_m_per_100m",
    "gnss_fixes",
    "h_acc_mean_m",
]


def run_overlay(overlay_script: Path, bag_dir: Path, out_subdir: Path,
                max_h_acc: float) -> int:
    out_subdir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(overlay_script),
        str(bag_dir),
        "--max-h-acc", str(max_h_acc),
        "--output-dir", str(out_subdir),
    ]
    result = subprocess.run(cmd, check=False)
    return result.returncode


def load_metadata(json_path: Path) -> dict | None:
    try:
        with json_path.open() as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"ERROR reading {json_path}: {exc}", file=sys.stderr)
        return None


def extract_row(bag: str, status: str, meta: dict | None) -> dict:
    row = {"bag": bag, "status": status,
           "duration_s": None, "distance_m": None,
           "drift_rate_m_per_100m": None, "gnss_fixes": None,
           "h_acc_mean_m": None}
    if meta is None:
        return row
    track = meta.get("track", {})
    drift = meta.get("drift", {})
    gnss = meta.get("gnss", {})
    row["duration_s"] = track.get("total_time_s")
    row["distance_m"] = track.get("total_distance_m")
    row["drift_rate_m_per_100m"] = drift.get("rate_m_per_100m")
    row["gnss_fixes"] = gnss.get("accepted_fixes")
    row["h_acc_mean_m"] = gnss.get("h_acc_mean_m")
    return row


def fmt(val, spec: str) -> str:
    if val is None:
        return "-"
    return format(val, spec)


def print_markdown_table(rows: list[dict]) -> None:
    print()
    print("| Bag | Status | Duration (s) | Distance (m) | "
          "Drift rate (m/100m) | GNSS fixes | h_acc mean (m) |")
    print("|-----|--------|--------------|--------------|"
          "---------------------|------------|----------------|")
    for r in rows:
        print(
            f"| {r['bag']} | {r['status']} "
            f"| {fmt(r['duration_s'], '.1f')} "
            f"| {fmt(r['distance_m'], '.1f')} "
            f"| {fmt(r['drift_rate_m_per_100m'], '.3f')} "
            f"| {r['gnss_fixes'] if r['gnss_fixes'] is not None else '-'} "
            f"| {fmt(r['h_acc_mean_m'], '.3f')} |"
        )


def write_csv(rows: list[dict], csv_path: Path) -> None:
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("recordings_dir", type=Path,
                    help="Parent directory containing the bag subdirectories")
    ap.add_argument("--output-dir", type=Path, required=True,
                    help="Where per-bag artifacts and summary.csv are written")
    ap.add_argument("--max-h-acc", type=float, default=2.0,
                    help="GNSS quality gate (m), forwarded to overlay script")
    ap.add_argument("--overlay-script", type=Path,
                    default=Path(__file__).parent / "odom_to_gnss_overlay.py",
                    help="Path to odom_to_gnss_overlay.py")
    ap.add_argument("--bags-list", type=str, default=None,
                    help="Comma-separated bag names to override the default BAGS")
    args = ap.parse_args()

    if not args.overlay_script.exists():
        print(f"ERROR: overlay script not found: {args.overlay_script}",
              file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)

    bags = [b.strip() for b in args.bags_list.split(",")] if args.bags_list else BAGS
    n = len(bags)

    rows: list[dict] = []
    for i, bag in enumerate(bags, 1):
        bag_dir = args.recordings_dir / bag
        out_subdir = args.output_dir / bag
        json_path = out_subdir / f"{bag}_odom_gnss_metadata.json"

        if not bag_dir.is_dir() or not list(bag_dir.glob("*.mcap")):
            print(f"[{i}/{n}] {bag} -> MISSING (no bag dir or .mcap)")
            rows.append(extract_row(bag, "MISSING", None))
            continue

        print(f"[{i}/{n}] {bag} -> running overlay")
        rc = run_overlay(args.overlay_script, bag_dir, out_subdir, args.max_h_acc)
        print(f"[{i}/{n}] {bag} -> exit {rc}")

        if rc != 0:
            rows.append(extract_row(bag, "FAILED", None))
            continue

        meta = load_metadata(json_path)
        if meta is None:
            rows.append(extract_row(bag, "FAILED", None))
            continue

        rows.append(extract_row(bag, "OK", meta))

    print_markdown_table(rows)

    csv_path = args.output_dir / "summary.csv"
    write_csv(rows, csv_path)
    print(f"\nSummary CSV written to {csv_path}")

    return 0 if all(r["status"] == "OK" for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
