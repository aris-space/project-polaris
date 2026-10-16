"""Quick chronological dump of Zermatt diagnosis health profiles."""
import json
import sys
from pathlib import Path

ROOT = Path("diagnosis/zermatt")
ORDER = [
    "zermatt_rectangle_01_2026_04_29-12_17_06",
    "zermatt_rectangle_02_2026_04_29-13_00_00",
    "zermatt_rectangle_04_2026_04_29-13_00_20",
    "zermatt_rectangle_06_2026_04_29-13_22_13",
    "zermatt_rectangle_07_2026_04_29-13_38_44",
    "zermatt_rectangle_08_2026_04_29-13_48_42",
    "zermatt_grid_01_2026_04_30-12_04_16",
    "zermatt_grid_02_2026_04_30-13_00_44",
]

print(f"{'bag':50s} {'class':22s} {'mean':>7s} {'max':>7s} {'lock':>6s} "
      f"{'unlock':>8s} {'cov_i':>10s} {'cov_max':>10s} {'cov_f':>10s} "
      f"{'growth':>10s} {'cov_yaw':>10s} {'spd':>5s} {'sp95':>5s} {'depth':>6s}")
for name in ORDER:
    p = ROOT / name / "diagnosis.json"
    if not p.exists():
        print(f"{name:50s}  MISSING")
        continue
    d = json.loads(p.read_text())
    m = d.get("metrics", {}) or {}
    h = d.get("health", {}) or {}

    def fmt(v, dec=2, sci=False):
        if v is None: return "-"
        if sci: return f"{v:.2e}"
        return f"{v:.{dec}f}"

    print(f"{name[:50]:50s} {d['classification'][:22]:22s} "
          f"{fmt(m.get('mean_error_m')):>7s} {fmt(m.get('max_error_m')):>7s} "
          f"{fmt(h.get('dvl_lock_fraction'),3):>6s} "
          f"{fmt(h.get('dvl_longest_unlock_s'),1):>8s} "
          f"{fmt(h.get('cov_xy_initial'),sci=True):>10s} "
          f"{fmt(h.get('cov_xy_max'),sci=True):>10s} "
          f"{fmt(h.get('cov_xy_final'),sci=True):>10s} "
          f"{fmt(h.get('cov_xy_growth_ratio'),1):>10s} "
          f"{fmt(h.get('cov_yaw_max'),sci=True):>10s} "
          f"{fmt(h.get('speed_mean_mps'),2):>5s} "
          f"{fmt(h.get('speed_p95_mps'),2):>5s} "
          f"{fmt(h.get('depth_z_range_m'),2):>6s}")

# Also print number of high-error intervals
print()
print("--- high-error intervals (>3m) per bag ---")
for name in ORDER:
    p = ROOT / name / "diagnosis.json"
    if not p.exists(): continue
    d = json.loads(p.read_text())
    metas = ROOT / name
    meta_p = metas / f"ekf_sbl_{name}_metadata.json"
    if meta_p.exists():
        meta = json.loads(meta_p.read_text())
        ivs = meta.get("high_error_intervals_s", []) or []
        if ivs:
            print(f"{name}:")
            for iv in ivs[:5]:
                print(f"  t={iv[0]:.1f}-{iv[1]:.1f}s  peak={iv[2]:.2f}m  dur={iv[1]-iv[0]:.1f}s")
        else:
            print(f"{name}: no >3m intervals")
