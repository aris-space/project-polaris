"""Unit tests for the ROS-free SBL quick-check statistics (run: pytest, or python this file)."""

import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "uwgpsg2_ros2_interface"))
import sbl_quick_math as q  # noqa: E402


def make_pings(n=60, rate_hz=2.0, base=(10.0, 10.2, 10.4, 10.1), sigma=0.05, valid=(1, 1, 1, 1),
               drift_mps=0.0, seed=0, frozen_rx=None, dropout=None):
    """n pings at rate_hz; receiver j range = base[j] + drift + Gaussian noise; dropout = {j: share}."""
    rng = random.Random(seed)
    pings = []
    for k in range(n):
        t = k / rate_hz
        dist, val = [], []
        for j, b in enumerate(base):
            r = b + drift_mps * t + rng.gauss(0.0, sigma)
            if frozen_rx is not None and j == frozen_rx:
                r = b
            ok = valid[j] and not (dropout and rng.random() < dropout.get(j, 0.0))
            dist.append(r if ok else q.NO_MEASUREMENT)
            val.append(1 if ok else 0)
        pings.append({"t": t, "distance": dist, "valid": val, "rssi": [-60.0] * 4, "nsd": [-100.0] * 4,
                      "position_valid": sum(val) >= 3})
    return pings


def test_ping_rate_and_valid_shares():
    s = q.window_stats(make_pings(n=61, rate_hz=2.0))
    assert abs(s["ping_rate_hz"] - 2.0) < 1e-9
    assert s["n_pings"] == 61 and abs(s["duration_s"] - 30.0) < 1e-9
    assert all(r["valid_share"] == 1.0 for r in s["receivers"])
    assert s["share_4"] == 1.0 and s["share_ge3"] == 1.0 and s["share_position_valid"] == 1.0


def test_scatter_recovers_sigma_and_ignores_slow_motion():
    still = q.window_stats(make_pings(n=2000, sigma=0.05, seed=1))
    moving = q.window_stats(make_pings(n=2000, sigma=0.05, drift_mps=0.25, seed=1))   # 0.25 m/s radial drift
    for s in (still, moving):
        for r in s["receivers"]:
            assert abs(r["scatter_m"] - 0.05) < 0.006
    assert abs(moving["receivers"][0]["range_m"] - (10.0 + 0.25 * 999.5 / 2.0)) < 1.0   # median moved, scatter not


def test_scatter_ignores_a_single_outlier():
    pings = make_pings(n=200, sigma=0.05, seed=2)
    pings[100]["distance"][0] += 5.0
    assert abs(q.window_stats(pings)["receivers"][0]["scatter_m"] - 0.05) < 0.015


def test_minus_one_is_not_a_measurement_even_if_flagged_valid():
    pings = make_pings(n=20)
    for p in pings:
        p["distance"][2] = q.NO_MEASUREMENT          # valid flag left at 1
    s = q.window_stats(pings)
    assert s["receivers"][2]["valid_share"] == 0.0 and s["receivers"][2]["range_m"] is None
    assert s["share_4"] == 0.0 and s["share_ge3"] == 1.0


def test_sanity_verdict_levels():
    assert q.sanity_verdict(q.window_stats(make_pings()))[0] == "OK"
    assert q.sanity_verdict(q.window_stats([]))[0] == "BAD"
    slow = q.window_stats(make_pings(n=3, rate_hz=1.0 / 66.0))          # the 22.10. locator fault
    level, why = q.sanity_verdict(slow)
    assert level == "BAD" and "ping rate" in why[0]
    two = q.window_stats(make_pings(valid=(1, 1, 0, 0)))
    assert q.sanity_verdict(two)[0] == "DEGRADED"
    none = q.window_stats(make_pings(valid=(0, 0, 0, 0)))
    assert q.sanity_verdict(none)[0] == "BAD"
    frozen = q.window_stats(make_pings(frozen_rx=1))
    level, why = q.sanity_verdict(frozen)
    assert level == "DEGRADED" and "frozen" in why[0]
    all_frozen = q.window_stats(make_pings(sigma=0.0))
    assert q.sanity_verdict(all_frozen)[0] == "BAD"


def test_compare_with_reference():
    ref = q.window_stats(make_pings(n=400, sigma=0.05, seed=3), fix_errors_m=[1.0, 1.2, 0.9])
    same = q.window_stats(make_pings(n=400, sigma=0.06, seed=4), fix_errors_m=[1.3, 1.1])
    assert q.compare_verdict(same, ref)[0] == "comparable"
    noisy = q.window_stats(make_pings(n=400, sigma=0.20, seed=5))
    word, why = q.compare_verdict(noisy, ref)
    assert word == "clearly worse" and "scatter" in why[0]
    drop = q.window_stats(make_pings(n=400, sigma=0.05, seed=6, dropout={0: 0.5}))
    word, why = q.compare_verdict(drop, ref)
    assert word == "clearly worse" and "receiver 0: valid" in why[0]
    far = q.window_stats(make_pings(n=400, sigma=0.05, seed=7), fix_errors_m=[4.0, 5.0, 6.0])
    assert q.compare_verdict(far, ref)[0] == "clearly worse"


def test_report_and_json_roundtrip(tmp_path):
    s = q.window_stats(make_pings(), fix_errors_m=[0.8, 1.0])
    text = q.format_report(s, label="station1_top", ref=s, ref_label="station1_top")
    assert "VERDICT: OK" in text and "COMPARABLE" in text and "box fix - RTK: 0.90 m" in text
    path = os.path.join(str(tmp_path), "ref.json")
    q.save_stats(path, s)
    assert q.load_stats(path) == s
    empty = q.format_report(q.window_stats([]))
    assert "VERDICT: BAD" in empty


if __name__ == "__main__":
    import tempfile
    import pathlib
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            if "tmp_path" in fn.__code__.co_varnames[:fn.__code__.co_argcount]:
                fn(pathlib.Path(tempfile.mkdtemp()))
            else:
                fn()
    print("ok")
