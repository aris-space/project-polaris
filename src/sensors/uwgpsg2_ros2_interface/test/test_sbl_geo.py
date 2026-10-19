"""Unit tests for the ROS-free geo helpers (run: pytest, or python this file)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "uwgpsg2_ros2_interface"))
import sbl_geo as g  # noqa: E402


def test_haversine_zero():
    assert g.haversine_m(47.32, 8.55, 47.32, 8.55) == 0.0


def test_haversine_lat_delta():
    # 1e-5 deg latitude ~= 1.111 m anywhere.
    d = g.haversine_m(47.32, 8.55, 47.32 + 1e-5, 8.55)
    assert abs(d - 1.111) < 0.02


def test_haversine_known_baseline():
    # ~ 1 deg longitude at 47.32N ~= 111320*cos(47.32) ~= 75.4 km
    d = g.haversine_m(47.32, 8.55, 47.32, 9.55)
    assert abs(d - 75400.0) < 500.0


def test_mean_latlon():
    pts = [(47.3200, 8.5500), (47.3202, 8.5502), (47.3201, 8.5501)]
    lat, lon = g.mean_latlon(pts)
    assert abs(lat - 47.3201) < 1e-6 and abs(lon - 8.5501) < 1e-6


def test_max_spread():
    center = (47.3201, 8.5501)
    pts = [(47.3201, 8.5501), (47.3201 + 1e-5, 8.5501)]
    assert abs(g.max_spread_m(pts, center) - 1.111) < 0.02


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"\nAll {len(fns)} tests passed.")
