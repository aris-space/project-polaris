"""Unit tests for the ROS-free water helpers (run: pytest, or python this file)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "uwgpsg2_ros2_interface"))
import sbl_water as w  # noqa: E402


def test_sound_speed_freshwater_table():
    # Marczak (2007) pure-water reference values [m/s], tolerance 0.5.
    for temp, ref in [(0.0, 1402.4), (10.0, 1447.3), (20.0, 1482.3), (25.0, 1496.7), (30.0, 1509.1)]:
        c = w.sound_speed_freshwater(temp)
        assert abs(c - ref) <= 0.5, f"c({temp})={c:.2f} vs {ref}"


def test_sound_speed_monotonic():
    temps = range(0, 41, 2)
    cs = [w.sound_speed_freshwater(t) for t in temps]
    assert all(b > a for a, b in zip(cs, cs[1:])), "speed of sound must increase with T over 0-40 C"


def test_default_constant():
    assert w.DEFAULT_SOUND_SPEED_MPS == 1486.0


def test_depth_from_gauge_pa():
    # 1 bar gauge (1e5 Pa) ~= 10.194 m in fresh water
    assert abs(w.depth_from_gauge_pa(1.0e5, 1000.0, 9.80665) - 10.1972) < 1e-3
    # 1 m of fresh water ~= 9806.65 Pa
    assert abs(w.depth_from_gauge_pa(9806.65, 1000.0, 9.80665) - 1.0) < 1e-6
    assert w.depth_from_gauge_pa(0.0) == 0.0


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"\nAll {len(fns)} tests passed.")
