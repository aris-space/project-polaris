"""Unit tests for the ROS-free SBL heading math (run: pytest, or python this file)."""

import math
import os
import sys

import numpy as np

# Import the ROS-free module directly (no rclpy needed).
sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "uwgpsg2_ros2_interface"),
)
import sbl_heading_math as m  # noqa: E402


def _close(a, b, tol=1e-6):
    return abs(m.norm180(a - b)) <= tol


def test_norm():
    assert m.norm360(370.0) == 10.0
    assert m.norm360(-10.0) == 350.0
    assert m.norm180(190.0) == -170.0
    assert m.norm180(-190.0) == 170.0
    assert m.norm180(180.0) == 180.0


def test_enu_to_compass():
    assert _close(m.enu_yaw_to_compass_deg(0.0), 90.0)            # East -> 90
    assert _close(m.enu_yaw_to_compass_deg(math.pi / 2), 0.0)     # North -> 0
    assert _close(m.enu_yaw_to_compass_deg(-math.pi / 2), 180.0)  # South -> 180
    assert _close(m.enu_yaw_to_compass_deg(math.pi), 270.0)       # West -> 270


def test_yaw_from_quat():
    for a in (0.0, 0.5, 1.2, -2.0, math.pi):
        q = (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))
        assert abs(m.norm180(math.degrees(m.yaw_enu_from_quat(*q)) - math.degrees(a))) < 1e-6


def _line(gamma_deg, n=40, length=4.0, noise=0.0, seed=0):
    """Points (forward,right) along CW-from-forward direction gamma, start at origin."""
    rng = np.random.default_rng(seed)
    v = np.array([math.cos(math.radians(gamma_deg)), math.sin(math.radians(gamma_deg))])
    t = np.linspace(0, length, n)
    pts = np.outer(t, v)
    if noise:
        pts = pts + rng.normal(0, noise, pts.shape)
    return pts


def test_fit_course_axes():
    g, length, ratio = m.fit_course(_line(0.0))     # along forward
    assert _close(g, 0.0, 1e-6) and length > 3.9 and ratio < 1e-6
    g, _, _ = m.fit_course(_line(90.0))             # along right
    assert _close(g, 90.0, 1e-6)


def test_fit_course_disambiguation():
    # points going in -forward direction should read as 180, not 0
    pts = _line(0.0)[::-1] * np.array([1.0, 1.0])  # reverse order -> displacement points -forward
    pts = pts[::-1]  # keep order but flip sign
    pts = -_line(0.0)
    g, _, _ = m.fit_course(pts)
    assert _close(g, 180.0, 1e-6)


def test_fit_course_noisy_straightness():
    g, length, ratio = m.fit_course(_line(33.0, noise=0.02, seed=1))
    assert _close(g, 33.0, 3.0)   # within a few deg under small noise
    assert ratio < 0.15           # still counts as straight


def test_end_to_end_scenarios():
    # (true_course, gamma_body) -> expected corrected topside heading
    cases = [
        (90.0, 90.0, 0.0),      # forward points North, drive East
        (0.0, 270.0, 90.0),     # forward points East, drive North
        (45.0, 15.0, 30.0),     # forward at 30, travel 15 CW of forward
        (10.0, 350.0, 20.0),    # wrap-around
    ]
    for true_c, gamma, expected in cases:
        assert _close(m.corrected_heading(true_c, gamma), expected, 1e-6)


def test_full_pipeline():
    # Build a straight leg in the body frame for a topside whose Forward truly points at 30 deg,
    # while the vehicle travels true course 45 deg -> gamma must be 15, corrected must be 30.
    pts = _line(15.0, noise=0.01, seed=2)
    gamma, _, _ = m.fit_course(pts)
    corrected = m.corrected_heading(45.0, gamma)
    assert _close(corrected, 30.0, 2.0)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"\nAll {len(fns)} tests passed.")
