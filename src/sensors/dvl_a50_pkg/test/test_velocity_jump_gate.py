import math

from dvl_a50_pkg.velocity_jump_gate import (
    ACCEPT,
    REACCEPT,
    REJECT_JUMP,
    REJECT_SPEED,
    VelocityJumpGate,
)


def _gate(**kw):
    p = dict(
        max_speed=1.2, max_jump=0.5, max_accel=2.0, max_jump_limit=1.0, reaccept_samples=5
    )
    p.update(kw)
    return VelocityJumpGate(**p)


def test_normal_motion_accepted():
    g = _gate()
    # 0.3 m/s surge with 0.5 m/s amplitude heave at 0.2 Hz, 10 Hz samples.
    for i in range(200):
        t = 0.1 * i
        vz = 0.5 * math.sin(2 * math.pi * 0.2 * t)
        assert g.check((0.3, 0.0, vz), t) == (True, ACCEPT)


def test_single_spike_rejected_then_recovers():
    g = _gate()
    for i in range(10):
        assert g.check((0.2, 0.0, 0.0), 0.1 * i)[0]
    assert g.check((0.9, 0.6, 0.0), 1.0) == (False, REJECT_JUMP)
    assert g.check((0.2, 0.0, 0.0), 1.1) == (True, ACCEPT)


def test_speed_cap():
    g = _gate()
    assert g.check((1.9, 0.5, 0.0), 0.0) == (False, REJECT_SPEED)
    assert g.check((float("nan"), 0.0, 0.0), 0.1) == (False, REJECT_SPEED)
    assert g.check((0.1, 0.0, 0.0), 0.2) == (True, ACCEPT)


def test_persistent_change_reaccepted():
    g = _gate(max_accel=0.0)
    assert g.check((0.0, 0.0, 0.0), 0.0)[0]
    results = [g.check((1.0, 0.0, 0.0), 0.1 * (i + 1)) for i in range(5)]
    assert [r[1] for r in results] == [REJECT_JUMP] * 4 + [REACCEPT]
    assert g.check((1.0, 0.0, 0.0), 0.6) == (True, ACCEPT)


def test_threshold_grows_with_gap():
    g = _gate()
    assert g.check((0.0, 0.0, 0.0), 0.0)[0]
    # 0.9 m/s change: rejected after 0.1 s (thr 0.7), accepted after 1 s (thr 1.0).
    assert not g.check((0.9, 0.0, 0.0), 0.1)[0]
    assert g.check((0.9, 0.0, 0.0), 1.0)[0]


def test_threshold_limited_after_long_gap():
    # June 2026 grid_03: 1.95 m/s 4-beam glitch 4 s after the last 4-beam sample.
    g = _gate(max_speed=0.0)
    assert g.check((0.3, -0.2, 0.0), 100.0)[0]
    assert g.check((-1.6, 0.1, 0.5), 104.0) == (False, REJECT_JUMP)
    assert g.check((0.3, -0.1, 0.0), 104.3) == (True, ACCEPT)


def test_stale_stamps_do_not_count_for_reaccept():
    g = _gate(max_accel=0.0, reaccept_samples=2)
    assert g.check((0.0, 0.0, 0.0), 1.0)[0]
    assert not g.check((1.0, 0.0, 0.0), 1.1)[0]
    # Re-published copy with an older stamp must not complete the re-accept run.
    assert g.check((1.0, 0.0, 0.0), 0.5) == (False, REJECT_JUMP)
    assert g.check((1.0, 0.0, 0.0), 1.2) == (True, REACCEPT)


def test_disabled():
    g = _gate(max_speed=0.0, max_jump=0.0)
    assert g.check((5.0, 0.0, 0.0), 0.0)[0]
    assert g.check((-5.0, 0.0, 0.0), 0.1)[0]
