"""
Plausibility gate for DVL velocity samples (pure Python, no ROS dependency).

Two checks, applied to samples that already passed the lock / beam-count gate:

1. Absolute cap: |v| > max_speed  -> reject. The vehicle cannot move that fast; a
   4-beam solution above it is a glitch (e.g. the 1.95 m/s sample in
   aut_no_z_surface_big_40_grid_03 at t=1629.67 s, reported variance 3.5e-4).

2. Jump check against the last accepted sample (3D norm):
       |v - v_ref| > min(max_jump + max_accel * dt, max_jump_limit)   -> reject
   dt = stamp - stamp_ref (clamped at >= 0, so duplicate / out-of-order stamps get
   the tightest threshold). `max_jump` covers sensor noise at the shortest sample
   interval, `max_accel` covers real vehicle + wave accelerations, so the threshold
   opens up after short gaps. `max_jump_limit` bounds it: a glitch right after a
   long lock gap (e.g. 1.95 m/s after 4 s without a 4-beam sample) must still fail.

   Re-acceptance: if `reaccept_samples` consecutive rejected samples are mutually
   consistent (each within the jump threshold of the previous rejected one), the
   velocity really changed: accept the latest one and make it the new reference.
   This prevents locking out forever if the reference itself was wrong.

A rejected sample is not used as reference. Set max_speed <= 0 / max_jump <= 0 to
disable the respective check.
"""

import math

ACCEPT = "ok"
REJECT_SPEED = "speed_cap"
REJECT_JUMP = "jump"
REACCEPT = "reaccept"


def _norm(a, b=(0.0, 0.0, 0.0)):
    return math.sqrt(sum((x - y) * (x - y) for x, y in zip(a, b)))


class VelocityJumpGate:

    def __init__(self, max_speed, max_jump, max_accel, max_jump_limit, reaccept_samples):
        self.max_speed = float(max_speed)
        self.max_jump = float(max_jump)
        self.max_accel = float(max_accel)
        self.max_jump_limit = float(max_jump_limit)
        self.reaccept_samples = max(1, int(reaccept_samples))
        self.reset()

    def reset(self):
        self._ref_v = None
        self._ref_t = None
        self._cand_v = None
        self._cand_t = None
        self._cand_n = 0

    def _threshold(self, dt):
        thr = self.max_jump + self.max_accel * max(0.0, dt)
        if self.max_jump_limit > 0.0:
            thr = min(thr, max(self.max_jump_limit, self.max_jump))
        return thr

    def check(self, v, t):
        """Return (accepted: bool, reason: str) for velocity v=(vx,vy,vz) at stamp t [s]."""
        v = tuple(float(x) for x in v)
        t = float(t)
        if not all(math.isfinite(x) for x in v):
            return False, REJECT_SPEED
        if self.max_speed > 0.0 and _norm(v) > self.max_speed:
            return False, REJECT_SPEED
        if self.max_jump <= 0.0 or self._ref_v is None:
            self._accept(v, t)
            return True, ACCEPT

        if _norm(v, self._ref_v) <= self._threshold(t - self._ref_t):
            self._accept(v, t)
            return True, ACCEPT

        # Rejected: track a run of mutually consistent rejected samples. Only newer
        # stamps count, so a re-published (stale-stamped) copy cannot speed this up.
        if self._cand_v is not None and t <= self._cand_t:
            return False, REJECT_JUMP
        if self._cand_v is not None and _norm(v, self._cand_v) <= self._threshold(
            t - self._cand_t
        ):
            self._cand_n += 1
        else:
            self._cand_n = 1
        self._cand_v = v
        self._cand_t = t
        if self._cand_n >= self.reaccept_samples:
            self._accept(v, t)
            return True, REACCEPT
        return False, REJECT_JUMP

    def _accept(self, v, t):
        self._ref_v = v
        # Keep the reference stamp monotonic (old driver re-published stale stamps).
        self._ref_t = t if self._ref_t is None else max(t, self._ref_t)
        self._cand_v = None
        self._cand_t = None
        self._cand_n = 0
