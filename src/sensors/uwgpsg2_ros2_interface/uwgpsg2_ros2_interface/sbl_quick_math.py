"""Pure (ROS-free) statistics for the SBL quick check (thesis lake test). Unit-testable off-robot.

The quick check answers on site, without any filter: "is the SBL delivering usable raw ranges right now?"
It is a SANITY check: it rules out a gross failure (locator not pinging, receivers dropping out, frozen
ranges). It does not prove that the ranges are as good as with another mount; a small systematic range
error is invisible here and only shows in post-processing.

Per window (default 30 s) of sbl_raw pings:
    ping rate                 pings / s                        expected about 2 Hz
    per receiver  valid       share of pings with a valid range
                  range       median range [m]
                  scatter     robust ping-to-ping scatter [m]: 1.4826 * MAD(range differences) / sqrt(2)
                              (differences, so slow vehicle motion does not count as noise)
                  frozen      share of valid pings whose range is identical to the previous one
                  rssi / nsd  medians
    pings with >= 3 / 4 valid receivers, share of pings with a valid box position
    box fix - RTK             median horizontal distance [m] between the box's own fix and the vehicle RTK
                              (only at the surface with RTK; includes the antenna-to-locator lever arm)

Verdict (plain words):
    absolute sanity limits  -> "BAD" / "DEGRADED" / "OK" (limits are for gross failures only)
    against a reference     -> "comparable" / "clearly worse" than a saved window (e.g. the top-mount
                               numbers of the morning at the same station); factors are parameters.
"""

import json
import math
import statistics

NO_MEASUREMENT = -1.0          # Waterlinked: receiver_distance == -1.0 means "no measurement"
MAD_TO_SIGMA = 1.4826


def _median(values):
    return statistics.median(values) if values else float("nan")


def robust_scatter(ranges):
    """Robust ping-to-ping scatter [m] of one receiver's valid ranges (in time order).

    sigma = 1.4826 * MAD(first differences) / sqrt(2). Differencing removes slow motion of the vehicle;
    the MAD ignores single outliers. Needs at least 3 ranges, else nan.
    """
    if len(ranges) < 3:
        return float("nan")
    d = [b - a for a, b in zip(ranges[:-1], ranges[1:])]
    med = statistics.median(d)
    mad = statistics.median([abs(x - med) for x in d])
    return MAD_TO_SIGMA * mad / math.sqrt(2.0)


def window_stats(pings, fix_errors_m=None):
    """Statistics of one window.

    pings: list of dicts in time order with keys
        t (float, s), distance (list of float), valid (list of 0/1), rssi (list), nsd (list),
        position_valid (bool).
    fix_errors_m: optional list of horizontal distances box fix - RTK [m] collected in the window.
    Returns a dict (JSON-serialisable).
    """
    n = len(pings)
    out = {"n_pings": n, "duration_s": 0.0, "ping_rate_hz": 0.0, "receivers": [],
           "share_ge3": 0.0, "share_4": 0.0, "share_position_valid": 0.0,
           "fix_minus_rtk_m": None, "fix_minus_rtk_n": 0}
    if n == 0:
        return out
    duration = pings[-1]["t"] - pings[0]["t"]
    out["duration_s"] = duration
    out["ping_rate_hz"] = (n - 1) / duration if duration > 0 and n > 1 else 0.0

    n_rx = max(len(p["distance"]) for p in pings)
    n_valid_per_ping = []
    for p in pings:
        n_valid_per_ping.append(sum(1 for j in range(len(p["distance"])) if _is_valid(p, j)))
    out["share_ge3"] = sum(1 for k in n_valid_per_ping if k >= 3) / n
    out["share_4"] = sum(1 for k in n_valid_per_ping if k >= 4) / n
    out["share_position_valid"] = sum(1 for p in pings if p.get("position_valid")) / n

    for j in range(n_rx):
        ranges, rssi, nsd = [], [], []
        frozen, prev = 0, None
        for p in pings:
            if j < len(p["distance"]) and _is_valid(p, j):
                r = p["distance"][j]
                ranges.append(r)
                if prev is not None and r == prev:
                    frozen += 1
                prev = r
                if j < len(p.get("rssi", [])):
                    rssi.append(p["rssi"][j])
                if j < len(p.get("nsd", [])):
                    nsd.append(p["nsd"][j])
        out["receivers"].append({
            "valid_share": len(ranges) / n,
            "range_m": _nan_to_none(_median(ranges)),
            "scatter_m": _nan_to_none(robust_scatter(ranges)),
            "frozen_share": (frozen / (len(ranges) - 1)) if len(ranges) > 1 else 0.0,
            "rssi": _nan_to_none(_median(rssi)),
            "nsd": _nan_to_none(_median(nsd)),
        })

    if fix_errors_m:
        out["fix_minus_rtk_m"] = _median(fix_errors_m)
        out["fix_minus_rtk_n"] = len(fix_errors_m)
    return out


def _is_valid(p, j):
    if j >= len(p["valid"]) or not p["valid"][j]:
        return False
    return p["distance"][j] != NO_MEASUREMENT


def _nan_to_none(x):
    return None if (x is None or (isinstance(x, float) and math.isnan(x))) else x


# --- verdicts -----------------------------------------------------------------------------------

def sanity_verdict(stats, min_rate_hz=1.0, min_valid_share=0.8, max_frozen_share=0.5):
    """Absolute sanity limits for gross failures. Returns (level, [reasons]); level OK / DEGRADED / BAD.

    BAD:      no pings, ping rate below min_rate_hz (22.10.: one fix per 66 s), no receiver usable,
              or every usable receiver frozen.
    DEGRADED: fewer than 3 receivers with valid_share >= min_valid_share, or a frozen receiver.
    """
    reasons = []
    if stats["n_pings"] == 0:
        return "BAD", ["no sbl_raw pings in this window"]
    if stats["ping_rate_hz"] < min_rate_hz:
        reasons.append(f"ping rate {stats['ping_rate_hz']:.2f} Hz (expected about 2 Hz)")
    good = [r for r in stats["receivers"] if r["valid_share"] >= min_valid_share]
    frozen = [i for i, r in enumerate(stats["receivers"])
              if r["valid_share"] > 0 and r["frozen_share"] > max_frozen_share]
    if not good:
        reasons.append(f"no receiver valid in >= {min_valid_share:.0%} of the pings")
    if reasons or (good and len(frozen) >= len(good)):
        if frozen and len(frozen) >= max(1, len(good)):
            reasons.append("ranges frozen on receivers " + ", ".join(str(i) for i in frozen))
        return "BAD", reasons
    level = "OK"
    if len(good) < 3:
        level = "DEGRADED"
        reasons.append(f"only {len(good)} receiver(s) valid in >= {min_valid_share:.0%} of the pings")
    if frozen:
        level = "DEGRADED"
        reasons.append("ranges frozen on receivers " + ", ".join(str(i) for i in frozen))
    return level, reasons


def compare_verdict(stats, ref, scatter_factor=2.0, valid_drop=0.2, fix_factor=2.0, fix_min_m=1.0):
    """Compare a window with a saved reference window (same station, other mount / earlier in the day).

    "clearly worse" if, for any receiver that was usable in the reference:
        valid share dropped by more than valid_drop, or scatter grew by more than scatter_factor;
    or the box fix - RTK distance grew by more than fix_factor (and is above fix_min_m).
    Returns (word, [reasons]) with word "comparable" or "clearly worse".
    """
    reasons = []
    for i, (r, q) in enumerate(zip(stats["receivers"], ref["receivers"])):
        if q["valid_share"] < 0.5:
            continue                                   # was not usable in the reference either
        if r["valid_share"] < q["valid_share"] - valid_drop:
            reasons.append(f"receiver {i}: valid {r['valid_share']:.0%} (reference {q['valid_share']:.0%})")
        if r["scatter_m"] is not None and q["scatter_m"] is not None and q["scatter_m"] > 0 \
                and r["scatter_m"] > scatter_factor * q["scatter_m"]:
            reasons.append(f"receiver {i}: scatter {r['scatter_m'] * 100:.1f} cm "
                           f"(reference {q['scatter_m'] * 100:.1f} cm)")
    a, b = stats.get("fix_minus_rtk_m"), ref.get("fix_minus_rtk_m")
    if a is not None and b is not None and a > fix_min_m and a > fix_factor * b:
        reasons.append(f"box fix - RTK {a:.1f} m (reference {b:.1f} m)")
    if stats["ping_rate_hz"] < 0.5 * ref["ping_rate_hz"]:
        reasons.append(f"ping rate {stats['ping_rate_hz']:.2f} Hz (reference {ref['ping_rate_hz']:.2f} Hz)")
    return ("clearly worse" if reasons else "comparable"), reasons


# --- text -----------------------------------------------------------------------------------------

def _fmt(x, spec, scale=1.0, dash="   -"):
    return dash if x is None else format(x * scale, spec)


def format_report(stats, label="", ref=None, ref_label=""):
    """Multi-line plain-text report of one window, ending with the verdict line(s)."""
    lines = []
    head = f"SBL quick check{(' [' + label + ']') if label else ''}: {stats['n_pings']} pings in " \
           f"{stats['duration_s']:.0f} s = {stats['ping_rate_hz']:.2f} Hz"
    lines.append(head)
    lines.append("  rx  valid   range    scatter  frozen   rssi    nsd")
    for i, r in enumerate(stats["receivers"]):
        lines.append(f"  {i}   {r['valid_share']:4.0%}  {_fmt(r['range_m'], '6.2f')} m  "
                     f"{_fmt(r['scatter_m'], '5.1f', 100.0)} cm  {r['frozen_share']:4.0%}  "
                     f"{_fmt(r['rssi'], '6.1f')}  {_fmt(r['nsd'], '6.1f')}")
    lines.append(f"  pings with >= 3 receivers: {stats['share_ge3']:.0%}, with 4: {stats['share_4']:.0%}; "
                 f"box position valid: {stats['share_position_valid']:.0%}")
    if stats.get("fix_minus_rtk_m") is not None:
        lines.append(f"  box fix - RTK: {stats['fix_minus_rtk_m']:.2f} m (median of {stats['fix_minus_rtk_n']}, "
                     "includes the antenna-locator lever arm)")
    else:
        lines.append("  box fix - RTK: not available (no RTK fix or no box fix in this window)")
    level, reasons = sanity_verdict(stats)
    detail = ": " + "; ".join(reasons) if reasons else " - ranges arrive, receivers valid, nothing frozen"
    lines.append(f"  VERDICT: {level}" + detail)
    if ref is not None:
        word, why = compare_verdict(stats, ref)
        lines.append(f"  vs reference{(' [' + ref_label + ']') if ref_label else ''}: {word.upper()}"
                     + (": " + "; ".join(why) if why else ""))
    return "\n".join(lines)


def save_stats(path, stats):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(stats, f, indent=1)


def load_stats(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)
