# Raw per-receiver SBL acoustic measurement from a Waterlinked UGPS G2.
# Source: GET /api/v1/position/acoustic/raw
#
# This is additive telemetry for the tightly-coupled SBL particle filter thesis: it exposes the
# individual topside-receiver slant ranges (and signal quality) BEFORE Waterlinked solves them into
# a position, so a downstream estimator can fuse the raw ranges directly. The existing solved-
# position topics (locator_position_*) are unaffected.

std_msgs/Header header          # stamp = poll wall-clock time (raw endpoint carries no measurement time)

# Per-receiver arrays. Index i corresponds to receiver i in SblReceiverConfig (same UGPS ordering).
float64[] receiver_distance     # slant range topside receiver -> locator [m]
float64[] receiver_nsd          # noise spectral density per receiver (lower = cleaner)
float64[] receiver_rssi         # received signal strength per receiver
uint8[]   receiver_valid        # 1 if that receiver's range is valid, else 0

# Solved acoustic position reported alongside the raw ranges, in the topside acoustic frame.
float64 x                       # forward [m]
float64 y                       # right of forward [m]
float64 z                       # down [m]
float64 std                     # Waterlinked-reported horizontal accuracy [m], -1.0 if unknown
bool    position_valid          # Waterlinked solution-valid flag (false if not reported by raw API)

# Timing
float64 poll_unix_time          # UNIX time when this HTTP poll was issued [s] (same instant as header.stamp)
