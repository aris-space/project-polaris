# Topside receiver geometry + generic acoustic config of a Waterlinked UGPS G2.
# Source: GET /api/v1/config/receivers  and  GET /api/v1/config/generic
#
# Published once, latched (TRANSIENT_LOCAL), so a late-joining subscriber (or a rosbag replay) still
# receives the fixed receiver layout needed to interpret SblRaw.receiver_distance for range fusion.

std_msgs/Header header          # stamp = time the config was polled

# Receiver positions in the topside acoustic frame [m] (x forward, y right, z down).
# Arrays are parallel; index i is the same receiver i as in SblRaw's per-receiver arrays.
int32[]   receiver_id           # Waterlinked receiver id (as reported by /config/receivers)
float64[] receiver_x
float64[] receiver_y
float64[] receiver_z
uint16    num_receivers         # length of the arrays above

# Generic config
float64 speed_of_sound          # configured sound speed [m/s] from /config/generic, -1.0 if not reported
