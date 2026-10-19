"""Pure (ROS-free) water/acoustic helpers for the SBL API tools.

Kept free of rclpy / ROS message imports so it can be unit-tested off-robot.
"""

# Marczak (2007) speed of sound in PURE water at 1 atm, valid ~0-95 deg C [m/s].
# c(T) = a0 + a1 T + a2 T^2 + a3 T^3 + a4 T^4 + a5 T^5
_MARCZAK = (
    1.402385e3,
    5.038813e0,
    -5.799136e-2,
    3.287156e-4,
    -1.398845e-6,
    2.787860e-9,
)

DEFAULT_SOUND_SPEED_MPS = 1486.0  # used when no temperature is supplied


def sound_speed_freshwater(temp_c):
    """Speed of sound in fresh (pure) water at temperature `temp_c` [deg C] -> m/s.

    Freshwater / near-surface assumption (Lake Zurich). Salinity and pressure terms are omitted.
    """
    t = float(temp_c)
    c = 0.0
    for i, a in enumerate(_MARCZAK):
        c += a * (t ** i)
    return c


def depth_from_gauge_pa(gauge_pa, water_density_kg_m3=1000.0, gravity=9.80665):
    """Convert gauge pressure [Pa] to depth below surface [m, positive down]."""
    return float(gauge_pa) / (float(water_density_kg_m3) * float(gravity))
