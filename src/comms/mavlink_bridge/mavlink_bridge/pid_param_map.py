"""
Single source of truth: ROS 2 tuning parameter name -> ArduPilot MAVLink param name.
"""

PID_PARAM_MAP = {
    # --- Attitude rate PIDs (P, I, D, feed-forward) ---
    "tuning/roll_rate_p": "ATC_RAT_RLL_P",
    "tuning/roll_rate_i": "ATC_RAT_RLL_I",
    "tuning/roll_rate_d": "ATC_RAT_RLL_D",
    "tuning/roll_rate_ff": "ATC_RAT_RLL_FF",
    "tuning/pitch_rate_p": "ATC_RAT_PIT_P",
    "tuning/pitch_rate_i": "ATC_RAT_PIT_I",
    "tuning/pitch_rate_d": "ATC_RAT_PIT_D",
    "tuning/pitch_rate_ff": "ATC_RAT_PIT_FF",
    "tuning/yaw_rate_p": "ATC_RAT_YAW_P",
    "tuning/yaw_rate_i": "ATC_RAT_YAW_I",
    "tuning/yaw_rate_d": "ATC_RAT_YAW_D",
    "tuning/yaw_rate_ff": "ATC_RAT_YAW_FF",
    # --- Attitude angle P ---
    "tuning/roll_angle_p": "ATC_ANG_RLL_P",
    "tuning/pitch_angle_p": "ATC_ANG_PIT_P",
    "tuning/yaw_angle_p": "ATC_ANG_YAW_P",
    # --- Depth (Z) position / velocity / acceleration cascade ---
    "tuning/depth_pos_p": "PSC_POSZ_P",
    "tuning/depth_vel_p": "PSC_VELZ_P",
    "tuning/depth_vel_i": "PSC_VELZ_I",
    "tuning/depth_vel_d": "PSC_VELZ_D",
    "tuning/depth_vel_ff": "PSC_VELZ_FF",
    "tuning/depth_accel_z_p": "PSC_ACCZ_P",
    "tuning/depth_accel_z_i": "PSC_ACCZ_I",
    "tuning/depth_accel_z_d": "PSC_ACCZ_D",
    "tuning/pilot_speed_up": "PILOT_SPEED_UP",
    "tuning/pilot_speed_dn": "PILOT_SPEED_DN",
    "tuning/pilot_accel_z": "PILOT_ACCEL_Z",
    # --- XY position / velocity cascade ---
    "tuning/pos_xy_p": "PSC_POSXY_P",
    "tuning/vel_xy_p": "PSC_VELXY_P",
    "tuning/vel_xy_i": "PSC_VELXY_I",
    "tuning/vel_xy_d": "PSC_VELXY_D",
    "tuning/vel_xy_ff": "PSC_VELXY_FF",
}


def normalize_mavlink_param_id(param_id) -> str:
    """PARAM_VALUE.param_id: bytes (null-padded) or str."""
    if param_id is None:
        return ""
    if isinstance(param_id, (bytes, bytearray)):
        return bytes(param_id).split(b"\x00")[0].decode("ascii", errors="replace").strip()
    return str(param_id).strip().rstrip("\x00").strip()
