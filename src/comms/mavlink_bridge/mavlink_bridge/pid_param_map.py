"""
Single source of truth: ROS 2 tuning parameter name -> ArduPilot MAVLink param name.
"""

PID_PARAM_MAP = {
    "tuning/roll_rate_p": "ATC_RAT_RLL_P",
    "tuning/roll_rate_i": "ATC_RAT_RLL_I",
    "tuning/roll_rate_d": "ATC_RAT_RLL_D",
    "tuning/pitch_rate_p": "ATC_RAT_PIT_P",
    "tuning/pitch_rate_i": "ATC_RAT_PIT_I",
    "tuning/pitch_rate_d": "ATC_RAT_PIT_D",
    "tuning/yaw_rate_p": "ATC_RAT_YAW_P",
    "tuning/yaw_rate_i": "ATC_RAT_YAW_I",
    "tuning/yaw_rate_d": "ATC_RAT_YAW_D",
    "tuning/roll_angle_p": "ATC_ANG_RLL_P",
    "tuning/pitch_angle_p": "ATC_ANG_PIT_P",
    "tuning/yaw_angle_p": "ATC_ANG_YAW_P",
    "tuning/depth_pos_p": "PSC_POSZ_P",
    "tuning/depth_vel_p": "PSC_VELZ_P",
    "tuning/depth_vel_i": "PSC_VELZ_I",
    "tuning/depth_vel_d": "PSC_VELZ_D",
    "tuning/pos_xy_p": "PSC_POSXY_P",
    "tuning/vel_xy_p": "PSC_VELXY_P",
    "tuning/vel_xy_i": "PSC_VELXY_I",
}


def normalize_mavlink_param_id(param_id) -> str:
    """PARAM_VALUE.param_id: bytes (null-padded) or str."""
    if param_id is None:
        return ""
    if isinstance(param_id, (bytes, bytearray)):
        return bytes(param_id).split(b"\x00")[0].decode("ascii", errors="replace").strip()
    return str(param_id).strip().rstrip("\x00").strip()
