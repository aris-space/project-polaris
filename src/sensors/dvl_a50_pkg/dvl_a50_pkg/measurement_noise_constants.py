"""
Constants derived from pool stationary analysis (TEP Section 6).

Source bag: stationary_02_2026_03_26-14_24_06, bottom-lock-masked segments,
header.stamp timeline, /sensors/dvl/odometry_cov twist.linear — sample variance
around the segment mean (see scripts/stationary_tep_stats.py and
recordings/stationary_tep_stats.json).

Regenerate JSON after new pool data and update these literals if needed.
"""

# Twist linear velocity variance [m^2/s^2] when bottom lock is valid (diagonal only).
# odometry_covariance_node multiplies these by lock_linear_variance_bias_drift_inflation_factor
# (ROS param, default 1.15) for runtime tuning without changing the recorded stationary stats.
DVL_LOCK_LINEAR_VARIANCE_X_M2_S2 = 5.581725289274474e-05
DVL_LOCK_LINEAR_VARIANCE_Y_M2_S2 = 7.83722133250522e-06
DVL_LOCK_LINEAR_VARIANCE_Z_RAW_M2_S2 = 9.434414856439562e-08

# Raw vz variance is extremely small; EKF can become ill-conditioned if trusted blindly.
DEFAULT_LOCK_LINEAR_VARIANCE_Z_FLOOR_M2_S2 = 1.0e-06

# IMU (same bag, lock-masked): sample variance of angular velocity [ (rad/s)^2 ].
# Local EKF fuses gyro (imu0_config); values published as stddev on /imu/data.
IMU_GYRO_VARIANCE_RAD2_S2_X = 2.072151772377615e-06
IMU_GYRO_VARIANCE_RAD2_S2_Y = 2.1996416310635546e-06
IMU_GYRO_VARIANCE_RAD2_S2_Z = 2.1043841520647106e-06

# Not fused in ekf_local.yaml imu0_config; kept for REP-145 completeness on /imu/data.
IMU_ACCEL_VARIANCE_M2_S4_X = 0.0002065498273821981
IMU_ACCEL_VARIANCE_M2_S4_Y = 2.4543638199825566e-05
IMU_ACCEL_VARIANCE_M2_S4_Z = 5.0828777411559534e-05
