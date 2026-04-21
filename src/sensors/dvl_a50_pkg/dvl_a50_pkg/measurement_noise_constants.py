"""
DVL and IMU measurement noise constants.

DVL vx (X): updated from stationary_11_2026_04_19 (St. Moritz lake, 64 min, 100% lock).
  Both stationary_02 and stationary_11 were standalone (not AUV-mounted) at the same lake
  location. stationary_11 had worse wave conditions yet measured 12.6x lower vx variance,
  indicating better mechanical mounting rigidity. Allan OADEV R = N^2 * f = 4.42e-6.
  Source: scripts/stationary_allan_variance.py, stationary11_allan_params.json.

DVL vy, vz: retained from stationary_02_2026_03_26-14_24_06 (stationary_11 vy agrees
  within 1.3x; vz remains below floor regardless).
  Bottom-lock-masked segments, header.stamp, /sensors/dvl/odometry_cov twist.linear.
  See scripts/stationary_tep_stats.py and recordings/stationary_tep_stats.json.
"""

# Twist linear velocity variance [m^2/s^2] when bottom lock is valid (diagonal only).
# odometry_covariance_node uses these directly (with optional z-floor safeguard).
DVL_LOCK_LINEAR_VARIANCE_X_M2_S2 = 4.42e-06
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
