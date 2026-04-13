import time
import math
import os

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

# --- connect to Pixhawk ---
master = mavutil.mavlink_connection("/dev/ttyTHS1", baud=57600)
master.wait_heartbeat()
print(f"Heartbeat from system={master.target_system} component={master.target_component}")

# --- helper: yaw-only quaternion (roll = pitch = 0) ---
def yaw_to_quat(yaw_rad):
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    # MAVLink ODOMETRY expects quaternion order: w, x, y, z
    return [cy, 0.0, 0.0, sy]

# 21-element covariance arrays
pose_cov = [float("nan")] * 21
vel_cov = [float("nan")] * 21

# Only fill the important ones for testing
pose_cov[0]  = 0.01   # x error
pose_cov[6]  = 0.01   # y error
pose_cov[11] = 0.01   # z error
pose_cov[15] = 0.05   # roll error [rad]
pose_cov[18] = 0.05   # pitch error [rad]
pose_cov[20] = 0.05   # yaw error [rad]

start = time.time()

while True:
    t = time.time() - start

    # Simulate abrupt yaw changes by hopping between large headings.
    # Each heading is held for a short interval to make message acceptance obvious.
    step_interval_s = 0.3
    yaw_steps_deg = [0, 120, -120, 60, -60, 170, -170, 30, -30]
    step_idx = int(t / step_interval_s) % len(yaw_steps_deg)
    yaw = math.radians(yaw_steps_deg[step_idx])
    q = yaw_to_quat(yaw)

    master.mav.odometry_send(
        int(time.time() * 1e6),                        # time_usec
        mavutil.mavlink.MAV_FRAME_BODY_FRD,           # frame_id
        mavutil.mavlink.MAV_FRAME_BODY_FRD,           # child_frame_id
        0.0, 0.0, 0.0,                                # x, y, z
        q,                                            # quaternion [w, x, y, z]
        0.0, 0.0, 0.0,                                # vx, vy, vz
        0.0, 0.0, 0.0,                                # rollspeed, pitchspeed, yawspeed
        pose_cov,                                     # pose covariance
        vel_cov,                                      # velocity covariance
        0,                                            # reset_counter
        mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,    # estimator_type
        100                                           # quality
    )

    print(f"sent yaw={math.degrees(yaw):6.1f} deg")
    time.sleep(0.1)   # 10 Hz