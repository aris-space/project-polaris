#!/usr/bin/env python3
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
master.wait_heartbeat()
print(f"Heartbeat from sysid {master.target_system}")

# Set ALT_HOLD mode
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    2
)
time.sleep(0.5)

# Check mode was accepted
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
if msg:
    mode = msg.custom_mode
    print(f"Current mode: {mode} (expected 2 for ALT_HOLD)")

# Arm with force flag to bypass pre-arm checks
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,      # arm
    21196,  # force arm magic number — bypasses pre-arm checks
    0, 0, 0, 0, 0
)

# Read ACK
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
if ack:
    print(f"ARM ACK result: {ack.result}")
    # 0 = MAV_RESULT_ACCEPTED
    # 4 = MAV_RESULT_FAILED (pre-arm check failed)
else:
    print("No ACK received — check connection")

# Wait for armed heartbeat
t0 = time.time()
while time.time() - t0 < 5.0:
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
    if msg:
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        mode  = msg.custom_mode
        print(f"  mode={mode} armed={bool(armed)}")
        if armed:
            print("Armed and in ALT_HOLD")
            break
else:
    print("Timed out waiting for armed state")