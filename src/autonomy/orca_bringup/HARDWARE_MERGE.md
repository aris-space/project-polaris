# Hardware Merge — Critical Details


---


### Where it appears and what to do

| File | Location | Status |
|---|---|---|
| `comms/mavlink_bridge/launch/mavlink_bridge.launch.py` | line 35, default `'true'` | Passed `False` by `hardware_launch.py` |
---

## 2. MAVLink connection URLs

The MAVLink bridge defaults to SITL on localhost.  Override with
environment variables **before** launching:

```bash
# Example: Pixhawk reachable via UDP (BlueROV2 companion computer default)
export MAVLINK_PUBLISHER_URL="udp:192.168.2.2:14550"   # read telemetry
export MAVLINK_RECEIVER_URL="udp:192.168.2.2:14551"    # send commands

# Or serial (direct USB/UART):
export MAVLINK_PUBLISHER_URL="serial:/dev/ttyUSB0:115200"
export MAVLINK_RECEIVER_URL="serial:/dev/ttyUSB0:115200"

```

Source files: `mavlink_publisher.py` line 80, `ros2_receiver.py` line 72.
---

## Quick checklist before first hardware run

- [ ] `MAVLINK_PUBLISHER_URL` and `MAVLINK_RECEIVER_URL` set in the shell
- [ ] `/pixhawk/heartbeat` topic is publishing before starting navigation
- [ ] `map → odom → base_link` TF chain visible in `ros2 run tf2_tools view_frames`
