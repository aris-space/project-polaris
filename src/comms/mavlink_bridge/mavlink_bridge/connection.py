"""MAVLink connection helper shared by ros2_receiver and mavlink_publisher.

On the vehicle nothing here is configured: the URL falls back to the caller's
default and a single connection attempt is made, exactly as before.

Simulation (SITL) overrides via environment variables:
  <url_env>                  e.g. MAVLINK_RECEIVER_URL=tcp:127.0.0.1:5762
  MAVLINK_CONNECT_RETRIES    attempts before giving up (default 1 = no retry)
  MAVLINK_CONNECT_DELAY_SEC  pause between attempts (default 1.0)
Retries exist because ArduSub SITL may start after the bridge.
"""
import os
import time

from pymavlink import mavutil


def connect(url_env, default_url):
    url = os.getenv(url_env, default_url)
    retries = max(1, int(os.getenv("MAVLINK_CONNECT_RETRIES", "1")))
    delay = float(os.getenv("MAVLINK_CONNECT_DELAY_SEC", "1.0"))
    for attempt in range(1, retries + 1):
        try:
            return mavutil.mavlink_connection(url)
        except OSError:
            if attempt == retries:
                raise
            time.sleep(delay)
