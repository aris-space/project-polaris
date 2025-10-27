# ROS 2 Jazzy (Ubuntu 24.04). Multi-arch (amd64 + arm64) is available.
FROM ros:jazzy-ros-base

# System deps: pip and mavros_msgs package (sensor_msgs is already in ros-base)
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      ros-jazzy-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONUNBUFFERED=1
WORKDIR /app

# If you also keep a requirements.txt for extra Python libs, copy/install it
# (optional; safe to leave if empty or remove if unused)
COPY src/requirements.txt /app/src/requirements.txt
RUN if [ -s /app/requirements.txt ]; then pip3 install --no-cache-dir -r /app/requirements.txt; fi

# Your node sources
COPY src app/src

# Keep the ROS entrypoint (sets up ROS environment) (for source ...setup.bash)
ENTRYPOINT ["/ros_entrypoint.sh"]

# The ROS images include /ros_entrypoint.sh that sources ROS env for you.
# We'll keep ENTRYPOINT as in the base image and only set a default CMD.
# Default to the joystick node (you can override at docker run).
CMD ["python3", "/app/src/joystick_node.py"]
