# ROS 2 Jazzy (Ubuntu 24.04). Multi-arch (amd64 + arm64).
FROM ros:jazzy-ros-base

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      ros-jazzy-mavros-msgs \
      pigpiod \
      python3-pigpio \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONUNBUFFERED=1
WORKDIR /app

# Python deps (you keep them under src/requirements.txt)
# COPY src/requirements.txt /app/src/requirements.txt
# RUN if [ -s /app/src/requirements.txt ]; then \
#      pip3 install --no-cache-dir -r /app/src/requirements.txt; \
#    fi

# App sources
COPY src /app/src

# Keep ROS env
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default: run your launcher (you can override at `docker run ...`)
CMD ["bash", "/app/src/run_all.sh"]
