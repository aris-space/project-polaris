###############################################################################
# Stage 1: Full development (GUI + simulation + desktop tools)
###############################################################################
FROM ros:humble-desktop-full AS full-dev
#FROM osrf/ros:humble-desktop-full AS full-dev


SHELL ["/bin/bash","-c"]

# Install dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    build-essential \
    git \
    python3-pip \
    vim nano less \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

WORKDIR /workspace

# Copy entrypoint script
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

###############################################################################
# Stage 2: Lightweight development (build + code editing)
###############################################################################
FROM ros:humble-ros-base AS dev
SHELL ["/bin/bash","-c"]

WORKDIR /workspace

# Install minimal build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    build-essential \
    git \
    python3-pip \
    vim nano less \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Copy entrypoint script
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

###############################################################################
# Stage 3: Runtime / Jetson deploy
###############################################################################
FROM ros:humble-ros-core AS runtime
SHELL ["/bin/bash","-c"]

WORKDIR /workspace

# Copy entrypoint script
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# NVIDIA runtime vars (for Jetson / GPU)
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Use non-root user for security
RUN useradd -ms /bin/bash rosuser && \
    mkdir -p /workspace/src && \
    chown -R rosuser:rosuser /workspace
USER rosuser

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "submarine_bringup", "mission.launch.py"]
