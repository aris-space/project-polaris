# ROS 2 Jazzy (Ubuntu 24.04) — multi-arch
FROM ros:jazzy-ros-base

# Use bash for RUN so we can `source` easily
SHELL ["/bin/bash", "-lc"]

ENV PYTHONUNBUFFERED=1

# System deps you need at runtime/build
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      ros-jazzy-joy \
      ros-jazzy-mavros-msgs \
      python3-pigpio \
    && rm -rf /var/lib/apt/lists/*

# Create workspace (standard ROS layout, host-agnostic)
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Build context is the OUTER prototype_gelb_package/
# Copy everything (code + launch) into src as a package dir
COPY . /ros2_ws/src/prototype_gelb_package/

# Resolve ROS deps from package.xml
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build overlay (sequential is Pi-friendly)
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --executor sequential

# Auto-source underlay + overlay for interactive shells
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# ros:jazzy images already provide this entrypoint, but keep explicit
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command (can override with `docker run ... -- ros2 ...`)
CMD ["bash", "-lc", "source /ros2_ws/install/setup.bash && ros2 launch prototype_gelb_package gelb_launch.py"]

