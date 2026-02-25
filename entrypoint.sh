#!/bin/bash
set -e

# 1. Source base ROS environment first (required for colcon and package discovery)
if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi

cd "${ROS_WS:-/ros2_ws}"

# Optional clean build: set CLEAN_BUILD=1 in compose/environment when needed
if [ "${CLEAN_BUILD:-0}" = "1" ]; then
    rm -rf build install log
fi

# 2. Build workspace
colcon build --symlink-install

# 3. Source local workspace for this process tree
if [ -f "${ROS_WS:-/ros2_ws}/install/setup.bash" ]; then
    source "${ROS_WS:-/ros2_ws}/install/setup.bash"
fi

# 4. Ensure interactive shells (e.g. docker exec -it ... bash) are sourced too
if ! grep -qxF 'source ${ROS_WS:-/ros2_ws}/install/setup.bash' /root/.bashrc 2>/dev/null; then
    echo 'source ${ROS_WS:-/ros2_ws}/install/setup.bash' >> /root/.bashrc
fi

# 5. Execute command passed to docker (e.g., sleep infinity)
exec "$@"