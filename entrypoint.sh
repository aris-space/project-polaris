#!/bin/bash
set -e

# 1. Build the workspace if needed (or every time)
# Using --symlink-install is great for python/resource changes
colcon build --symlink-install

# 2. Source the local workspace
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# 3. Execute the command passed to docker (e.g., sleep infinity)
exec "$@"