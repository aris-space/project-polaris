#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source the workspace if it exists
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# If the workspace src exists and is not empty, build it
if [ -d /workspace/src ] && [ "$(ls -A /workspace/src)" ]; then
    if [ ! -f /workspace/install/setup.bash ]; then
        echo "Building workspace..."
        colcon build --symlink-install
    fi
fi

# Execute the command passed to the container
exec "$@"
