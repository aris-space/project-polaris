#!/bin/bash
set -euo pipefail

# --- Config defaults ---
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_WS="${ROS_WS:-/ros2_ws}"
AUTO_BUILD="${AUTO_BUILD:-0}"
ROSDEP_INSTALL="${ROSDEP_INSTALL:-0}"

# 1) Source base ROS env (already present in image, but keep explicit here).
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ ! -d "${ROS_WS}" ]; then
  echo "Workspace directory not found: ${ROS_WS}"
  exit 1
fi

cd "${ROS_WS}"

# 2) Optional dependency install for mounted workspaces.
if [ "${ROSDEP_INSTALL}" = "1" ] && command -v rosdep-install-workspace >/dev/null 2>&1; then
  rosdep-install-workspace "${ROS_WS}"
fi

# 3) Optional build step (disabled by default for runtime images).
if [ "${AUTO_BUILD}" = "1" ]; then
  if [ "${CLEAN_BUILD:-0}" = "1" ]; then
    rm -rf build install log
  fi

  if ! command -v colcon >/dev/null 2>&1; then
    echo "AUTO_BUILD=1 but 'colcon' was not found in PATH."
    exit 1
  fi

  colcon build --symlink-install
fi

# 4) Source overlay for this process tree when available.
if [ -f "${ROS_WS}/install/setup.bash" ]; then
  source "${ROS_WS}/install/setup.bash"
fi

# 5) Execute command passed by docker/compose.
exec "$@"