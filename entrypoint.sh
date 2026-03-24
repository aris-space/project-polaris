#!/bin/bash
set -euo pipefail

# ROS setup scripts may reference optional vars that are unset.
# Temporarily disable nounset while sourcing them.
source_with_relaxed_nounset() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

# --- Config defaults ---
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_WS="${ROS_WS:-/ros2_ws}"
AUTO_BUILD="${AUTO_BUILD:-1}"
ROSDEP_INSTALL="${ROSDEP_INSTALL:0}"
REFRESH_PY_PACKAGES="${REFRESH_PY_PACKAGES:-1}"
ROSDEP_SKIP_KEYS="${ROSDEP_SKIP_KEYS:-pymavlink dvl_a50 python3-jetson-gpio}"

# 1) Source base ROS env (already present in image, but keep explicit here).
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source_with_relaxed_nounset "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ ! -d "${ROS_WS}" ]; then
  echo "Workspace directory not found: ${ROS_WS}"
  exit 1
fi

cd "${ROS_WS}"

# 1.5) Initialize and repair submodules if git is available.
if command -v git >/dev/null 2>&1 && [ -f "${ROS_WS}/setup_submodules.sh" ]; then
  # Ensure the script is executable.
  chmod +x "${ROS_WS}/setup_submodules.sh"
  # Run the standalone submodule setup script.
  "${ROS_WS}/setup_submodules.sh"
fi

# 2) Optional dependency install for mounted workspaces.
if [ "${ROSDEP_INSTALL}" = "1" ]; then
  if command -v rosdep >/dev/null 2>&1; then
    echo "[entrypoint] Updating package lists..."
    apt-get update
    
    echo "[entrypoint] Updating rosdep..."
    rosdep update || true
    
    if command -v rosdep-install-workspace >/dev/null 2>&1; then
      rosdep-install-workspace "${ROS_WS}"
    else
      echo "[entrypoint] Installing dependencies with rosdep..."
      rosdep install --from-paths src --ignore-src -r -y --skip-keys "${ROSDEP_SKIP_KEYS}"
    fi
  else
    echo "ROSDEP_INSTALL=1 but neither 'rosdep-install-workspace' nor 'rosdep' was found."
    exit 1
  fi
fi

# 3) Optional build step (disabled by default for runtime images).
if [ "${AUTO_BUILD}" = "1" ]; then
  if [ "${CLEAN_BUILD:-0}" = "1" ]; then
    rm -rf build install log
  elif [ "${REFRESH_PY_PACKAGES}" = "1" ]; then
    # Generic safeguard for Python package entry-point changes:
    # rebuild Python packages from a clean package-local state.
    # This avoids stale install artifacts when setup.py console_scripts change.
    while IFS= read -r -d '' setup_py; do
      pkg_dir="$(dirname "${setup_py}")"
      pkg_name="$(basename "${pkg_dir}")"
      rm -rf "build/${pkg_name}" "install/${pkg_name}"
    done < <(find "${ROS_WS}/src" -name setup.py -print0)
  fi

  if ! command -v colcon >/dev/null 2>&1; then
    echo "AUTO_BUILD=1 but 'colcon' was not found in PATH."
    exit 1
  fi

  # Old layouts used src/sensors/dvl-a50 (hyphen). Submodule path is dvl_a50 (underscore).
  # Stale CMakeCache keeps the old path and breaks colcon until build/ is removed.
  if [ -f "${ROS_WS}/build/dvl_a50/CMakeCache.txt" ] \
    && grep -q 'dvl-a50' "${ROS_WS}/build/dvl_a50/CMakeCache.txt" 2>/dev/null; then
    echo "[entrypoint] Clearing dvl_a50 build/install (CMake cache referenced removed dvl-a50 path)."
    rm -rf "${ROS_WS}/build/dvl_a50" "${ROS_WS}/install/dvl_a50"
  fi

  colcon build --symlink-install
fi

# 4) Source overlay for this process tree when available.
if [ -f "${ROS_WS}/install/setup.bash" ]; then
  source_with_relaxed_nounset "${ROS_WS}/install/setup.bash"
fi

# 5) Execute command passed by docker/compose.
exec "$@"