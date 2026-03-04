#!/bin/bash
set -e

# --- Config defaults ---
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_WS="${ROS_WS:-/ros2_ws}"

# 1) Source base ROS env (needed for colcon/package discovery)
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

cd "${ROS_WS}"

# Ensure submodules are available before build (e.g. ublox_dgnss).
# Bind-mounts can trigger git "dubious ownership", so mark workspace as safe.
if [ -d .git ]; then
  git config --global --add safe.directory "${ROS_WS}" || true
  git submodule sync --recursive
  git submodule update --init --recursive
fi

# Optional clean build: set CLEAN_BUILD=1 in compose/environment when needed
if [ "${CLEAN_BUILD:-0}" = "1" ]; then
  rm -rf build install log
fi

# 2) Build workspace
colcon build --symlink-install

# 3) Source overlay for this process tree
if [ -f "${ROS_WS}/install/setup.bash" ]; then
  source "${ROS_WS}/install/setup.bash"
fi

# 4) Make *future* shells (docker exec -it ... bash) automatically source ROS + overlay
# Works regardless of whether the container runs as root or a non-root user.
BASHRC="${HOME}/.bashrc"
LINE1='source /opt/ros/${ROS_DISTRO:-humble}/setup.bash'
LINE2='source ${ROS_WS:-/ros2_ws}/install/setup.bash'

mkdir -p "$(dirname "$BASHRC")"
touch "$BASHRC"

grep -qxF "$LINE1" "$BASHRC" || echo "$LINE1" >> "$BASHRC"
grep -qxF "$LINE2" "$BASHRC" || echo "$LINE2" >> "$BASHRC"

# Also cover login shells (bash -l / bash -lc)
BASHPROFILE="${HOME}/.bash_profile"
if [ ! -f "$BASHPROFILE" ]; then
  echo 'if [ -f ~/.bashrc ]; then . ~/.bashrc; fi' > "$BASHPROFILE"
fi

# 5) (Optional but robust) System-wide sourcing for any user + any shell that reads /etc/profile
# If you run as non-root, this will fail silently and that's fine.
if [ "$(id -u)" = "0" ]; then
  cat >/etc/profile.d/ros2_ws.sh <<'EOF'
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source ${ROS_WS:-/ros2_ws}/install/setup.bash
EOF
  chmod +x /etc/profile.d/ros2_ws.sh
fi

# 6) Execute the command passed by docker/compose (e.g., sleep infinity)
exec "$@"