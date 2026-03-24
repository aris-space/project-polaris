#!/bin/bash
set -euo pipefail

# This script initializes and repairs git submodules.
# It is intended to be run from the repository root.

# Ensure we are in a git repository
if [ ! -d .git ] && [ ! -f .git ]; then
  echo "Error: Not in a git repository root."
  exit 1
fi

# Bind-mounted workspaces often have host ownership that differs from the
# container user. Mark workspace and key submodules as safe for git.
git config --global --add safe.directory "$(pwd)" || true

echo "[setup_submodules] Syncing submodules..."
git submodule sync --recursive

echo "[setup_submodules] Updating submodules..."
# Try a standard update first. If it fails, we'll try a more aggressive approach.
if ! git submodule update --init --recursive; then
  echo "[setup_submodules] Submodule update failed. Attempting to fix by cleaning submodules..."
  # Clean and reset submodules to a known good state
  git submodule foreach --recursive 'git clean -ffdx && git reset --hard'
  git submodule update --init --recursive
fi

# Specific repair logic for dvl_a50 and its nested json submodule
if [ -d "src/sensors/dvl_a50" ]; then
    git config --global --add safe.directory "$(pwd)/src/sensors/dvl_a50" || true
    git config --global --add safe.directory "$(pwd)/src/sensors/dvl_a50/include/dvl_a50/json" || true
    
    if ! git submodule update --init --recursive -- "src/sensors/dvl_a50"; then
        echo "[setup_submodules] Repairing nested dvl_a50 json submodule checkout..."
        rm -rf "src/sensors/dvl_a50/include/dvl_a50/json"
        git submodule update --init --recursive --force -- "src/sensors/dvl_a50"
    fi
fi

# Apply sparse-checkout logic for Foxglove Bridge to minimize disk usage/build time
if [ -d "src/comms/foxglove_bridge" ]; then
    echo "[setup_submodules] Configuring Foxglove Bridge sparse-checkout (ros only)..."
    # We use a subshell to avoid changing the main script's working directory permanently
    (
        cd src/comms/foxglove_bridge
        git sparse-checkout init --cone
        git sparse-checkout set ros
        git reset --hard HEAD
    )
fi

echo "[setup_submodules] Submodules are successfully initialized and configured."