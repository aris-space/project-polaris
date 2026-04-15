#!/bin/bash
set -euo pipefail

# This script initializes and repairs git submodules.
# It is intended to be run from the repository root.

# Returns 0 if a basic internet connection is available, 1 otherwise.
has_internet() {
  timeout 3 bash -c 'echo >/dev/tcp/8.8.8.8/53' 2>/dev/null
}

# Ensure we are in a git repository
if [ ! -d .git ] && [ ! -f .git ]; then
  echo "Error: Not in a git repository root."
  exit 1
fi

echo "[setup_submodules] Marking directories as safe for git..."
# Mark the current directory as safe
git config --global --add safe.directory "$(pwd)" || true

# Proactively mark all submodules as safe to avoid "dubious ownership" errors
# We extract paths from .gitmodules if it exists
if [ -f .gitmodules ]; then
    grep path .gitmodules | sed 's/.*= //' | while read -r sub_path; do
        full_path="$(pwd)/$sub_path"
        echo "[setup_submodules] Marking $full_path as safe..."
        git config --global --add safe.directory "$full_path" || true
    done
fi

# Specifically for nested submodules like dvl_a50/include/dvl_a50/json
# These might not be directly in the top-level .gitmodules
if [ -d "src/sensors/dvl_a50/include/dvl_a50/json" ]; then
    git config --global --add safe.directory "$(pwd)/src/sensors/dvl_a50/include/dvl_a50/json" || true
fi

echo "[setup_submodules] Syncing submodules..."
git submodule sync --recursive

echo "[setup_submodules] Updating submodules..."
if has_internet; then
  # Try a standard update first.
  if ! git submodule update --init --recursive; then
    echo "[setup_submodules] Submodule update failed. Attempting to fix by cleaning submodules..."
    # If update fails, we try to clean. foreach might also hit ownership issues,
    # but we've tried to mark them safe above.
    git submodule foreach --recursive 'git clean -ffdx && git reset --hard'
    git submodule update --init --recursive
  fi
else
  echo "[setup_submodules] No internet access — skipping submodule fetch."
  # --no-fetch uses only already-downloaded objects; safe when offline.
  git submodule update --init --recursive --no-fetch 2>/dev/null \
    || echo "[setup_submodules] Warning: Some submodules may not be initialized (offline)."
fi

# Apply sparse-checkout logic for Foxglove Bridge
if [ -d "src/comms/foxglove_bridge" ]; then
    echo "[setup_submodules] Configuring Foxglove Bridge sparse-checkout (ros only)..."
    (
        cd src/comms/foxglove_bridge
        git sparse-checkout init --cone
        git sparse-checkout set ros
        git reset --hard HEAD
    )
fi

echo "[setup_submodules] Submodules are successfully initialized and configured."
