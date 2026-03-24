#!/bin/bash
# 1. Initialize the submodule entry
git submodule update --init --recursive
echo "Initialized submodules."

# 2. Apply the sparse-checkout logic
cd src/comms/foxglove_bridge
git sparse-checkout init --cone
git sparse-checkout set ros
git reset --hard HEAD
echo "Foxglove Bridge submodule is ready at src/comms/foxglove_bridge/ros"

