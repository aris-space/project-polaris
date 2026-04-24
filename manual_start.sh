#!/bin/bash
set -euo pipefail

BUILD_START=$(date -u +%Y-%m-%dT%H:%M:%SZ)
docker compose up -d

echo "Waiting for build to complete... (if nothing prints check if container already running)"
while IFS= read -r line; do
    echo "$line"
    if echo "$line" | grep -q "\[POLARIS\] BUILD COMPLETE"; then
        break
    fi
done < <(docker logs -f --since "$BUILD_START" jetson-container 2>&1)

echo "Starting tmux session with launch..."
docker exec jetson-container tmux kill-session -t polaris 2>/dev/null || true
docker exec jetson-container tmux new-session -d -s polaris \
    "bash -c 'source /ros2_ws/install/setup.bash && ros2 launch config_pkg start_system.launch.py; exec bash'"

docker exec jetson-container tmux split-window -h -t polaris

docker exec -it jetson-container tmux attach -t polaris
