#!/bin/bash
set -euo pipefail

docker compose up -d

echo "Waiting for build to complete..."
until docker exec jetson-container test -f /ros2_ws/.build_complete 2>/dev/null; do
    docker logs --tail 5 jetson-container 2>&1
    sleep 3
done

echo "Starting tmux session with launch..."
docker exec jetson-container tmux new-session -d -s polaris \
    "bash -c 'source /ros2_ws/install/setup.bash && ros2 launch config_pkg start_system.launch.py; exec bash'"

docker exec jetson-container tmux split-window -h -t polaris

docker exec -it jetson-container tmux attach -t polaris
