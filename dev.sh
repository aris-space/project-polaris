#!/bin/bash
set -euo pipefail

docker compose up -d

echo "Starting dev tmux session..."
tmux new-session -d -s polaris-dev \
    "docker logs -f jetson-container"

tmux split-window -h -t polaris-dev \
    "bash -c 'echo Waiting for build...; until docker exec jetson-container test -f /ros2_ws/.build_complete 2>/dev/null; do sleep 3; done; echo Build complete — opening shell; docker exec -it jetson-container bash'"

tmux attach -t polaris-dev
