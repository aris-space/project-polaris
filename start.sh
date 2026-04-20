#!/bin/bash
set -euo pipefail

docker compose up -d

echo "Waiting for build and launch to complete..."
until docker exec jetson-container tmux has-session -t polaris 2>/dev/null; do
    docker logs --tail 20 jetson-container 2>&1
    sleep 3
done

docker exec -it jetson-container tmux attach -t polaris
