#!/bin/bash
set -euo pipefail

BUILD_START=$(date -u +%Y-%m-%dT%H:%M:%SZ)
docker compose up -d

echo "Waiting for build to complete... (if nothing prints, old container might still be running)"
while IFS= read -r line; do
    echo "$line"
    if echo "$line" | grep -q "\[POLARIS\] BUILD COMPLETE"; then
        break
    fi
done < <(docker logs -f --since "$BUILD_START" jetson-container 2>&1)

echo "Build complete — opening tmux session..."
docker exec -it jetson-container tmux new-session


