#!/usr/bin/env bash
# Replay all 2026-04-23 bags sequentially with a fresh EKF.
# Run INSIDE the Docker container: docker exec -it jetson-container bash
#
# Set BAG_ROOT to the directory that contains the bag folders.

set -eo pipefail

BAG_ROOT="/ros2_ws/recordings/rosbags/2026-04-23"
PATTERN="2026_04_23"
RATE="${1:-3.0}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

shopt -s nullglob
BAGS=( "$BAG_ROOT"/*"$PATTERN"*/ )
shopt -u nullglob

if [ "${#BAGS[@]}" -eq 0 ]; then
    echo "[replay_all_bags] No bags matching *${PATTERN}* found in $BAG_ROOT" >&2
    exit 1
fi

echo "[replay_all_bags] Found ${#BAGS[@]} bags — rate ${RATE}x"
echo ""

for bag_dir in "${BAGS[@]}"; do
    echo "========================================================"
    echo "[replay_all_bags] Processing: $bag_dir"
    echo "========================================================"
    bash "$SCRIPT_DIR/replay_ekf_bag.sh" "$bag_dir" "$RATE"
    echo ""
done

echo "[replay_all_bags] All done."
