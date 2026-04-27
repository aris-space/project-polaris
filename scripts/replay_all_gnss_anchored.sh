#!/usr/bin/env bash
# One-shot: build the package, then replay every 23 April global-run GNSS bag
# with the anchored-pose pipeline (local EKF + gnss_anchored_pose, no global EKF).
#
# Run INSIDE the container:  docker exec -it jetson-container bash
#                            bash scripts/replay_all_gnss_anchored.sh [rate]
#
# Output per bag: <bag>/anchored_replay/<bag>_anchored/  (recorded mcap)
# Plus per-bag EKF launch log: <bag>/anchored_replay/ekf_replay.log
#
# After this finishes, on the Windows host run anchored_overlay.py against each
# anchored_replay/<bag>_anchored directory to produce the satellite overlays.

set -eo pipefail

# BAG_ROOT can be overridden via env var, e.g.:
#   BAG_ROOT=/ros2_ws/recordings/rosbags/2026-04-23 bash scripts/replay_all_gnss_anchored.sh
BAG_ROOT="${BAG_ROOT:-/ros2_ws/recordings/rosbags/2026-04-23_patched}"
RATE="${1:-2.0}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

GNSS_BAGS=(
    global_run_gnss_01_2026_04_23-14_00_14
    global_run_gnss_02_2026_04_23-14_02_40
    global_run_gnss_02_2026_04_23-14_03_32
    global_run_gnss_03_2026_04_23-14_03_53
    global_run_gnss_04_2026_04_23-14_06_35
)

echo "[replay_all_gnss_anchored] Source: $BAG_ROOT"
echo "[replay_all_gnss_anchored] Rate:   ${RATE}x"
echo "[replay_all_gnss_anchored] Bags:   ${#GNSS_BAGS[@]}"
echo ""

# --- Sanity-check that all bags exist before starting -------------------------
missing=0
for name in "${GNSS_BAGS[@]}"; do
    if [ ! -d "$BAG_ROOT/$name" ]; then
        echo "[replay_all_gnss_anchored] MISSING: $BAG_ROOT/$name" >&2
        missing=$((missing + 1))
    fi
done
if [ "$missing" -gt 0 ]; then
    echo "[replay_all_gnss_anchored] ERROR: $missing bag(s) not found." >&2
    exit 1
fi

# --- Build ekf_localization_pkg once before the loop --------------------------
echo "[replay_all_gnss_anchored] Building ekf_localization_pkg..."
(
    cd /ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select ekf_localization_pkg \
        2>&1 | tail -10
)
echo ""

# --- Replay each bag ----------------------------------------------------------
for name in "${GNSS_BAGS[@]}"; do
    echo "========================================================"
    echo "[replay_all_gnss_anchored] Processing: $name"
    echo "========================================================"
    bash "$SCRIPT_DIR/replay_anchored_bag.sh" "$BAG_ROOT/$name" "$RATE"
    echo ""
done

echo "[replay_all_gnss_anchored] All ${#GNSS_BAGS[@]} bags processed."
echo ""
echo "Next: on the Windows host, run the overlay for each:"
for name in "${GNSS_BAGS[@]}"; do
    echo "  python scripts\\anchored_overlay.py recordings\\rosbags\\2026-04-23_patched\\$name\\anchored_replay\\${name}_anchored"
done
