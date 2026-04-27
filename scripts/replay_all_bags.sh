#!/usr/bin/env bash
# Replay the 16 "good bags" (reliable IMU+DVL+pressure) from the patched
# 2026-04-23 set with a fresh EKF and record the output.
# Run INSIDE the Docker container: docker exec -it jetson-container bash
#
# Good-bag selection: recordings/rosbags/2026-04-23/ekf_residual_analysis_good_bags/
# Patched source:     recordings/rosbags/2026-04-23_patched/
# See POLARIS/research/EKF_RESEARCH_NOTES.md §3.3 for selection criteria.

set -eo pipefail

BAG_ROOT="/ros2_ws/recordings/rosbags/2026-04-23_patched"
RATE="${1:-2.0}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 16 bags with reliable IMU + DVL + pressure (from ekf_residual_analysis_good_bags/)
GOOD_BAGS=(
    depth_hold_02_2026_04_23-19_53_12
    depth_hold_2026_04_23-19_05_38
    stationary_01_2026_04_23-14_19_03
    stationary_02_2026_04_23-12_57_41
    straight_surge_02_2026_04_23-13_35_18
    straight_surge_03_2026_04_23-13_37_29
    straight_surge_04_2026_04_23-13_40_15
    straight_surge_06_2026_04_23-13_45_45
    straight_surge_07_2026_04_23-13_49_11
    straight_surge_09_2026_04_23-13_52_13
    straight_surge_10_2026_04_23-13_56_09
    vertical_05_2026_04_23-15_21_15
    vertical_06_2026_04_23-15_21_49
    yaw_turns_01_2026_04_23-13_11_12
    yaw_turns_02_2026_04_23-13_13_52
    yaw_turns_03_2026_04_23-13_17_48
)

echo "[replay_all_bags] Source: $BAG_ROOT"
echo "[replay_all_bags] Rate:   ${RATE}x"
echo "[replay_all_bags] Bags:   ${#GOOD_BAGS[@]}"
echo ""

missing=0
for name in "${GOOD_BAGS[@]}"; do
    if [ ! -d "$BAG_ROOT/$name" ]; then
        echo "[replay_all_bags] MISSING: $BAG_ROOT/$name" >&2
        missing=$((missing + 1))
    fi
done
if [ "$missing" -gt 0 ]; then
    echo "[replay_all_bags] ERROR: $missing bag(s) not found in $BAG_ROOT" >&2
    exit 1
fi

for name in "${GOOD_BAGS[@]}"; do
    echo "========================================================"
    echo "[replay_all_bags] Processing: $name"
    echo "========================================================"
    bash "$SCRIPT_DIR/replay_ekf_bag.sh" "$BAG_ROOT/$name" "$RATE"
    echo ""
done

echo "[replay_all_bags] All done."
