#!/usr/bin/env bash
# Replay a single rosbag with a fresh EKF and record the output.
# Run INSIDE the Docker container: docker exec -it jetson-container bash
#
# Usage: bash scripts/replay_ekf_bag.sh <bag_dir> [rate]
#   bag_dir  path to bag directory (contains metadata.yaml + *.mcap)
#   rate     playback multiplier, default 3.0

set -eo pipefail

BAG_DIR="${1:?Usage: $0 <bag_dir> [rate]}"
RATE="${2:-3.0}"

# Strip trailing slash for consistent basename
BAG_DIR="${BAG_DIR%/}"
BAG_NAME="$(basename "$BAG_DIR")"
OUT_DIR="${BAG_DIR}/ekf_replay/${BAG_NAME}_ekf"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "[replay_ekf_bag] Bag:  $BAG_DIR"
echo "[replay_ekf_bag] Out:  $OUT_DIR"
echo "[replay_ekf_bag] Rate: ${RATE}x"

# --- Build topic whitelist (exclude stale EKF outputs from the bag) -----------
# Write the helper to a temp file first to avoid bash parser issues that arise
# when a heredoc is nested inside $() on some bash versions.
_TOPICS_PY=$(mktemp /tmp/ekf_topics_XXXX.py)
cat > "$_TOPICS_PY" <<'PYEOF'
import sys
from pathlib import Path

EKF_EXCLUDE = {
    "/odometry/filtered/local",
    "/odometry/filtered/global",
    "/odometry/gps",
    "/gps/filtered",
    "/gps/filtered/global",
    "/filter/euler",
    "/filter/free_acceleration",
    "/filter/quaternion",
    "/tf",
}

SYSTEM_EXCLUDE = {
    "/rosout",
    "/parameter_events",
    "/events/write_split",
    "/clock",
}

bag_dir = Path(sys.argv[1])

try:
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    storage_opts = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="")
    conv_opts = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_opts, conv_opts)
    topics = {t.name for t in reader.get_all_topics_and_types()}
    del reader
except Exception:
    import yaml
    meta = yaml.safe_load((bag_dir / "metadata.yaml").read_text())
    topics = {t["topic_metadata"]["name"]
              for t in meta["rosbag2_bagfile_information"]["topics_with_message_count"]}

whitelist = sorted(topics - EKF_EXCLUDE - SYSTEM_EXCLUDE)
print(" ".join(whitelist))
PYEOF
TOPICS_RAW=$(python3 "$_TOPICS_PY" "$BAG_DIR")
rm -f "$_TOPICS_PY"

if [ -z "$TOPICS_RAW" ]; then
    echo "[replay_ekf_bag] ERROR: could not read topics from bag" >&2
    exit 1
fi

# --- Detect GPS topic ---------------------------------------------------------
GPS_TOPIC="/fix"
case " $TOPICS_RAW " in
    *" /gps/selected "*) GPS_TOPIC="/gps/selected" ;;
esac
echo "[replay_ekf_bag] GPS topic: $GPS_TOPIC"

# h_acc gate is disabled for offline replay: the gate guards against null-island
# startup in live deployment, but offline bags were recorded with real GPS data.
# GPS status ≥ 0 + coordinate check (|lat|>0.1°) is sufficient here.
H_ACC_TOPIC=""
echo "[replay_ekf_bag] h_acc gate: disabled (offline replay)"

# --- Pre-run cleanup ----------------------------------------------------------
# Kill any EKF/navsat processes lingering from a previous bag run.
# A diverged /odometry/filtered/local from a prior run stays on the DDS bus and
# corrupts navsat_transform's datum anchor the moment the next run starts.
pkill -f "ekf_node" 2>/dev/null || true
pkill -f "navsat_transform_node" 2>/dev/null || true
pkill -f "gnss_datum_watchdog" 2>/dev/null || true
pkill -f "navsat_global_ekf.launch" 2>/dev/null || true
sleep 1

# --- Start recorder -----------------------------------------------------------
# Record everything played from the bag (sensor data) plus the freshly
# generated EKF topics. TOPICS_RAW is already (all_bag_topics - EKF_EXCLUDE),
# so appending the EKF output topics produces no duplicates.
mkdir -p "$(dirname "$OUT_DIR")"
# shellcheck disable=SC2086
ros2 bag record \
    -o "$OUT_DIR" \
    $TOPICS_RAW \
    /odometry/filtered/local \
    /odometry/filtered/global \
    /odometry/gps \
    /gps/filtered \
    /gps/filtered/global \
    /filter/euler \
    /filter/free_acceleration \
    /filter/quaternion \
    /tf &
REC_PID=$!
echo "[replay_ekf_bag] Recorder PID: $REC_PID"

# --- Play bag (background) ----------------------------------------------------
# /clock must be publishing at the bag timestamp BEFORE the EKF starts.
# If EKF starts first with use_sim_time=true and no /clock, it initialises its
# filter at t=0; the first sensor message then produces dt ≈ unix_epoch seconds
# (≈1.77e9 s), causing DVL noise to integrate to ~20,000 km of spurious position.
echo "[replay_ekf_bag] Playing bag at ${RATE}x..."
# shellcheck disable=SC2086
ros2 bag play "$BAG_DIR" \
    --clock \
    --rate "$RATE" \
    --read-ahead-queue-size 5000 \
    --topics $TOPICS_RAW &
BAG_PID=$!
echo "[replay_ekf_bag] Bag PID: $BAG_PID — waiting for /clock to be live..."
# ros2 bag play takes ~2 s to initialise before it publishes anything.
# Poll using 'ros2 topic list' (DDS discovery only — fast, no message receive)
# until /clock appears, then add a grace period for sensor buffers to fill.
_t=0
until ros2 topic list 2>/dev/null | grep -qx '/clock'; do
    if [ $_t -ge 30 ]; then
        echo "[replay_ekf_bag] ERROR: /clock not found after 30 s; aborting" >&2
        kill "$BAG_PID" "$REC_PID" 2>/dev/null || true
        exit 1
    fi
    sleep 1; _t=$((_t + 1))
done
echo "[replay_ekf_bag] /clock is live (${_t}s) — grace period 3s..."
sleep 3

# --- Launch EKF ---------------------------------------------------------------
# /clock is now at the bag timestamp; EKF initialises with a realistic dt.
ros2 launch ekf_localization_pkg offline_ekf_replay.launch.py \
    gps_fix_topic:="$GPS_TOPIC" &
EKF_PID=$!
echo "[replay_ekf_bag] EKF PID: $EKF_PID"

wait "$BAG_PID"
echo "[replay_ekf_bag] Playback complete — flushing 2s..."
sleep 2

# --- Teardown -----------------------------------------------------------------
# Kill publishers first so no new messages arrive after the recorder closes.
kill "$EKF_PID" 2>/dev/null || true
pkill -f "navsat_global_ekf.launch" 2>/dev/null || true
wait "$EKF_PID" 2>/dev/null || true
sleep 1   # let the last in-flight messages reach the recorder
kill "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true

echo "[replay_ekf_bag] Done. Output: $OUT_DIR"
