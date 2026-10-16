#!/usr/bin/env bash
# Replay a single rosbag with a fresh EKF and record the output.
# Run INSIDE the Docker container: docker exec -it jetson-container bash
#
# Usage: bash scripts/replay_ekf_bag.sh <bag_dir> [rate] [launch_file] [extra_launch_args...]
#   bag_dir            path to bag directory (contains metadata.yaml + *.mcap)
#   rate               playback multiplier, default 2.0
#   launch_file        one of:
#                        offline_ekf_replay.launch.py   (default — global EKF stack)
#                        offline_anchored_replay.launch.py   (gnss_anchored_pose stack)
#   extra_launch_args  additional 'key:=value' args passed to ros2 launch.
#                      e.g. imu_yaw_offset_deg:=-140  yaw_offset_deg:=0
#
# Outputs (per run):
#   <bag_dir>/ekf_replay/<bag_name>_<run_label>/        recorded bag (MCAP)
#   <bag_dir>/ekf_replay/<bag_name>_<run_label>.log     EKF stack log
#   <bag_dir>/ekf_replay/<bag_name>_<run_label>_diag/   diagnostic CSV (diag.csv)
#
# RUN_LABEL is computed as <launch_basename>_rate<rate> (e.g.
# offline_ekf_replay_rate2.0). Override via env: RUN_LABEL=foo bash scripts/...

set -eo pipefail

BAG_DIR="${1:?Usage: $0 <bag_dir> [rate] [launch_file] [extra_launch_args...]}"
RATE="${2:-2.0}"
LAUNCH_FILE="${3:-offline_ekf_replay.launch.py}"
# Everything after position 3 is forwarded verbatim to ros2 launch. Empty
# if not provided. Used for things like imu_yaw_offset_deg:=-140.
EXTRA_LAUNCH_ARGS=("${@:4}")

# Strip trailing slash for consistent basename
BAG_DIR="${BAG_DIR%/}"
BAG_NAME="$(basename "$BAG_DIR")"
LAUNCH_LABEL="${LAUNCH_FILE%.launch.py}"
RUN_LABEL="${RUN_LABEL:-${LAUNCH_LABEL}_rate${RATE}}"
OUT_DIR="${BAG_DIR}/ekf_replay/${BAG_NAME}_${RUN_LABEL}"
DIAG_DIR="${BAG_DIR}/ekf_replay/${BAG_NAME}_${RUN_LABEL}_diag"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "[replay_ekf_bag] Bag:    $BAG_DIR"
echo "[replay_ekf_bag] Out:    $OUT_DIR"
echo "[replay_ekf_bag] Diag:   $DIAG_DIR"
echo "[replay_ekf_bag] Rate:   ${RATE}x"
echo "[replay_ekf_bag] Launch: $LAUNCH_FILE"
if [ ${#EXTRA_LAUNCH_ARGS[@]} -gt 0 ]; then
    echo "[replay_ekf_bag] Extra:  ${EXTRA_LAUNCH_ARGS[*]}"
fi

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
    "/gps/validated",
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
# The list must cover ALL nodes that publish on /odometry/filtered/* or
# /gps/filtered/* — any leftover publisher creates a duplicate stream that
# the recorder + ekf_offline_diagnostic interleave into the new run's data,
# producing inflated bbox / path-length numbers (observed: 30 m vs 14 m
# real bbox on rect_01 because a stale gnss_anchored_pose from a prior
# replay was still publishing alongside the fresh one).
# IMPORTANT: do NOT pkill -f on "offline_ekf_replay.launch" or
# "offline_anchored_replay.launch" -- those substrings appear in THIS
# script's own argv (we were invoked as "bash scripts/replay_ekf_bag.sh
# ... offline_ekf_replay.launch.py ..."), and pkill -f matches the full
# command line, so the script would SIGTERM itself. Killing the node
# executables below is sufficient; the parent "ros2 launch" exits on its
# own when its children are gone.
#
# Self-protection: pgrep -v with $$ filters anything matched against this
# pid out of the kill set. Belt-and-braces in case a future pattern is
# also too loose.
_self_pid=$$
_safe_pkill() {
    local pattern="$1"
    # Find PIDs matching the pattern, excluding self.
    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null | grep -v "^${_self_pid}$" || true)
    if [ -n "$pids" ]; then
        # shellcheck disable=SC2086
        kill $pids 2>/dev/null || true
    fi
}
_safe_pkill "ekf_node"
_safe_pkill "navsat_transform_node"
_safe_pkill "gnss_datum_watchdog"
_safe_pkill "navsat_global_ekf.launch"
_safe_pkill "gnss_anchored_pose"
_safe_pkill "imu_yaw_correction"
_safe_pkill "odometry_validator"
_safe_pkill "ekf_offline_diagnostic"
_safe_pkill "global_ekf_to_navsatfix"
sleep 1

# --- Start recorder -----------------------------------------------------------
# Record everything played from the bag (sensor data) plus the freshly
# generated EKF topics. TOPICS_RAW is already (all_bag_topics - EKF_EXCLUDE),
# so appending the EKF output topics produces no duplicates.
mkdir -p "$(dirname "$OUT_DIR")"
if [ -d "$OUT_DIR" ]; then
    echo "[replay_ekf_bag] Removing previous output: $OUT_DIR"
    rm -rf "$OUT_DIR"
fi
# shellcheck disable=SC2086
# -s mcap: match the storage of the input bags so all downstream analysis
# scripts (which use mcap-ros2-support / rosbags-reading-mcap) work out of
# the box without a sqlite3 conversion step.
ros2 bag record \
    -o "$OUT_DIR" \
    -s mcap \
    $TOPICS_RAW \
    /odometry/filtered/local \
    /odometry/filtered/global \
    /odometry/gps \
    /gps/filtered \
    /gps/filtered/global \
    /gps/validated \
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
mkdir -p "$DIAG_DIR"
EKF_LOG="${BAG_DIR}/ekf_replay/${BAG_NAME}_${RUN_LABEL}.log"
ros2 launch ekf_localization_pkg "$LAUNCH_FILE" \
    gps_fix_topic:="$GPS_TOPIC" \
    diag_output_dir:="$DIAG_DIR" \
    "${EXTRA_LAUNCH_ARGS[@]}" >"$EKF_LOG" 2>&1 &
EKF_PID=$!
echo "[replay_ekf_bag] EKF PID: $EKF_PID — logging to $EKF_LOG"

wait "$BAG_PID"
echo "[replay_ekf_bag] Playback complete — killing EKF stack before /clock dies..."

# --- Teardown -----------------------------------------------------------------
# CRITICAL ORDER: kill publishers (EKF + navsat) IMMEDIATELY after bag ends,
# BEFORE any flush sleep.
#
# When ros2 bag play exits, /clock stops being published. Any node still
# running with use_sim_time=true then falls back to wall-clock for
# rclcpp::Clock::now(). For the EKF that means the next 30 Hz periodic
# predict computes dt = wall_clock_now - last_sim_time ~= 3 days for offline
# replay of a recent bag, integrates velocity over that bogus dt, blows up
# its state to millions of metres, and publishes that garbage out -- which
# the recorder happily captures and the diagnostic summary then reports as
# a [DIAG] >10 km divergence at the very end of every run.
# (Confirmed via debug log: a single predict-step delta of 315544 s
# carrying an odom0_twist measurement with todays wall-clock stamp instead
# of bag sim-time.)
#
# Killing publishers first stops the bad messages at the source. Recorder
# then drains its buffer with the last in-bag (good) messages only.
#
# Default kill (SIGTERM): each Python node installs a SIGTERM handler
# in main() that converts it into a KeyboardInterrupt, so the existing
# try/finally: destroy_node clause runs -- flushing the diag CSV and
# printing the final-summary log lines. SIGINT to ros2 launch works too
# but is much slower (graceful tree-walk through every child); SIGTERM
# with the handler in place gives us fast-and-graceful.
kill "$EKF_PID" 2>/dev/null || true
pkill -f "navsat_global_ekf.launch" 2>/dev/null || true
wait "$EKF_PID" 2>/dev/null || true
sleep 2   # recorder drains in-flight buffered messages
kill "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true

# --- Diagnostic summary -------------------------------------------------------
echo ""
echo "[replay_ekf_bag] === Diagnostic summary ==="
grep -m1 "local EKF odom at datum-set" "$EKF_LOG" 2>/dev/null \
    || echo "  WARNING: no datum-set line found (no GPS fix?)"
echo "  navsat_transform /odometry/gps samples:"
grep "\[GPS odom\]" "$EKF_LOG" 2>/dev/null | head -4 | sed 's/^/    /' \
    || echo "    (none — navsat_transform may not have published)"
echo "  [DIAG-onset] crossings (first time global EKF crosses 100 m / 1 km / 10 km):"
grep "\[DIAG-onset\]" "$EKF_LOG" 2>/dev/null | sed 's/^/    /' \
    || echo "    (none — global EKF stayed within 100 m of map origin)"
echo "  [DIAG-summary] watchdog final counters:"
grep "\[DIAG-summary\]" "$EKF_LOG" 2>/dev/null | tail -1 | sed 's/^/    /' \
    || echo "    (none — watchdog summary did not fire; check that node was running)"
echo "  [DIAG] late-stage warnings (>10 km divergence):"
grep "\[DIAG\] " "$EKF_LOG" 2>/dev/null | tail -3 | sed 's/^/    /' \
    || echo "    (none)"
echo "  [odom_validator] final stamp-jump counts:"
grep "\[odom_validator\] final" "$EKF_LOG" 2>/dev/null | tail -1 | sed 's/^/    /' \
    || echo "    (none — validator did not log final summary)"
echo "  ekf_offline_diagnostic CSV size:"
if [ -f "$DIAG_DIR/diag.csv" ]; then
    wc -l "$DIAG_DIR/diag.csv" | sed 's/^/    /'
else
    echo "    (no diag.csv at $DIAG_DIR)"
fi
echo "[replay_ekf_bag] Full EKF log: $EKF_LOG"
echo "[replay_ekf_bag] Diag CSV:    $DIAG_DIR/diag.csv"
echo ""
echo "[replay_ekf_bag] Done. Output: $OUT_DIR"
