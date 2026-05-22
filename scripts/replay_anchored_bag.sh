#!/usr/bin/env bash
# Replay a single rosbag with local EKF + gnss_anchored_pose (no global EKF).
# Output bag contains the bag's sensor topics + the new /odometry/filtered/global
# from gnss_anchored_pose. Use scripts/anchored_overlay.py to evaluate.
#
# Run INSIDE the Docker container: docker exec -it jetson-container bash
#
# Usage:
#   bash scripts/replay_anchored_bag.sh <bag_dir> [rate] \
#        [--params-file PATH] [--out-label LABEL]
#
#   bag_dir         path to bag directory (contains metadata.yaml + *.mcap)
#   rate            playback multiplier, default 2.0
#   --params-file   override the EKF params yaml (default: package default
#                   ekf_local.yaml). Used for Q tuning campaigns — pass a
#                   campaign yaml like ekf_local_q_approx_2026_05_12.yaml.
#   --out-label     suffix for the output bag dir; default "anchored" produces
#                   <bag>/anchored_replay/<bag>_anchored. Use e.g.
#                   --out-label q_approx to write to
#                   <bag>/anchored_replay/<bag>_anchored_q_approx and preserve
#                   any earlier replay output.

set -eo pipefail

BAG_DIR=""
RATE="2.0"
PARAMS_FILE=""
OUT_LABEL=""

# Parse args. Keep bag_dir + rate as positionals; --params-file and --out-label
# can appear anywhere.
_positional=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --params-file)
            PARAMS_FILE="$2"; shift 2 ;;
        --params-file=*)
            PARAMS_FILE="${1#*=}"; shift ;;
        --out-label)
            OUT_LABEL="$2"; shift 2 ;;
        --out-label=*)
            OUT_LABEL="${1#*=}"; shift ;;
        --help|-h)
            sed -n '1,30p' "$0"; exit 0 ;;
        *)
            _positional+=("$1"); shift ;;
    esac
done
if [ "${#_positional[@]}" -lt 1 ]; then
    echo "Usage: $0 <bag_dir> [rate] [--params-file PATH] [--out-label LABEL]" >&2
    exit 1
fi
BAG_DIR="${_positional[0]}"
if [ "${#_positional[@]}" -ge 2 ]; then
    RATE="${_positional[1]}"
fi

# Optional anchor-gate overrides from env vars.
# H_ACC_MAX_M=''  → keep launch default (0.5 m, gnss_datum_watchdog parity)
# H_ACC_MAX_M=5.0 → loosen to 5 m (April-23-style surface antenna)
# H_ACC_TOPIC=''  → disable h_acc gate entirely (anchor on first non-null-island fix)
H_ACC_MAX_M="${H_ACC_MAX_M:-}"
H_ACC_TOPIC="${H_ACC_TOPIC-/ubx_nav_hp_pos_llh}"

BAG_DIR="${BAG_DIR%/}"
BAG_NAME="$(basename "$BAG_DIR")"
# Build output dir. Default suffix "anchored"; with --out-label q_approx it
# becomes anchored_q_approx so the original output is preserved.
if [ -n "$OUT_LABEL" ]; then
    OUT_SUFFIX="anchored_${OUT_LABEL}"
else
    OUT_SUFFIX="anchored"
fi
OUT_DIR="${BAG_DIR}/anchored_replay/${BAG_NAME}_${OUT_SUFFIX}"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "[replay_anchored_bag] Bag:  $BAG_DIR"
echo "[replay_anchored_bag] Out:  $OUT_DIR"
echo "[replay_anchored_bag] Rate: ${RATE}x"

# --- Build topic whitelist (exclude stale filter outputs from the bag) --------
_TOPICS_PY=$(mktemp /tmp/anchored_topics_XXXX.py)
cat > "$_TOPICS_PY" <<'PYEOF'
import sys
from pathlib import Path

EKF_EXCLUDE = {
    "/odometry/filtered/local",
    "/odometry/filtered/local_validated",
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
    echo "[replay_anchored_bag] ERROR: could not read topics from bag" >&2
    exit 1
fi

# --- Detect GPS topic ---------------------------------------------------------
GPS_TOPIC="/fix"
case " $TOPICS_RAW " in
    *" /gps/selected "*) GPS_TOPIC="/gps/selected" ;;
esac
echo "[replay_anchored_bag] GPS topic: $GPS_TOPIC"

# --- Pre-run cleanup ----------------------------------------------------------
pkill -f "ekf_node" 2>/dev/null || true
pkill -f "gnss_anchored_pose" 2>/dev/null || true
pkill -f "odometry_validator" 2>/dev/null || true
sleep 1

# --- Pre-launch EKF -----------------------------------------------------------
# `ros2 launch` Python startup costs ~20s on the Jetson. Doing it inside the
# replay window leaves the local EKF only seconds to actually process the bag.
# We launch first, wait until ekf_node advertises /odometry/filtered/local
# (proof that import + node init are done), THEN start the bag.
# use_sim_time=true means the nodes idle harmlessly until /clock arrives.
EKF_LOG="${OUT_DIR}/../ekf_replay.log"
mkdir -p "$(dirname "$EKF_LOG")"
LAUNCH_ARGS=( "gps_fix_topic:=$GPS_TOPIC" "h_acc_topic:=$H_ACC_TOPIC" )
if [ -n "$H_ACC_MAX_M" ]; then
    LAUNCH_ARGS+=( "h_acc_max_m:=$H_ACC_MAX_M" )
fi
if [ -n "$PARAMS_FILE" ]; then
    if [ ! -f "$PARAMS_FILE" ]; then
        echo "[replay_anchored_bag] ERROR: --params-file not found: $PARAMS_FILE" >&2
        exit 1
    fi
    LAUNCH_ARGS+=( "params_file:=$PARAMS_FILE" )
    echo "[replay_anchored_bag] params_file: $PARAMS_FILE"
fi
echo "[replay_anchored_bag] Launch args: ${LAUNCH_ARGS[*]}"
if [ -n "${DEBUG:-}" ]; then
    PYTHONUNBUFFERED=1 stdbuf -oL -eL \
    ros2 launch ekf_localization_pkg offline_anchored_replay.launch.py \
        "${LAUNCH_ARGS[@]}" 2>&1 | tee "$EKF_LOG" &
    EKF_PID=$!
    echo "[replay_anchored_bag] DEBUG mode — EKF output live + tee to $EKF_LOG"
else
    PYTHONUNBUFFERED=1 stdbuf -oL -eL \
    ros2 launch ekf_localization_pkg offline_anchored_replay.launch.py \
        "${LAUNCH_ARGS[@]}" >"$EKF_LOG" 2>&1 &
    EKF_PID=$!
    echo "[replay_anchored_bag] EKF PID: $EKF_PID — logging to $EKF_LOG"
fi

echo "[replay_anchored_bag] Waiting for ekf_node to advertise /odometry/filtered/local..."
_t=0
until ros2 topic list 2>/dev/null | grep -qx '/odometry/filtered/local'; do
    if [ $_t -ge 60 ]; then
        echo "[replay_anchored_bag] ERROR: ekf_node didn't come up after 60 s; aborting" >&2
        kill "$EKF_PID" 2>/dev/null || true
        exit 1
    fi
    sleep 1; _t=$((_t + 1))
done
echo "[replay_anchored_bag] EKF is up (${_t}s)"

# --- Start recorder -----------------------------------------------------------
mkdir -p "$(dirname "$OUT_DIR")"
if [ -d "$OUT_DIR" ]; then
    echo "[replay_anchored_bag] Removing previous output: $OUT_DIR"
    rm -rf "$OUT_DIR"
fi
# shellcheck disable=SC2086
ros2 bag record \
    -s mcap \
    -o "$OUT_DIR" \
    $TOPICS_RAW \
    /odometry/filtered/local \
    /odometry/filtered/local_validated \
    /odometry/filtered/global \
    /tf &
REC_PID=$!
echo "[replay_anchored_bag] Recorder PID: $REC_PID"
sleep 2  # let recorder discover existing topics before bag publishes

# --- Play bag (background) ----------------------------------------------------
echo "[replay_anchored_bag] Playing bag at ${RATE}x..."
# shellcheck disable=SC2086
ros2 bag play "$BAG_DIR" \
    --clock \
    --rate "$RATE" \
    --read-ahead-queue-size 5000 \
    --topics $TOPICS_RAW &
BAG_PID=$!
echo "[replay_anchored_bag] Bag PID: $BAG_PID"

wait "$BAG_PID"
echo "[replay_anchored_bag] Playback complete — flushing 2s..."
sleep 2

# --- Teardown -----------------------------------------------------------------
kill "$EKF_PID" 2>/dev/null || true
pkill -f "gnss_anchored_pose" 2>/dev/null || true
pkill -f "odometry_validator" 2>/dev/null || true
pkill -f "ekf_node" 2>/dev/null || true
wait "$EKF_PID" 2>/dev/null || true
sleep 1
kill "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true

# --- Diagnostic summary -------------------------------------------------------
echo ""
echo "[replay_anchored_bag] === Diagnostic summary ==="
if grep -q "Anchored: datum" "$EKF_LOG" 2>/dev/null; then
    grep -m1 "Anchored: datum" "$EKF_LOG"
    grep -m1 "Datum set"      "$EKF_LOG" 2>/dev/null
    grep "Re-anchored" "$EKF_LOG" 2>/dev/null | tail -3 \
        || echo "  (no re-anchor events — reanchor_on_each_fix=false)"
else
    echo "  WARNING: no anchor line — gnss_anchored_pose did not anchor."
    n_wait=$(grep -c "anchor wait" "$EKF_LOG" 2>/dev/null || true)
    n_wait=${n_wait:-0}
    echo "  [anchor wait] rejections: $n_wait"
    if [ "$n_wait" -gt 0 ]; then
        echo "  First 3 rejections (showing h_acc):"
        grep "anchor wait" "$EKF_LOG" 2>/dev/null | head -3 | sed 's/^/    /'
        echo "  Last 3 rejections:"
        grep "anchor wait" "$EKF_LOG" 2>/dev/null | tail -3 | sed 's/^/    /'
        echo "  → fix h_acc never met threshold. Re-run with looser gate, e.g.:"
        echo "       ros2 launch ekf_localization_pkg offline_anchored_replay.launch.py \\"
        echo "             gps_fix_topic:=$GPS_TOPIC h_acc_max_m:=5.0"
        echo "     OR disable the gate entirely:  h_acc_topic:=''"
    else
        echo "  No anchor-wait rejections logged. Likely causes:"
        echo "    • Bag has no $GPS_TOPIC messages (check with 'ros2 bag info')."
        echo "    • Bag has no /ubx_nav_hp_pos_llh and h_acc gate is waiting forever."
        echo "      → re-run with h_acc_topic:='' to disable the gate."
        echo "    • Local EKF never produced odometry (no anchor without it)."
    fi
fi
echo "[replay_anchored_bag] Full EKF log: $EKF_LOG"
echo ""
echo "[replay_anchored_bag] Done. Output: $OUT_DIR"
