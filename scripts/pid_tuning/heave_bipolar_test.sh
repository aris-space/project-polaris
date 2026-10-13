#!/usr/bin/env bash
# Heave tuning run: GUIDED + arm -> activate Nav2 -> dive to z=-5 -> disarm/arm cycle
#   -> bipolar heave ramp -> disarm.
#
# Usage: ./scripts/pid_tuning/heave_bipolar_test.sh
# Requires the workspace to be sourced and the bringup (autonomy:=true) running.
# Disarms on any exit, including errors and Ctrl-C.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_TIMEOUT_S=30
STATE_RESEND_S=3

# --once waits for a subscriber; the timeout keeps a dead bridge from hanging the script.
pub_mode() { timeout 5 ros2 topic pub --once /pixhawk/mode_cmd std_msgs/msg/String "{data: $1}" >/dev/null || true; }
pub_arm()  { timeout 5 ros2 topic pub --once /pixhawk/arm_cmd  std_msgs/msg/Bool   "{data: $1}" >/dev/null || true; }

disarm() {
  echo "[heave_test] Disarming"
  pub_arm false
}
trap disarm EXIT

# set_state <true|false>: command GUIDED + armed=<arg>, poll the heartbeat and resend
# whichever command doesn't match yet (every STATE_RESEND_S) until STATE_TIMEOUT_S.
set_state() {
  local want_armed="$1" deadline=$((SECONDS + STATE_TIMEOUT_S)) last_send=-999 hb mode="" armed=""
  echo "[heave_test] Commanding GUIDED + armed=${want_armed} (up to ${STATE_TIMEOUT_S}s)"
  while (( SECONDS < deadline )); do
    if (( SECONDS - last_send >= STATE_RESEND_S )); then
      if [[ "$mode" != "GUIDED" ]]; then pub_mode GUIDED; fi
      if [[ "$armed" != "$want_armed" ]]; then pub_arm "$want_armed"; fi
      last_send=$SECONDS
    fi
    hb="$(timeout 3 ros2 topic echo --once /pixhawk/heartbeat mavros_msgs/msg/State 2>/dev/null || true)"
    mode="$(awk '/^mode:/ {print $2}' <<<"$hb")"
    armed="$(awk '/^armed:/ {print $2}' <<<"$hb")"
    if [[ "$mode" == "GUIDED" && "$armed" == "$want_armed" ]]; then
      echo "[heave_test] GUIDED + armed=${want_armed}"
      return 0
    fi
    echo "[heave_test]   heartbeat: mode='${mode}' armed='${armed}', retrying"
    sleep 1
  done
  echo "[heave_test] Expected GUIDED + armed=${want_armed}, got mode='${mode}', armed='${armed}'. Aborting."
  exit 1
}

set_state true

# STARTUP only works from unconfigured; after nav2_deactivate (PAUSE) the nodes sit
# in inactive and need RESUME instead.
nav2_state="$(timeout 10 ros2 lifecycle get /waypoint_follower 2>/dev/null | awk '{print $1}' || true)"
echo "[heave_test] Nav2 state: '${nav2_state}'"
case "$nav2_state" in
  active)
    echo "[heave_test] Nav2 already active"
    ;;
  inactive)
    echo "[heave_test] Resuming Nav2"
    resume_out="$(timeout 60 ros2 service call /lifecycle_manager_navigation/manage_nodes \
      nav2_msgs/srv/ManageLifecycleNodes "{command: 2}" 2>&1 || true)"
    if ! grep -q "success=True" <<<"$resume_out"; then
      echo "$resume_out"
      echo "[heave_test] Nav2 RESUME failed. Aborting."
      exit 1
    fi
    ;;
  *)
    echo "[heave_test] Activating Nav2"
    ros2 run autonomy_bringup_pkg nav2_activate
    ;;
esac

echo "[heave_test] Sending goal z=-5.0 and waiting for result"
goal_out="$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {z: -5.0}, orientation: {w: 1.0}}}}" \
  | tee /dev/stderr)"
if ! grep -q "SUCCEEDED" <<<"$goal_out"; then
  echo "[heave_test] Goal did not succeed. Aborting."
  exit 1
fi

echo "[heave_test] Goal reached, cycling disarm/arm"
set_state false
set_state true

echo "[heave_test] Waiting 5s before ramp"
sleep 5

echo "[heave_test] Running bipolar heave ramp"
"${SCRIPT_DIR}/cmd_vel_ramp.py" bipolar heave 0.08 --ramp 5 --hold 5 --exit-when-done

echo "[heave_test] Done"
# disarm runs via the EXIT trap
