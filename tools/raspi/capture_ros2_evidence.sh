#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SERVICE_NAME="${MOTIONBRAIN_SERVICE_NAME:-motionbrain-ros-bridge.service}"
CHECK_SERVICE="${CHECK_SERVICE:-1}"
SAMPLE_TIMEOUT_SECONDS="${SAMPLE_TIMEOUT_SECONDS:-12}"
CAPTURE_COMPAT_JSON="${CAPTURE_COMPAT_JSON:-1}"
CAPTURE_MISSION_BOUNDARY="${CAPTURE_MISSION_BOUNDARY:-0}"
CAPTURE_ROUTINE_COMMAND_BOUNDARY="${CAPTURE_ROUTINE_COMMAND_BOUNDARY:-0}"
CAPTURE_ROUTINE_SERVICE_BOUNDARY="${CAPTURE_ROUTINE_SERVICE_BOUNDARY:-0}"
CAPTURE_ROUTINE_ACTION_BOUNDARY="${CAPTURE_ROUTINE_ACTION_BOUNDARY:-0}"
CAPTURE_ROSBAG="${CAPTURE_ROSBAG:-0}"
ROSBAG_DURATION_SECONDS="${ROSBAG_DURATION_SECONDS:-10}"
COMMAND_ECHO_SETTLE_SECONDS="${COMMAND_ECHO_SETTLE_SECONDS:-1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${MOTIONBRAIN_REPO:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-${REPO_DIR}/ros2_ws}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${MOTIONBRAIN_EVIDENCE_OUTPUT:-/tmp/motionbrain_ros2_evidence_${STAMP}.txt}"
ROSBAG_OUTPUT="${MOTIONBRAIN_ROSBAG_OUTPUT:-/tmp/motionbrain_ros2_bag_${STAMP}}"

failures=0

mkdir -p "$(dirname "${OUTPUT}")"
exec > >(tee "${OUTPUT}") 2>&1

section() {
  echo
  echo "## $*"
}

run_step() {
  local label="$1"
  local rc=0
  shift
  section "${label}"
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  set +e
  "$@"
  rc=$?
  set -e
  if (( rc != 0 )); then
    echo "FAIL ${label}: exit ${rc}"
    failures=$((failures + 1))
  fi
}

capture_topic() {
  local topic="$1"
  run_step "Topic sample ${topic}" timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo "${topic}" --once
}

capture_command_result() {
  local label="$1"
  local command_topic="$2"
  local command_type="$3"
  local command_payload="$4"
  local result_topic="$5"
  shift 5
  local expected_patterns=("$@")
  local echo_output
  local echo_pid
  local pub_rc=0
  local echo_rc=0
  local missing=0

  section "${label}"
  echo_output="$(mktemp)"
  printf '+ timeout %q ros2 topic echo %q --once\n' "${SAMPLE_TIMEOUT_SECONDS}" "${result_topic}"
  set +e
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo "${result_topic}" --once > "${echo_output}" 2>&1 &
  echo_pid=$!
  sleep "${COMMAND_ECHO_SETTLE_SECONDS}"
  printf '+ timeout %q ros2 topic pub --once --wait-matching-subscriptions 1 %q %q %q\n' \
    "${SAMPLE_TIMEOUT_SECONDS}" "${command_topic}" "${command_type}" "${command_payload}"
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic pub --once --wait-matching-subscriptions 1 \
    "${command_topic}" "${command_type}" "${command_payload}"
  pub_rc=$?
  wait "${echo_pid}"
  echo_rc=$?
  set -e
  cat "${echo_output}"
  for pattern in "${expected_patterns[@]}"; do
    if ! grep -Fq "${pattern}" "${echo_output}"; then
      echo "FAIL ${label}: missing expected output pattern: ${pattern}"
      missing=$((missing + 1))
    fi
  done
  rm -f "${echo_output}"
  if (( pub_rc != 0 || echo_rc != 0 || missing != 0 )); then
    echo "FAIL ${label}: publish exit ${pub_rc}, echo exit ${echo_rc}, missing ${missing}"
    failures=$((failures + 1))
  fi
}

capture_service_result() {
  local label="$1"
  local service_name="$2"
  local service_type="$3"
  local service_request="$4"
  shift 4
  local expected_patterns=("$@")
  local service_output
  local rc=0
  local missing=0

  section "${label}"
  service_output="$(mktemp)"
  printf '+ timeout %q ros2 service call %q %q %q\n' \
    "${SAMPLE_TIMEOUT_SECONDS}" "${service_name}" "${service_type}" "${service_request}"
  set +e
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 service call \
    "${service_name}" "${service_type}" "${service_request}" > "${service_output}" 2>&1
  rc=$?
  set -e
  cat "${service_output}"
  for pattern in "${expected_patterns[@]}"; do
    if ! grep -Fq "${pattern}" "${service_output}"; then
      echo "FAIL ${label}: missing expected output pattern: ${pattern}"
      missing=$((missing + 1))
    fi
  done
  rm -f "${service_output}"
  if (( rc != 0 || missing != 0 )); then
    echo "FAIL ${label}: service exit ${rc}, missing ${missing}"
    failures=$((failures + 1))
  fi
}

capture_action_result() {
  local label="$1"
  local action_name="$2"
  local action_type="$3"
  local action_goal="$4"
  shift 4
  local expected_patterns=("$@")
  local action_output
  local rc=0
  local missing=0

  section "${label}"
  action_output="$(mktemp)"
  printf '+ timeout %q ros2 action send_goal %q %q %q\n' \
    "${SAMPLE_TIMEOUT_SECONDS}" "${action_name}" "${action_type}" "${action_goal}"
  set +e
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 action send_goal \
    "${action_name}" "${action_type}" "${action_goal}" > "${action_output}" 2>&1
  rc=$?
  set -e
  cat "${action_output}"
  for pattern in "${expected_patterns[@]}"; do
    if ! grep -Fq "${pattern}" "${action_output}"; then
      echo "FAIL ${label}: missing expected output pattern: ${pattern}"
      missing=$((missing + 1))
    fi
  done
  rm -f "${action_output}"
  if (( rc != 0 || missing != 0 )); then
    echo "FAIL ${label}: action exit ${rc}, missing ${missing}"
    failures=$((failures + 1))
  fi
}

capture_rosbag() {
  local rc=0
  local topics=(
    "/motionbrain/status_typed"
    "/motionbrain/routine_typed"
    "/motionbrain/diagnostics"
    "/motionbrain/events_typed"
    "/camera/detection_typed"
    "/joint_states"
    "/motionbrain/end_effector_pose"
    "/motionbrain/kinematics_typed"
    "/motionbrain/control_guard_typed"
    "/motionbrain/mission_state_typed"
  )

  section "ROS2 bag read-only capture"
  echo "Output directory: ${ROSBAG_OUTPUT}"
  printf '+ timeout --signal=SIGINT %q ros2 bag record -o %q' \
    "${ROSBAG_DURATION_SECONDS}" "${ROSBAG_OUTPUT}"
  printf ' %q' "${topics[@]}"
  printf '\n'

  set +e
  timeout --signal=SIGINT "${ROSBAG_DURATION_SECONDS}" ros2 bag record \
    -o "${ROSBAG_OUTPUT}" \
    "${topics[@]}"
  rc=$?
  set -e

  if (( rc != 0 && rc != 124 )); then
    echo "FAIL ROS2 bag read-only capture: exit ${rc}"
    failures=$((failures + 1))
    return
  fi

  if [[ ! -d "${ROSBAG_OUTPUT}" ]]; then
    echo "FAIL ROS2 bag read-only capture: missing output directory"
    failures=$((failures + 1))
    return
  fi

  if [[ ! -f "${ROSBAG_OUTPUT}/metadata.yaml" ]]; then
    echo "FAIL ROS2 bag read-only capture: missing metadata.yaml"
    failures=$((failures + 1))
    return
  fi

  run_step "ROS2 bag output files" find "${ROSBAG_OUTPUT}" -maxdepth 2 -type f -print
}

section "MotionBrain ROS2 Evidence"
date --iso-8601=seconds
hostname
uname -a

if [[ -f /etc/os-release ]]; then
  run_step "OS release" sed -n '1,8p' /etc/os-release
fi

run_step "Git HEAD" git -C "${REPO_DIR}" log --oneline -1
run_step "Git status" git -C "${REPO_DIR}" status --short
section "ROS distro"
echo "ROS_DISTRO=${ROS_DISTRO}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "FAIL ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 2
fi

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "FAIL workspace setup not found: ${WORKSPACE}/install/setup.bash"
  echo "Build first: colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description"
  exit 2
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE}/install/setup.bash"

set -u

if [[ "${CHECK_SERVICE}" == "1" ]]; then
  run_step "Service active" systemctl is-active "${SERVICE_NAME}"
  run_step "Service status" systemctl status "${SERVICE_NAME}" --no-pager
fi

run_step "Bridge health check" env \
  CHECK_SERVICE="${CHECK_SERVICE}" \
  ROS_DISTRO="${ROS_DISTRO}" \
  MOTIONBRAIN_ROS_WS="${WORKSPACE}" \
  "${REPO_DIR}/tools/raspi/check_ros_bridge_health.sh"

run_step "MotionBrain ROS2 packages" bash -lc "ros2 pkg list | grep '^motionbrain'"
run_step "MotionBrain ROS2 interfaces" bash -lc "ros2 interface list | grep 'motionbrain_msgs/msg'"
run_step "MotionBrain ROS2 services" bash -lc "ros2 interface list | grep 'motionbrain_msgs/srv'"
run_step "MotionBrain ROS2 actions" bash -lc "ros2 interface list | grep 'motionbrain_msgs/action'"
run_step "ROS2 topic list" ros2 topic list
run_step "ROS2 service list" ros2 service list
run_step "ROS2 action list" ros2 action list

capture_topic "/motionbrain/status_typed"
capture_topic "/motionbrain/routine_typed"
capture_topic "/motionbrain/diagnostics"
capture_topic "/camera/detection_typed"
capture_topic "/joint_states"
capture_topic "/motionbrain/end_effector_pose"
capture_topic "/motionbrain/kinematics_typed"
capture_topic "/motionbrain/control_guard_typed"
capture_topic "/motionbrain/mission_state_typed"

if [[ "${CAPTURE_COMPAT_JSON}" == "1" ]]; then
  capture_topic "/motionbrain/status"
  capture_topic "/motionbrain/routine"
  capture_topic "/camera/detection"
  capture_topic "/motionbrain/kinematics"
  capture_topic "/motionbrain/control_guard"
  capture_topic "/motionbrain/mission_state"
fi

if [[ "${CAPTURE_MISSION_BOUNDARY}" == "1" ]]; then
  run_step "Mission command start" timeout "${SAMPLE_TIMEOUT_SECONDS}" \
    ros2 topic pub --once --wait-matching-subscriptions 1 \
    /motionbrain/mission_cmd_typed motionbrain_msgs/msg/MissionCommand \
    "{command: start}"
  sleep 1
  capture_topic "/motionbrain/mission_state_typed"

  run_step "Mission command reset" timeout "${SAMPLE_TIMEOUT_SECONDS}" \
    ros2 topic pub --once --wait-matching-subscriptions 1 \
    /motionbrain/mission_cmd_typed motionbrain_msgs/msg/MissionCommand \
    "{command: reset}"
  sleep 1
  capture_topic "/motionbrain/mission_state_typed"
fi

if [[ "${CAPTURE_ROUTINE_COMMAND_BOUNDARY}" == "1" ]]; then
  capture_command_result "Routine command status result" \
    /motionbrain/routine_cmd_typed motionbrain_msgs/msg/RoutineCommand \
    "{action: status}" \
    /motionbrain/routine_result_typed \
    "success: true" "action: status" "forwarded: true"

  capture_command_result "Routine command run rejection result" \
    /motionbrain/routine_cmd_typed motionbrain_msgs/msg/RoutineCommand \
    "{action: run, routine_name: inspect, confirm_code: confirm-inspect}" \
    /motionbrain/routine_result_typed \
    "success: false" "forwarded: false" "routine_execute_disabled_by_bridge_policy"
fi

if [[ "${CAPTURE_ROUTINE_SERVICE_BOUNDARY}" == "1" ]]; then
  capture_service_result "Routine command service status result" \
    /motionbrain/routine_command motionbrain_msgs/srv/GuardedRoutineCommand \
    "{action: status}" \
    "success=True" "action='status'" "forwarded=True"

  capture_service_result "Routine command service run rejection result" \
    /motionbrain/routine_command motionbrain_msgs/srv/GuardedRoutineCommand \
    "{action: run, routine_name: inspect, confirm_code: confirm-inspect}" \
    "success=False" "forwarded=False" "routine_execute_disabled_by_bridge_policy"
fi

if [[ "${CAPTURE_ROUTINE_ACTION_BOUNDARY}" == "1" ]]; then
  capture_action_result "Guarded routine action status result" \
    /motionbrain/guarded_routine motionbrain_msgs/action/GuardedRoutine \
    "{action: status}" \
    "success=True" "action='status'" "forwarded=True"

  capture_action_result "Guarded routine action run rejection result" \
    /motionbrain/guarded_routine motionbrain_msgs/action/GuardedRoutine \
    "{action: run, routine_name: inspect, confirm_code: confirm-inspect}" \
    "success=False" "forwarded=False" "routine_execute_disabled_by_bridge_policy"
fi

if [[ "${CAPTURE_ROSBAG}" == "1" ]]; then
  capture_rosbag
fi

section "Summary"
echo "Output: ${OUTPUT}"
if [[ "${CAPTURE_ROSBAG}" == "1" ]]; then
  echo "ROS2 bag: ${ROSBAG_OUTPUT}"
fi
if (( failures > 0 )); then
  echo "Result: FAIL (${failures} step(s) failed)"
  exit 1
fi

echo "Result: OK"
