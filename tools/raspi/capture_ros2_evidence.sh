#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SERVICE_NAME="${MOTIONBRAIN_SERVICE_NAME:-motionbrain-ros-bridge.service}"
CHECK_SERVICE="${CHECK_SERVICE:-1}"
SAMPLE_TIMEOUT_SECONDS="${SAMPLE_TIMEOUT_SECONDS:-12}"
CAPTURE_COMPAT_JSON="${CAPTURE_COMPAT_JSON:-1}"
CAPTURE_MISSION_BOUNDARY="${CAPTURE_MISSION_BOUNDARY:-0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${MOTIONBRAIN_REPO:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-${REPO_DIR}/ros2_ws}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${MOTIONBRAIN_EVIDENCE_OUTPUT:-/tmp/motionbrain_ros2_evidence_${STAMP}.txt}"

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
run_step "ROS2 topic list" ros2 topic list

capture_topic "/motionbrain/status_typed"
capture_topic "/camera/detection_typed"
capture_topic "/joint_states"
capture_topic "/motionbrain/end_effector_pose"
capture_topic "/motionbrain/kinematics_typed"
capture_topic "/motionbrain/control_guard_typed"
capture_topic "/motionbrain/mission_state_typed"

if [[ "${CAPTURE_COMPAT_JSON}" == "1" ]]; then
  capture_topic "/motionbrain/status"
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

section "Summary"
echo "Output: ${OUTPUT}"
if (( failures > 0 )); then
  echo "Result: FAIL (${failures} step(s) failed)"
  exit 1
fi

echo "Result: OK"
