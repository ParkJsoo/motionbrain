#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
HARDWARE_ROS_DOMAIN_ID="${HARDWARE_ROS_DOMAIN_ID:-43}"
SAMPLE_TIMEOUT_SECONDS="${SAMPLE_TIMEOUT_SECONDS:-15}"
STARTUP_TIMEOUT_SECONDS="${STARTUP_TIMEOUT_SECONDS:-35}"
TRAJECTORY_SETTLE_SECONDS="${TRAJECTORY_SETTLE_SECONDS:-3}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${MOTIONBRAIN_REPO:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-${REPO_DIR}/ros2_ws}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${MOTIONBRAIN_EVIDENCE_OUTPUT:-/tmp/motionbrain_ros2_control_hardware_evidence_${STAMP}.txt}"
LAUNCH_LOG="${MOTIONBRAIN_HARDWARE_LAUNCH_LOG:-/tmp/motionbrain_ros2_control_hardware_launch_${STAMP}.log}"

failures=0
launch_pid=""

mkdir -p "$(dirname "${OUTPUT}")"
exec > >(tee "${OUTPUT}") 2>&1

section() {
  echo
  echo "## $*"
}

print_command() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
}

run_step() {
  local label="$1"
  local rc=0
  shift
  section "${label}"
  print_command "$@"
  set +e
  "$@"
  rc=$?
  set -e
  if (( rc != 0 )); then
    echo "FAIL ${label}: exit ${rc}"
    failures=$((failures + 1))
  fi
}

capture_step() {
  local label="$1"
  shift
  local expected_patterns=()
  local output
  local rc=0
  local missing=0

  while (( $# > 0 )); do
    if [[ "$1" == "--" ]]; then
      shift
      break
    fi
    expected_patterns+=("$1")
    shift
  done

  section "${label}"
  output="$(mktemp)"
  print_command "$@"
  set +e
  "$@" > "${output}" 2>&1
  rc=$?
  set -e
  cat "${output}"

  for pattern in "${expected_patterns[@]}"; do
    if ! grep -Fq -- "${pattern}" "${output}"; then
      echo "FAIL ${label}: missing expected output pattern: ${pattern}"
      missing=$((missing + 1))
    fi
  done
  rm -f "${output}"

  if (( rc != 0 || missing != 0 )); then
    echo "FAIL ${label}: exit ${rc}, missing ${missing}"
    failures=$((failures + 1))
  fi
}

cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" >/dev/null 2>&1; then
    section "Stop hardware-interface launch"
    kill -INT "-${launch_pid}" >/dev/null 2>&1 || kill -INT "${launch_pid}" >/dev/null 2>&1 || true
    for _ in {1..10}; do
      if ! kill -0 "${launch_pid}" >/dev/null 2>&1; then
        break
      fi
      sleep 1
    done
    if kill -0 "${launch_pid}" >/dev/null 2>&1; then
      kill -TERM "-${launch_pid}" >/dev/null 2>&1 || kill -TERM "${launch_pid}" >/dev/null 2>&1 || true
      sleep 2
    fi
    if kill -0 "${launch_pid}" >/dev/null 2>&1; then
      kill -KILL "-${launch_pid}" >/dev/null 2>&1 || kill -KILL "${launch_pid}" >/dev/null 2>&1 || true
    fi
    wait "${launch_pid}" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

section "MotionBrain ros2_control Hardware Interface Evidence"
date --iso-8601=seconds
hostname
uname -a
echo "ROS_DISTRO=${ROS_DISTRO}"
echo "ROS_DOMAIN_ID=${HARDWARE_ROS_DOMAIN_ID}"
echo "Workspace=${WORKSPACE}"
echo "Physical actuation: disabled; hardware interface transport_mode is dry_run"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "FAIL ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 2
fi

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "FAIL workspace setup not found: ${WORKSPACE}/install/setup.bash"
  echo "Build first: colcon build --packages-select motionbrain_hardware_interface"
  exit 2
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE}/install/setup.bash"
export ROS_DOMAIN_ID="${HARDWARE_ROS_DOMAIN_ID}"

required_packages=(
  "controller_manager"
  "hardware_interface"
  "joint_state_broadcaster"
  "joint_trajectory_controller"
  "ros2_control"
  "ros2controlcli"
  "motionbrain_hardware_interface"
)

missing_packages=()
section "Runtime package check"
for package in "${required_packages[@]}"; do
  if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "OK package: ${package}"
  else
    echo "MISSING package: ${package}"
    missing_packages+=("${package}")
  fi
done

if (( ${#missing_packages[@]} > 0 )); then
  echo "Install missing Jazzy runtime packages on the Pi, then rerun this helper."
  exit 2
fi

run_step "Git HEAD" git -C "${REPO_DIR}" log --oneline -1
run_step "Git status" git -C "${REPO_DIR}" status --short
run_step "Hardware package prefix" ros2 pkg prefix motionbrain_hardware_interface
run_step "Dry-run transport contract" grep -F "transport_mode\">dry_run" \
  "${WORKSPACE}/install/motionbrain_hardware_interface/share/motionbrain_hardware_interface/urdf/motionbrain_hardware_interface.urdf"

section "Start hardware-interface launch"
echo "Launch log: ${LAUNCH_LOG}"
print_command ros2 launch motionbrain_hardware_interface hardware_interface.launch.py
setsid ros2 launch motionbrain_hardware_interface hardware_interface.launch.py > "${LAUNCH_LOG}" 2>&1 &
launch_pid=$!

section "Wait for controller manager"
deadline=$((SECONDS + STARTUP_TIMEOUT_SECONDS))
controller_ready=0
while (( SECONDS < deadline )); do
  if timeout 5 ros2 control list_controllers >/tmp/motionbrain_hardware_controllers.$$ 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
cat /tmp/motionbrain_hardware_controllers.$$ 2>/dev/null || true
rm -f /tmp/motionbrain_hardware_controllers.$$

if (( controller_ready == 0 )); then
  echo "WARN controller manager readiness probe did not complete within ${STARTUP_TIMEOUT_SECONDS}s"
  echo "WARN continuing because the validated controller list step below is authoritative"
fi

section "Wait for active controllers"
deadline=$((SECONDS + STARTUP_TIMEOUT_SECONDS))
active_controllers_output="$(mktemp)"
controllers_active=0
while (( SECONDS < deadline )); do
  if timeout 5 ros2 control list_controllers > "${active_controllers_output}" 2>&1 &&
     grep -Fq "joint_state_broadcaster" "${active_controllers_output}" &&
     grep -Fq "motionbrain_arm_controller" "${active_controllers_output}" &&
     grep -Fq "active" "${active_controllers_output}"; then
    controllers_active=1
    break
  fi
  sleep 1
done
cat "${active_controllers_output}" || true
rm -f "${active_controllers_output}"
if (( controllers_active == 0 )); then
  echo "FAIL active controllers were not ready within ${STARTUP_TIMEOUT_SECONDS}s"
  failures=$((failures + 1))
fi

capture_step "Controller list" \
  "joint_state_broadcaster" "active" "motionbrain_arm_controller" "active" -- \
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 control list_controllers

capture_step "Hardware interfaces" \
  "base_yaw_joint/position [available] [claimed]" \
  "shoulder_pitch_joint/position [available] [claimed]" \
  "gripper_joint/position [available] [claimed]" \
  "base_yaw_joint/velocity" \
  "wrist_pitch_joint/velocity" -- \
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 control list_hardware_interfaces

capture_step "Joint states before trajectory" \
  "base_yaw_joint" "shoulder_pitch_joint" "gripper_joint" "position:" -- \
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /joint_states --once

run_step "Send open-loop trajectory goal" timeout "${SAMPLE_TIMEOUT_SECONDS}" \
  ros2 action send_goal \
  /motionbrain_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [base_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, gripper_joint], points: [{positions: [0.2, 0.1, -0.1, 0.05, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}}"

sleep "${TRAJECTORY_SETTLE_SECONDS}"

capture_step "Joint states after trajectory" \
  "base_yaw_joint" "shoulder_pitch_joint" "gripper_joint" \
  "- 0.2" "- 0.1" "- -0.1" "- 0.05" -- \
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /joint_states --once

section "Launch log tail"
tail -140 "${LAUNCH_LOG}" || true

section "Result"
if (( failures == 0 )); then
  echo "OK"
else
  echo "FAIL failures=${failures}"
  exit 1
fi
