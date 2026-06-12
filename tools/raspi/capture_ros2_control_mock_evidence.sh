#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
MOCK_ROS_DOMAIN_ID="${MOCK_ROS_DOMAIN_ID:-42}"
SAMPLE_TIMEOUT_SECONDS="${SAMPLE_TIMEOUT_SECONDS:-12}"
MOCK_STARTUP_TIMEOUT_SECONDS="${MOCK_STARTUP_TIMEOUT_SECONDS:-30}"
CAPTURE_MOCK_TRAJECTORY="${CAPTURE_MOCK_TRAJECTORY:-0}"
EXTRA_ROS_PREFIX="${MOTIONBRAIN_ROS2_CONTROL_OVERLAY_PREFIX:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${MOTIONBRAIN_REPO:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-${REPO_DIR}/ros2_ws}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${MOTIONBRAIN_EVIDENCE_OUTPUT:-/tmp/motionbrain_ros2_control_mock_evidence_${STAMP}.txt}"
LAUNCH_LOG="${MOTIONBRAIN_MOCK_LAUNCH_LOG:-/tmp/motionbrain_ros2_control_mock_launch_${STAMP}.log}"

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
    if ! grep -Fq "${pattern}" "${output}"; then
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
  if [[ -n "${launch_pid}" ]]; then
    if kill -0 "${launch_pid}" >/dev/null 2>&1; then
      section "Stop mock launch"
      kill -INT "${launch_pid}" >/dev/null 2>&1 || true
      wait "${launch_pid}" >/dev/null 2>&1 || true
    fi
  fi
}

trap cleanup EXIT

section "MotionBrain ros2_control Mock Evidence"
date --iso-8601=seconds
hostname
uname -a
echo "ROS_DISTRO=${ROS_DISTRO}"
echo "ROS_DOMAIN_ID=${MOCK_ROS_DOMAIN_ID}"
echo "Workspace=${WORKSPACE}"
if [[ -n "${EXTRA_ROS_PREFIX}" ]]; then
  echo "Extra ROS prefix=${EXTRA_ROS_PREFIX}"
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "FAIL ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 2
fi

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "FAIL workspace setup not found: ${WORKSPACE}/install/setup.bash"
  echo "Build first: colcon build --packages-select motionbrain_ros2_control_mock"
  exit 2
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -n "${EXTRA_ROS_PREFIX}" ]]; then
  if [[ ! -d "${EXTRA_ROS_PREFIX}" ]]; then
    echo "FAIL extra ROS prefix not found: ${EXTRA_ROS_PREFIX}"
    exit 2
  fi
  python_site="$(
    python3 - <<'PY'
import sys
print(f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages")
PY
  )"
  export AMENT_PREFIX_PATH="${EXTRA_ROS_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${EXTRA_ROS_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export COLCON_PREFIX_PATH="${EXTRA_ROS_PREFIX}:${COLCON_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${EXTRA_ROS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  export PATH="${EXTRA_ROS_PREFIX}/bin:${PATH:-}"
  export PYTHONPATH="${EXTRA_ROS_PREFIX}/${python_site}:${PYTHONPATH:-}"
  if [[ -d "${EXTRA_ROS_PREFIX}/opt" ]]; then
    while IFS= read -r vendor_lib_dir; do
      export LD_LIBRARY_PATH="${vendor_lib_dir}:${LD_LIBRARY_PATH:-}"
    done < <(find "${EXTRA_ROS_PREFIX}/opt" -type d -name lib)
  fi
fi
source "${WORKSPACE}/install/setup.bash"
export ROS_DOMAIN_ID="${MOCK_ROS_DOMAIN_ID}"

required_packages=(
  "controller_manager"
  "hardware_interface"
  "joint_state_broadcaster"
  "joint_trajectory_controller"
  "ros2_control"
  "ros2_control_test_assets"
  "ros2controlcli"
  "motionbrain_ros2_control_mock"
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
  echo
  echo "Install missing Jazzy runtime packages on the Pi, then rerun this helper:"
  cat <<'EOF'
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-ros2-control-test-assets \
  ros-jazzy-ros2controlcli
EOF
  exit 2
fi

run_step "Git HEAD" git -C "${REPO_DIR}" log --oneline -1
run_step "Git status" git -C "${REPO_DIR}" status --short
run_step "Mock package prefix" ros2 pkg prefix motionbrain_ros2_control_mock
run_step "Mock launch file" ros2 pkg prefix motionbrain_ros2_control_mock

section "Start mock launch"
echo "Launch log: ${LAUNCH_LOG}"
print_command ros2 launch motionbrain_ros2_control_mock mock_control.launch.py
ros2 launch motionbrain_ros2_control_mock mock_control.launch.py > "${LAUNCH_LOG}" 2>&1 &
launch_pid=$!

section "Wait for controller manager"
deadline=$((SECONDS + MOCK_STARTUP_TIMEOUT_SECONDS))
controller_ready=0
while (( SECONDS < deadline )); do
  if timeout 5 ros2 control list_controllers >/tmp/motionbrain_mock_controllers.$$ 2>&1; then
    controller_ready=1
    break
  fi
  sleep 1
done
cat /tmp/motionbrain_mock_controllers.$$ 2>/dev/null || true
rm -f /tmp/motionbrain_mock_controllers.$$

if (( controller_ready == 0 )); then
  echo "FAIL controller manager did not become ready within ${MOCK_STARTUP_TIMEOUT_SECONDS}s"
  failures=$((failures + 1))
fi

capture_step "Controller list" \
  "joint_state_broadcaster" "active" "motionbrain_arm_controller" "active" -- \
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 control list_controllers

capture_step "Hardware interfaces" \
  "base_yaw_joint/position" "shoulder_pitch_joint/position" "gripper_joint/position" -- \
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 control list_hardware_interfaces

capture_step "Mock joint states" \
  "base_yaw_joint" "shoulder_pitch_joint" "gripper_joint" -- \
  timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /joint_states --once

if [[ "${CAPTURE_MOCK_TRAJECTORY}" == "1" ]]; then
  run_step "Publish mock-only trajectory" timeout "${SAMPLE_TIMEOUT_SECONDS}" \
    ros2 topic pub --once --wait-matching-subscriptions 1 \
    /motionbrain_arm_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [base_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, gripper_joint], points: [{positions: [0.2, 0.1, -0.1, 0.05, 0.0], time_from_start: {sec: 2}}]}"

  sleep 3
  capture_step "Mock joint states after trajectory" \
    "base_yaw_joint" "shoulder_pitch_joint" "gripper_joint" -- \
    timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /joint_states --once
fi

section "Launch log tail"
tail -120 "${LAUNCH_LOG}" || true

section "Summary"
echo "Output: ${OUTPUT}"
echo "Launch log: ${LAUNCH_LOG}"
if (( failures > 0 )); then
  echo "Result: FAIL (${failures} step(s) failed)"
  exit 1
fi

echo "Result: OK"
