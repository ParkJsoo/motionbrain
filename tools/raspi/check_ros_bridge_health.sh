#!/usr/bin/env bash
set -eo pipefail

SERVICE_NAME="${MOTIONBRAIN_SERVICE_NAME:-motionbrain-ros-bridge.service}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-/home/motionbrain/develop/arduino/motionbrain/ros2_ws}"
CHECK_SERVICE="${CHECK_SERVICE:-1}"
STRICT_CAMERA_AVAILABLE="${STRICT_CAMERA_AVAILABLE:-0}"
TOPIC_WAIT_SECONDS="${TOPIC_WAIT_SECONDS:-20}"
TOPIC_POLL_SECONDS="${TOPIC_POLL_SECONDS:-1}"
SAMPLE_TIMEOUT_SECONDS="${SAMPLE_TIMEOUT_SECONDS:-20}"
EXPECTED_FEEDBACK_SELECTED_TARGET="${EXPECTED_FEEDBACK_SELECTED_TARGET:-base_yaw_reference}"
EXPECTED_FEEDBACK_READY="${EXPECTED_FEEDBACK_READY:-false}"
EXPECTED_PHYSICAL_ROUTINE_ALLOWED="${EXPECTED_PHYSICAL_ROUTINE_ALLOWED:-false}"
EXPECTED_BASE_YAW_FEEDBACK_FAULT="${EXPECTED_BASE_YAW_FEEDBACK_FAULT:-not_installed}"
EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY="${EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY:-}"
EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE="${EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE:-}"
EXPECTED_BASE_YAW_FEEDBACK_PIN="${EXPECTED_BASE_YAW_FEEDBACK_PIN:-36}"
EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW="${EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW:-true}"
EXPECTED_JOINT_STATES_PUBLISHERS="${EXPECTED_JOINT_STATES_PUBLISHERS:-1}"
EXPECTED_ESTIMATED_JOINT_STATES_PUBLISHERS="${EXPECTED_ESTIMATED_JOINT_STATES_PUBLISHERS:-1}"

required_topics=(
  "/motionbrain/status_typed"
  "/motionbrain/routine"
  "/motionbrain/routine_typed"
  "/motionbrain/lifecycle_typed"
  "/motionbrain/diagnostics"
  "/camera/detection_typed"
  "/motionbrain/estimated_joint_states"
  "/joint_states"
  "/motionbrain/end_effector_pose"
  "/motionbrain/kinematics_typed"
  "/motionbrain/control_guard_typed"
  "/motionbrain/mission_state_typed"
)

required_services=(
  "/motionbrain/routine_command"
)

required_actions=(
  "/motionbrain/guarded_routine"
)

expected_lifecycle_nodes=(
  "motionbrain_status_node"
  "motionbrain_joint_state_node"
  "motionbrain_kinematics_node"
  "motionbrain_control_guard_node"
  "motionbrain_mission_supervisor"
)

if [[ "${CHECK_SERVICE}" == "1" ]]; then
  if ! systemctl is-active --quiet "${SERVICE_NAME}"; then
    echo "FAIL service inactive: ${SERVICE_NAME}" >&2
    systemctl status "${SERVICE_NAME}" --no-pager || true
    exit 1
  fi
  echo "OK service active: ${SERVICE_NAME}"
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE}/install/setup.bash"

set -u

check_topic_publisher_count() {
  local topic="$1"
  local expected_count="$2"
  local label="$3"
  local topic_info=""
  local observed_count=""

  if [[ -z "${expected_count}" ]]; then
    return 0
  fi

  local deadline=$((SECONDS + TOPIC_WAIT_SECONDS))
  while (( SECONDS <= deadline )); do
    topic_info="$(timeout 8 ros2 topic info --verbose "${topic}" 2>/dev/null || true)"
    observed_count="$(
      awk -F: '/Publisher count:/ { gsub(/^[[:space:]]+/, "", $2); print $2; exit }' \
        <<< "${topic_info}"
    )"
    if [[ "${observed_count}" == "${expected_count}" ]]; then
      echo "OK ${label} publisher count: ${topic}=${expected_count}"
      return 0
    fi
    sleep "${TOPIC_POLL_SECONDS}"
  done

  echo "FAIL ${label} publisher count for ${topic}: expected ${expected_count}, got ${observed_count:-unknown}" >&2
  echo "${topic_info}" >&2
  exit 1
}

for topic in "${required_topics[@]}"; do
  topic_deadline=$((SECONDS + TOPIC_WAIT_SECONDS))
  topics=""
  while (( SECONDS <= topic_deadline )); do
    topics="$(timeout 8 ros2 topic list || true)"
    if grep -qx "${topic}" <<< "${topics}"; then
      break
    fi
    sleep "${TOPIC_POLL_SECONDS}"
  done
  if ! grep -qx "${topic}" <<< "${topics}"; then
    echo "FAIL missing topic: ${topic}" >&2
    echo "${topics}" >&2
    exit 1
  fi
  echo "OK topic: ${topic}"
done

check_topic_publisher_count \
  "/joint_states" \
  "${EXPECTED_JOINT_STATES_PUBLISHERS}" \
  "joint_states"
check_topic_publisher_count \
  "/motionbrain/estimated_joint_states" \
  "${EXPECTED_ESTIMATED_JOINT_STATES_PUBLISHERS}" \
  "estimated_joint_states"

for service in "${required_services[@]}"; do
  service_deadline=$((SECONDS + TOPIC_WAIT_SECONDS))
  services=""
  while (( SECONDS <= service_deadline )); do
    services="$(timeout 8 ros2 service list || true)"
    if grep -qx "${service}" <<< "${services}"; then
      break
    fi
    sleep "${TOPIC_POLL_SECONDS}"
  done
  if ! grep -qx "${service}" <<< "${services}"; then
    echo "FAIL missing service: ${service}" >&2
    echo "${services}" >&2
    exit 1
  fi
  echo "OK service: ${service}"
done

for action in "${required_actions[@]}"; do
  action_deadline=$((SECONDS + TOPIC_WAIT_SECONDS))
  actions=""
  while (( SECONDS <= action_deadline )); do
    actions="$(timeout 8 ros2 action list || true)"
    if grep -qx "${action}" <<< "${actions}"; then
      break
    fi
    sleep "${TOPIC_POLL_SECONDS}"
  done
  if ! grep -qx "${action}" <<< "${actions}"; then
    echo "FAIL missing action: ${action}" >&2
    echo "${actions}" >&2
    exit 1
  fi
  echo "OK action: ${action}"
done

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/status_typed --once >/dev/null
echo "OK status typed sample"

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/routine --once >/dev/null
echo "OK routine diagnostics sample"

routine_typed_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/routine_typed --once)"
expect_routine_typed_pattern() {
  local pattern="$1"
  local label="$2"
  if ! grep -Fq "${pattern}" <<< "${routine_typed_sample}"; then
    echo "FAIL routine typed sample missing ${label}: ${pattern}" >&2
    echo "${routine_typed_sample}" >&2
    exit 1
  fi
}
expect_routine_typed_pattern \
  "feedback_selected_target: ${EXPECTED_FEEDBACK_SELECTED_TARGET}" \
  "feedback selected target"
expect_routine_typed_pattern \
  "feedback_ready: ${EXPECTED_FEEDBACK_READY}" \
  "feedback ready state"
expect_routine_typed_pattern \
  "physical_routine_execution_allowed: ${EXPECTED_PHYSICAL_ROUTINE_ALLOWED}" \
  "physical routine execution gate"
expect_routine_typed_pattern \
  "base_yaw_feedback_fault: ${EXPECTED_BASE_YAW_FEEDBACK_FAULT}" \
  "base yaw feedback fault"
expect_routine_typed_pattern \
  "base_yaw_feedback_pin: ${EXPECTED_BASE_YAW_FEEDBACK_PIN}" \
  "base yaw feedback pin"
expect_routine_typed_pattern \
  "base_yaw_feedback_active_low: ${EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW}" \
  "base yaw feedback polarity"
if [[ -n "${EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY}" ]]; then
  expect_routine_typed_pattern \
    "base_yaw_feedback_hardware_ready: ${EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY}" \
    "base yaw hardware readiness"
fi
if [[ -n "${EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE}" ]]; then
  expect_routine_typed_pattern \
    "base_yaw_feedback_signal_active: ${EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE}" \
    "base yaw signal state"
fi
echo "OK routine typed feedback readiness sample"

lifecycle_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/lifecycle_typed || true)"
for lifecycle_node in "${expected_lifecycle_nodes[@]}"; do
  if ! awk -v node="${lifecycle_node}" '
    BEGIN { RS="---"; found=0 }
    $0 ~ "node_name: " node && $0 ~ "state_label: active" && $0 ~ "active: true" {
      found=1
    }
    END { exit found ? 0 : 1 }
  ' <<< "${lifecycle_sample}"; then
    echo "FAIL lifecycle sample missing active node: ${lifecycle_node}" >&2
    echo "${lifecycle_sample}" >&2
    exit 1
  fi
done
echo "OK lifecycle active samples"

diagnostics_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/diagnostics --once)"
if ! grep -Fq 'name: motionbrain/controller' <<< "${diagnostics_sample}"; then
  echo "FAIL diagnostics sample missing motionbrain/controller" >&2
  echo "${diagnostics_sample}" >&2
  exit 1
fi
if ! grep -Fq 'name: motionbrain/routine_executor' <<< "${diagnostics_sample}"; then
  echo "FAIL diagnostics sample missing motionbrain/routine_executor" >&2
  echo "${diagnostics_sample}" >&2
  exit 1
fi
if ! grep -Fq 'name: motionbrain/feedback' <<< "${diagnostics_sample}"; then
  echo "FAIL diagnostics sample missing motionbrain/feedback" >&2
  echo "${diagnostics_sample}" >&2
  exit 1
fi
if ! grep -Fq 'base_yaw_fault' <<< "${diagnostics_sample}"; then
  echo "FAIL diagnostics sample missing base_yaw_fault key" >&2
  echo "${diagnostics_sample}" >&2
  exit 1
fi
echo "OK diagnostics sample"

routine_service_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 service call /motionbrain/routine_command \
  motionbrain_msgs/srv/GuardedRoutineCommand "{action: status}")"
if ! grep -Eq 'success[:=][[:space:]]*(true|True)' <<< "${routine_service_sample}"; then
  echo "FAIL routine command service status sample is not success=true" >&2
  echo "${routine_service_sample}" >&2
  exit 1
fi
echo "OK routine command service status sample"

routine_action_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 action send_goal /motionbrain/guarded_routine \
  motionbrain_msgs/action/GuardedRoutine "{action: status}")"
if ! grep -Eq 'success[:=][[:space:]]*(true|True)' <<< "${routine_action_sample}"; then
  echo "FAIL guarded routine action status sample is not success=true" >&2
  echo "${routine_action_sample}" >&2
  exit 1
fi
echo "OK guarded routine action status sample"

camera_detection_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /camera/detection_typed --once)"
if [[ "${STRICT_CAMERA_AVAILABLE}" == "1" ]] && ! grep -Eq '^available: true$' <<< "${camera_detection_sample}"; then
  echo "FAIL camera detection typed sample is not available=true" >&2
  echo "${camera_detection_sample}" >&2
  exit 1
fi
echo "OK camera detection typed sample"

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /joint_states --once >/dev/null
echo "OK joint state sample"

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/estimated_joint_states --once >/dev/null
echo "OK estimated joint state sample"

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/end_effector_pose --once >/dev/null
echo "OK end-effector pose sample"

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/kinematics_typed --once >/dev/null
echo "OK kinematics typed sample"

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/control_guard_typed --once >/dev/null
echo "OK control guard typed sample"

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/mission_state_typed --once >/dev/null
echo "OK mission state typed sample"
