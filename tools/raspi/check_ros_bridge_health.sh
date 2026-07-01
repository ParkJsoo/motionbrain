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
EXPECTED_CONTROLLER_DIAGNOSTIC_MAX_LEVEL="${EXPECTED_CONTROLLER_DIAGNOSTIC_MAX_LEVEL:-0}"
EXPECTED_SHOULDER_DIAGNOSTIC_MAX_LEVEL="${EXPECTED_SHOULDER_DIAGNOSTIC_MAX_LEVEL:-1}"
EXPECTED_ROUTINE_EXECUTOR_DIAGNOSTIC_MAX_LEVEL="${EXPECTED_ROUTINE_EXECUTOR_DIAGNOSTIC_MAX_LEVEL:-0}"
EXPECTED_FEEDBACK_DIAGNOSTIC_MAX_LEVEL="${EXPECTED_FEEDBACK_DIAGNOSTIC_MAX_LEVEL:-1}"
EXPECTED_TELEOP_SENSOR_DIAGNOSTIC_MAX_LEVEL="${EXPECTED_TELEOP_SENSOR_DIAGNOSTIC_MAX_LEVEL:-0}"
EXPECTED_CAMERA_PERCEPTION_DIAGNOSTIC_MAX_LEVEL="${EXPECTED_CAMERA_PERCEPTION_DIAGNOSTIC_MAX_LEVEL:-0}"
EXPECTED_KINEMATICS_JOINT_STATES_TOPIC="${EXPECTED_KINEMATICS_JOINT_STATES_TOPIC:-/joint_states}"
EXPECTED_KINEMATICS_JOINT_NAMES="${EXPECTED_KINEMATICS_JOINT_NAMES:-base_yaw_joint shoulder_pitch_joint elbow_pitch_joint wrist_pitch_joint gripper_joint}"
CHECK_ROUTINE_RUN_REJECTION="${CHECK_ROUTINE_RUN_REJECTION:-1}"

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

diagnostic_block() {
  local diagnostic_name="$1"

  awk -v name="${diagnostic_name}" '
    BEGIN { RS="\n- "; found=0 }
    $0 ~ "name: " name "([[:space:]]|$)" {
      print $0
      found=1
      exit
    }
    END { exit found ? 0 : 1 }
  ' <<< "${diagnostics_sample}"
}

diagnostic_level_number() {
  local diagnostic_text="$1"
  local level_value=""

  level_value="$(
    awk -F': ' '/^[[:space:]]*level:/ { print $2; exit }' <<< "${diagnostic_text}" \
      | tr -d '[:space:]"'
  )"
  case "${level_value}" in
    0|\\0) echo 0 ;;
    1|\\x01) echo 1 ;;
    2|\\x02) echo 2 ;;
    3|\\x03) echo 3 ;;
    *) echo 99 ;;
  esac
}

check_diagnostic_max_level() {
  local diagnostic_name="$1"
  local max_level="$2"
  local label="$3"
  local block=""
  local level=""

  if ! block="$(diagnostic_block "${diagnostic_name}")"; then
    echo "FAIL diagnostics sample missing ${diagnostic_name}" >&2
    echo "${diagnostics_sample}" >&2
    return 1
  fi

  if [[ -z "${max_level}" ]]; then
    echo "OK diagnostic sample: ${diagnostic_name} (${label})"
    return 0
  fi

  level="$(diagnostic_level_number "${block}")"
  if (( level > max_level )); then
    echo "FAIL diagnostic level too high for ${diagnostic_name}: expected <= ${max_level}, got ${level} (${label})" >&2
    echo "${block}" >&2
    return 1
  fi

  echo "OK diagnostic level: ${diagnostic_name}=${level} <= ${max_level} (${label})"
}

check_joint_state_required_sample() {
  local topic="$1"
  local label="$2"
  local expected_names_string="$3"
  local names_sample=""
  local positions_sample=""
  local observed_count="0"
  local expected_names=()

  read -r -a expected_names <<< "${expected_names_string}"
  if (( ${#expected_names[@]} == 0 )); then
    return 0
  fi

  names_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo "${topic}" --field name --once)"
  positions_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo "${topic}" --field position --once)"

  if ! printf "%s\n" "${names_sample}" | EXPECTED_NAMES="${expected_names_string}" python3 -c '
import ast
import os
import sys

expected = os.environ["EXPECTED_NAMES"].split()
observed = []
for line in sys.stdin:
    stripped = line.strip()
    if not stripped or stripped == "---":
        continue
    if stripped.startswith("["):
        try:
            value = ast.literal_eval(stripped)
        except (SyntaxError, ValueError):
            print(f"cannot parse JointState name list: {stripped}", file=sys.stderr)
            sys.exit(1)
        if not isinstance(value, list):
            print(f"JointState name field is not a list: {stripped}", file=sys.stderr)
            sys.exit(1)
        observed.extend(str(item) for item in value)
    elif stripped.startswith("-"):
        observed.append(stripped[1:].strip().strip(chr(34)).strip(chr(39)))

if len(observed) < len(expected):
    print(f"too few JointState names: expected at least {len(expected)}, got {len(observed)}", file=sys.stderr)
    sys.exit(1)

missing = [name for name in expected if name not in observed]
if missing:
    print("missing JointState names: " + ", ".join(missing), file=sys.stderr)
    sys.exit(1)
'; then
    echo "FAIL ${label} JointState names do not contain required joints on ${topic}" >&2
    echo "${names_sample}" >&2
    exit 1
  fi

  if ! printf "%s\n" "${positions_sample}" | EXPECTED_COUNT="${#expected_names[@]}" python3 -c '
import ast
import math
import os
import sys

values = []
for line in sys.stdin:
    stripped = line.strip()
    if not stripped or stripped == "---":
        continue
    if stripped.startswith("array("):
        start = stripped.find("[")
        end = stripped.rfind("]")
        if start < 0 or end < start:
            print(f"cannot parse JointState position array: {stripped}", file=sys.stderr)
            sys.exit(1)
        try:
            parsed = ast.literal_eval(stripped[start : end + 1])
        except (SyntaxError, ValueError):
            print(f"cannot parse JointState position array: {stripped}", file=sys.stderr)
            sys.exit(1)
        if not isinstance(parsed, list):
            print(f"JointState position array is not backed by a list: {stripped}", file=sys.stderr)
            sys.exit(1)
        raw_values = parsed
    elif stripped.startswith("["):
        try:
            parsed = ast.literal_eval(stripped)
        except (SyntaxError, ValueError):
            print(f"cannot parse JointState position list: {stripped}", file=sys.stderr)
            sys.exit(1)
        if not isinstance(parsed, list):
            print(f"JointState position field is not a list: {stripped}", file=sys.stderr)
            sys.exit(1)
        raw_values = parsed
    elif stripped.startswith("-"):
        raw_values = [stripped[1:].strip()]
    else:
        continue
    for raw in raw_values:
        try:
            value = float(raw)
        except (TypeError, ValueError):
            print(f"non-numeric JointState position: {raw}", file=sys.stderr)
            sys.exit(1)
        if not math.isfinite(value):
            print(f"non-finite JointState position: {raw}", file=sys.stderr)
            sys.exit(1)
        values.append(value)

expected_count = int(os.environ["EXPECTED_COUNT"])
if len(values) < expected_count:
    print(f"too few JointState positions: expected at least {expected_count}, got {len(values)}", file=sys.stderr)
    sys.exit(1)
'; then
    echo "FAIL ${label} JointState positions are not finite on ${topic}" >&2
    echo "${positions_sample}" >&2
    exit 1
  fi

  echo "OK ${label} JointState required joints and finite positions: ${topic}"
}

check_kinematics_typed_finite_sample() {
  local sample=""
  sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/kinematics_typed --once)"

  if ! printf "%s\n" "${sample}" | python3 -c '
import math
import re
import sys

text = sys.stdin.read()
required_fields = [
    "x_m",
    "y_m",
    "z_m",
    "yaw_rad",
    "pitch_rad",
    "radial_reach_m",
    "base_yaw_rad",
    "shoulder_pitch_rad",
    "elbow_pitch_rad",
    "wrist_pitch_rad",
    "gripper_rad",
]
for field in required_fields:
    match = re.search(rf"^{field}:\s*([^\s]+)\s*$", text, re.MULTILINE)
    if not match:
        print(f"missing kinematics field: {field}", file=sys.stderr)
        sys.exit(1)
    try:
        value = float(match.group(1))
    except ValueError:
        print(f"non-numeric kinematics field {field}: {match.group(1)}", file=sys.stderr)
        sys.exit(1)
    if not math.isfinite(value):
        print(f"non-finite kinematics field {field}: {match.group(1)}", file=sys.stderr)
        sys.exit(1)
'; then
    echo "FAIL kinematics typed sample has missing or non-finite fields" >&2
    echo "${sample}" >&2
    exit 1
  fi

  echo "OK kinematics typed finite sample"
}

run_diagnostics_checks() {
  check_diagnostic_max_level \
    "motionbrain/controller" \
    "${EXPECTED_CONTROLLER_DIAGNOSTIC_MAX_LEVEL}" \
    "controller"
  check_diagnostic_max_level \
    "motionbrain/shoulder_feedback" \
    "${EXPECTED_SHOULDER_DIAGNOSTIC_MAX_LEVEL}" \
    "M4 shoulder feedback"
  check_diagnostic_max_level \
    "motionbrain/routine_executor" \
    "${EXPECTED_ROUTINE_EXECUTOR_DIAGNOSTIC_MAX_LEVEL}" \
    "routine executor"
  check_diagnostic_max_level \
    "motionbrain/feedback" \
    "${EXPECTED_FEEDBACK_DIAGNOSTIC_MAX_LEVEL}" \
    "routine feedback readiness"
  check_diagnostic_max_level \
    "motionbrain/teleop_sensor" \
    "${EXPECTED_TELEOP_SENSOR_DIAGNOSTIC_MAX_LEVEL}" \
    "teleop and STM32 sensor"
  check_diagnostic_max_level \
    "motionbrain/camera_perception" \
    "${EXPECTED_CAMERA_PERCEPTION_DIAGNOSTIC_MAX_LEVEL}" \
    "camera perception"
  if ! grep -Fq 'base_yaw_fault' <<< "${diagnostics_sample}"; then
    echo "FAIL diagnostics sample missing base_yaw_fault key" >&2
    echo "${diagnostics_sample}" >&2
    return 1
  fi
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

for lifecycle_node in "${expected_lifecycle_nodes[@]}"; do
  lifecycle_state="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 lifecycle get "/${lifecycle_node}")"
  if ! grep -Fq "active" <<< "${lifecycle_state}"; then
    echo "FAIL lifecycle get not active: /${lifecycle_node}" >&2
    echo "${lifecycle_state}" >&2
    exit 1
  fi
  echo "OK lifecycle get active: /${lifecycle_node}"
done

diagnostics_deadline=$((SECONDS + SAMPLE_TIMEOUT_SECONDS))
diagnostics_check_output=""
diagnostics_last_error=""
diagnostics_sample=""
while (( SECONDS <= diagnostics_deadline )); do
  diagnostics_sample="$(timeout 8 ros2 topic echo /motionbrain/diagnostics --once 2>/dev/null || true)"
  if [[ -z "${diagnostics_sample}" ]]; then
    diagnostics_last_error="diagnostics sample unavailable"
  elif diagnostics_check_output="$(run_diagnostics_checks 2>&1)"; then
    printf "%s\n" "${diagnostics_check_output}"
    diagnostics_last_error=""
    break
  else
    diagnostics_last_error="${diagnostics_check_output}"
  fi
  sleep "${TOPIC_POLL_SECONDS}"
done
if [[ -n "${diagnostics_last_error}" ]]; then
  echo "FAIL diagnostics did not reach expected levels before timeout" >&2
  echo "${diagnostics_last_error}" >&2
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

if [[ "${CHECK_ROUTINE_RUN_REJECTION}" == "1" ]]; then
  routine_run_service_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 service call /motionbrain/routine_command \
    motionbrain_msgs/srv/GuardedRoutineCommand "{action: run, routine_name: inspect}")"
  if ! grep -Eq 'success[:=][[:space:]]*(false|False)' <<< "${routine_run_service_sample}"; then
    echo "FAIL routine command service run rejection is not success=false" >&2
    echo "${routine_run_service_sample}" >&2
    exit 1
  fi
  if ! grep -Eq 'forwarded[:=][[:space:]]*(false|False)' <<< "${routine_run_service_sample}"; then
    echo "FAIL routine command service run rejection is not forwarded=false" >&2
    echo "${routine_run_service_sample}" >&2
    exit 1
  fi
  if ! grep -Fq 'routine_execute_disabled_by_bridge_policy' <<< "${routine_run_service_sample}"; then
    echo "FAIL routine command service run rejection missing bridge policy result" >&2
    echo "${routine_run_service_sample}" >&2
    exit 1
  fi
  echo "OK routine command service run rejection sample"
fi

routine_action_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 action send_goal /motionbrain/guarded_routine \
  motionbrain_msgs/action/GuardedRoutine "{action: status}")"
if ! grep -Eq 'success[:=][[:space:]]*(true|True)' <<< "${routine_action_sample}"; then
  echo "FAIL guarded routine action status sample is not success=true" >&2
  echo "${routine_action_sample}" >&2
  exit 1
fi
echo "OK guarded routine action status sample"

if [[ "${CHECK_ROUTINE_RUN_REJECTION}" == "1" ]]; then
  routine_run_action_sample="$(timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 action send_goal /motionbrain/guarded_routine \
    motionbrain_msgs/action/GuardedRoutine "{action: run, routine_name: inspect}")"
  if ! grep -Eq 'success[:=][[:space:]]*(false|False)' <<< "${routine_run_action_sample}"; then
    echo "FAIL guarded routine action run rejection is not success=false" >&2
    echo "${routine_run_action_sample}" >&2
    exit 1
  fi
  if ! grep -Eq 'forwarded[:=][[:space:]]*(false|False)' <<< "${routine_run_action_sample}"; then
    echo "FAIL guarded routine action run rejection is not forwarded=false" >&2
    echo "${routine_run_action_sample}" >&2
    exit 1
  fi
  if ! grep -Fq 'routine_execute_disabled_by_bridge_policy' <<< "${routine_run_action_sample}"; then
    echo "FAIL guarded routine action run rejection missing bridge policy result" >&2
    echo "${routine_run_action_sample}" >&2
    exit 1
  fi
  echo "OK guarded routine action run rejection sample"
fi

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

check_joint_state_required_sample \
  "${EXPECTED_KINEMATICS_JOINT_STATES_TOPIC}" \
  "kinematics input" \
  "${EXPECTED_KINEMATICS_JOINT_NAMES}"

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/end_effector_pose --once >/dev/null
echo "OK end-effector pose sample"

check_kinematics_typed_finite_sample

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/control_guard_typed --once >/dev/null
echo "OK control guard typed sample"

timeout "${SAMPLE_TIMEOUT_SECONDS}" ros2 topic echo /motionbrain/mission_state_typed --once >/dev/null
echo "OK mission state typed sample"
