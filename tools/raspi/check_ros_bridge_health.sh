#!/usr/bin/env bash
set -eo pipefail

SERVICE_NAME="${MOTIONBRAIN_SERVICE_NAME:-motionbrain-ros-bridge.service}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-/home/motionbrain/develop/arduino/motionbrain/ros2_ws}"
CHECK_SERVICE="${CHECK_SERVICE:-1}"
STRICT_CAMERA_AVAILABLE="${STRICT_CAMERA_AVAILABLE:-0}"
TOPIC_WAIT_SECONDS="${TOPIC_WAIT_SECONDS:-20}"
TOPIC_POLL_SECONDS="${TOPIC_POLL_SECONDS:-1}"

required_topics=(
  "/motionbrain/status_typed"
  "/motionbrain/routine"
  "/motionbrain/routine_typed"
  "/camera/detection_typed"
  "/joint_states"
  "/motionbrain/end_effector_pose"
  "/motionbrain/kinematics_typed"
  "/motionbrain/control_guard_typed"
  "/motionbrain/mission_state_typed"
)

required_services=(
  "/motionbrain/routine_command"
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

timeout 10 ros2 topic echo /motionbrain/status_typed --once >/dev/null
echo "OK status typed sample"

timeout 10 ros2 topic echo /motionbrain/routine --once >/dev/null
echo "OK routine diagnostics sample"

timeout 10 ros2 topic echo /motionbrain/routine_typed --once >/dev/null
echo "OK routine typed diagnostics sample"

routine_service_sample="$(timeout 10 ros2 service call /motionbrain/routine_command \
  motionbrain_msgs/srv/GuardedRoutineCommand "{action: status}")"
if ! grep -Eq 'success[:=][[:space:]]*(true|True)' <<< "${routine_service_sample}"; then
  echo "FAIL routine command service status sample is not success=true" >&2
  echo "${routine_service_sample}" >&2
  exit 1
fi
echo "OK routine command service status sample"

camera_detection_sample="$(timeout 10 ros2 topic echo /camera/detection_typed --once)"
if [[ "${STRICT_CAMERA_AVAILABLE}" == "1" ]] && ! grep -Eq '^available: true$' <<< "${camera_detection_sample}"; then
  echo "FAIL camera detection typed sample is not available=true" >&2
  echo "${camera_detection_sample}" >&2
  exit 1
fi
echo "OK camera detection typed sample"

timeout 10 ros2 topic echo /joint_states --once >/dev/null
echo "OK joint state sample"

timeout 10 ros2 topic echo /motionbrain/end_effector_pose --once >/dev/null
echo "OK end-effector pose sample"

timeout 10 ros2 topic echo /motionbrain/kinematics_typed --once >/dev/null
echo "OK kinematics typed sample"

timeout 10 ros2 topic echo /motionbrain/control_guard_typed --once >/dev/null
echo "OK control guard typed sample"

timeout 10 ros2 topic echo /motionbrain/mission_state_typed --once >/dev/null
echo "OK mission state typed sample"
