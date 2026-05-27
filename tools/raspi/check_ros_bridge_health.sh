#!/usr/bin/env bash
set -eo pipefail

SERVICE_NAME="${MOTIONBRAIN_SERVICE_NAME:-motionbrain-ros-bridge.service}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-/home/motionbrain/develop/arduino/motionbrain/ros2_ws}"
CHECK_SERVICE="${CHECK_SERVICE:-1}"

required_topics=(
  "/motionbrain/status_typed"
  "/camera/detection_typed"
  "/joint_states"
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

topics="$(timeout 8 ros2 topic list)"
for topic in "${required_topics[@]}"; do
  if ! grep -qx "${topic}" <<< "${topics}"; then
    echo "FAIL missing topic: ${topic}" >&2
    echo "${topics}" >&2
    exit 1
  fi
  echo "OK topic: ${topic}"
done

timeout 10 ros2 topic echo /motionbrain/status_typed --once >/dev/null
echo "OK status typed sample"

timeout 10 ros2 topic echo /joint_states --once >/dev/null
echo "OK joint state sample"
