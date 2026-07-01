#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REPO="${MOTIONBRAIN_REPO:-/home/motionbrain/develop/arduino/motionbrain}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-/home/motionbrain/develop/arduino/motionbrain/ros2_ws}"
DISCOVERY_PYTHON="${MOTIONBRAIN_DISCOVERY_PYTHON:-/usr/bin/python3}"
MOTION_HOST="${MOTIONBRAIN_HOST:-motionbrain.local}"
CAMERA_URL="${MOTIONBRAIN_CAMERA_URL:-http://motionbrain-cam.local}"
PERCEPTION_URL="${MOTIONBRAIN_PERCEPTION_URL:-}"
DETECT_COLOR="${MOTIONBRAIN_DETECT_COLOR:-red}"
POLL_INTERVAL="${MOTIONBRAIN_POLL_INTERVAL:-1.0}"
HTTP_TIMEOUT="${MOTIONBRAIN_HTTP_TIMEOUT:-4.0}"
EVENTS_LIMIT="${MOTIONBRAIN_EVENTS_LIMIT:-8}"
ENABLE_JOINT_STATE_BRIDGE="${MOTIONBRAIN_ENABLE_JOINT_STATE_BRIDGE:-true}"
JOINT_STATE_AUTOSTART="${MOTIONBRAIN_JOINT_STATE_AUTOSTART:-true}"
JOINT_STATES_TOPIC="${MOTIONBRAIN_JOINT_STATES_TOPIC:-/joint_states}"
ESTIMATED_JOINT_STATES_TOPIC="${MOTIONBRAIN_ESTIMATED_JOINT_STATES_TOPIC:-/motionbrain/estimated_joint_states}"
KINEMATICS_JOINT_STATES_TOPIC="${MOTIONBRAIN_KINEMATICS_JOINT_STATES_TOPIC:-${JOINT_STATES_TOPIC}}"
JOINT_STATES_OUTPUT="${MOTIONBRAIN_JOINT_STATES_OUTPUT:-estimated}"
SHOULDER_FEEDBACK_CALIBRATION_ENABLED="${MOTIONBRAIN_SHOULDER_FEEDBACK_CALIBRATION_ENABLED:-false}"
SHOULDER_SENSOR_ZERO_DEG="${MOTIONBRAIN_SHOULDER_SENSOR_ZERO_DEG:-0.0}"
SHOULDER_DIRECTION_SIGN="${MOTIONBRAIN_SHOULDER_DIRECTION_SIGN:-1}"
SHOULDER_ROS_JOINT_ZERO_RAD="${MOTIONBRAIN_SHOULDER_ROS_JOINT_ZERO_RAD:-0.0}"
KINEMATICS_AUTOSTART="${MOTIONBRAIN_KINEMATICS_AUTOSTART:-true}"
CONTROL_GUARD_AUTOSTART="${MOTIONBRAIN_CONTROL_GUARD_AUTOSTART:-true}"
MISSION_SUPERVISOR_AUTOSTART="${MOTIONBRAIN_MISSION_SUPERVISOR_AUTOSTART:-true}"

discover_device_url() {
  local kind="$1"
  local preferred="$2"
  if [[ "${MOTIONBRAIN_DISCOVERY:-1}" == "0" || -z "${preferred}" ]]; then
    printf "%s" "${preferred}"
    return 0
  fi

  local discovery_args=(
    --kind "${kind}"
    --preferred "${preferred}"
    --timeout "${MOTIONBRAIN_DISCOVERY_TIMEOUT:-0.35}"
    --workers "${MOTIONBRAIN_DISCOVERY_WORKERS:-64}"
  )
  if [[ -n "${MOTIONBRAIN_DISCOVERY_CIDR:-}" ]]; then
    discovery_args+=(--cidr "${MOTIONBRAIN_DISCOVERY_CIDR}")
  fi

  local resolved_url
  if resolved_url="$(
    "${DISCOVERY_PYTHON}" "${REPO}/tools/raspi/discover_device_url.py" "${discovery_args[@]}" 2>/dev/null
  )"; then
    printf "%s" "${resolved_url}"
    return 0
  fi
  printf "%s" "${preferred}"
}

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 2
fi

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Workspace setup not found: ${WORKSPACE}/install/setup.bash" >&2
  echo "Build first: colcon build --packages-select motionbrain_msgs motionbrain_control motionbrain_mission motionbrain_ros_bridge motionbrain_description" >&2
  exit 2
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE}/install/setup.bash"

set -u

MOTION_URL="$(discover_device_url controller "http://${MOTION_HOST}:80")"
MOTION_AUTHORITY="${MOTION_URL#http://}"
MOTION_AUTHORITY="${MOTION_AUTHORITY#https://}"
MOTION_AUTHORITY="${MOTION_AUTHORITY%%/*}"
if [[ "${MOTION_AUTHORITY}" == *:* ]]; then
  MOTION_HOST="${MOTION_AUTHORITY%%:*}"
else
  MOTION_HOST="${MOTION_AUTHORITY}"
fi

CAMERA_URL="$(discover_device_url camera "${CAMERA_URL}")"

echo "ROS2 bridge controller URL: http://${MOTION_HOST}:80" >&2
echo "ROS2 bridge camera URL: ${CAMERA_URL}" >&2

launch_args=(
  "motion_host:=${MOTION_HOST}"
  "camera_url:=${CAMERA_URL}"
  "detect_color:=${DETECT_COLOR}"
  "poll_interval:=${POLL_INTERVAL}"
  "http_timeout:=${HTTP_TIMEOUT}"
  "events_limit:=${EVENTS_LIMIT}"
  "enable_joint_state_bridge:=${ENABLE_JOINT_STATE_BRIDGE}"
  "joint_state_autostart:=${JOINT_STATE_AUTOSTART}"
  "joint_states_topic:=${JOINT_STATES_TOPIC}"
  "estimated_joint_states_topic:=${ESTIMATED_JOINT_STATES_TOPIC}"
  "kinematics_joint_states_topic:=${KINEMATICS_JOINT_STATES_TOPIC}"
  "joint_states_output:=${JOINT_STATES_OUTPUT}"
  "shoulder_feedback_calibration_enabled:=${SHOULDER_FEEDBACK_CALIBRATION_ENABLED}"
  "shoulder_sensor_zero_deg:=${SHOULDER_SENSOR_ZERO_DEG}"
  "shoulder_direction_sign:=${SHOULDER_DIRECTION_SIGN}"
  "shoulder_ros_joint_zero_rad:=${SHOULDER_ROS_JOINT_ZERO_RAD}"
  "kinematics_autostart:=${KINEMATICS_AUTOSTART}"
  "control_guard_autostart:=${CONTROL_GUARD_AUTOSTART}"
  "mission_supervisor_autostart:=${MISSION_SUPERVISOR_AUTOSTART}"
)

if [[ -n "${PERCEPTION_URL}" ]]; then
  launch_args+=("perception_url:=${PERCEPTION_URL}")
fi

exec ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py "${launch_args[@]}"
