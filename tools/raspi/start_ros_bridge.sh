#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-/home/motionbrain/develop/arduino/motionbrain/ros2_ws}"
MOTION_HOST="${MOTIONBRAIN_HOST:-motionbrain.local}"
CAMERA_URL="${MOTIONBRAIN_CAMERA_URL:-http://motionbrain-cam.local}"
DETECT_COLOR="${MOTIONBRAIN_DETECT_COLOR:-red}"
POLL_INTERVAL="${MOTIONBRAIN_POLL_INTERVAL:-1.0}"
HTTP_TIMEOUT="${MOTIONBRAIN_HTTP_TIMEOUT:-4.0}"
EVENTS_LIMIT="${MOTIONBRAIN_EVENTS_LIMIT:-8}"

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

exec ros2 launch motionbrain_ros_bridge motionbrain_home_wifi.launch.py \
  motion_host:="${MOTION_HOST}" \
  camera_url:="${CAMERA_URL}" \
  detect_color:="${DETECT_COLOR}" \
  poll_interval:="${POLL_INTERVAL}" \
  http_timeout:="${HTTP_TIMEOUT}" \
  events_limit:="${EVENTS_LIMIT}"
