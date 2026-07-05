#!/usr/bin/env bash
set -euo pipefail

REPO="${MOTIONBRAIN_REPO:-/home/motionbrain/develop/arduino/motionbrain}"
PYTHON="${MOTIONBRAIN_DASHBOARD_PYTHON:-/usr/bin/python3}"
DISCOVERY_PYTHON="${MOTIONBRAIN_DISCOVERY_PYTHON:-/usr/bin/python3}"
CAMERA_PROFILE_PYTHON="${MOTIONBRAIN_CAMERA_PROFILE_PYTHON:-${DISCOVERY_PYTHON}}"
MOTION_HOST="${MOTIONBRAIN_MOTION_HOST:-${MOTIONBRAIN_HOST:-motionbrain.local}}"
MOTION_PORT="${MOTIONBRAIN_MOTION_PORT:-80}"
CAMERA_URL="${MOTIONBRAIN_CAMERA_URL-http://motionbrain-cam.local}"
PERCEPTION_URL="${MOTIONBRAIN_PERCEPTION_URL-http://127.0.0.1:8766}"
CAMERA_QUALITY="${MOTIONBRAIN_CAMERA_QUALITY:-15}"
CAMERA_MIN_STABLE_QUALITY="${MOTIONBRAIN_CAMERA_MIN_STABLE_QUALITY:-15}"
CAMERA_STABLE_QUALITY_FLOOR="${MOTIONBRAIN_CAMERA_STABLE_QUALITY_FLOOR:-15}"

if [[ "${MOTIONBRAIN_ALLOW_UNSTABLE_CAMERA_QUALITY:-0}" != "1" ]] &&
   [[ "${CAMERA_MIN_STABLE_QUALITY}" =~ ^[0-9]+$ ]] &&
   [[ "${CAMERA_STABLE_QUALITY_FLOOR}" =~ ^[0-9]+$ ]] &&
   (( CAMERA_MIN_STABLE_QUALITY < CAMERA_STABLE_QUALITY_FLOOR )); then
  CAMERA_MIN_STABLE_QUALITY="${CAMERA_STABLE_QUALITY_FLOOR}"
fi

if [[ "${MOTIONBRAIN_ALLOW_UNSTABLE_CAMERA_QUALITY:-0}" != "1" ]] &&
   [[ "${CAMERA_QUALITY}" =~ ^[0-9]+$ ]] &&
   [[ "${CAMERA_MIN_STABLE_QUALITY}" =~ ^[0-9]+$ ]] &&
   (( CAMERA_QUALITY < CAMERA_MIN_STABLE_QUALITY )); then
  echo "Warning: raising ESP32-CAM JPEG quality from ${CAMERA_QUALITY} to stable minimum ${CAMERA_MIN_STABLE_QUALITY}" >&2
  CAMERA_QUALITY="${CAMERA_MIN_STABLE_QUALITY}"
fi

cd "${REPO}"

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

MOTION_URL="$(discover_device_url controller "http://${MOTION_HOST}:${MOTION_PORT}")"
MOTION_AUTHORITY="${MOTION_URL#http://}"
MOTION_AUTHORITY="${MOTION_AUTHORITY#https://}"
MOTION_AUTHORITY="${MOTION_AUTHORITY%%/*}"
if [[ "${MOTION_AUTHORITY}" == *:* ]]; then
  MOTION_HOST="${MOTION_AUTHORITY%%:*}"
  MOTION_PORT="${MOTION_AUTHORITY##*:}"
else
  MOTION_HOST="${MOTION_AUTHORITY}"
fi

CAMERA_URL="$(discover_device_url camera "${CAMERA_URL}")"

if [[ "${MOTIONBRAIN_CAMERA_PROFILE:-1}" != "0" ]]; then
  profile_args=(
    --camera-url "${CAMERA_URL}"
    --framesize "${MOTIONBRAIN_CAMERA_FRAMESIZE:-qvga}"
    --quality "${CAMERA_QUALITY}"
    --timeout "${MOTIONBRAIN_CAMERA_PROFILE_TIMEOUT:-3.0}"
  )
  if ! "${CAMERA_PROFILE_PYTHON}" "${REPO}/tools/raspi/apply_camera_profile.py" "${profile_args[@]}" >&2; then
    echo "Warning: ESP32-CAM profile could not be applied for ${CAMERA_URL}" >&2
  fi
fi

args=(
  "${REPO}/tools/motionbrain_dashboard.py"
  --host "${MOTIONBRAIN_DASHBOARD_HOST:-0.0.0.0}"
  --port "${MOTIONBRAIN_DASHBOARD_PORT:-8765}"
  --motion-host "${MOTION_HOST}"
  --motion-port "${MOTION_PORT}"
  --camera-url "${CAMERA_URL}"
  --perception-url "${PERCEPTION_URL}"
  --detect-color "${MOTIONBRAIN_DETECT_COLOR:-red}"
  --timeout "${MOTIONBRAIN_DASHBOARD_TIMEOUT:-2.0}"
  --events-limit "${MOTIONBRAIN_EVENTS_LIMIT:-12}"
  --align-nudge-ms "${MOTIONBRAIN_ALIGN_NUDGE_MS:-250}"
  --align-percent "${MOTIONBRAIN_ALIGN_PERCENT:-25}"
  --grasp-target-label "${MOTIONBRAIN_GRASP_TARGET_LABEL:-cup}"
  --grasp-min-confidence "${MOTIONBRAIN_GRASP_MIN_CONFIDENCE:-0.5}"
)

exec "${PYTHON}" "${args[@]}"
