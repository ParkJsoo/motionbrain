#!/usr/bin/env bash
set -euo pipefail

REPO="${MOTIONBRAIN_REPO:-/home/motionbrain/develop/arduino/motionbrain}"
DASHBOARD_ENV="${MOTIONBRAIN_DASHBOARD_ENV:-/etc/motionbrain/dashboard.env}"
PERCEPTION_ENV="${MOTIONBRAIN_PERCEPTION_ENV:-/etc/motionbrain/perception.env}"

if [[ -f "${DASHBOARD_ENV}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${DASHBOARD_ENV}"
  set +a
fi

if [[ -f "${PERCEPTION_ENV}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${PERCEPTION_ENV}"
  set +a
fi

DISCOVERY_PYTHON="${MOTIONBRAIN_DISCOVERY_PYTHON:-/usr/bin/python3}"
CAMERA_PROFILE_PYTHON="${MOTIONBRAIN_CAMERA_PROFILE_PYTHON:-${DISCOVERY_PYTHON}}"
DASHBOARD_URL="${MOTIONBRAIN_DASHBOARD_URL:-http://127.0.0.1:${MOTIONBRAIN_DASHBOARD_PORT:-8765}}"
PERCEPTION_URL="${MOTIONBRAIN_PERCEPTION_URL:-http://127.0.0.1:8766}"
MOTION_HOST="${MOTIONBRAIN_MOTION_HOST:-${MOTIONBRAIN_HOST:-motionbrain.local}}"
MOTION_PORT="${MOTIONBRAIN_MOTION_PORT:-80}"
CAMERA_URL="${MOTIONBRAIN_CAMERA_URL:-http://motionbrain-cam.local}"

DASHBOARD_URL="${DASHBOARD_URL%/}"
PERCEPTION_URL="${PERCEPTION_URL%/}"

discover_device_url() {
  local kind="$1"
  local preferred="$2"
  local discovery_args=(
    --kind "${kind}"
    --preferred "${preferred}"
    --timeout "${MOTIONBRAIN_DISCOVERY_TIMEOUT:-0.35}"
    --workers "${MOTIONBRAIN_DISCOVERY_WORKERS:-64}"
  )
  if [[ -n "${MOTIONBRAIN_DISCOVERY_CIDR:-}" ]]; then
    discovery_args+=(--cidr "${MOTIONBRAIN_DISCOVERY_CIDR}")
  fi

  "${DISCOVERY_PYTHON}" "${REPO}/tools/raspi/discover_device_url.py" "${discovery_args[@]}" 2>/dev/null || true
}

json_field() {
  local field="$1"
  "${DISCOVERY_PYTHON}" -c '
import json
import sys

field = sys.argv[1]
try:
    payload = json.load(sys.stdin)
except json.JSONDecodeError:
    sys.exit(1)
value = payload.get(field, "")
if isinstance(value, bool):
    print("true" if value else "false")
elif value is None:
    print("")
else:
    print(value)
' "${field}"
}

url_host() {
  "${DISCOVERY_PYTHON}" -c '
import sys
import urllib.parse

parsed = urllib.parse.urlparse(sys.argv[1])
print(parsed.hostname or "")
' "$1"
}

controller_url="$(discover_device_url controller "http://${MOTION_HOST}:${MOTION_PORT}")"
camera_url="$(discover_device_url camera "${CAMERA_URL}")"
if [[ -z "${controller_url}" && -z "${camera_url}" ]]; then
  echo "No MotionBrain devices discovered; leaving services unchanged"
  exit 0
fi

restart_needed=0
reasons=()
camera_profile_result=""
if [[ -n "${camera_url}" && "${MOTIONBRAIN_CAMERA_PROFILE:-1}" != "0" ]]; then
  profile_args=(
    --camera-url "${camera_url}"
    --framesize "${MOTIONBRAIN_CAMERA_FRAMESIZE:-qvga}"
    --quality "${MOTIONBRAIN_CAMERA_QUALITY:-4}"
    --timeout "${MOTIONBRAIN_CAMERA_PROFILE_TIMEOUT:-3.0}"
  )
  if camera_profile_result="$("${CAMERA_PROFILE_PYTHON}" "${REPO}/tools/raspi/apply_camera_profile.py" "${profile_args[@]}" 2>/dev/null)"; then
    if [[ "${camera_profile_result}" == "updated" ]]; then
      restart_needed=1
      reasons+=("camera_profile_updated")
    fi
  else
    echo "Warning: ESP32-CAM profile could not be applied for ${camera_url}" >&2
  fi
fi

dashboard_config="$(curl -fsS --max-time "${MOTIONBRAIN_RECONCILE_TIMEOUT:-4}" "${DASHBOARD_URL}/api/config" 2>/dev/null || true)"
perception_health="$(curl -fsS --max-time "${MOTIONBRAIN_RECONCILE_TIMEOUT:-4}" "${PERCEPTION_URL}/health" 2>/dev/null || true)"

if [[ -z "${dashboard_config}" ]]; then
  restart_needed=1
  reasons+=("dashboard_config_unavailable")
else
  dashboard_motion_url="$(printf "%s" "${dashboard_config}" | json_field motionBaseUrl || true)"
  dashboard_camera_url="$(printf "%s" "${dashboard_config}" | json_field cameraUrl || true)"
  if [[ -n "${controller_url}" && "$(url_host "${dashboard_motion_url}")" != "$(url_host "${controller_url}")" ]]; then
    restart_needed=1
    reasons+=("controller_url_changed")
  fi
  if [[ -n "${camera_url}" && "${dashboard_camera_url}" != "${camera_url}" ]]; then
    restart_needed=1
    reasons+=("dashboard_camera_url_changed")
  fi
fi

if [[ -z "${perception_health}" ]]; then
  restart_needed=1
  reasons+=("perception_health_unavailable")
else
  perception_ok="$(printf "%s" "${perception_health}" | json_field ok || true)"
  perception_camera_url="$(printf "%s" "${perception_health}" | json_field cameraUrl || true)"
  if [[ -n "${camera_url}" && "${perception_camera_url}" != "${camera_url}" ]]; then
    restart_needed=1
    reasons+=("perception_camera_url_changed")
  fi
  if [[ "${perception_ok}" != "true" && -n "${camera_url}" ]]; then
    restart_needed=1
    reasons+=("perception_not_ok")
  fi
fi

if [[ "${restart_needed}" == "0" ]]; then
  echo "MotionBrain dashboard services already match discovered devices"
  exit 0
fi

echo "Restarting MotionBrain dashboard services: ${reasons[*]}"
systemctl restart motionbrain-perception.service motionbrain-dashboard.service
