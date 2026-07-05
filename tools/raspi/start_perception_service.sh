#!/usr/bin/env bash
set -euo pipefail

REPO="${MOTIONBRAIN_REPO:-/home/motionbrain/develop/arduino/motionbrain}"
PYTHON="${MOTIONBRAIN_PERCEPTION_PYTHON:-${MOTIONBRAIN_OPENCV_PYTHON:-/home/motionbrain/.cache/motionbrain/opencv-venv/bin/python}}"
DISCOVERY_PYTHON="${MOTIONBRAIN_DISCOVERY_PYTHON:-/usr/bin/python3}"
CAMERA_PROFILE_PYTHON="${MOTIONBRAIN_CAMERA_PROFILE_PYTHON:-${DISCOVERY_PYTHON}}"
CAMERA_URL="${MOTIONBRAIN_CAMERA_URL:-http://motionbrain-cam.local}"
CAMERA_QUALITY="${MOTIONBRAIN_CAMERA_QUALITY:-15}"
CAMERA_MIN_STABLE_QUALITY="${MOTIONBRAIN_CAMERA_MIN_STABLE_QUALITY:-15}"
CAMERA_STABLE_QUALITY_FLOOR="${MOTIONBRAIN_CAMERA_STABLE_QUALITY_FLOOR:-15}"
PERCEPTION_TIMEOUT="${MOTIONBRAIN_PERCEPTION_TIMEOUT:-1.5}"
PERCEPTION_MAX_CAMERA_TIMEOUT="${MOTIONBRAIN_PERCEPTION_MAX_CAMERA_TIMEOUT:-3.0}"

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

if [[ "${MOTIONBRAIN_ALLOW_LONG_CAMERA_TIMEOUT:-0}" != "1" ]]; then
  PERCEPTION_TIMEOUT="$(
    "${DISCOVERY_PYTHON}" -c '
import sys

timeout = float(sys.argv[1])
maximum = float(sys.argv[2])
print(min(timeout, maximum))
' "${PERCEPTION_TIMEOUT}" "${PERCEPTION_MAX_CAMERA_TIMEOUT}"
  )"
fi

cd "${REPO}"

if [[ "${MOTIONBRAIN_DISCOVERY:-1}" != "0" ]]; then
  discovery_args=(
    --kind camera
    --preferred "${CAMERA_URL}"
    --timeout "${MOTIONBRAIN_DISCOVERY_TIMEOUT:-0.35}"
    --workers "${MOTIONBRAIN_DISCOVERY_WORKERS:-64}"
  )
  if [[ -n "${MOTIONBRAIN_DISCOVERY_CIDR:-}" ]]; then
    discovery_args+=(--cidr "${MOTIONBRAIN_DISCOVERY_CIDR}")
  fi
  if resolved_camera_url="$(
    "${DISCOVERY_PYTHON}" "${REPO}/tools/raspi/discover_device_url.py" "${discovery_args[@]}" 2>/dev/null
  )"; then
    CAMERA_URL="${resolved_camera_url}"
  fi
fi

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
  "${REPO}/tools/motionbrain_perception_service.py"
  --host "${MOTIONBRAIN_PERCEPTION_HOST:-127.0.0.1}"
  --port "${MOTIONBRAIN_PERCEPTION_PORT:-8766}"
  --camera-url "${CAMERA_URL}"
  --timeout "${PERCEPTION_TIMEOUT}"
  --interval "${MOTIONBRAIN_PERCEPTION_INTERVAL:-2.0}"
  --stale-seconds "${MOTIONBRAIN_PERCEPTION_STALE_SECONDS:-8.0}"
  --failure-backoff-initial "${MOTIONBRAIN_PERCEPTION_FAILURE_BACKOFF_INITIAL:-1.0}"
  --failure-backoff-max "${MOTIONBRAIN_PERCEPTION_FAILURE_BACKOFF_MAX:-8.0}"
  --display-hold-seconds "${MOTIONBRAIN_DISPLAY_HOLD_SECONDS:-2.5}"
  --detector-mode "${MOTIONBRAIN_DETECTOR_MODE:-object}"
  --detect-color "${MOTIONBRAIN_DETECT_COLOR:-red}"
  --align-deadband "${MOTIONBRAIN_ALIGN_DEADBAND:-0.15}"
  --object-backend "${MOTIONBRAIN_OBJECT_BACKEND:-opencv-dnn}"
  --object-model "${MOTIONBRAIN_OBJECT_MODEL:-/home/motionbrain/.cache/motionbrain/models/yolov5s.onnx}"
  --object-labels "${MOTIONBRAIN_OBJECT_LABELS:-${REPO}/config/coco80.labels}"
  --object-target "${MOTIONBRAIN_OBJECT_TARGET:-cup}"
  --object-min-confidence "${MOTIONBRAIN_OBJECT_MIN_CONFIDENCE:-0.5}"
  --object-nms-threshold "${MOTIONBRAIN_OBJECT_NMS_THRESHOLD:-0.45}"
  --object-input-size "${MOTIONBRAIN_OBJECT_INPUT_SIZE:-640}"
  --target-policy "${MOTIONBRAIN_TARGET_POLICY:-largest}"
  --opencv-threads "${MOTIONBRAIN_OPENCV_THREADS:-1}"
)

if [[ -n "${MOTIONBRAIN_OBJECT_TARGET_ALIASES:-}" ]]; then
  args+=(--object-target-alias "${MOTIONBRAIN_OBJECT_TARGET_ALIASES}")
fi

exec "${PYTHON}" "${args[@]}"
