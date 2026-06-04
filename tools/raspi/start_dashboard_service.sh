#!/usr/bin/env bash
set -euo pipefail

REPO="${MOTIONBRAIN_REPO:-/home/motionbrain/develop/arduino/motionbrain}"
PYTHON="${MOTIONBRAIN_DASHBOARD_PYTHON:-/usr/bin/python3}"
MOTION_HOST="${MOTIONBRAIN_MOTION_HOST:-${MOTIONBRAIN_HOST:-motionbrain.local}}"
CAMERA_URL="${MOTIONBRAIN_CAMERA_URL-http://motionbrain-cam.local}"
PERCEPTION_URL="${MOTIONBRAIN_PERCEPTION_URL-http://127.0.0.1:8766}"

cd "${REPO}"

args=(
  "${REPO}/tools/motionbrain_dashboard.py"
  --host "${MOTIONBRAIN_DASHBOARD_HOST:-0.0.0.0}"
  --port "${MOTIONBRAIN_DASHBOARD_PORT:-8765}"
  --motion-host "${MOTION_HOST}"
  --motion-port "${MOTIONBRAIN_MOTION_PORT:-80}"
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
