#!/usr/bin/env bash
set -euo pipefail

DASHBOARD_URL="${MOTIONBRAIN_DASHBOARD_URL:-http://127.0.0.1:8765}"
PERCEPTION_URL="${MOTIONBRAIN_PERCEPTION_URL:-http://127.0.0.1:8766}"
CHECK_SERVICE="${CHECK_SERVICE:-0}"

DASHBOARD_URL="${DASHBOARD_URL%/}"
PERCEPTION_URL="${PERCEPTION_URL%/}"

if [[ "${CHECK_SERVICE}" == "1" ]]; then
  systemctl is-active --quiet motionbrain-perception.service
  echo "OK service active: motionbrain-perception.service"
  systemctl is-active --quiet motionbrain-dashboard.service
  echo "OK service active: motionbrain-dashboard.service"
fi

curl -fsS "${PERCEPTION_URL}/health" >/dev/null
echo "OK perception health: ${PERCEPTION_URL}/health"

curl -fsS "${PERCEPTION_URL}/api/detection" >/dev/null
echo "OK perception detection: ${PERCEPTION_URL}/api/detection"

curl -fsS "${DASHBOARD_URL}/api/config" >/dev/null
echo "OK dashboard config: ${DASHBOARD_URL}/api/config"

curl -fsS "${DASHBOARD_URL}/api/status" >/dev/null
echo "OK dashboard status: ${DASHBOARD_URL}/api/status"
